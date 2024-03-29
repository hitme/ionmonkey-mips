/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sw=4 et tw=80:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/*
 * JS execution context.
 */

#include <limits.h> /* make sure that <features.h> is included and we can use
                       __GLIBC__ to detect glibc presence */
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#ifdef ANDROID
# include <android/log.h>
# include <fstream>
# include <string>
#endif  // ANDROID

#include "jstypes.h"
#include "jsutil.h"
#include "jsclist.h"
#include "jsprf.h"
#include "jsatom.h"
#include "jscntxt.h"
#include "jsversion.h"
#include "jsdbgapi.h"
#include "jsexn.h"
#include "jsfun.h"
#include "jsgc.h"
#include "jsiter.h"
#include "jslock.h"
#include "jsmath.h"
#include "jsnum.h"
#include "jsobj.h"
#include "jsopcode.h"
#include "jspubtd.h"
#include "jsscope.h"
#include "jsscript.h"
#include "jsstr.h"
#include "jsworkers.h"
#ifdef JS_ION
#include "ion/Ion.h"
#include "ion/IonFrames.h"
#endif

#ifdef JS_METHODJIT
# include "assembler/assembler/MacroAssembler.h"
# include "methodjit/MethodJIT.h"
#endif
#include "gc/Marking.h"
#include "js/MemoryMetrics.h"
#include "frontend/TokenStream.h"
#include "frontend/ParseMaps.h"
#include "yarr/BumpPointerAllocator.h"

#include "jsatominlines.h"
#include "jscntxtinlines.h"
#include "jscompartment.h"
#include "jsobjinlines.h"

#include "selfhosted.out.h"

using namespace js;
using namespace js::gc;

using mozilla::DebugOnly;

bool
js::AutoCycleDetector::init()
{
    ObjectSet &set = cx->cycleDetectorSet;
    hashsetAddPointer = set.lookupForAdd(obj);
    if (!hashsetAddPointer) {
        if (!set.add(hashsetAddPointer, obj))
            return false;
        cyclic = false;
        hashsetGenerationAtInit = set.generation();
    }
    return true;
}

js::AutoCycleDetector::~AutoCycleDetector()
{
    if (!cyclic) {
        if (hashsetGenerationAtInit == cx->cycleDetectorSet.generation())
            cx->cycleDetectorSet.remove(hashsetAddPointer);
        else
            cx->cycleDetectorSet.remove(obj);
    }
}

void
js::TraceCycleDetectionSet(JSTracer *trc, js::ObjectSet &set)
{
    for (js::ObjectSet::Enum e(set); !e.empty(); e.popFront()) {
        JSObject *prior = e.front();
        MarkObjectRoot(trc, const_cast<JSObject **>(&e.front()), "cycle detector table entry");
        if (prior != e.front())
            e.rekeyFront(e.front());
    }
}

void
JSRuntime::sizeOfIncludingThis(JSMallocSizeOfFun mallocSizeOf, RuntimeSizes *rtSizes)
{
    rtSizes->object = mallocSizeOf(this);

    rtSizes->atomsTable = atoms.sizeOfExcludingThis(mallocSizeOf);

    rtSizes->contexts = 0;
    for (ContextIter acx(this); !acx.done(); acx.next())
        rtSizes->contexts += acx->sizeOfIncludingThis(mallocSizeOf);

    rtSizes->dtoa = mallocSizeOf(dtoaState);

    rtSizes->temporary = tempLifoAlloc.sizeOfExcludingThis(mallocSizeOf);

    if (execAlloc_) {
        execAlloc_->sizeOfCode(&rtSizes->jaegerCode, &rtSizes->ionCode, &rtSizes->regexpCode,
                               &rtSizes->unusedCode);
    } else {
        rtSizes->jaegerCode = 0;
        rtSizes->ionCode    = 0;
        rtSizes->regexpCode = 0;
        rtSizes->unusedCode = 0;
    }

    rtSizes->stack = stackSpace.sizeOf();

    rtSizes->gcMarker = gcMarker.sizeOfExcludingThis(mallocSizeOf);

    rtSizes->mathCache = mathCache_ ? mathCache_->sizeOfIncludingThis(mallocSizeOf) : 0;

    rtSizes->scriptFilenames = scriptFilenameTable.sizeOfExcludingThis(mallocSizeOf);
    for (ScriptFilenameTable::Range r = scriptFilenameTable.all(); !r.empty(); r.popFront())
        rtSizes->scriptFilenames += mallocSizeOf(r.front());
}

size_t
JSRuntime::sizeOfExplicitNonHeap()
{
    size_t size = stackSpace.sizeOf();

    if (execAlloc_) {
        size_t jaegerCode, ionCode, regexpCode, unusedCode;
        execAlloc_->sizeOfCode(&jaegerCode, &ionCode, &regexpCode, &unusedCode);
        size += jaegerCode + ionCode + regexpCode + unusedCode;
    }

    return size;
}

void
JSRuntime::triggerOperationCallback()
{
    /*
     * Invalidate ionTop to trigger its over-recursion check. Note this must be
     * set before interrupt, to avoid racing with js_InvokeOperationCallback,
     * into a weird state where interrupt is stuck at 0 but ionStackLimit is
     * MAXADDR.
     */
    ionStackLimit = -1;

    /*
     * Use JS_ATOMIC_SET in the hope that it ensures the write will become
     * immediately visible to other processors polling the flag.
     */
    JS_ATOMIC_SET(&interrupt, 1);
}

void
JSRuntime::setJitHardening(bool enabled)
{
    jitHardening = enabled;
    if (execAlloc_)
        execAlloc_->setRandomize(enabled);
}

JSC::ExecutableAllocator *
JSRuntime::createExecutableAllocator(JSContext *cx)
{
    JS_ASSERT(!execAlloc_);
    JS_ASSERT(cx->runtime == this);

    JSC::AllocationBehavior randomize =
        jitHardening ? JSC::AllocationCanRandomize : JSC::AllocationDeterministic;
    execAlloc_ = js_new<JSC::ExecutableAllocator>(randomize);
    if (!execAlloc_)
        js_ReportOutOfMemory(cx);
    return execAlloc_;
}

WTF::BumpPointerAllocator *
JSRuntime::createBumpPointerAllocator(JSContext *cx)
{
    JS_ASSERT(!bumpAlloc_);
    JS_ASSERT(cx->runtime == this);

    bumpAlloc_ = js_new<WTF::BumpPointerAllocator>();
    if (!bumpAlloc_)
        js_ReportOutOfMemory(cx);
    return bumpAlloc_;
}

MathCache *
JSRuntime::createMathCache(JSContext *cx)
{
    JS_ASSERT(!mathCache_);
    JS_ASSERT(cx->runtime == this);

    MathCache *newMathCache = js_new<MathCache>();
    if (!newMathCache) {
        js_ReportOutOfMemory(cx);
        return NULL;
    }

    mathCache_ = newMathCache;
    return mathCache_;
}

#ifdef JS_METHODJIT
mjit::JaegerRuntime *
JSRuntime::createJaegerRuntime(JSContext *cx)
{
    JS_ASSERT(!jaegerRuntime_);
    JS_ASSERT(cx->runtime == this);

    mjit::JaegerRuntime *jr = js_new<mjit::JaegerRuntime>();
    if (!jr || !jr->init(cx)) {
        js_ReportOutOfMemory(cx);
        js_delete(jr);
        return NULL;
    }

    jaegerRuntime_ = jr;
    return jaegerRuntime_;
}
#endif

static void
selfHosting_ErrorReporter(JSContext *cx, const char *message, JSErrorReport *report)
{
    PrintError(cx, stderr, message, report, true);
}

static JSClass self_hosting_global_class = {
    "self-hosting-global", JSCLASS_GLOBAL_FLAGS,
    JS_PropertyStub,  JS_PropertyStub,
    JS_PropertyStub,  JS_StrictPropertyStub,
    JS_EnumerateStub, JS_ResolveStub,
    JS_ConvertStub,   NULL
};

static JSBool
intrinsic_ToObject(JSContext *cx, unsigned argc, Value *vp)
{
    CallArgs args = CallArgsFromVp(argc, vp);
    RootedValue val(cx, args[0]);
    RootedObject obj(cx, ToObject(cx, val));
    if (!obj)
        return false;
    args.rval().set(OBJECT_TO_JSVAL(obj));
    return true;
}

static JSBool
intrinsic_ToInteger(JSContext *cx, unsigned argc, Value *vp)
{
    CallArgs args = CallArgsFromVp(argc, vp);
    double result;
    if (!ToInteger(cx, args[0], &result))
        return false;
    args.rval().set(DOUBLE_TO_JSVAL(result));
    return true;
}

static JSBool
intrinsic_IsCallable(JSContext *cx, unsigned argc, Value *vp)
{
    CallArgs args = CallArgsFromVp(argc, vp);
    Value val = args[0];
    bool isCallable = val.isObject() && val.toObject().isCallable();
    args.rval().set(BOOLEAN_TO_JSVAL(isCallable));
    return true;
}

static JSBool
intrinsic_ThrowError(JSContext *cx, unsigned argc, Value *vp)
{
    CallArgs args = CallArgsFromVp(argc, vp);
    JS_ASSERT(args.length() >= 1);
    uint32_t errorNumber = args[0].toInt32();

    char *errorArgs[3] = {NULL, NULL, NULL};
    for (unsigned i = 1; i < 4 && i < args.length(); i++) {
        RootedValue val(cx, args[i]);
        if (val.isInt32()) {
            JSString *str = ToString(cx, val);
            if (!str)
                return false;
            errorArgs[i - 1] = JS_EncodeString(cx, str);
        } else if (val.isString()) {
            errorArgs[i - 1] = JS_EncodeString(cx, ToString(cx, val));
        } else {
            errorArgs[i - 1] = DecompileValueGenerator(cx, JSDVG_SEARCH_STACK, val, NullPtr());
        }
        if (!errorArgs[i - 1])
            return false;
    }

    JS_ReportErrorNumber(cx, js_GetErrorMessage, NULL, errorNumber,
                         errorArgs[0], errorArgs[1], errorArgs[2]);
    for (unsigned i = 0; i < 3; i++)
        js_free(errorArgs[i]);
    return false;
}

/*
 * Used to decompile values in the nearest non-builtin stack frame, falling
 * back to decompiling in the current frame. Helpful for printing higher-order
 * function arguments.
 * 
 * The user must supply the argument number of the value in question; it
 * _cannot_ be automatically determined.
 */
static JSBool
intrinsic_DecompileArg(JSContext *cx, unsigned argc, Value *vp)
{
    CallArgs args = CallArgsFromVp(argc, vp);
    JS_ASSERT(args.length() == 2);

    RootedValue value(cx, args[1]);
    ScopedFreePtr<char> str(DecompileArgument(cx, args[0].toInt32(), value));
    if (!str)
        return false;
    RootedAtom atom(cx, Atomize(cx, str, strlen(str)));
    if (!atom)
        return false;
    args.rval().setString(atom);
    return true;
}

static JSBool
intrinsic_MakeConstructible(JSContext *cx, unsigned argc, Value *vp)
{
    CallArgs args = CallArgsFromVp(argc, vp);
    JS_ASSERT(args.length() >= 1);
    JS_ASSERT(args[0].isObject());
    RootedObject obj(cx, &args[0].toObject());
    JS_ASSERT(obj->isFunction());
    obj->toFunction()->setIsSelfHostedConstructor();
    return true;
}

JSFunctionSpec intrinsic_functions[] = {
    JS_FN("ToObject",           intrinsic_ToObject,             1,0),
    JS_FN("ToInteger",          intrinsic_ToInteger,            1,0),
    JS_FN("IsCallable",         intrinsic_IsCallable,           1,0),
    JS_FN("ThrowError",         intrinsic_ThrowError,           4,0),
    JS_FN("_MakeConstructible", intrinsic_MakeConstructible,    1,0),
    JS_FN("_DecompileArg",      intrinsic_DecompileArg,         2,0),
    JS_FS_END
};
bool
JSRuntime::initSelfHosting(JSContext *cx)
{
    JS_ASSERT(!selfHostedGlobal_);
    RootedObject savedGlobal(cx, JS_GetGlobalObject(cx));
    if (!(selfHostedGlobal_ = JS_NewGlobalObject(cx, &self_hosting_global_class, NULL)))
        return false;
    JS_SetGlobalObject(cx, selfHostedGlobal_);
    JSAutoCompartment ac(cx, cx->global());
    RootedObject shg(cx, selfHostedGlobal_);

    if (!JS_DefineFunctions(cx, shg, intrinsic_functions))
        return false;

    CompileOptions options(cx);
    options.setFileAndLine("self-hosted", 1);
    options.setSelfHostingMode(true);

    /*
     * Set a temporary error reporter printing to stderr because it is too
     * early in the startup process for any other reporter to be registered
     * and we don't want errors in self-hosted code to be silently swallowed.
     */
    JSErrorReporter oldReporter = JS_SetErrorReporter(cx, selfHosting_ErrorReporter);
    Value rv;
    bool ok = false;

    char *filename = getenv("MOZ_SELFHOSTEDJS");
    if (filename) {
        RootedScript script(cx, Compile(cx, shg, options, filename));
        if (script)
            ok = Execute(cx, script, *shg.get(), &rv);
    } else {
        const char *src = selfhosted::raw_sources;
        uint32_t srcLen = selfhosted::GetRawScriptsSize();
        ok = Evaluate(cx, shg, options, src, srcLen, &rv);
    }
    JS_SetErrorReporter(cx, oldReporter);
    JS_SetGlobalObject(cx, savedGlobal);
    return ok;
}

void
JSRuntime::markSelfHostedGlobal(JSTracer *trc)
{
    MarkObjectRoot(trc, &selfHostedGlobal_, "self-hosting global");
}

bool
JSRuntime::getUnclonedSelfHostedValue(JSContext *cx, Handle<PropertyName*> name,
                                      MutableHandleValue vp)
{
    RootedObject shg(cx, selfHostedGlobal_);
    AutoCompartment ac(cx, shg);
    return JS_GetPropertyById(cx, shg, NameToId(name), vp.address());
}

bool
JSRuntime::cloneSelfHostedFunctionScript(JSContext *cx, Handle<PropertyName*> name,
                                         Handle<JSFunction*> targetFun)
{
    RootedValue funVal(cx);
    if (!getUnclonedSelfHostedValue(cx, name, &funVal))
        return false;

    RootedFunction sourceFun(cx, funVal.toObject().toFunction());
    Rooted<JSScript*> sourceScript(cx, sourceFun->nonLazyScript());
    JS_ASSERT(!sourceScript->enclosingStaticScope());
    RawScript cscript = CloneScript(cx, NullPtr(), targetFun, sourceScript);
    if (!cscript)
        return false;
    targetFun->setScript(cscript);
    cscript->setFunction(targetFun);
    JS_ASSERT(sourceFun->nargs == targetFun->nargs);
    targetFun->flags = sourceFun->flags | JSFunction::EXTENDED;
    return true;
}

bool
JSRuntime::cloneSelfHostedValue(JSContext *cx, Handle<PropertyName*> name, MutableHandleValue vp)
{
    RootedValue funVal(cx);
    if (!getUnclonedSelfHostedValue(cx, name, &funVal))
        return false;

    /*
     * We don't clone if we're operating in the self-hosting global, as that
     * means we're currently executing the self-hosting script while
     * initializing the runtime (see JSRuntime::initSelfHosting).
     */
    if (cx->global() == selfHostedGlobal_) {
        vp.set(funVal);
    } else if (funVal.isObject() && funVal.toObject().isFunction()) {
        RootedFunction fun(cx, funVal.toObject().toFunction());
        RootedObject clone(cx, CloneFunctionObject(cx, fun, cx->global(), fun->getAllocKind()));
        if (!clone)
            return false;
        vp.set(ObjectValue(*clone));
    } else {
        vp.set(UndefinedValue());
    }
    return true;
}

JSContext *
js::NewContext(JSRuntime *rt, size_t stackChunkSize)
{
    JS_AbortIfWrongThread(rt);

    JSContext *cx = js_new<JSContext>(rt);
    if (!cx)
        return NULL;

    JS_ASSERT(cx->findVersion() == JSVERSION_DEFAULT);

    if (!cx->cycleDetectorSet.init()) {
        js_delete(cx);
        return NULL;
    }

    /*
     * Here the GC lock is still held after js_InitContextThreadAndLockGC took it and
     * the GC is not running on another thread.
     */
    bool first = rt->contextList.isEmpty();
    rt->contextList.insertBack(cx);

    js_InitRandom(cx);

    /*
     * If cx is the first context on this runtime, initialize well-known atoms,
     * keywords, numbers, strings and self-hosted scripts. If one of these
     * steps should fail, the runtime will be left in a partially initialized
     * state, with zeroes and nulls stored in the default-initialized remainder
     * of the struct. We'll clean the runtime up under DestroyContext, because
     * cx will be "last" as well as "first".
     */
    if (first) {
#ifdef JS_THREADSAFE
        JS_BeginRequest(cx);
#endif
        bool ok = rt->staticStrings.init(cx);
        if (ok)
            ok = InitCommonNames(cx);
        if (ok)
            ok = rt->initSelfHosting(cx);

#ifdef JS_THREADSAFE
        JS_EndRequest(cx);
#endif
        if (!ok) {
            DestroyContext(cx, DCM_NEW_FAILED);
            return NULL;
        }
    }

    JSContextCallback cxCallback = rt->cxCallback;
    if (cxCallback && !cxCallback(cx, JSCONTEXT_NEW)) {
        DestroyContext(cx, DCM_NEW_FAILED);
        return NULL;
    }

    return cx;
}

void
js::DestroyContext(JSContext *cx, DestroyContextMode mode)
{
    JSRuntime *rt = cx->runtime;
    JS_AbortIfWrongThread(rt);

    JS_ASSERT(!cx->enumerators);

#ifdef JS_THREADSAFE
    JS_ASSERT(cx->outstandingRequests == 0);
#endif

    if (mode != DCM_NEW_FAILED) {
        if (JSContextCallback cxCallback = rt->cxCallback) {
            /*
             * JSCONTEXT_DESTROY callback is not allowed to fail and must
             * return true.
             */
            JS_ALWAYS_TRUE(cxCallback(cx, JSCONTEXT_DESTROY));
        }
    }

    cx->remove();
    bool last = !rt->hasContexts();
    if (last) {
        JS_ASSERT(!rt->isHeapBusy());

        /*
         * Dump remaining type inference results first. This printing
         * depends on atoms still existing.
         */
        for (CompartmentsIter c(rt); !c.done(); c.next())
            c->types.print(cx, false);

        /* Off thread ion compilations depend on atoms still existing. */
        for (CompartmentsIter c(rt); !c.done(); c.next())
            CancelOffThreadIonCompile(c, NULL);

        /* Unpin all common names before final GC. */
        FinishCommonNames(rt);

        /* Clear debugging state to remove GC roots. */
        for (CompartmentsIter c(rt); !c.done(); c.next())
            c->clearTraps(rt->defaultFreeOp());
        JS_ClearAllWatchPoints(cx);

        /* Clear the statics table to remove GC roots. */
        rt->staticStrings.finish();

        PrepareForFullGC(rt);
        GC(rt, GC_NORMAL, gcreason::LAST_CONTEXT);
    } else if (mode == DCM_FORCE_GC) {
        JS_ASSERT(!rt->isHeapBusy());
        PrepareForFullGC(rt);
        GC(rt, GC_NORMAL, gcreason::DESTROY_CONTEXT);
    }
    js_delete(cx);
}

bool
AutoResolving::alreadyStartedSlow() const
{
    JS_ASSERT(link);
    AutoResolving *cursor = link;
    do {
        JS_ASSERT(this != cursor);
        if (object.get() == cursor->object && id.get() == cursor->id && kind == cursor->kind)
            return true;
    } while (!!(cursor = cursor->link));
    return false;
}

static void
ReportError(JSContext *cx, const char *message, JSErrorReport *reportp,
            JSErrorCallback callback, void *userRef)
{
    AssertCanGC();

    /*
     * Check the error report, and set a JavaScript-catchable exception
     * if the error is defined to have an associated exception.  If an
     * exception is thrown, then the JSREPORT_EXCEPTION flag will be set
     * on the error report, and exception-aware hosts should ignore it.
     */
    JS_ASSERT(reportp);
    if ((!callback || callback == js_GetErrorMessage) &&
        reportp->errorNumber == JSMSG_UNCAUGHT_EXCEPTION)
        reportp->flags |= JSREPORT_EXCEPTION;

    /*
     * Call the error reporter only if an exception wasn't raised.
     *
     * If an exception was raised, then we call the debugErrorHook
     * (if present) to give it a chance to see the error before it
     * propagates out of scope.  This is needed for compatibility
     * with the old scheme.
     */
    if (!JS_IsRunning(cx) ||
        !js_ErrorToException(cx, message, reportp, callback, userRef)) {
        js_ReportErrorAgain(cx, message, reportp);
    } else if (JSDebugErrorHook hook = cx->runtime->debugHooks.debugErrorHook) {
        /*
         * If we've already chewed up all the C stack, don't call into the
         * error reporter since this may trigger an infinite recursion where
         * the reporter triggers an over-recursion.
         */
        int stackDummy;
        if (!JS_CHECK_STACK_SIZE(cx->runtime->nativeStackLimit, &stackDummy))
            return;

        if (cx->errorReporter)
            hook(cx, message, reportp, cx->runtime->debugHooks.debugErrorHookData);
    }
}

/*
 * The given JSErrorReport object have been zeroed and must not outlive
 * cx->fp() (otherwise report->originPrincipals may become invalid).
 */
static void
PopulateReportBlame(JSContext *cx, JSErrorReport *report)
{
    AutoAssertNoGC nogc;

    /*
     * Walk stack until we find a frame that is associated with a non-builtin
     * rather than a builtin frame.
     */
    NonBuiltinScriptFrameIter iter(cx);
    if (iter.done())
        return;

    report->filename = iter.script()->filename;
    report->lineno = PCToLineNumber(iter.script().get(nogc), iter.pc(), &report->column);
    report->originPrincipals = iter.script()->originPrincipals;
}

/*
 * We don't post an exception in this case, since doing so runs into
 * complications of pre-allocating an exception object which required
 * running the Exception class initializer early etc.
 * Instead we just invoke the errorReporter with an "Out Of Memory"
 * type message, and then hope the process ends swiftly.
 */
void
js_ReportOutOfMemory(JSContext *cx)
{
    AutoAssertNoGC nogc;

    cx->runtime->hadOutOfMemory = true;

    JSErrorReport report;
    JSErrorReporter onError = cx->errorReporter;

    /* Get the message for this error, but we won't expand any arguments. */
    const JSErrorFormatString *efs =
        js_GetLocalizedErrorMessage(cx, NULL, NULL, JSMSG_OUT_OF_MEMORY);
    const char *msg = efs ? efs->format : "Out of memory";

    /* Fill out the report, but don't do anything that requires allocation. */
    PodZero(&report);
    report.flags = JSREPORT_ERROR;
    report.errorNumber = JSMSG_OUT_OF_MEMORY;
    PopulateReportBlame(cx, &report);

    /*
     * We clear a pending exception, if any, now so the hook can replace the
     * out-of-memory error by a script-catchable exception.
     */
    cx->clearPendingException();
    if (onError) {
        AutoSuppressGC suppressGC(cx);
        onError(cx, msg, &report);
    }
}

JS_FRIEND_API(void)
js_ReportOverRecursed(JSContext *maybecx)
{
#ifdef JS_MORE_DETERMINISTIC
    /*
     * We cannot make stack depth deterministic across different
     * implementations (e.g. JIT vs. interpreter will differ in
     * their maximum stack depth).
     * However, we can detect externally when we hit the maximum
     * stack depth which is useful for external testing programs
     * like fuzzers.
     */
    fprintf(stderr, "js_ReportOverRecursed called\n");
#endif
    if (maybecx)
        JS_ReportErrorNumber(maybecx, js_GetErrorMessage, NULL, JSMSG_OVER_RECURSED);
}

void
js_ReportAllocationOverflow(JSContext *maybecx)
{
    if (maybecx)
        JS_ReportErrorNumber(maybecx, js_GetErrorMessage, NULL, JSMSG_ALLOC_OVERFLOW);
}

/*
 * Given flags and the state of cx, decide whether we should report an
 * error, a warning, or just continue execution normally.  Return
 * true if we should continue normally, without reporting anything;
 * otherwise, adjust *flags as appropriate and return false.
 */
static bool
checkReportFlags(JSContext *cx, unsigned *flags)
{
    if (JSREPORT_IS_STRICT_MODE_ERROR(*flags)) {
        /*
         * Error in strict code; warning with strict option; okay otherwise.
         * We assume that if the top frame is a native, then it is strict if
         * the nearest scripted frame is strict, see bug 536306.
         */
        JSScript *script = cx->stack.currentScript();
        if (script && script->strictModeCode)
            *flags &= ~JSREPORT_WARNING;
        else if (cx->hasStrictOption())
            *flags |= JSREPORT_WARNING;
        else
            return true;
    } else if (JSREPORT_IS_STRICT(*flags)) {
        /* Warning/error only when JSOPTION_STRICT is set. */
        if (!cx->hasStrictOption())
            return true;
    }

    /* Warnings become errors when JSOPTION_WERROR is set. */
    if (JSREPORT_IS_WARNING(*flags) && cx->hasWErrorOption())
        *flags &= ~JSREPORT_WARNING;

    return false;
}

JSBool
js_ReportErrorVA(JSContext *cx, unsigned flags, const char *format, va_list ap)
{
    char *message;
    jschar *ucmessage;
    size_t messagelen;
    JSErrorReport report;
    JSBool warning;

    if (checkReportFlags(cx, &flags))
        return JS_TRUE;

    message = JS_vsmprintf(format, ap);
    if (!message)
        return JS_FALSE;
    messagelen = strlen(message);

    PodZero(&report);
    report.flags = flags;
    report.errorNumber = JSMSG_USER_DEFINED_ERROR;
    report.ucmessage = ucmessage = InflateString(cx, message, &messagelen);
    PopulateReportBlame(cx, &report);

    warning = JSREPORT_IS_WARNING(report.flags);

    ReportError(cx, message, &report, NULL, NULL);
    js_free(message);
    js_free(ucmessage);
    return warning;
}

/* |callee| requires a usage string provided by JS_DefineFunctionsWithHelp. */
void
js::ReportUsageError(JSContext *cx, HandleObject callee, const char *msg)
{
    const char *usageStr = "usage";
    PropertyName *usageAtom = Atomize(cx, usageStr, strlen(usageStr))->asPropertyName();
    DebugOnly<Shape *> shape = callee->nativeLookup(cx, NameToId(usageAtom));
    JS_ASSERT(!shape->configurable());
    JS_ASSERT(!shape->writable());
    JS_ASSERT(shape->hasDefaultGetter());

    jsval usage;
    if (!JS_LookupProperty(cx, callee, "usage", &usage))
        return;

    if (JSVAL_IS_VOID(usage)) {
        JS_ReportError(cx, "%s", msg);
    } else {
        JSString *str = JSVAL_TO_STRING(usage);
        JS::Anchor<JSString *> a_str(str);
        const jschar *chars = JS_GetStringCharsZ(cx, str);
        if (!chars)
            return;
        JS_ReportError(cx, "%s. Usage: %hs", msg, chars);
    }
}

bool
js::PrintError(JSContext *cx, FILE *file, const char *message, JSErrorReport *report,
               bool reportWarnings)
{
    if (!report) {
        fprintf(file, "%s\n", message);
        fflush(file);
        return false;
    }

    /* Conditionally ignore reported warnings. */
    if (JSREPORT_IS_WARNING(report->flags) && !reportWarnings)
        return false;

    char *prefix = NULL;
    if (report->filename)
        prefix = JS_smprintf("%s:", report->filename);
    if (report->lineno) {
        char *tmp = prefix;
        prefix = JS_smprintf("%s%u:%u ", tmp ? tmp : "", report->lineno, report->column);
        JS_free(cx, tmp);
    }
    if (JSREPORT_IS_WARNING(report->flags)) {
        char *tmp = prefix;
        prefix = JS_smprintf("%s%swarning: ",
                             tmp ? tmp : "",
                             JSREPORT_IS_STRICT(report->flags) ? "strict " : "");
        JS_free(cx, tmp);
    }

    /* embedded newlines -- argh! */
    const char *ctmp;
    while ((ctmp = strchr(message, '\n')) != 0) {
        ctmp++;
        if (prefix)
            fputs(prefix, file);
        fwrite(message, 1, ctmp - message, file);
        message = ctmp;
    }

    /* If there were no filename or lineno, the prefix might be empty */
    if (prefix)
        fputs(prefix, file);
    fputs(message, file);

    if (report->linebuf) {
        /* report->linebuf usually ends with a newline. */
        int n = strlen(report->linebuf);
        fprintf(file, ":\n%s%s%s%s",
                prefix,
                report->linebuf,
                (n > 0 && report->linebuf[n-1] == '\n') ? "" : "\n",
                prefix);
        n = report->tokenptr - report->linebuf;
        for (int i = 0, j = 0; i < n; i++) {
            if (report->linebuf[i] == '\t') {
                for (int k = (j + 8) & ~7; j < k; j++) {
                    fputc('.', file);
                }
                continue;
            }
            fputc('.', file);
            j++;
        }
        fputc('^', file);
    }
    fputc('\n', file);
    fflush(file);
    JS_free(cx, prefix);
    return true;
}

/*
 * The arguments from ap need to be packaged up into an array and stored
 * into the report struct.
 *
 * The format string addressed by the error number may contain operands
 * identified by the format {N}, where N is a decimal digit. Each of these
 * is to be replaced by the Nth argument from the va_list. The complete
 * message is placed into reportp->ucmessage converted to a JSString.
 *
 * Returns true if the expansion succeeds (can fail if out of memory).
 */
JSBool
js_ExpandErrorArguments(JSContext *cx, JSErrorCallback callback,
                        void *userRef, const unsigned errorNumber,
                        char **messagep, JSErrorReport *reportp,
                        bool charArgs, va_list ap)
{
    const JSErrorFormatString *efs;
    int i;
    int argCount;
    bool messageArgsPassed = !!reportp->messageArgs;

    *messagep = NULL;

    /* Most calls supply js_GetErrorMessage; if this is so, assume NULL. */
    if (!callback || callback == js_GetErrorMessage)
        efs = js_GetLocalizedErrorMessage(cx, userRef, NULL, errorNumber);
    else
        efs = callback(userRef, NULL, errorNumber);
    if (efs) {
        reportp->exnType = efs->exnType;

        size_t totalArgsLength = 0;
        size_t argLengths[10]; /* only {0} thru {9} supported */
        argCount = efs->argCount;
        JS_ASSERT(argCount <= 10);
        if (argCount > 0) {
            /*
             * Gather the arguments into an array, and accumulate
             * their sizes. We allocate 1 more than necessary and
             * null it out to act as the caboose when we free the
             * pointers later.
             */
            if (messageArgsPassed) {
                JS_ASSERT(!reportp->messageArgs[argCount]);
            } else {
                reportp->messageArgs = cx->pod_malloc<const jschar*>(argCount + 1);
                if (!reportp->messageArgs)
                    return JS_FALSE;
                /* NULL-terminate for easy copying. */
                reportp->messageArgs[argCount] = NULL;
            }
            for (i = 0; i < argCount; i++) {
                if (messageArgsPassed) {
                    /* Do nothing. */
                } else if (charArgs) {
                    char *charArg = va_arg(ap, char *);
                    size_t charArgLength = strlen(charArg);
                    reportp->messageArgs[i] = InflateString(cx, charArg, &charArgLength);
                    if (!reportp->messageArgs[i])
                        goto error;
                } else {
                    reportp->messageArgs[i] = va_arg(ap, jschar *);
                }
                argLengths[i] = js_strlen(reportp->messageArgs[i]);
                totalArgsLength += argLengths[i];
            }
        }
        /*
         * Parse the error format, substituting the argument X
         * for {X} in the format.
         */
        if (argCount > 0) {
            if (efs->format) {
                jschar *buffer, *fmt, *out;
                int expandedArgs = 0;
                size_t expandedLength;
                size_t len = strlen(efs->format);

                buffer = fmt = InflateString(cx, efs->format, &len);
                if (!buffer)
                    goto error;
                expandedLength = len
                                 - (3 * argCount)       /* exclude the {n} */
                                 + totalArgsLength;

                /*
                * Note - the above calculation assumes that each argument
                * is used once and only once in the expansion !!!
                */
                reportp->ucmessage = out = cx->pod_malloc<jschar>(expandedLength + 1);
                if (!out) {
                    js_free(buffer);
                    goto error;
                }
                while (*fmt) {
                    if (*fmt == '{') {
                        if (isdigit(fmt[1])) {
                            int d = JS7_UNDEC(fmt[1]);
                            JS_ASSERT(d < argCount);
                            js_strncpy(out, reportp->messageArgs[d],
                                       argLengths[d]);
                            out += argLengths[d];
                            fmt += 3;
                            expandedArgs++;
                            continue;
                        }
                    }
                    *out++ = *fmt++;
                }
                JS_ASSERT(expandedArgs == argCount);
                *out = 0;
                js_free(buffer);
                *messagep = DeflateString(cx, reportp->ucmessage,
                                          size_t(out - reportp->ucmessage));
                if (!*messagep)
                    goto error;
            }
        } else {
            /* Non-null messageArgs should have at least one non-null arg. */
            JS_ASSERT(!reportp->messageArgs);
            /*
             * Zero arguments: the format string (if it exists) is the
             * entire message.
             */
            if (efs->format) {
                size_t len;
                *messagep = JS_strdup(cx, efs->format);
                if (!*messagep)
                    goto error;
                len = strlen(*messagep);
                reportp->ucmessage = InflateString(cx, *messagep, &len);
                if (!reportp->ucmessage)
                    goto error;
            }
        }
    }
    if (*messagep == NULL) {
        /* where's the right place for this ??? */
        const char *defaultErrorMessage
            = "No error message available for error number %d";
        size_t nbytes = strlen(defaultErrorMessage) + 16;
        *messagep = cx->pod_malloc<char>(nbytes);
        if (!*messagep)
            goto error;
        JS_snprintf(*messagep, nbytes, defaultErrorMessage, errorNumber);
    }
    return JS_TRUE;

error:
    if (!messageArgsPassed && reportp->messageArgs) {
        /* free the arguments only if we allocated them */
        if (charArgs) {
            i = 0;
            while (reportp->messageArgs[i])
                js_free((void *)reportp->messageArgs[i++]);
        }
        js_free((void *)reportp->messageArgs);
        reportp->messageArgs = NULL;
    }
    if (reportp->ucmessage) {
        js_free((void *)reportp->ucmessage);
        reportp->ucmessage = NULL;
    }
    if (*messagep) {
        js_free((void *)*messagep);
        *messagep = NULL;
    }
    return JS_FALSE;
}

JSBool
js_ReportErrorNumberVA(JSContext *cx, unsigned flags, JSErrorCallback callback,
                       void *userRef, const unsigned errorNumber,
                       JSBool charArgs, va_list ap)
{
    JSErrorReport report;
    char *message;
    JSBool warning;

    if (checkReportFlags(cx, &flags))
        return JS_TRUE;
    warning = JSREPORT_IS_WARNING(flags);

    PodZero(&report);
    report.flags = flags;
    report.errorNumber = errorNumber;
    PopulateReportBlame(cx, &report);

    if (!js_ExpandErrorArguments(cx, callback, userRef, errorNumber,
                                 &message, &report, !!charArgs, ap)) {
        return JS_FALSE;
    }

    ReportError(cx, message, &report, callback, userRef);

    if (message)
        js_free(message);
    if (report.messageArgs) {
        /*
         * js_ExpandErrorArguments owns its messageArgs only if it had to
         * inflate the arguments (from regular |char *|s).
         */
        if (charArgs) {
            int i = 0;
            while (report.messageArgs[i])
                js_free((void *)report.messageArgs[i++]);
        }
        js_free((void *)report.messageArgs);
    }
    if (report.ucmessage)
        js_free((void *)report.ucmessage);

    return warning;
}

bool
js_ReportErrorNumberUCArray(JSContext *cx, unsigned flags, JSErrorCallback callback,
                            void *userRef, const unsigned errorNumber,
                            const jschar **args)
{
    if (checkReportFlags(cx, &flags))
        return true;
    bool warning = JSREPORT_IS_WARNING(flags);

    JSErrorReport report;
    PodZero(&report);
    report.flags = flags;
    report.errorNumber = errorNumber;
    PopulateReportBlame(cx, &report);
    report.messageArgs = args;

    char *message;
    va_list dummy;
    if (!js_ExpandErrorArguments(cx, callback, userRef, errorNumber,
                                 &message, &report, JS_FALSE, dummy)) {
        return false;
    }

    ReportError(cx, message, &report, callback, userRef);

    if (message)
        js_free(message);
    if (report.ucmessage)
        js_free((void *)report.ucmessage);

    return warning;
}

JS_FRIEND_API(void)
js_ReportErrorAgain(JSContext *cx, const char *message, JSErrorReport *reportp)
{
    JSErrorReporter onError;

    if (!message)
        return;

    onError = cx->errorReporter;

    /*
     * If debugErrorHook is present then we give it a chance to veto
     * sending the error on to the regular ErrorReporter.
     */
    if (onError) {
        JSDebugErrorHook hook = cx->runtime->debugHooks.debugErrorHook;
        if (hook && !hook(cx, message, reportp, cx->runtime->debugHooks.debugErrorHookData))
            onError = NULL;
    }
    if (onError)
        onError(cx, message, reportp);
}

void
js_ReportIsNotDefined(JSContext *cx, const char *name)
{
    JS_ReportErrorNumber(cx, js_GetErrorMessage, NULL, JSMSG_NOT_DEFINED, name);
}

JSBool
js_ReportIsNullOrUndefined(JSContext *cx, int spindex, HandleValue v,
                           HandleString fallback)
{
    char *bytes;
    JSBool ok;

    bytes = DecompileValueGenerator(cx, spindex, v, fallback);
    if (!bytes)
        return JS_FALSE;

    if (strcmp(bytes, js_undefined_str) == 0 ||
        strcmp(bytes, js_null_str) == 0) {
        ok = JS_ReportErrorFlagsAndNumber(cx, JSREPORT_ERROR,
                                          js_GetErrorMessage, NULL,
                                          JSMSG_NO_PROPERTIES, bytes,
                                          NULL, NULL);
    } else if (v.isUndefined()) {
        ok = JS_ReportErrorFlagsAndNumber(cx, JSREPORT_ERROR,
                                          js_GetErrorMessage, NULL,
                                          JSMSG_UNEXPECTED_TYPE, bytes,
                                          js_undefined_str, NULL);
    } else {
        JS_ASSERT(v.isNull());
        ok = JS_ReportErrorFlagsAndNumber(cx, JSREPORT_ERROR,
                                          js_GetErrorMessage, NULL,
                                          JSMSG_UNEXPECTED_TYPE, bytes,
                                          js_null_str, NULL);
    }

    js_free(bytes);
    return ok;
}

void
js_ReportMissingArg(JSContext *cx, HandleValue v, unsigned arg)
{
    char argbuf[11];
    char *bytes;
    RootedAtom atom(cx);

    JS_snprintf(argbuf, sizeof argbuf, "%u", arg);
    bytes = NULL;
    if (IsFunctionObject(v)) {
        atom = v.toObject().toFunction()->atom();
        bytes = DecompileValueGenerator(cx, JSDVG_SEARCH_STACK,
                                        v, atom);
        if (!bytes)
            return;
    }
    JS_ReportErrorNumber(cx, js_GetErrorMessage, NULL,
                         JSMSG_MISSING_FUN_ARG, argbuf,
                         bytes ? bytes : "");
    js_free(bytes);
}

JSBool
js_ReportValueErrorFlags(JSContext *cx, unsigned flags, const unsigned errorNumber,
                         int spindex, HandleValue v, HandleString fallback,
                         const char *arg1, const char *arg2)
{
    char *bytes;
    JSBool ok;

    JS_ASSERT(js_ErrorFormatString[errorNumber].argCount >= 1);
    JS_ASSERT(js_ErrorFormatString[errorNumber].argCount <= 3);
    bytes = DecompileValueGenerator(cx, spindex, v, fallback);
    if (!bytes)
        return JS_FALSE;

    ok = JS_ReportErrorFlagsAndNumber(cx, flags, js_GetErrorMessage,
                                      NULL, errorNumber, bytes, arg1, arg2);
    js_free(bytes);
    return ok;
}

JSErrorFormatString js_ErrorFormatString[JSErr_Limit] = {
#define MSG_DEF(name, number, count, exception, format) \
    { format, count, exception } ,
#include "js.msg"
#undef MSG_DEF
};

JS_FRIEND_API(const JSErrorFormatString *)
js_GetErrorMessage(void *userRef, const char *locale, const unsigned errorNumber)
{
    if ((errorNumber > 0) && (errorNumber < JSErr_Limit))
        return &js_ErrorFormatString[errorNumber];
    return NULL;
}

JSBool
js_InvokeOperationCallback(JSContext *cx)
{
    JS_ASSERT_REQUEST_DEPTH(cx);

    JSRuntime *rt = cx->runtime;
    JS_ASSERT(rt->interrupt != 0);

    /*
     * Reset the callback counter first, then run GC and yield. If another
     * thread is racing us here we will accumulate another callback request
     * which will be serviced at the next opportunity.
     */
    JS_ATOMIC_SET(&rt->interrupt, 0);

    /* IonMonkey sets its stack limit to NULL to trigger operaton callbacks. */
    rt->resetIonStackLimit();

    if (rt->gcIsNeeded)
        GCSlice(rt, GC_NORMAL, rt->gcTriggerReason);

#ifdef JS_ION
    /*
     * A worker thread may have set the callback after finishing an Ion
     * compilation.
     */
    ion::AttachFinishedCompilations(cx);
#endif

    /*
     * Important: Additional callbacks can occur inside the callback handler
     * if it re-enters the JS engine. The embedding must ensure that the
     * callback is disconnected before attempting such re-entry.
     */
    JSOperationCallback cb = cx->operationCallback;
    return !cb || cb(cx);
}

JSBool
js_HandleExecutionInterrupt(JSContext *cx)
{
    JSBool result = JS_TRUE;
    if (cx->runtime->interrupt)
        result = js_InvokeOperationCallback(cx) && result;
    return result;
}

jsbytecode*
js_GetCurrentBytecodePC(JSContext* cx)
{
    return cx->hasfp() ? cx->regs().pc : NULL;
}

void
DSTOffsetCache::purge()
{
    /*
     * NB: The initial range values are carefully chosen to result in a cache
     *     miss on first use given the range of possible values.  Be careful
     *     to keep these values and the caching algorithm in sync!
     */
    offsetMilliseconds = 0;
    rangeStartSeconds = rangeEndSeconds = INT64_MIN;
    oldOffsetMilliseconds = 0;
    oldRangeStartSeconds = oldRangeEndSeconds = INT64_MIN;

    sanityCheck();
}

/*
 * Since getDSTOffsetMilliseconds guarantees that all times seen will be
 * positive, we can initialize the range at construction time with large
 * negative numbers to ensure the first computation is always a cache miss and
 * doesn't return a bogus offset.
 */
DSTOffsetCache::DSTOffsetCache()
{
    purge();
}

JSContext::JSContext(JSRuntime *rt)
  : ContextFriendFields(rt),
    defaultVersion(JSVERSION_DEFAULT),
    hasVersionOverride(false),
    throwing(false),
    exception(UndefinedValue()),
    runOptions(0),
    reportGranularity(JS_DEFAULT_JITREPORT_GRANULARITY),
    localeCallbacks(NULL),
    resolvingList(NULL),
    generatingError(false),
    compartment(NULL),
    enterCompartmentDepth_(0),
    savedFrameChains_(),
    defaultCompartmentObject_(NULL),
    stack(thisDuringConstruction()),
    parseMapPool_(NULL),
    cycleDetectorSet(thisDuringConstruction()),
    errorReporter(NULL),
    operationCallback(NULL),
    data(NULL),
    data2(NULL),
#ifdef JS_THREADSAFE
    outstandingRequests(0),
#endif
    resolveFlags(0),
    rngSeed(0),
    iterValue(MagicValue(JS_NO_ITER_VALUE)),
#ifdef JS_METHODJIT
    methodJitEnabled(false),
#endif
#ifdef MOZ_TRACE_JSCALLS
    functionCallback(NULL),
#endif
    enumerators(NULL),
    innermostGenerator_(NULL),
#ifdef DEBUG
    stackIterAssertionEnabled(true),
#endif
    activeCompilations(0)
{
#ifdef JSGC_ROOT_ANALYSIS
    PodArrayZero(thingGCRooters);
#if defined(JS_GC_ZEAL) && defined(DEBUG) && !defined(JS_THREADSAFE)
    skipGCRooters = NULL;
#endif
#endif
}

JSContext::~JSContext()
{
    /* Free the stuff hanging off of cx. */
    if (parseMapPool_)
        js_delete(parseMapPool_);

    JS_ASSERT(!resolvingList);
}

/*
 * Since this function is only called in the context of a pending exception,
 * the caller must subsequently take an error path. If wrapping fails, it will
 * set a new (uncatchable) exception to be used in place of the original.
 */
void
JSContext::wrapPendingException()
{
    Value v = getPendingException();
    clearPendingException();
    if (compartment->wrap(this, &v))
        setPendingException(v);
}


void
JSContext::enterGenerator(JSGenerator *gen)
{
    JS_ASSERT(!gen->prevGenerator);
    gen->prevGenerator = innermostGenerator_;
    innermostGenerator_ = gen;
}

void
JSContext::leaveGenerator(JSGenerator *gen)
{
    JS_ASSERT(innermostGenerator_ == gen);
    innermostGenerator_ = innermostGenerator_->prevGenerator;
    gen->prevGenerator = NULL;
}


bool
JSContext::runningWithTrustedPrincipals() const
{
    return !compartment || compartment->principals == runtime->trustedPrincipals();
}

bool
JSContext::saveFrameChain()
{
    if (!stack.saveFrameChain())
        return false;

    if (!savedFrameChains_.append(SavedFrameChain(compartment, enterCompartmentDepth_))) {
        stack.restoreFrameChain();
        return false;
    }

    if (defaultCompartmentObject_)
        compartment = defaultCompartmentObject_->compartment();
    else
        compartment = NULL;
    enterCompartmentDepth_ = 0;

    if (isExceptionPending())
        wrapPendingException();
    return true;
}

void
JSContext::restoreFrameChain()
{
    SavedFrameChain sfc = savedFrameChains_.popCopy();
    compartment = sfc.compartment;
    enterCompartmentDepth_ = sfc.enterCompartmentCount;

    stack.restoreFrameChain();

    if (isExceptionPending())
        wrapPendingException();
}

void
JSRuntime::setGCMaxMallocBytes(size_t value)
{
    /*
     * For compatibility treat any value that exceeds PTRDIFF_T_MAX to
     * mean that value.
     */
    gcMaxMallocBytes = (ptrdiff_t(value) >= 0) ? value : size_t(-1) >> 1;
    for (CompartmentsIter c(this); !c.done(); c.next())
        c->setGCMaxMallocBytes(value);
}

void
JSRuntime::updateMallocCounter(JSContext *cx, size_t nbytes)
{
    /* We tolerate any thread races when updating gcMallocBytes. */
    ptrdiff_t oldCount = gcMallocBytes;
    ptrdiff_t newCount = oldCount - ptrdiff_t(nbytes);
    gcMallocBytes = newCount;
    if (JS_UNLIKELY(newCount <= 0 && oldCount > 0))
        onTooMuchMalloc();
    else if (cx && cx->compartment)
        cx->compartment->updateMallocCounter(nbytes);
}

JS_FRIEND_API(void)
JSRuntime::onTooMuchMalloc()
{
    TriggerGC(this, gcreason::TOO_MUCH_MALLOC);
}

JS_FRIEND_API(void *)
JSRuntime::onOutOfMemory(void *p, size_t nbytes, JSContext *cx)
{
    if (isHeapBusy())
        return NULL;

    /*
     * Retry when we are done with the background sweeping and have stopped
     * all the allocations and released the empty GC chunks.
     */
    ShrinkGCBuffers(this);
    gcHelperThread.waitBackgroundSweepOrAllocEnd();
    if (!p)
        p = js_malloc(nbytes);
    else if (p == reinterpret_cast<void *>(1))
        p = js_calloc(nbytes);
    else
      p = js_realloc(p, nbytes);
    if (p)
        return p;
    if (cx)
        js_ReportOutOfMemory(cx);
    return NULL;
}

void
JSContext::purge()
{
    if (!activeCompilations) {
        js_delete(parseMapPool_);
        parseMapPool_ = NULL;
    }
}

#if defined(JS_METHODJIT)
static bool
ComputeIsJITBroken()
{
#if !defined(ANDROID) || defined(GONK)
    return false;
#else  // ANDROID
    if (getenv("JS_IGNORE_JIT_BROKENNESS")) {
        return false;
    }

    std::string line;

    // Check for the known-bad kernel version (2.6.29).
    std::ifstream osrelease("/proc/sys/kernel/osrelease");
    std::getline(osrelease, line);
    __android_log_print(ANDROID_LOG_INFO, "Gecko", "Detected osrelease `%s'",
                        line.c_str());

    if (line.npos == line.find("2.6.29")) {
        // We're using something other than 2.6.29, so the JITs should work.
        __android_log_print(ANDROID_LOG_INFO, "Gecko", "JITs are not broken");
        return false;
    }

    // We're using 2.6.29, and this causes trouble with the JITs on i9000.
    line = "";
    bool broken = false;
    std::ifstream cpuinfo("/proc/cpuinfo");
    do {
        if (0 == line.find("Hardware")) {
            const char* blacklist[] = {
                "SCH-I400",     // Samsung Continuum
                "SGH-T959",     // Samsung i9000, Vibrant device
                "SGH-I897",     // Samsung i9000, Captivate device
                "SCH-I500",     // Samsung i9000, Fascinate device
                "SPH-D700",     // Samsung i9000, Epic device
                "GT-I9000",     // Samsung i9000, UK/Europe device
                NULL
            };
            for (const char** hw = &blacklist[0]; *hw; ++hw) {
                if (line.npos != line.find(*hw)) {
                    __android_log_print(ANDROID_LOG_INFO, "Gecko",
                                        "Blacklisted device `%s'", *hw);
                    broken = true;
                    break;
                }
            }
            break;
        }
        std::getline(cpuinfo, line);
    } while(!cpuinfo.fail() && !cpuinfo.eof());

    __android_log_print(ANDROID_LOG_INFO, "Gecko", "JITs are %sbroken",
                        broken ? "" : "not ");

    return broken;
#endif  // ifndef ANDROID
}

static bool
IsJITBrokenHere()
{
    static bool computedIsBroken = false;
    static bool isBroken = false;
    if (!computedIsBroken) {
        isBroken = ComputeIsJITBroken();
        computedIsBroken = true;
    }
    return isBroken;
}
#endif

void
JSContext::updateJITEnabled()
{
#ifdef JS_METHODJIT
    methodJitEnabled = (runOptions & JSOPTION_METHODJIT) && !IsJITBrokenHere();
#endif
}

size_t
JSContext::sizeOfIncludingThis(JSMallocSizeOfFun mallocSizeOf) const
{
    /*
     * There are other JSContext members that could be measured; the following
     * ones have been found by DMD to be worth measuring.  More stuff may be
     * added later.
     */
    return mallocSizeOf(this) + cycleDetectorSet.sizeOfExcludingThis(mallocSizeOf);
}

void
JSContext::mark(JSTracer *trc)
{
    /* Stack frames and slots are traced by StackSpace::mark. */

    /* Mark other roots-by-definition in the JSContext. */
    if (defaultCompartmentObject_ && !hasRunOption(JSOPTION_UNROOTED_GLOBAL))
        MarkObjectRoot(trc, &defaultCompartmentObject_, "default compartment object");
    if (isExceptionPending())
        MarkValueRoot(trc, &exception, "exception");

    TraceCycleDetectionSet(trc, cycleDetectorSet);

    MarkValueRoot(trc, &iterValue, "iterValue");
}

#if defined JS_THREADSAFE && defined DEBUG

AutoCheckRequestDepth::AutoCheckRequestDepth(JSContext *cx)
    : cx(cx)
{
    JS_ASSERT(cx->runtime->requestDepth || cx->runtime->isHeapBusy());
    cx->runtime->assertValidThread();
    cx->runtime->checkRequestDepth++;
}

AutoCheckRequestDepth::~AutoCheckRequestDepth()
{
    JS_ASSERT(cx->runtime->checkRequestDepth != 0);
    cx->runtime->checkRequestDepth--;
}

#endif
