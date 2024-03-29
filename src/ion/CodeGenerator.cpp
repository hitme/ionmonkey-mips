/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "CodeGenerator.h"
#include "IonLinker.h"
#include "IonSpewer.h"
#include "MIRGenerator.h"
#include "shared/CodeGenerator-shared-inl.h"
#include "jsnum.h"
#include "jsmath.h"
#include "jsinterpinlines.h"
#include "ExecutionModeInlines.h"

#include "vm/StringObject-inl.h"

using namespace js;
using namespace js::ion;

using mozilla::DebugOnly;

namespace js {
namespace ion {

StringObject *
MNewStringObject::templateObj() const {
    return &templateObj_->asString();
}

CodeGenerator::CodeGenerator(MIRGenerator *gen, LIRGraph *graph)
  : CodeGeneratorSpecific(gen, graph)
{
}

bool
CodeGenerator::visitValueToInt32(LValueToInt32 *lir)
{
    ValueOperand operand = ToValue(lir, LValueToInt32::Input);
    Register output = ToRegister(lir->output());

    Label done, simple, isInt32, isBool, notDouble;

    // Type-check switch.
    masm.branchTestInt32(Assembler::Equal, operand, &isInt32);
    masm.branchTestBoolean(Assembler::Equal, operand, &isBool);
    masm.branchTestDouble(Assembler::NotEqual, operand, &notDouble);

    // If the value is a double, see if it fits in a 32-bit int. We need to ask
    // the platform-specific codegenerator to do this.
    FloatRegister temp = ToFloatRegister(lir->tempFloat());
    masm.unboxDouble(operand, temp);

    Label fails;
    switch (lir->mode()) {
      case LValueToInt32::TRUNCATE:
        if (!emitTruncateDouble(temp, output))
            return false;
        break;
      default:
        JS_ASSERT(lir->mode() == LValueToInt32::NORMAL);
        emitDoubleToInt32(temp, output, &fails, lir->mir()->canBeNegativeZero());
        break;
    }
    masm.jump(&done);

    masm.bind(&notDouble);

    if (lir->mode() == LValueToInt32::NORMAL) {
        // If the value is not null, it's a string, object, or undefined,
        // which we can't handle here.
        masm.branchTestNull(Assembler::NotEqual, operand, &fails);
    } else {
        // Test for string or object - then fallthrough to null, which will
        // also handle undefined.
        masm.branchTestObject(Assembler::Equal, operand, &fails);
        masm.branchTestString(Assembler::Equal, operand, &fails);
    }

    if (fails.used() && !bailoutFrom(&fails, lir->snapshot()))
        return false;

    // The value is null - just emit 0.
    masm.mov(Imm32(0), output);
    masm.jump(&done);

    // Just unbox a bool, the result is 0 or 1.
    masm.bind(&isBool);
    masm.unboxBoolean(operand, output);
    masm.jump(&done);

    // Integers can be unboxed.
    masm.bind(&isInt32);
    masm.unboxInt32(operand, output);

    masm.bind(&done);

    return true;
}

static const double DoubleZero = 0.0;

bool
CodeGenerator::visitValueToDouble(LValueToDouble *lir)
{
    ValueOperand operand = ToValue(lir, LValueToDouble::Input);
    FloatRegister output = ToFloatRegister(lir->output());

    Label isDouble, isInt32, isBool, isNull, done;

    // Type-check switch.
    masm.branchTestDouble(Assembler::Equal, operand, &isDouble);
    masm.branchTestInt32(Assembler::Equal, operand, &isInt32);
    masm.branchTestBoolean(Assembler::Equal, operand, &isBool);
    masm.branchTestNull(Assembler::Equal, operand, &isNull);

    Assembler::Condition cond = masm.testUndefined(Assembler::NotEqual, operand);
    if (!bailoutIf(cond, lir->snapshot()))
        return false;
    masm.loadStaticDouble(&js_NaN, output);
    masm.jump(&done);

    masm.bind(&isNull);
    masm.loadStaticDouble(&DoubleZero, output);
    masm.jump(&done);

    masm.bind(&isBool);
    masm.boolValueToDouble(operand, output);
    masm.jump(&done);

    masm.bind(&isInt32);
    masm.int32ValueToDouble(operand, output);
    masm.jump(&done);

    masm.bind(&isDouble);
    masm.unboxDouble(operand, output);
    masm.bind(&done);

    return true;
}

bool
CodeGenerator::visitInt32ToDouble(LInt32ToDouble *lir)
{
    masm.convertInt32ToDouble(ToRegister(lir->input()), ToFloatRegister(lir->output()));
    return true;
}

bool
CodeGenerator::visitDoubleToInt32(LDoubleToInt32 *lir)
{
    Label fail;
    FloatRegister input = ToFloatRegister(lir->input());
    Register output = ToRegister(lir->output());
    emitDoubleToInt32(input, output, &fail, lir->mir()->canBeNegativeZero());
    if (!bailoutFrom(&fail, lir->snapshot()))
        return false;
    return true;
}

bool
CodeGenerator::visitTestVAndBranch(LTestVAndBranch *lir)
{
    const ValueOperand value = ToValue(lir, LTestVAndBranch::Input);
    masm.branchTestValueTruthy(value, lir->ifTrue(), ToFloatRegister(lir->tempFloat()));
    masm.jump(lir->ifFalse());
    return true;
}
 
bool
CodeGenerator::visitPolyInlineDispatch(LPolyInlineDispatch *lir)
{
    MPolyInlineDispatch *mir = lir->mir();
    Register inputReg = ToRegister(lir->input());

    InlinePropertyTable *inlinePropTable = mir->inlinePropertyTable();
    if (inlinePropTable) {
        Register tempReg = ToRegister(lir->temp());

        masm.loadPtr(Address(inputReg, JSObject::offsetOfType()), tempReg);
        for (size_t i = 0; i < inlinePropTable->numEntries(); i++) {
            types::TypeObject *typeObj = inlinePropTable->getTypeObject(i);
            JSFunction *func = inlinePropTable->getFunction(i);
            LBlock *target = mir->getFunctionBlock(func)->lir();
            masm.branchPtr(Assembler::Equal, tempReg, ImmGCPtr(typeObj), target->label());
        }
        // Jump to fallback block
        LBlock *fallback = mir->fallbackPrepBlock()->lir();
        masm.jump(fallback->label());
    } else {
        for (size_t i = 0; i < mir->numCallees(); i++) {
            JSFunction *func = mir->getFunction(i);
            LBlock *target = mir->getFunctionBlock(i)->lir();
            if (i < mir->numCallees() - 1) {
                masm.branchPtr(Assembler::Equal, inputReg, ImmGCPtr(func), target->label());
            } else {
                // Don't generate guard for final case
                masm.jump(target->label());
            }
        }
    }
    return true;
}

typedef JSFlatString *(*IntToStringFn)(JSContext *, int);
static const VMFunction IntToStringInfo =
    FunctionInfo<IntToStringFn>(Int32ToString);

bool
CodeGenerator::visitIntToString(LIntToString *lir)
{
    Register input = ToRegister(lir->input());
    Register output = ToRegister(lir->output());

    OutOfLineCode *ool = oolCallVM(IntToStringInfo, lir, (ArgList(), input),
                                   StoreRegisterTo(output));
    if (!ool)
        return false;

    masm.branch32(Assembler::AboveOrEqual, input, Imm32(StaticStrings::INT_STATIC_LIMIT),
                  ool->entry());

    masm.movePtr(ImmWord(&gen->compartment->rt->staticStrings.intStaticTable), output);
    masm.loadPtr(BaseIndex(output, input, ScalePointer), output);

    masm.bind(ool->rejoin());
    return true;
}

typedef JSObject *(*CloneRegExpObjectFn)(JSContext *, JSObject *, JSObject *);
static const VMFunction CloneRegExpObjectInfo =
    FunctionInfo<CloneRegExpObjectFn>(CloneRegExpObject);

bool
CodeGenerator::visitRegExp(LRegExp *lir)
{
    JSObject *proto = lir->mir()->getRegExpPrototype();

    pushArg(ImmGCPtr(proto));
    pushArg(ImmGCPtr(lir->mir()->source()));
    return callVM(CloneRegExpObjectInfo, lir);
}

typedef bool (*ExecuteRegExpFn)(JSContext *cx, RegExpExecType type, HandleObject regexp,
                                HandleString string, MutableHandleValue rval);
static const VMFunction ExecuteRegExpInfo = FunctionInfo<ExecuteRegExpFn>(ExecuteRegExp);

bool
CodeGenerator::visitRegExpTest(LRegExpTest *lir)
{
    pushArg(ToRegister(lir->string()));
    pushArg(ToRegister(lir->regexp()));
    pushArg(Imm32(RegExpTest));
    if (!callVM(ExecuteRegExpInfo, lir))
        return false;

    Register output = ToRegister(lir->output());
    Label notBool, end;
    masm.branchTestBoolean(Assembler::NotEqual, JSReturnOperand, &notBool);
    masm.unboxBoolean(JSReturnOperand, output);
    masm.jump(&end);
    masm.bind(&notBool);
    masm.mov(Imm32(0), output);
    masm.bind(&end);
    return true;
}

typedef JSObject *(*LambdaFn)(JSContext *, HandleFunction, HandleObject);
static const VMFunction LambdaInfo =
    FunctionInfo<LambdaFn>(js::Lambda);

bool
CodeGenerator::visitLambdaForSingleton(LLambdaForSingleton *lir)
{
    pushArg(ToRegister(lir->scopeChain()));
    pushArg(ImmGCPtr(lir->mir()->fun()));
    return callVM(LambdaInfo, lir);
}

bool
CodeGenerator::visitLambda(LLambda *lir)
{
    Register scopeChain = ToRegister(lir->scopeChain());
    Register output = ToRegister(lir->output());
    JSFunction *fun = lir->mir()->fun();

    OutOfLineCode *ool = oolCallVM(LambdaInfo, lir, (ArgList(), ImmGCPtr(fun), scopeChain),
                                   StoreRegisterTo(output));
    if (!ool)
        return false;

    JS_ASSERT(gen->compartment == fun->compartment());
    JS_ASSERT(!fun->hasSingletonType());

    masm.newGCThing(output, fun, ool->entry());
    masm.initGCThing(output, fun);

    // Initialize nargs and flags. We do this with a single uint32 to avoid
    // 16-bit writes.
    union {
        struct S {
            uint16_t nargs;
            uint16_t flags;
        } s;
        uint32_t word;
    } u;
    u.s.nargs = fun->nargs;
    u.s.flags = fun->flags & ~JSFunction::EXTENDED;

    JS_STATIC_ASSERT(offsetof(JSFunction, flags) == offsetof(JSFunction, nargs) + 2);
    masm.store32(Imm32(u.word), Address(output, offsetof(JSFunction, nargs)));
    masm.storePtr(ImmGCPtr(fun->nonLazyScript().unsafeGet()),
                  Address(output, JSFunction::offsetOfNativeOrScript()));
    masm.storePtr(scopeChain, Address(output, JSFunction::offsetOfEnvironment()));
    masm.storePtr(ImmGCPtr(fun->displayAtom()), Address(output, JSFunction::offsetOfAtom()));

    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitLabel(LLabel *lir)
{
    masm.bind(lir->label());
    return true;
}

bool
CodeGenerator::visitNop(LNop *lir)
{
    return true;
}

bool
CodeGenerator::visitOsiPoint(LOsiPoint *lir)
{
    // Note: markOsiPoint ensures enough space exists between the last
    // LOsiPoint and this one to patch adjacent call instructions.

    JS_ASSERT(masm.framePushed() == frameSize());

    uint32 osiCallPointOffset;
    if (!markOsiPoint(lir, &osiCallPointOffset))
        return false;

    LSafepoint *safepoint = lir->associatedSafepoint();
    JS_ASSERT(!safepoint->osiCallPointOffset());
    safepoint->setOsiCallPointOffset(osiCallPointOffset);
    return true;
}

bool
CodeGenerator::visitGoto(LGoto *lir)
{
    LBlock *target = lir->target()->lir();

    // No jump necessary if we can fall through to the next block.
    if (isNextBlock(target))
        return true;

    masm.jump(target->label());
    return true;
}

bool
CodeGenerator::visitTableSwitch(LTableSwitch *ins)
{
    MTableSwitch *mir = ins->mir();
    Label *defaultcase = mir->getDefault()->lir()->label();
    const LAllocation *temp;

    if (ins->index()->isDouble()) {
        temp = ins->tempInt();

        // The input is a double, so try and convert it to an integer.
        // If it does not fit in an integer, take the default case.
        emitDoubleToInt32(ToFloatRegister(ins->index()), ToRegister(temp), defaultcase, false);
    } else {
        temp = ins->index();
    }

    return emitTableSwitchDispatch(mir, ToRegister(temp), ToRegisterOrInvalid(ins->tempPointer()));
}

bool
CodeGenerator::visitTableSwitchV(LTableSwitchV *ins)
{
    MTableSwitch *mir = ins->mir();
    Label *defaultcase = mir->getDefault()->lir()->label();

    Register index = ToRegister(ins->tempInt());
    ValueOperand value = ToValue(ins, LTableSwitchV::InputValue);
    Register tag = masm.extractTag(value, index);
    masm.branchTestNumber(Assembler::NotEqual, tag, defaultcase);

    Label unboxInt, isInt;
    masm.branchTestInt32(Assembler::Equal, tag, &unboxInt);
    {
        FloatRegister floatIndex = ToFloatRegister(ins->tempFloat());
        masm.unboxDouble(value, floatIndex);
        emitDoubleToInt32(floatIndex, index, defaultcase, false);
        masm.jump(&isInt);
    }

    masm.bind(&unboxInt);
    masm.unboxInt32(value, index);

    masm.bind(&isInt);

    return emitTableSwitchDispatch(mir, index, ToRegisterOrInvalid(ins->tempPointer()));
}

bool
CodeGenerator::visitParameter(LParameter *lir)
{
    return true;
}

bool
CodeGenerator::visitCallee(LCallee *lir)
{
    return true;
}

bool
CodeGenerator::visitStart(LStart *lir)
{
    return true;
}

bool
CodeGenerator::visitReturn(LReturn *lir)
{
#if defined(JS_NUNBOX32)
    DebugOnly<LAllocation *> type    = lir->getOperand(TYPE_INDEX);
    DebugOnly<LAllocation *> payload = lir->getOperand(PAYLOAD_INDEX);
    JS_ASSERT(ToRegister(type)    == JSReturnReg_Type);
    JS_ASSERT(ToRegister(payload) == JSReturnReg_Data);
#elif defined(JS_PUNBOX64)
    DebugOnly<LAllocation *> result = lir->getOperand(0);
    JS_ASSERT(ToRegister(result) == JSReturnReg);
#endif
    // Don't emit a jump to the return label if this is the last block.
    if (current->mir() != *gen->graph().poBegin())
        masm.jump(returnLabel_);
    return true;
}

bool
CodeGenerator::visitOsrEntry(LOsrEntry *lir)
{
    // Remember the OSR entry offset into the code buffer.
    masm.flushBuffer();
    setOsrEntryOffset(masm.size());

    // Allocate the full frame for this function.
    masm.subPtr(Imm32(frameSize()), StackPointer);
    return true;
}

bool
CodeGenerator::visitOsrScopeChain(LOsrScopeChain *lir)
{
    const LAllocation *frame   = lir->getOperand(0);
    const LDefinition *object  = lir->getDef(0);

    const ptrdiff_t frameOffset = StackFrame::offsetOfScopeChain();

    masm.loadPtr(Address(ToRegister(frame), frameOffset), ToRegister(object));
    return true;
}

bool
CodeGenerator::visitStackArgT(LStackArgT *lir)
{
    const LAllocation *arg = lir->getArgument();
    MIRType argType = lir->mir()->getArgument()->type();
    uint32 argslot = lir->argslot();

    int32 stack_offset = StackOffsetOfPassedArg(argslot);
    Address dest(StackPointer, stack_offset);

    if (arg->isFloatReg())
        masm.storeDouble(ToFloatRegister(arg), dest);
    else if (arg->isRegister())
        masm.storeValue(ValueTypeFromMIRType(argType), ToRegister(arg), dest);
    else
        masm.storeValue(*(arg->toConstant()), dest);

    return pushedArgumentSlots_.append(StackOffsetToSlot(stack_offset));
}

bool
CodeGenerator::visitStackArgV(LStackArgV *lir)
{
    ValueOperand val = ToValue(lir, 0);
    uint32 argslot = lir->argslot();
    int32 stack_offset = StackOffsetOfPassedArg(argslot);

    masm.storeValue(val, Address(StackPointer, stack_offset));
    return pushedArgumentSlots_.append(StackOffsetToSlot(stack_offset));
}

bool
CodeGenerator::visitInteger(LInteger *lir)
{
    masm.move32(Imm32(lir->getValue()), ToRegister(lir->output()));
    return true;
}

bool
CodeGenerator::visitPointer(LPointer *lir)
{
    if (lir->kind() == LPointer::GC_THING)
        masm.movePtr(ImmGCPtr(lir->gcptr()), ToRegister(lir->output()));
    else
        masm.movePtr(ImmWord(lir->ptr()), ToRegister(lir->output()));
    return true;
}

bool
CodeGenerator::visitSlots(LSlots *lir)
{
    Address slots(ToRegister(lir->object()), JSObject::offsetOfSlots());
    masm.loadPtr(slots, ToRegister(lir->output()));
    return true;
}

bool
CodeGenerator::visitStoreSlotV(LStoreSlotV *store)
{
    Register base = ToRegister(store->slots());
    int32 offset  = store->mir()->slot() * sizeof(Value);

    const ValueOperand value = ToValue(store, LStoreSlotV::Value);

    if (store->mir()->needsBarrier())
       emitPreBarrier(Address(base, offset), MIRType_Value);

    masm.storeValue(value, Address(base, offset));
    return true;
}

bool
CodeGenerator::visitElements(LElements *lir)
{
    Address elements(ToRegister(lir->object()), JSObject::offsetOfElements());
    masm.loadPtr(elements, ToRegister(lir->output()));
    return true;
}

bool
CodeGenerator::visitFunctionEnvironment(LFunctionEnvironment *lir)
{
    Address environment(ToRegister(lir->function()), JSFunction::offsetOfEnvironment());
    masm.loadPtr(environment, ToRegister(lir->output()));
    return true;
}

bool
CodeGenerator::visitTypeBarrier(LTypeBarrier *lir)
{
    ValueOperand operand = ToValue(lir, LTypeBarrier::Input);
    Register scratch = ToRegister(lir->temp());

    Label mismatched;
    masm.guardTypeSet(operand, lir->mir()->typeSet(), scratch, &mismatched);
    if (!bailoutFrom(&mismatched, lir->snapshot()))
        return false;
    return true;
}

bool
CodeGenerator::visitMonitorTypes(LMonitorTypes *lir)
{
    ValueOperand operand = ToValue(lir, LMonitorTypes::Input);
    Register scratch = ToRegister(lir->temp());

    Label mismatched;
    masm.guardTypeSet(operand, lir->mir()->typeSet(), scratch, &mismatched);
    if (!bailoutFrom(&mismatched, lir->snapshot()))
        return false;
    return true;
}

bool
CodeGenerator::visitCallNative(LCallNative *call)
{
    JSFunction *target = call->getSingleTarget();
    JS_ASSERT(target);
    JS_ASSERT(target->isNative());

    int callargslot = call->argslot();
    int unusedStack = StackOffsetOfPassedArg(callargslot);

    // Registers used for callWithABI() argument-passing.
    const Register argJSContextReg = ToRegister(call->getArgJSContextReg());
    const Register argUintNReg     = ToRegister(call->getArgUintNReg());
    const Register argVpReg        = ToRegister(call->getArgVpReg());

    // Misc. temporary registers.
    const Register tempReg = ToRegister(call->getTempReg());

    DebugOnly<uint32> initialStack = masm.framePushed();

    masm.checkStackAlignment();

    // Native functions have the signature:
    //  bool (*)(JSContext *, unsigned, Value *vp)
    // Where vp[0] is space for an outparam, vp[1] is |this|, and vp[2] onward
    // are the function arguments.

    // Allocate space for the outparam, moving the StackPointer to what will be &vp[1].
    masm.adjustStack(unusedStack);

    // Push a Value containing the callee object: natives are allowed to access their callee before
    // setitng the return value. The StackPointer is moved to &vp[0].
    masm.Push(ObjectValue(*target));

    // Preload arguments into registers.
    masm.loadJSContext(argJSContextReg);
    masm.move32(Imm32(call->numStackArgs()), argUintNReg);
    masm.movePtr(StackPointer, argVpReg);

    masm.Push(argUintNReg);

    // Construct native exit frame.
    uint32 safepointOffset;
    if (!masm.buildFakeExitFrame(tempReg, &safepointOffset))
        return false;
    masm.enterFakeExitFrame();

    if (!markSafepointAt(safepointOffset, call))
        return false;

    // Construct and execute call.
    masm.setupUnalignedABICall(3, tempReg);
    masm.passABIArg(argJSContextReg);
    masm.passABIArg(argUintNReg);
    masm.passABIArg(argVpReg);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, target->native()));

    // Test for failure.
    Label success, exception;
    masm.branchTest32(Assembler::Zero, ReturnReg, ReturnReg, &exception);

    // Load the outparam vp[0] into output register(s).
    masm.loadValue(Address(StackPointer, IonNativeExitFrameLayout::offsetOfResult()), JSReturnOperand);
    masm.jump(&success);

    // Handle exception case.
    {
        masm.bind(&exception);
// Asse.. buf grow invalidates labels(256*16): JSC::AssemblerBuffer::grow(int) at ../../assembler/assembler/AssemblerBuffer.h:240
        masm.handleException();
    }
    masm.bind(&success);

    // The next instruction is removing the footer of the exit frame, so there
    // is no need for leaveFakeExitFrame.

    // Move the StackPointer back to its original location, unwinding the native exit frame.
    masm.adjustStack(IonNativeExitFrameLayout::Size() - unusedStack);
    JS_ASSERT(masm.framePushed() == initialStack);

    dropArguments(call->numStackArgs() + 1);
    return true;
}

bool
CodeGenerator::visitCallDOMNative(LCallDOMNative *call)
{
    JSFunction *target = call->getSingleTarget();
    JS_ASSERT(target);
    JS_ASSERT(target->isNative());
    JS_ASSERT(target->jitInfo());
    JS_ASSERT(call->mir()->isDOMFunction());

    int callargslot = call->argslot();
    int unusedStack = StackOffsetOfPassedArg(callargslot);

    // Registers used for callWithABI() argument-passing.
    const Register argJSContext = ToRegister(call->getArgJSContext());
    const Register argObj       = ToRegister(call->getArgObj());
    const Register argPrivate   = ToRegister(call->getArgPrivate());
    const Register argArgc      = ToRegister(call->getArgArgc());
    const Register argVp        = ToRegister(call->getArgVp());

    DebugOnly<uint32> initialStack = masm.framePushed();

    masm.checkStackAlignment();

    // DOM methods have the signature:
    //  bool (*)(JSContext *, HandleObject, void *private, unsigned argc, Value *vp)
    // Where vp[0] is space for an outparam and the callee, vp[1] is |this|, and vp[2] onward
    // are the function arguments.

    // Nestle the stack up against the pushed arguments, leaving StackPointer at
    // &vp[1]
    masm.adjustStack(unusedStack);
    // argObj is filled with the extracted object, then returned.
    Register obj = masm.extractObject(Address(StackPointer, 0), argObj);
    JS_ASSERT(obj == argObj);

    // Push a Value containing the callee object: natives are allowed to access their callee before
    // setitng the return value. The StackPointer is moved to &vp[0].
    masm.Push(ObjectValue(*target));
    masm.movePtr(StackPointer, argVp);

    // GetReservedSlot(obj, DOM_PROTO_INSTANCE_CLASS_SLOT).toPrivate()
    masm.loadPrivate(Address(obj, JSObject::getFixedSlotOffset(0)), argPrivate);

    // Load argc from the call instruction.
    masm.move32(Imm32(call->numStackArgs()), argArgc);
    // Push argument into what will become the IonExitFrame
    masm.Push(argArgc);

    // Push |this| object for passing HandleObject. We push after argc to
    // maintain the same sp-relative location of the object pointer with other
    // DOMExitFrames.
    masm.Push(argObj);
    masm.movePtr(StackPointer, argObj);

    // Construct native exit frame.
    uint32 safepointOffset;
    if (!masm.buildFakeExitFrame(argJSContext, &safepointOffset))
        return false;
    masm.enterFakeExitFrame(ION_FRAME_DOMMETHOD);

    if (!markSafepointAt(safepointOffset, call))
        return false;

    // Construct and execute call.
    masm.setupUnalignedABICall(5, argJSContext);

    masm.loadJSContext(argJSContext);

    masm.passABIArg(argJSContext);
    masm.passABIArg(argObj);
    masm.passABIArg(argPrivate);
    masm.passABIArg(argArgc);
    masm.passABIArg(argVp);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, target->jitInfo()->op));

    if (target->jitInfo()->isInfallible) {
        masm.loadValue(Address(StackPointer, IonDOMMethodExitFrameLayout::offsetOfResult()),
                       JSReturnOperand);
    } else {
        // Test for failure.
        Label success, exception;
        masm.branchTest32(Assembler::Zero, ReturnReg, ReturnReg, &exception);

        // Load the outparam vp[0] into output register(s).
        masm.loadValue(Address(StackPointer, IonDOMMethodExitFrameLayout::offsetOfResult()),
                       JSReturnOperand);
        masm.jump(&success);

        // Handle exception case.
        {
            masm.bind(&exception);
            masm.handleException();
        }
        masm.bind(&success);
    }

    // The next instruction is removing the footer of the exit frame, so there
    // is no need for leaveFakeExitFrame.

    // Move the StackPointer back to its original location, unwinding the native exit frame.
    masm.adjustStack(IonDOMMethodExitFrameLayout::Size() - unusedStack);
    JS_ASSERT(masm.framePushed() == initialStack);

    dropArguments(call->numStackArgs() + 1);
    return true;
}

typedef bool (*GetIntrinsicValueFn)(JSContext *cx, HandlePropertyName, MutableHandleValue);
static const VMFunction GetIntrinsicValueInfo =
    FunctionInfo<GetIntrinsicValueFn>(GetIntrinsicValue);

bool
CodeGenerator::visitCallGetIntrinsicValue(LCallGetIntrinsicValue *lir)
{
    pushArg(ImmGCPtr(lir->mir()->name()));
    return callVM(GetIntrinsicValueInfo, lir);
}

typedef bool (*InvokeFunctionFn)(JSContext *, JSFunction *, uint32, Value *, Value *);
static const VMFunction InvokeFunctionInfo = FunctionInfo<InvokeFunctionFn>(InvokeFunction);

bool
CodeGenerator::emitCallInvokeFunction(LInstruction *call, Register calleereg,
                                      uint32 argc, uint32 unusedStack)
{
    // Nestle %esp up to the argument vector.
    // Each path must account for framePushed_ separately, for callVM to be valid.
    masm.freeStack(unusedStack);

    pushArg(StackPointer); // argv.
    pushArg(Imm32(argc));  // argc.
    pushArg(calleereg);    // JSFunction *.

    if (!callVM(InvokeFunctionInfo, call))
        return false;

    // Un-nestle %esp from the argument vector. No prefix was pushed.
    masm.reserveStack(unusedStack);
    return true;
}

static inline int32_t ionOffset(ExecutionMode executionMode)
{
    switch (executionMode) {
      case SequentialExecution: return offsetof(JSScript, ion);
      case ParallelExecution: return offsetof(JSScript, parallelIon);
    }

    JS_ASSERT(false);
    return offsetof(JSScript, ion);
}

bool
CodeGenerator::visitCallGeneric(LCallGeneric *call)
{
    Register calleereg = ToRegister(call->getFunction());
    Register objreg    = ToRegister(call->getTempObject());
    Register nargsreg  = ToRegister(call->getNargsReg());
    uint32 unusedStack = StackOffsetOfPassedArg(call->argslot());
    Label invoke, thunk, makeCall, end;

    // Known-target case is handled by LCallKnown.
    JS_ASSERT(!call->hasSingleTarget());
    // Unknown constructor case is handled by LCallConstructor.
    JS_ASSERT(!call->mir()->isConstructing());

    // Generate an ArgumentsRectifier.
    IonCompartment *ion = gen->ionCompartment();
    IonCode *argumentsRectifier = ion->getArgumentsRectifier();

    masm.checkStackAlignment();

    // Guard that calleereg is actually a function object.
    masm.loadObjClass(calleereg, nargsreg);
    masm.cmpPtr(nargsreg, ImmWord(&js::FunctionClass));
    if (!bailoutIf(Assembler::NotEqual, call->snapshot()))
        return false;

    // Guard that calleereg is an interpreted function with a JSScript:
    masm.branchIfFunctionHasNoScript(calleereg, &invoke);

    // Knowing that calleereg is a non-native function, load the JSScript.
    masm.loadPtr(Address(calleereg, offsetof(JSFunction, u.i.script_)), objreg);
    ExecutionMode executionMode = gen->info().executionMode();
    masm.loadPtr(Address(objreg, ionOffset(executionMode)), objreg);

    // Guard that the IonScript has been compiled.
    masm.branchPtr(Assembler::BelowOrEqual, objreg, ImmWord(ION_COMPILING_SCRIPT), &invoke);

    // Nestle the StackPointer up to the argument vector.
    masm.freeStack(unusedStack);

    // Construct the IonFramePrefix.
    uint32 descriptor = MakeFrameDescriptor(masm.framePushed(), IonFrame_OptimizedJS);
    masm.Push(Imm32(call->numActualArgs()));
    masm.Push(calleereg);
    masm.Push(Imm32(descriptor));

    // Check whether the provided arguments satisfy target argc.
    masm.load16ZeroExtend(Address(calleereg, offsetof(JSFunction, nargs)), nargsreg);
    masm.cmp32(nargsreg, Imm32(call->numStackArgs()));
    masm.j(Assembler::Above, &thunk);

    // No argument fixup needed. Load the start of the target IonCode.
    masm.loadPtr(Address(objreg, IonScript::offsetOfMethod()), objreg);
    masm.loadPtr(Address(objreg, IonCode::offsetOfCode()), objreg);
    masm.jump(&makeCall);

    // Argument fixed needed. Load the ArgumentsRectifier.
    masm.bind(&thunk);
    {
        JS_ASSERT(ArgumentsRectifierReg != objreg);
        masm.movePtr(ImmGCPtr(argumentsRectifier), objreg); // Necessary for GC marking.
        masm.loadPtr(Address(objreg, IonCode::offsetOfCode()), objreg);
        masm.move32(Imm32(call->numStackArgs()), ArgumentsRectifierReg);
    }

    // Finally call the function in objreg.
    masm.bind(&makeCall);
    uint32 callOffset = masm.callIon(objreg);
    if (!markSafepointAt(callOffset, call))
        return false;

    // Increment to remove IonFramePrefix; decrement to fill FrameSizeClass.
    // The return address has already been removed from the Ion frame.
    int prefixGarbage = sizeof(IonJSFrameLayout) - sizeof(void *);
    masm.adjustStack(prefixGarbage - unusedStack);
    masm.jump(&end);

    // Handle uncompiled or native functions.
    masm.bind(&invoke);
    if (!emitCallInvokeFunction(call, calleereg, call->numActualArgs(), unusedStack))
        return false;

    masm.bind(&end);
    dropArguments(call->numStackArgs() + 1);
    return true;
}

bool
CodeGenerator::visitCallKnown(LCallKnown *call)
{
    JSContext *cx = GetIonContext()->cx;
    Register calleereg = ToRegister(call->getFunction());
    Register objreg    = ToRegister(call->getTempObject());
    uint32 unusedStack = StackOffsetOfPassedArg(call->argslot());
    JSFunction *target = call->getSingleTarget();
    Label end, invoke;

    // Native single targets are handled by LCallNative.
    JS_ASSERT(!target->isNative());
    // Missing arguments must have been explicitly appended by the IonBuilder.
    JS_ASSERT(target->nargs <= call->numStackArgs());

    masm.checkStackAlignment();

    // If the function is known to be uncompilable, only emit the call to InvokeFunction.
    ExecutionMode executionMode = gen->info().executionMode();
    RootedScript targetScript(cx, target->nonLazyScript());
    if (GetIonScript(targetScript, executionMode) == ION_DISABLED_SCRIPT) {
        if (!emitCallInvokeFunction(call, calleereg, call->numActualArgs(), unusedStack))
            return false;

        if (call->mir()->isConstructing()) {
            Label notPrimitive;
            masm.branchTestPrimitive(Assembler::NotEqual, JSReturnOperand, &notPrimitive);
            masm.loadValue(Address(StackPointer, unusedStack), JSReturnOperand);
            masm.bind(&notPrimitive);
        }

        dropArguments(call->numStackArgs() + 1);
        return true;
    }

    // Knowing that calleereg is a non-native function, load the JSScript.
    masm.loadPtr(Address(calleereg, offsetof(JSFunction, u.i.script_)), objreg);
    masm.loadPtr(Address(objreg, ionOffset(executionMode)), objreg);

    // Guard that the IonScript has been compiled.
    masm.branchPtr(Assembler::BelowOrEqual, objreg, ImmWord(ION_COMPILING_SCRIPT), &invoke);

    // Load the start of the target IonCode.
    masm.loadPtr(Address(objreg, IonScript::offsetOfMethod()), objreg);
    masm.loadPtr(Address(objreg, IonCode::offsetOfCode()), objreg);

    // Nestle the StackPointer up to the argument vector.
    masm.freeStack(unusedStack);

    // Construct the IonFramePrefix.
    uint32 descriptor = MakeFrameDescriptor(masm.framePushed(), IonFrame_OptimizedJS);
    masm.Push(Imm32(call->numActualArgs()));
    masm.Push(calleereg);
    masm.Push(Imm32(descriptor));

    // Finally call the function in objreg.
    uint32 callOffset = masm.callIon(objreg);
    if (!markSafepointAt(callOffset, call))
        return false;

    // Increment to remove IonFramePrefix; decrement to fill FrameSizeClass.
    // The return address has already been removed from the Ion frame.
    int prefixGarbage = sizeof(IonJSFrameLayout) - sizeof(void *);
    masm.adjustStack(prefixGarbage - unusedStack);
    masm.jump(&end);

    // Handle uncompiled functions.
    masm.bind(&invoke);
    if (!emitCallInvokeFunction(call, calleereg, call->numActualArgs(), unusedStack))
        return false;

    masm.bind(&end);

    // If the return value of the constructing function is Primitive,
    // replace the return value with the Object from CreateThis.
    if (call->mir()->isConstructing()) {
        Label notPrimitive;
        masm.branchTestPrimitive(Assembler::NotEqual, JSReturnOperand, &notPrimitive);
        masm.loadValue(Address(StackPointer, unusedStack), JSReturnOperand);
        masm.bind(&notPrimitive);
    }

    dropArguments(call->numStackArgs() + 1);
    return true;
}

typedef bool (*InvokeConstructorFn)(JSContext *, JSObject *, uint32, Value *, Value *);
static const VMFunction InvokeConstructorInfo =
    FunctionInfo<InvokeConstructorFn>(ion::InvokeConstructor);

bool
CodeGenerator::visitCallConstructor(LCallConstructor *call)
{
    JS_ASSERT(call->mir()->isConstructing());

    // Holds the function object.
    const LAllocation *callee = call->getFunction();
    Register calleereg = ToRegister(callee);

    uint32 callargslot = call->argslot();
    uint32 unusedStack = StackOffsetOfPassedArg(callargslot);

    // Nestle %esp up to the argument vector.
    masm.freeStack(unusedStack);

    pushArg(StackPointer);                  // argv.
    pushArg(Imm32(call->numActualArgs()));  // argc.
    pushArg(calleereg);                     // JSFunction *.

    if (!callVM(InvokeConstructorInfo, call))
        return false;

    // Un-nestle %esp from the argument vector. No prefix was pushed.
    masm.reserveStack(unusedStack);

    dropArguments(call->numStackArgs() + 1);
    return true;
}

bool
CodeGenerator::emitCallInvokeFunction(LApplyArgsGeneric *apply, Register extraStackSize)
{
    Register objreg = ToRegister(apply->getTempObject());
    JS_ASSERT(objreg != extraStackSize);

    // Push the space used by the arguments.
    masm.movePtr(StackPointer, objreg);
    masm.Push(extraStackSize);

    pushArg(objreg);                           // argv.
    pushArg(ToRegister(apply->getArgc()));     // argc.
    pushArg(ToRegister(apply->getFunction())); // JSFunction *.

    // This specialization og callVM restore the extraStackSize after the call.
    if (!callVM(InvokeFunctionInfo, apply, &extraStackSize))
        return false;

    masm.Pop(extraStackSize);
    return true;
}

// Do not bailout after the execution of this function since the stack no longer
// correspond to what is expected by the snapshots.
void
CodeGenerator::emitPushArguments(LApplyArgsGeneric *apply, Register extraStackSpace)
{
    // Holds the function nargs. Initially undefined.
    Register argcreg = ToRegister(apply->getArgc());

    Register copyreg = ToRegister(apply->getTempObject());
    size_t argvOffset = frameSize() + IonJSFrameLayout::offsetOfActualArgs();
    Label end;

    // Initialize the loop counter AND Compute the stack usage (if == 0)
    masm.movePtr(argcreg, extraStackSpace);
    masm.branchTestPtr(Assembler::Zero, argcreg, argcreg, &end);

    // Copy arguments.
    {
        Register count = extraStackSpace; // <- argcreg
        Label loop;
        masm.bind(&loop);

        // We remove sizeof(void*) from argvOffset because withtout it we target
        // the address after the memory area that we want to copy.
        BaseIndex disp(StackPointer, argcreg, ScaleFromShift(sizeof(Value)), argvOffset - sizeof(void*));

        // Do not use Push here because other this account to 1 in the framePushed
        // instead of 0.  These push are only counted by argcreg.
        masm.loadPtr(disp, copyreg);
        masm.push(copyreg);

        // Handle 32 bits architectures.
        if (sizeof(Value) == 2 * sizeof(void*)) {
            masm.loadPtr(disp, copyreg);
            masm.push(copyreg);
        }

        masm.decBranchPtr(Assembler::NonZero, count, Imm32(1), &loop);
    }

    // Compute the stack usage.
    masm.movePtr(argcreg, extraStackSpace);
    masm.lshiftPtr(Imm32::ShiftOf(ScaleFromShift(sizeof(Value))), extraStackSpace);

    // Join with all arguments copied and the extra stack usage computed.
    masm.bind(&end);

    // Push |this|.
    masm.addPtr(Imm32(sizeof(Value)), extraStackSpace);
    masm.pushValue(ToValue(apply, LApplyArgsGeneric::ThisIndex));
}

void
CodeGenerator::emitPopArguments(LApplyArgsGeneric *apply, Register extraStackSpace)
{
    // Pop |this| and Arguments.
    masm.freeStack(extraStackSpace);
}

bool
CodeGenerator::visitApplyArgsGeneric(LApplyArgsGeneric *apply)
{
    JSContext *cx = GetIonContext()->cx;

    // Holds the function object.
    Register calleereg = ToRegister(apply->getFunction());

    // Temporary register for modifying the function object.
    Register objreg = ToRegister(apply->getTempObject());
    Register copyreg = ToRegister(apply->getTempCopy());

    // Holds the function nargs. Initially undefined.
    Register argcreg = ToRegister(apply->getArgc());

    // Unless already known, guard that calleereg is actually a function object.
    if (!apply->hasSingleTarget()) {
        masm.loadObjClass(calleereg, objreg);
        masm.cmpPtr(objreg, ImmWord(&js::FunctionClass));
        if (!bailoutIf(Assembler::NotEqual, apply->snapshot()))
            return false;
    }

    // Copy the arguments of the current function.
    emitPushArguments(apply, copyreg);

    masm.checkStackAlignment();

    // If the function is known to be uncompilable, only emit the call to InvokeFunction.
    ExecutionMode executionMode = gen->info().executionMode();
    if (apply->hasSingleTarget()) {
        RootedFunction target(cx, apply->getSingleTarget());
        if (!CanIonCompile(cx, target, executionMode)) {
            if (!emitCallInvokeFunction(apply, copyreg))
                return false;
            emitPopArguments(apply, copyreg);
            return true;
        }
    }

    Label end, invoke;

    // Guard that calleereg is an interpreted function with a JSScript:
    if (!apply->hasSingleTarget()) {
        masm.branchIfFunctionHasNoScript(calleereg, &invoke);
    } else {
        // Native single targets are handled by LCallNative.
        JS_ASSERT(!apply->getSingleTarget()->isNative());
    }

    // Knowing that calleereg is a non-native function, load the JSScript.
    masm.loadPtr(Address(calleereg, offsetof(JSFunction, u.i.script_)), objreg);
    masm.loadPtr(Address(objreg, ionOffset(executionMode)), objreg);

    // Guard that the IonScript has been compiled.
    masm.branchPtr(Assembler::BelowOrEqual, objreg, ImmWord(ION_COMPILING_SCRIPT), &invoke);

    // Call with an Ion frame or a rectifier frame.
    {
        // Create the frame descriptor.
        unsigned pushed = masm.framePushed();
        masm.addPtr(Imm32(pushed), copyreg);
        masm.makeFrameDescriptor(copyreg, IonFrame_OptimizedJS);

        masm.Push(argcreg);
        masm.Push(calleereg);
        masm.Push(copyreg); // descriptor

        Label underflow, rejoin;

        // Check whether the provided arguments satisfy target argc.
        if (!apply->hasSingleTarget()) {
            masm.load16ZeroExtend(Address(calleereg, offsetof(JSFunction, nargs)), copyreg);
            masm.cmp32(argcreg, copyreg);
            masm.j(Assembler::Below, &underflow);
        } else {
            masm.cmp32(argcreg, Imm32(apply->getSingleTarget()->nargs));
            masm.j(Assembler::Below, &underflow);
        }

        // No argument fixup needed. Load the start of the target IonCode.
        {
            masm.loadPtr(Address(objreg, IonScript::offsetOfMethod()), objreg);
            masm.loadPtr(Address(objreg, IonCode::offsetOfCode()), objreg);

            // Skip the construction of the rectifier frame because we have no
            // underflow.
            masm.jump(&rejoin);
        }

        // Argument fixup needed. Get ready to call the argumentsRectifier.
        {
            masm.bind(&underflow);

            // Hardcode the address of the argumentsRectifier code.
            IonCompartment *ion = gen->ionCompartment();
            IonCode *argumentsRectifier = ion->getArgumentsRectifier();

            JS_ASSERT(ArgumentsRectifierReg != objreg);
            masm.movePtr(ImmGCPtr(argumentsRectifier), objreg); // Necessary for GC marking.
            masm.loadPtr(Address(objreg, IonCode::offsetOfCode()), objreg);
            masm.movePtr(argcreg, ArgumentsRectifierReg);
        }

        masm.bind(&rejoin);

        // Finally call the function in objreg, as assigned by one of the paths above.
        uint32 callOffset = masm.callIon(objreg);
        if (!markSafepointAt(callOffset, apply))
            return false;

        // Recover the number of arguments from the frame descriptor.
        masm.loadPtr(Address(StackPointer, 0), copyreg);
        masm.rshiftPtr(Imm32(FRAMESIZE_SHIFT), copyreg);
        masm.subPtr(Imm32(pushed), copyreg);

        // Increment to remove IonFramePrefix; decrement to fill FrameSizeClass.
        // The return address has already been removed from the Ion frame.
        int prefixGarbage = sizeof(IonJSFrameLayout) - sizeof(void *);
        masm.adjustStack(prefixGarbage);
        masm.jump(&end);
    }

    // Handle uncompiled or native functions.
    {
        masm.bind(&invoke);
        if (!emitCallInvokeFunction(apply, copyreg))
            return false;
    }

    // Pop arguments and continue.
    masm.bind(&end);
    emitPopArguments(apply, copyreg);

    return true;
}

// Registers safe for use before generatePrologue().
static const uint32 EntryTempMask = Registers::TempMask & ~(1 << OsrFrameReg.code());

bool
CodeGenerator::generateArgumentsChecks()
{
    MIRGraph &mir = gen->graph();
    MResumePoint *rp = mir.entryResumePoint();

    // Reserve the amount of stack the actual frame will use. We have to undo
    // this before falling through to the method proper though, because the
    // monomorphic call case will bypass this entire path.

#if 0 && defined JS_CPU_MIPS
    masm.push(ra);
#endif
    masm.reserveStack(frameSize());

    // No registers are allocated yet, so it's safe to grab anything.
    Register temp = GeneralRegisterSet(EntryTempMask).getAny();

    CompileInfo &info = gen->info();

    // Indexes need to be shifted by one, to skip the scope chain slot.
    JS_ASSERT(info.scopeChainSlot() == 0);
    static const uint32 START_SLOT = 1;

    Label mismatched;
    for (uint32 i = START_SLOT; i < CountArgSlots(info.fun()); i++) {
        // All initial parameters are guaranteed to be MParameters.
        MParameter *param = rp->getOperand(i)->toParameter();
        types::TypeSet *types = param->typeSet();
        if (!types || types->unknown())
            continue;

        // Use ReturnReg as a scratch register here, since not all platforms
        // have an actual ScratchReg.
        int32 offset = ArgToStackOffset((i - START_SLOT) * sizeof(Value));
        masm.guardTypeSet(Address(StackPointer, offset), types, temp, &mismatched);
    }

    if (mismatched.used() && !bailoutFrom(&mismatched, graph.entrySnapshot()))
        return false;

    masm.freeStack(frameSize());

    return true;
}

// Out-of-line path to report over-recursed error and fail.
class CheckOverRecursedFailure : public OutOfLineCodeBase<CodeGenerator>
{
    LCheckOverRecursed *lir_;

  public:
    CheckOverRecursedFailure(LCheckOverRecursed *lir)
      : lir_(lir)
    { }

    bool accept(CodeGenerator *codegen) {
        return codegen->visitCheckOverRecursedFailure(this);
    }

    LCheckOverRecursed *lir() const {
        return lir_;
    }
};

bool
CodeGenerator::visitCheckOverRecursed(LCheckOverRecursed *lir)
{
    // Ensure that this frame will not cross the stack limit.
    // This is a weak check, justified by Ion using the C stack: we must always
    // be some distance away from the actual limit, since if the limit is
    // crossed, an error must be thrown, which requires more frames.
    //
    // It must always be possible to trespass past the stack limit.
    // Ion may legally place frames very close to the limit. Calling additional
    // C functions may then violate the limit without any checking.

    JSRuntime *rt = gen->compartment->rt;
    Register limitReg = ToRegister(lir->limitTemp());

    // Since Ion frames exist on the C stack, the stack limit may be
    // dynamically set by JS_SetThreadStackLimit() and JS_SetNativeStackQuota().
    uintptr_t *limitAddr = &rt->ionStackLimit;
    masm.loadPtr(AbsoluteAddress(limitAddr), limitReg);

    CheckOverRecursedFailure *ool = new CheckOverRecursedFailure(lir);
    if (!addOutOfLineCode(ool))
        return false;

    // Conditional forward (unlikely) branch to failure.
    masm.branchPtr(Assembler::BelowOrEqual, StackPointer, limitReg, ool->entry());
    masm.bind(ool->rejoin());

    return true;
}

typedef bool (*DefVarOrConstFn)(JSContext *, HandlePropertyName, unsigned, HandleObject);
static const VMFunction DefVarOrConstInfo =
    FunctionInfo<DefVarOrConstFn>(DefVarOrConst);

bool
CodeGenerator::visitDefVar(LDefVar *lir)
{
    Register scopeChain = ToRegister(lir->getScopeChain());
    Register nameTemp   = ToRegister(lir->nameTemp());

    masm.movePtr(ImmGCPtr(lir->mir()->name()), nameTemp);

    pushArg(scopeChain); // JSObject *
    pushArg(Imm32(lir->mir()->attrs())); // unsigned
    pushArg(nameTemp); // PropertyName *

    if (!callVM(DefVarOrConstInfo, lir))
        return false;

    return true;
}

typedef bool (*ReportOverRecursedFn)(JSContext *);
static const VMFunction CheckOverRecursedInfo =
    FunctionInfo<ReportOverRecursedFn>(CheckOverRecursed);

bool
CodeGenerator::visitCheckOverRecursedFailure(CheckOverRecursedFailure *ool)
{
    // The OOL path is hit if the recursion depth has been exceeded.
    // Throw an InternalError for over-recursion.

    // LFunctionEnvironment can appear before LCheckOverRecursed, so we have
    // to save all live registers to avoid crashes if CheckOverRecursed triggers
    // a GC.
    saveLive(ool->lir());

    if (!callVM(CheckOverRecursedInfo, ool->lir()))
        return false;

    restoreLive(ool->lir());
    masm.jump(ool->rejoin());
    return true;
}

IonScriptCounts *
CodeGenerator::maybeCreateScriptCounts()
{
    // If scripts are being profiled, create a new IonScriptCounts and attach
    // it to the script. This must be done on the main thread.
    JSContext *cx = GetIonContext()->cx;
    if (!cx)
        return NULL;

    IonScriptCounts *counts = NULL;

    CompileInfo *outerInfo = &gen->info();
    RawScript script = outerInfo->script();

    if (cx->runtime->profilingScripts && !script->hasScriptCounts) {
        if (!script->initScriptCounts(cx))
            return NULL;
    }

    if (!script->hasScriptCounts)
        return NULL;

    counts = js_new<IonScriptCounts>();
    if (!counts || !counts->init(graph.numBlocks())) {
        js_delete(counts);
        return NULL;
    }

    script->addIonCounts(counts);

    for (size_t i = 0; i < graph.numBlocks(); i++) {
        MBasicBlock *block = graph.getBlock(i)->mir();

        // Find a PC offset in the outermost script to use. If this block is
        // from an inlined script, find a location in the outer script to
        // associate information about the inling with.
        MResumePoint *resume = block->entryResumePoint();
        while (resume->caller())
            resume = resume->caller();
        uint32 offset = resume->pc() - script->code;
        JS_ASSERT(offset < script->length);

        if (!counts->block(i).init(block->id(), offset, block->numSuccessors()))
            return NULL;
        for (size_t j = 0; j < block->numSuccessors(); j++)
            counts->block(i).setSuccessor(j, block->getSuccessor(j)->id());
    }

    return counts;
}

// Structure for managing the state tracked for a block by script counters.
struct ScriptCountBlockState
{
    IonBlockCounts &block;
    MacroAssembler &masm;

    Sprinter printer;

    uint32 instructionBytes;
    uint32 spillBytes;

    // Pointer to instructionBytes, spillBytes, or NULL, depending on the last
    // instruction processed.
    uint32 *last;
    uint32 lastLength;

  public:
    ScriptCountBlockState(IonBlockCounts *block, MacroAssembler *masm)
      : block(*block), masm(*masm),
        printer(GetIonContext()->cx),
        instructionBytes(0), spillBytes(0), last(NULL), lastLength(0)
    {
    }

    bool init()
    {
        if (!printer.init())
            return false;

        // Bump the hit count for the block at the start. This code is not
        // included in either the text for the block or the instruction byte
        // counts.
        masm.inc64(AbsoluteAddress(block.addressOfHitCount()));

        // Collect human readable assembly for the code generated in the block.
        masm.setPrinter(&printer);

        return true;
    }

    void visitInstruction(LInstruction *ins)
    {
        if (last)
            *last += masm.size() - lastLength;
        lastLength = masm.size();
        last = ins->isMoveGroup() ? &spillBytes : &instructionBytes;

        // Prefix stream of assembly instructions with their LIR instruction name.
        printer.printf("[%s]\n", ins->opName());
    }

    ~ScriptCountBlockState()
    {
        masm.setPrinter(NULL);

        if (last)
            *last += masm.size() - lastLength;

        block.setCode(printer.string());
        block.setInstructionBytes(instructionBytes);
        block.setSpillBytes(spillBytes);
    }
};

bool
CodeGenerator::generateBody()
{
    IonScriptCounts *counts = maybeCreateScriptCounts();

    for (size_t i = 0; i < graph.numBlocks(); i++) {
        current = graph.getBlock(i);

        LInstructionIterator iter = current->begin();

        // Separately visit the label at the start of every block, so that
        // count instrumentation is inserted after the block label is bound.
        if (!iter->accept(this))
            return false;
        iter++;

        mozilla::Maybe<ScriptCountBlockState> blockCounts;
        if (counts) {
            blockCounts.construct(&counts->block(i), &masm);
            if (!blockCounts.ref().init())
                return false;
        }

        for (; iter != current->end(); iter++) {
            IonSpew(IonSpew_Codegen, "instruction %s", iter->opName());

            if (counts)
                blockCounts.ref().visitInstruction(*iter);

            if (iter->safepoint() && pushedArgumentSlots_.length()) {
                if (!markArgumentSlots(iter->safepoint()))
                    return false;
            }

            if (!iter->accept(this))
                return false;
        }
        if (masm.oom())
            return false;
    }

    JS_ASSERT(pushedArgumentSlots_.empty());
    return true;
}

// Out-of-line object allocation for LNewArray.
class OutOfLineNewArray : public OutOfLineCodeBase<CodeGenerator>
{
    LNewArray *lir_;

  public:
    OutOfLineNewArray(LNewArray *lir)
      : lir_(lir)
    { }

    bool accept(CodeGenerator *codegen) {
        return codegen->visitOutOfLineNewArray(this);
    }

    LNewArray *lir() const {
        return lir_;
    }
};

typedef JSObject *(*NewInitArrayFn)(JSContext *, uint32, types::TypeObject *);
static const VMFunction NewInitArrayInfo =
    FunctionInfo<NewInitArrayFn>(NewInitArray);

bool
CodeGenerator::visitNewArrayCallVM(LNewArray *lir)
{
    Register objReg = ToRegister(lir->output());

    JS_ASSERT(!lir->isCall());
    saveLive(lir);

    JSObject *templateObject = lir->mir()->templateObject();
    types::TypeObject *type = templateObject->hasSingletonType() ? NULL : templateObject->type();

    pushArg(ImmGCPtr(type));
    pushArg(Imm32(lir->mir()->count()));

    if (!callVM(NewInitArrayInfo, lir))
        return false;

    if (ReturnReg != objReg)
        masm.movePtr(ReturnReg, objReg);

    restoreLive(lir);

    return true;
}

bool
CodeGenerator::visitNewSlots(LNewSlots *lir)
{
    Register temp1 = ToRegister(lir->temp1());
    Register temp2 = ToRegister(lir->temp2());
    Register temp3 = ToRegister(lir->temp3());
    Register output = ToRegister(lir->output());

    masm.mov(ImmWord(gen->compartment->rt), temp1);
    masm.mov(Imm32(lir->mir()->nslots()), temp2);

    masm.setupUnalignedABICall(2, temp3);
    masm.passABIArg(temp1);
    masm.passABIArg(temp2);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, NewSlots));

    masm.testPtr(output, output);
    if (!bailoutIf(Assembler::Zero, lir->snapshot()))
        return false;

    return true;
}

bool
CodeGenerator::visitNewArray(LNewArray *lir)
{
    Register objReg = ToRegister(lir->output());
    JSObject *templateObject = lir->mir()->templateObject();
    uint32 count = lir->mir()->count();

    JS_ASSERT(count < JSObject::NELEMENTS_LIMIT);

    size_t maxArraySlots =
        gc::GetGCKindSlots(gc::FINALIZE_OBJECT_LAST) - ObjectElements::VALUES_PER_HEADER;

    // Allocate space using the VMCall
    // when mir hints it needs to get allocated immediatly,
    // but only when data doesn't fit the available array slots.
    bool allocating = lir->mir()->isAllocating() && count > maxArraySlots;

    if (templateObject->hasSingletonType() || allocating)
        return visitNewArrayCallVM(lir);

    OutOfLineNewArray *ool = new OutOfLineNewArray(lir);
    if (!addOutOfLineCode(ool))
        return false;

    masm.newGCThing(objReg, templateObject, ool->entry());
    masm.initGCThing(objReg, templateObject);

    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitOutOfLineNewArray(OutOfLineNewArray *ool)
{
    if (!visitNewArrayCallVM(ool->lir()))
        return false;
    masm.jump(ool->rejoin());
    return true;
}

// Out-of-line object allocation for JSOP_NEWOBJECT.
class OutOfLineNewObject : public OutOfLineCodeBase<CodeGenerator>
{
    LNewObject *lir_;

  public:
    OutOfLineNewObject(LNewObject *lir)
      : lir_(lir)
    { }

    bool accept(CodeGenerator *codegen) {
        return codegen->visitOutOfLineNewObject(this);
    }

    LNewObject *lir() const {
        return lir_;
    }
};

typedef JSObject *(*NewInitObjectFn)(JSContext *, HandleObject);
static const VMFunction NewInitObjectInfo = FunctionInfo<NewInitObjectFn>(NewInitObject);

bool
CodeGenerator::visitNewObjectVMCall(LNewObject *lir)
{
    Register objReg = ToRegister(lir->output());

    JS_ASSERT(!lir->isCall());
    saveLive(lir);

    pushArg(ImmGCPtr(lir->mir()->templateObject()));
    if (!callVM(NewInitObjectInfo, lir))
        return false;

    if (ReturnReg != objReg)
        masm.movePtr(ReturnReg, objReg);

    restoreLive(lir);
    return true;
}

bool
CodeGenerator::visitNewObject(LNewObject *lir)
{
    Register objReg = ToRegister(lir->output());

    JSObject *templateObject = lir->mir()->templateObject();

    if (templateObject->hasSingletonType() || templateObject->hasDynamicSlots())
        return visitNewObjectVMCall(lir);

    OutOfLineNewObject *ool = new OutOfLineNewObject(lir);
    if (!addOutOfLineCode(ool))
        return false;

    masm.newGCThing(objReg, templateObject, ool->entry());
    masm.initGCThing(objReg, templateObject);

    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitOutOfLineNewObject(OutOfLineNewObject *ool)
{
    if (!visitNewObjectVMCall(ool->lir()))
        return false;
    masm.jump(ool->rejoin());
    return true;
}

typedef JSObject *(*NewCallObjectFn)(JSContext *, HandleShape,
                                     HandleTypeObject, HeapSlot *);
static const VMFunction NewCallObjectInfo =
    FunctionInfo<NewCallObjectFn>(NewCallObject);

bool
CodeGenerator::visitNewCallObject(LNewCallObject *lir)
{
    Register obj = ToRegister(lir->output());

    JSObject *templateObj = lir->mir()->templateObj();

    // If we have a template object, we can inline call object creation.
    OutOfLineCode *ool;
    if (lir->slots()->isRegister()) {
        ool = oolCallVM(NewCallObjectInfo, lir,
                        (ArgList(), ImmGCPtr(templateObj->lastProperty()),
                                    ImmGCPtr(templateObj->type()),
                                    ToRegister(lir->slots())),
                        StoreRegisterTo(obj));
    } else {
        ool = oolCallVM(NewCallObjectInfo, lir,
                        (ArgList(), ImmGCPtr(templateObj->lastProperty()),
                                    ImmGCPtr(templateObj->type()),
                                    ImmWord((void *)NULL)),
                        StoreRegisterTo(obj));
    }
    if (!ool)
        return false;

    masm.newGCThing(obj, templateObj, ool->entry());
    masm.initGCThing(obj, templateObj);

    if (lir->slots()->isRegister())
        masm.storePtr(ToRegister(lir->slots()), Address(obj, JSObject::offsetOfSlots()));
    masm.bind(ool->rejoin());
    return true;
}

typedef JSObject *(*NewStringObjectFn)(JSContext *, HandleString);
static const VMFunction NewStringObjectInfo = FunctionInfo<NewStringObjectFn>(NewStringObject);

bool
CodeGenerator::visitNewStringObject(LNewStringObject *lir)
{
    Register input = ToRegister(lir->input());
    Register output = ToRegister(lir->output());
    Register temp = ToRegister(lir->temp());

    StringObject *templateObj = lir->mir()->templateObj();

    OutOfLineCode *ool = oolCallVM(NewStringObjectInfo, lir, (ArgList(), input),
                                   StoreRegisterTo(output));
    if (!ool)
        return false;

    masm.newGCThing(output, templateObj, ool->entry());
    masm.initGCThing(output, templateObj);

    masm.loadStringLength(input, temp);

    masm.storeValue(JSVAL_TYPE_STRING, input, Address(output, StringObject::offsetOfPrimitiveValue()));
    masm.storeValue(JSVAL_TYPE_INT32, temp, Address(output, StringObject::offsetOfLength()));

    masm.bind(ool->rejoin());
    return true;
}

typedef bool(*InitPropFn)(JSContext *cx, HandleObject obj,
                          HandlePropertyName name, HandleValue value);
static const VMFunction InitPropInfo =
    FunctionInfo<InitPropFn>(InitProp);

bool
CodeGenerator::visitInitProp(LInitProp *lir)
{
    Register objReg = ToRegister(lir->getObject());

    pushArg(ToValue(lir, LInitProp::ValueIndex));
    pushArg(ImmGCPtr(lir->mir()->propertyName()));
    pushArg(objReg);

    return callVM(InitPropInfo, lir);
}

typedef JSObject *(*NewGCThingFn)(JSContext *cx, gc::AllocKind allocKind, size_t thingSize);
static const VMFunction NewGCThingInfo =
    FunctionInfo<NewGCThingFn>(js::ion::NewGCThing);

bool
CodeGenerator::visitCreateThis(LCreateThis *lir)
{
    JS_ASSERT(lir->mir()->hasTemplateObject());

    JSObject *templateObject = lir->mir()->getTemplateObject();
    gc::AllocKind allocKind = templateObject->getAllocKind();
    int thingSize = (int)gc::Arena::thingSize(allocKind);
    Register objReg = ToRegister(lir->output());

    OutOfLineCode *ool = oolCallVM(NewGCThingInfo, lir,
                                   (ArgList(), Imm32(allocKind), Imm32(thingSize)),
                                   StoreRegisterTo(objReg));
    if (!ool)
        return false;

    // Allocate. If the FreeList is empty, call to VM, which may GC.
    masm.newGCThing(objReg, templateObject, ool->entry());

    // Initialize based on the templateObject.
    masm.bind(ool->rejoin());
    masm.initGCThing(objReg, templateObject);

    return true;
}

typedef JSObject *(*CreateThisFn)(JSContext *cx, HandleObject callee, JSObject *proto);
static const VMFunction CreateThisInfo =
    FunctionInfo<CreateThisFn>(js_CreateThisForFunctionWithProto);

bool
CodeGenerator::visitCreateThisVM(LCreateThisVM *lir)
{
    const LAllocation *proto = lir->getPrototype();
    const LAllocation *callee = lir->getCallee();

    // Push arguments.
    if (proto->isConstant())
        pushArg(ImmGCPtr(&proto->toConstant()->toObject()));
    else
        pushArg(ToRegister(proto));

    if (callee->isConstant())
        pushArg(ImmGCPtr(&callee->toConstant()->toObject()));
    else
        pushArg(ToRegister(callee));

    if (!callVM(CreateThisInfo, lir))
        return false;
    return true;
}

bool
CodeGenerator::visitReturnFromCtor(LReturnFromCtor *lir)
{
    ValueOperand value = ToValue(lir, LReturnFromCtor::ValueIndex);
    Register obj = ToRegister(lir->getObject());
    Register output = ToRegister(lir->output());

    Label valueIsObject, end;

    masm.branchTestObject(Assembler::Equal, value, &valueIsObject);

    // Value is not an object. Return that other object.
    masm.movePtr(obj, output);
    masm.jump(&end);

    // Value is an object. Return unbox(Value).
    masm.bind(&valueIsObject);
    Register payload = masm.extractObject(value, output);
    if (payload != output)
        masm.movePtr(payload, output);

    masm.bind(&end);
    return true;
}

bool
CodeGenerator::visitArrayLength(LArrayLength *lir)
{
    Address length(ToRegister(lir->elements()), ObjectElements::offsetOfLength());
    masm.load32(length, ToRegister(lir->output()));
    return true;
}

bool
CodeGenerator::visitTypedArrayLength(LTypedArrayLength *lir)
{
    Register obj = ToRegister(lir->object());
    Register out = ToRegister(lir->output());
    masm.unboxInt32(Address(obj, TypedArray::lengthOffset()), out);
    return true;
}

bool
CodeGenerator::visitTypedArrayElements(LTypedArrayElements *lir)
{
    Register obj = ToRegister(lir->object());
    Register out = ToRegister(lir->output());
    masm.loadPtr(Address(obj, TypedArray::dataOffset()), out);
    return true;
}

bool
CodeGenerator::visitStringLength(LStringLength *lir)
{
    Register input = ToRegister(lir->string());
    Register output = ToRegister(lir->output());

    masm.loadStringLength(input, output);
    return true;
}

bool
CodeGenerator::visitMinMaxI(LMinMaxI *ins)
{
    Register first = ToRegister(ins->first());
    Register output = ToRegister(ins->output());

    JS_ASSERT(first == output);

    if (ins->second()->isConstant())
        masm.cmp32(first, Imm32(ToInt32(ins->second())));
    else
        masm.cmp32(first, ToRegister(ins->second()));

    Label done;
    if (ins->mir()->isMax())
        masm.j(Assembler::GreaterThan, &done);
    else
        masm.j(Assembler::LessThan, &done);

    if (ins->second()->isConstant())
        masm.move32(Imm32(ToInt32(ins->second())), output);
    else
        masm.mov(ToRegister(ins->second()), output);


    masm.bind(&done);
    return true;
}

bool
CodeGenerator::visitAbsI(LAbsI *ins)
{
    Register input = ToRegister(ins->input());
    Label positive;

    JS_ASSERT(input == ToRegister(ins->output()));
    masm.test32(input, input);
    masm.j(Assembler::GreaterThanOrEqual, &positive);
    masm.neg32(input);
    if (ins->snapshot() && !bailoutIf(Assembler::Overflow, ins->snapshot()))
        return false;
    masm.bind(&positive);

    return true;
}

bool
CodeGenerator::visitPowI(LPowI *ins)
{
    FloatRegister value = ToFloatRegister(ins->value());
    Register power = ToRegister(ins->power());
    Register temp = ToRegister(ins->temp());

    JS_ASSERT(power != temp);

    // In all implementations, setupUnalignedABICall() relinquishes use of
    // its scratch register. We can therefore save an input register by
    // reusing the scratch register to pass constants to callWithABI.
    masm.setupUnalignedABICall(2, temp);
    masm.passABIArg(value);
    masm.passABIArg(power);

    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, js::powi), MacroAssembler::DOUBLE);
    JS_ASSERT(ToFloatRegister(ins->output()) == ReturnFloatReg);

    return true;
}

bool
CodeGenerator::visitPowD(LPowD *ins)
{
    FloatRegister value = ToFloatRegister(ins->value());
    FloatRegister power = ToFloatRegister(ins->power());
    Register temp = ToRegister(ins->temp());

    masm.setupUnalignedABICall(2, temp);
    masm.passABIArg(value);
    masm.passABIArg(power);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, ecmaPow), MacroAssembler::DOUBLE);

    JS_ASSERT(ToFloatRegister(ins->output()) == ReturnFloatReg);
    return true;
}

bool
CodeGenerator::visitRandom(LRandom *ins)
{
    Register temp = ToRegister(ins->temp());
    Register temp2 = ToRegister(ins->temp2());

    masm.loadJSContext(temp);

    masm.setupUnalignedABICall(1, temp2);
    masm.passABIArg(temp);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, math_random_no_outparam), MacroAssembler::DOUBLE);

    JS_ASSERT(ToFloatRegister(ins->output()) == ReturnFloatReg);
    return true;
}

bool
CodeGenerator::visitMathFunctionD(LMathFunctionD *ins)
{
    Register temp = ToRegister(ins->temp());
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(ToFloatRegister(ins->output()) == ReturnFloatReg);

    MathCache *mathCache = ins->mir()->cache();

    masm.setupUnalignedABICall(2, temp);
    masm.movePtr(ImmWord(mathCache), temp);
    masm.passABIArg(temp);
    masm.passABIArg(input);

    void *funptr = NULL;
    switch (ins->mir()->function()) {
      case MMathFunction::Log:
        funptr = JS_FUNC_TO_DATA_PTR(void *, js::math_log_impl);
        break;
      case MMathFunction::Sin:
        funptr = JS_FUNC_TO_DATA_PTR(void *, js::math_sin_impl);
        break;
      case MMathFunction::Cos:
        funptr = JS_FUNC_TO_DATA_PTR(void *, js::math_cos_impl);
        break;
      case MMathFunction::Tan:
        funptr = JS_FUNC_TO_DATA_PTR(void *, js::math_tan_impl);
        break;
      default:
        JS_NOT_REACHED("Unknown math function");
    }

    masm.callWithABI(funptr, MacroAssembler::DOUBLE);
    return true;
}

bool
CodeGenerator::visitModD(LModD *ins)
{
    FloatRegister lhs = ToFloatRegister(ins->lhs());
    FloatRegister rhs = ToFloatRegister(ins->rhs());
    Register temp = ToRegister(ins->temp());

    JS_ASSERT(ToFloatRegister(ins->output()) == ReturnFloatReg);

    masm.setupUnalignedABICall(2, temp);
    masm.passABIArg(lhs);
    masm.passABIArg(rhs);

    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, NumberMod), MacroAssembler::DOUBLE);
    return true;
}

typedef bool (*BinaryFn)(JSContext *, HandleScript, jsbytecode *,
                         HandleValue, HandleValue, Value *);

static const VMFunction AddInfo = FunctionInfo<BinaryFn>(js::AddValues);
static const VMFunction SubInfo = FunctionInfo<BinaryFn>(js::SubValues);
static const VMFunction MulInfo = FunctionInfo<BinaryFn>(js::MulValues);
static const VMFunction DivInfo = FunctionInfo<BinaryFn>(js::DivValues);
static const VMFunction ModInfo = FunctionInfo<BinaryFn>(js::ModValues);
static const VMFunction UrshInfo = FunctionInfo<BinaryFn>(js::UrshValues);

bool
CodeGenerator::visitBinaryV(LBinaryV *lir)
{
    pushArg(ToValue(lir, LBinaryV::RhsInput));
    pushArg(ToValue(lir, LBinaryV::LhsInput));
    pushArg(ImmWord(lir->mirRaw()->toInstruction()->resumePoint()->pc()));
    pushArg(ImmGCPtr(current->mir()->info().script()));

    switch (lir->jsop()) {
      case JSOP_ADD:
        return callVM(AddInfo, lir);

      case JSOP_SUB:
        return callVM(SubInfo, lir);

      case JSOP_MUL:
        return callVM(MulInfo, lir);

      case JSOP_DIV:
        return callVM(DivInfo, lir);

      case JSOP_MOD:
        return callVM(ModInfo, lir);

      case JSOP_URSH:
        return callVM(UrshInfo, lir);

      default:
        JS_NOT_REACHED("Unexpected binary op");
        return false;
    }
}

typedef bool (*StringCompareFn)(JSContext *, HandleString, HandleString, JSBool *);
static const VMFunction stringsEqualInfo =
    FunctionInfo<StringCompareFn>(ion::StringsEqual<true>);
static const VMFunction stringsNotEqualInfo =
    FunctionInfo<StringCompareFn>(ion::StringsEqual<false>);

bool
CodeGenerator::visitCompareS(LCompareS *lir)
{
    JSOp op = lir->mir()->jsop();
    Register left = ToRegister(lir->left());
    Register right = ToRegister(lir->right());
    Register output = ToRegister(lir->output());
    Register temp = ToRegister(lir->temp());

    OutOfLineCode *ool = NULL;
    if (op == JSOP_EQ || op == JSOP_STRICTEQ) {
        ool = oolCallVM(stringsEqualInfo, lir, (ArgList(), left, right),  StoreRegisterTo(output));
    } else {
        JS_ASSERT(op == JSOP_NE || op == JSOP_STRICTNE);
        ool = oolCallVM(stringsNotEqualInfo, lir, (ArgList(), left, right), StoreRegisterTo(output));
    }

    if (!ool)
        return false;

    Label notPointerEqual;
    // Fast path for identical strings
    masm.branchPtr(Assembler::NotEqual, left, right, &notPointerEqual);
    masm.move32(Imm32(op == JSOP_EQ || op == JSOP_STRICTEQ), output);
    masm.jump(ool->rejoin());

    masm.bind(&notPointerEqual);
    masm.loadPtr(Address(left, JSString::offsetOfLengthAndFlags()), output);
    masm.loadPtr(Address(right, JSString::offsetOfLengthAndFlags()), temp);

    Label notAtom;
    // We can optimize the equality operation to a pointer compare for
    // two atoms.
    Imm32 atomBit(JSString::ATOM_BIT);
    masm.branchTest32(Assembler::Zero, output, atomBit, &notAtom);
    masm.branchTest32(Assembler::Zero, temp, atomBit, &notAtom);

    masm.cmpPtr(left, right);
    emitSet(JSOpToCondition(op), output);
    masm.jump(ool->rejoin());

    masm.bind(&notAtom);
    // Strings of different length can never be equal.
    masm.rshiftPtr(Imm32(JSString::LENGTH_SHIFT), output);
    masm.rshiftPtr(Imm32(JSString::LENGTH_SHIFT), temp);
    masm.branchPtr(Assembler::Equal, output, temp, ool->entry());
    masm.move32(Imm32(op == JSOP_NE || op == JSOP_STRICTNE), output);

    masm.bind(ool->rejoin());
    return true;
}

typedef bool (*CompareFn)(JSContext *, HandleValue, HandleValue, JSBool *);
static const VMFunction EqInfo = FunctionInfo<CompareFn>(ion::LooselyEqual<true>);
static const VMFunction NeInfo = FunctionInfo<CompareFn>(ion::LooselyEqual<false>);
static const VMFunction StrictEqInfo = FunctionInfo<CompareFn>(ion::StrictlyEqual<true>);
static const VMFunction StrictNeInfo = FunctionInfo<CompareFn>(ion::StrictlyEqual<false>);
static const VMFunction LtInfo = FunctionInfo<CompareFn>(ion::LessThan);
static const VMFunction LeInfo = FunctionInfo<CompareFn>(ion::LessThanOrEqual);
static const VMFunction GtInfo = FunctionInfo<CompareFn>(ion::GreaterThan);
static const VMFunction GeInfo = FunctionInfo<CompareFn>(ion::GreaterThanOrEqual);

bool
CodeGenerator::visitCompareV(LCompareV *lir)
{
    pushArg(ToValue(lir, LBinaryV::RhsInput));
    pushArg(ToValue(lir, LBinaryV::LhsInput));

    switch (lir->mir()->jsop()) {
      case JSOP_EQ:
        return callVM(EqInfo, lir);

      case JSOP_NE:
        return callVM(NeInfo, lir);

      case JSOP_STRICTEQ:
        return callVM(StrictEqInfo, lir);

      case JSOP_STRICTNE:
        return callVM(StrictNeInfo, lir);

      case JSOP_LT:
        return callVM(LtInfo, lir);

      case JSOP_LE:
        return callVM(LeInfo, lir);

      case JSOP_GT:
        return callVM(GtInfo, lir);

      case JSOP_GE:
        return callVM(GeInfo, lir);

      default:
        JS_NOT_REACHED("Unexpected compare op");
        return false;
    }
}

bool
CodeGenerator::visitIsNullOrUndefined(LIsNullOrUndefined *lir)
{
    JSOp op = lir->mir()->jsop();
    MIRType specialization = lir->mir()->specialization();
    JS_ASSERT(IsNullOrUndefined(specialization));

    const ValueOperand value = ToValue(lir, LIsNullOrUndefined::Value);
    Register output = ToRegister(lir->output());

    if (op == JSOP_EQ || op == JSOP_NE) {
        Register tag = masm.splitTagForTest(value);

        Label nullOrUndefined, done;
        masm.branchTestNull(Assembler::Equal, tag, &nullOrUndefined);
        masm.branchTestUndefined(Assembler::Equal, tag, &nullOrUndefined);

        masm.move32(Imm32(op == JSOP_NE), output);
        masm.jump(&done);

        masm.bind(&nullOrUndefined);
        masm.move32(Imm32(op == JSOP_EQ), output);
        masm.bind(&done);
        return true;
    }

    JS_ASSERT(op == JSOP_STRICTEQ || op == JSOP_STRICTNE);

    Assembler::Condition cond = JSOpToCondition(op);
    if (specialization == MIRType_Null)
        cond = masm.testNull(cond, value);
    else
        cond = masm.testUndefined(cond, value);

    emitSet(cond, output);
    return true;
}

bool
CodeGenerator::visitIsNullOrUndefinedAndBranch(LIsNullOrUndefinedAndBranch *lir)
{
    JSOp op = lir->mir()->jsop();
    MIRType specialization = lir->mir()->specialization();
    JS_ASSERT(IsNullOrUndefined(specialization));

    const ValueOperand value = ToValue(lir, LIsNullOrUndefinedAndBranch::Value);

    if (op == JSOP_EQ || op == JSOP_NE) {
        MBasicBlock *ifTrue;
        MBasicBlock *ifFalse;

        if (op == JSOP_EQ) {
            ifTrue = lir->ifTrue();
            ifFalse = lir->ifFalse();
        } else {
            // Swap branches.
            ifTrue = lir->ifFalse();
            ifFalse = lir->ifTrue();
            op = JSOP_EQ;
        }

        Register tag = masm.splitTagForTest(value);
        masm.branchTestNull(Assembler::Equal, tag, ifTrue->lir()->label());

        Assembler::Condition cond = masm.testUndefined(Assembler::Equal, tag);
        emitBranch(cond, ifTrue, ifFalse);
        return true;
    }

    JS_ASSERT(op == JSOP_STRICTEQ || op == JSOP_STRICTNE);

    Assembler::Condition cond = JSOpToCondition(op);
    if (specialization == MIRType_Null)
        cond = masm.testNull(cond, value);
    else
        cond = masm.testUndefined(cond, value);

    emitBranch(cond, lir->ifTrue(), lir->ifFalse());
    return true;
}

typedef JSString *(*ConcatStringsFn)(JSContext *, HandleString, HandleString);
static const VMFunction ConcatStringsInfo = FunctionInfo<ConcatStringsFn>(js_ConcatStrings);

bool
CodeGenerator::visitConcat(LConcat *lir)
{
    pushArg(ToRegister(lir->rhs()));
    pushArg(ToRegister(lir->lhs()));
    if (!callVM(ConcatStringsInfo, lir))
        return false;
    return true;
}

typedef bool (*EnsureLinearFn)(JSContext *, JSString *);
static const VMFunction EnsureLinearInfo = FunctionInfo<EnsureLinearFn>(JSString::ensureLinear);

bool
CodeGenerator::visitCharCodeAt(LCharCodeAt *lir)
{
    Register str = ToRegister(lir->str());
    Register index = ToRegister(lir->index());
    Register output = ToRegister(lir->output());

    OutOfLineCode *ool = oolCallVM(EnsureLinearInfo, lir, (ArgList(), str), StoreNothing());
    if (!ool)
        return false;

    Address lengthAndFlagsAddr(str, JSString::offsetOfLengthAndFlags());
    masm.loadPtr(lengthAndFlagsAddr, output);

    masm.branchTest32(Assembler::Zero, output, Imm32(JSString::FLAGS_MASK), ool->entry());
    masm.bind(ool->rejoin());

    // getChars
    Address charsAddr(str, JSString::offsetOfChars());
    masm.loadPtr(charsAddr, output);
    masm.load16ZeroExtend(BaseIndex(output, index, TimesTwo, 0), output);

    return true;
}

typedef JSFlatString *(*StringFromCharCodeFn)(JSContext *, int32_t);
static const VMFunction StringFromCharCodeInfo = FunctionInfo<StringFromCharCodeFn>(ion::StringFromCharCode);

bool
CodeGenerator::visitFromCharCode(LFromCharCode *lir)
{
    Register code = ToRegister(lir->code());
    Register output = ToRegister(lir->output());

    OutOfLineCode *ool = oolCallVM(StringFromCharCodeInfo, lir, (ArgList(), code), StoreRegisterTo(output));
    if (!ool)
        return false;

    // OOL path if code >= UNIT_STATIC_LIMIT.
    masm.branch32(Assembler::AboveOrEqual, code, Imm32(StaticStrings::UNIT_STATIC_LIMIT),
                  ool->entry());

    masm.movePtr(ImmWord(&gen->compartment->rt->staticStrings.unitStaticTable), output);
    masm.loadPtr(BaseIndex(output, code, ScalePointer), output);

    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitInitializedLength(LInitializedLength *lir)
{
    Address initLength(ToRegister(lir->elements()), ObjectElements::offsetOfInitializedLength());
    masm.load32(initLength, ToRegister(lir->output()));
    return true;
}

bool
CodeGenerator::visitSetInitializedLength(LSetInitializedLength *lir)
{
    Address initLength(ToRegister(lir->elements()), ObjectElements::offsetOfInitializedLength());
    Int32Key index = ToInt32Key(lir->index());

    masm.bumpKey(&index, 1);
    masm.storeKey(index, initLength);
    // Restore register value if it is used/captured after.
    masm.bumpKey(&index, -1);
    return true;
}

bool
CodeGenerator::visitNotV(LNotV *lir)
{
    Label setFalse;
    Label join;
    masm.branchTestValueTruthy(ToValue(lir, LNotV::Input), &setFalse, ToFloatRegister(lir->tempFloat()));

    // fallthrough to set true
    masm.move32(Imm32(1), ToRegister(lir->getDef(0)));
    masm.jump(&join);

    // true case rediercts to setFalse
    masm.bind(&setFalse);
    masm.move32(Imm32(0), ToRegister(lir->getDef(0)));

    // both branches meet here.
    masm.bind(&join);
    return true;
}

bool
CodeGenerator::visitBoundsCheck(LBoundsCheck *lir)
{
    if (lir->index()->isConstant()) {
        // Use uint32_t so that the comparison is unsigned.
        uint32_t index = ToInt32(lir->index());
        if (lir->length()->isConstant()) {
            uint32_t length = ToInt32(lir->length());
            if (index < length)
                return true;
            return bailout(lir->snapshot());
        }
        masm.cmp32(ToOperand(lir->length()), Imm32(index));
        return bailoutIf(Assembler::BelowOrEqual, lir->snapshot());
    }
    if (lir->length()->isConstant()) {
        masm.cmp32(ToRegister(lir->index()), Imm32(ToInt32(lir->length())));
        return bailoutIf(Assembler::AboveOrEqual, lir->snapshot());
    }
    masm.cmp32(ToOperand(lir->length()), ToRegister(lir->index()));
    return bailoutIf(Assembler::BelowOrEqual, lir->snapshot());
}

bool
CodeGenerator::visitBoundsCheckRange(LBoundsCheckRange *lir)
{
    int32 min = lir->mir()->minimum();
    int32 max = lir->mir()->maximum();
    JS_ASSERT(max >= min);

    Register temp = ToRegister(lir->getTemp(0));
    if (lir->index()->isConstant()) {
        int32 nmin, nmax;
        int32 index = ToInt32(lir->index());
        if (SafeAdd(index, min, &nmin) && SafeAdd(index, max, &nmax) && nmin >= 0) {
            masm.cmp32(ToOperand(lir->length()), Imm32(nmax));
            return bailoutIf(Assembler::BelowOrEqual, lir->snapshot());
        }
        masm.mov(Imm32(index), temp);
    } else {
        masm.mov(ToRegister(lir->index()), temp);
    }

    // If the minimum and maximum differ then do an underflow check first.
    // If the two are the same then doing an unsigned comparison on the
    // length will also catch a negative index.
    if (min != max) {
        if (min != 0) {
            masm.add32(Imm32(min), temp);
            if (!bailoutIf(Assembler::Overflow, lir->snapshot()))
                return false;
            int32 diff;
            if (SafeSub(max, min, &diff))
                max = diff;
            else
                masm.sub32(Imm32(min), temp);
        }

        masm.cmp32(temp, Imm32(0));
        if (!bailoutIf(Assembler::LessThan, lir->snapshot()))
            return false;
    }

    // Compute the maximum possible index. No overflow check is needed when
    // max > 0. We can only wraparound to a negative number, which will test as
    // larger than all nonnegative numbers in the unsigned comparison, and the
    // length is required to be nonnegative (else testing a negative length
    // would succeed on any nonnegative index).
    if (max != 0) {
        masm.add32(Imm32(max), temp);
        if (max < 0 && !bailoutIf(Assembler::Overflow, lir->snapshot()))
            return false;
    }

    masm.cmp32(ToOperand(lir->length()), temp);
    return bailoutIf(Assembler::BelowOrEqual, lir->snapshot());
}

bool
CodeGenerator::visitBoundsCheckLower(LBoundsCheckLower *lir)
{
    int32 min = lir->mir()->minimum();
    masm.cmp32(ToRegister(lir->index()), Imm32(min));
    return bailoutIf(Assembler::LessThan, lir->snapshot());
}

class OutOfLineStoreElementHole : public OutOfLineCodeBase<CodeGenerator>
{
    LInstruction *ins_;
    Label rejoinStore_;

  public:
    OutOfLineStoreElementHole(LInstruction *ins)
      : ins_(ins)
    {
        JS_ASSERT(ins->isStoreElementHoleV() || ins->isStoreElementHoleT());
    }

    bool accept(CodeGenerator *codegen) {
        return codegen->visitOutOfLineStoreElementHole(this);
    }
    LInstruction *ins() const {
        return ins_;
    }
    Label *rejoinStore() {
        return &rejoinStore_;
    }
};

bool
CodeGenerator::visitStoreElementT(LStoreElementT *store)
{
    if (store->mir()->needsBarrier())
       emitPreBarrier(ToRegister(store->elements()), store->index(), store->mir()->elementType());

    storeElementTyped(store->value(), store->mir()->value()->type(), store->mir()->elementType(),
                      ToRegister(store->elements()), store->index());
    return true;
}

bool
CodeGenerator::visitStoreElementV(LStoreElementV *lir)
{
    const ValueOperand value = ToValue(lir, LStoreElementV::Value);
    Register elements = ToRegister(lir->elements());

    if (lir->mir()->needsBarrier())
        emitPreBarrier(elements, lir->index(), MIRType_Value);

    if (lir->index()->isConstant())
        masm.storeValue(value, Address(elements, ToInt32(lir->index()) * sizeof(js::Value)));
    else
        masm.storeValue(value, BaseIndex(elements, ToRegister(lir->index()), TimesEight));
    return true;
}

bool
CodeGenerator::visitStoreElementHoleT(LStoreElementHoleT *lir)
{
    OutOfLineStoreElementHole *ool = new OutOfLineStoreElementHole(lir);
    if (!addOutOfLineCode(ool))
        return false;

    Register elements = ToRegister(lir->elements());
    const LAllocation *index = lir->index();

    // OOL path if index >= initializedLength.
    Address initLength(elements, ObjectElements::offsetOfInitializedLength());
    masm.branchKey(Assembler::BelowOrEqual, initLength, ToInt32Key(index), ool->entry());

    if (lir->mir()->needsBarrier())
        emitPreBarrier(elements, index, lir->mir()->elementType());

    masm.bind(ool->rejoinStore());
    storeElementTyped(lir->value(), lir->mir()->value()->type(), lir->mir()->elementType(),
                      elements, index);

    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitStoreElementHoleV(LStoreElementHoleV *lir)
{
    OutOfLineStoreElementHole *ool = new OutOfLineStoreElementHole(lir);
    if (!addOutOfLineCode(ool))
        return false;

    Register elements = ToRegister(lir->elements());
    const LAllocation *index = lir->index();
    const ValueOperand value = ToValue(lir, LStoreElementHoleV::Value);

    // OOL path if index >= initializedLength.
    Address initLength(elements, ObjectElements::offsetOfInitializedLength());
    masm.branchKey(Assembler::BelowOrEqual, initLength, ToInt32Key(index), ool->entry());

    if (lir->mir()->needsBarrier())
        emitPreBarrier(elements, index, lir->mir()->elementType());

    masm.bind(ool->rejoinStore());
    if (lir->index()->isConstant())
        masm.storeValue(value, Address(elements, ToInt32(lir->index()) * sizeof(js::Value)));
    else
        masm.storeValue(value, BaseIndex(elements, ToRegister(lir->index()), TimesEight));

    masm.bind(ool->rejoin());
    return true;
}

typedef bool (*SetObjectElementFn)(JSContext *, HandleObject,
                                   HandleValue, HandleValue, JSBool strict);
static const VMFunction SetObjectElementInfo =
    FunctionInfo<SetObjectElementFn>(SetObjectElement);

bool
CodeGenerator::visitOutOfLineStoreElementHole(OutOfLineStoreElementHole *ool)
{
    Register object, elements;
    LInstruction *ins = ool->ins();
    const LAllocation *index;
    MIRType valueType;
    ConstantOrRegister value;

    if (ins->isStoreElementHoleV()) {
        LStoreElementHoleV *store = ins->toStoreElementHoleV();
        object = ToRegister(store->object());
        elements = ToRegister(store->elements());
        index = store->index();
        valueType = store->mir()->value()->type();
        value = TypedOrValueRegister(ToValue(store, LStoreElementHoleV::Value));
    } else {
        LStoreElementHoleT *store = ins->toStoreElementHoleT();
        object = ToRegister(store->object());
        elements = ToRegister(store->elements());
        index = store->index();
        valueType = store->mir()->value()->type();
        if (store->value()->isConstant())
            value = ConstantOrRegister(*store->value()->toConstant());
        else
            value = TypedOrValueRegister(valueType, ToAnyRegister(store->value()));
    }

    // If index == initializedLength, try to bump the initialized length inline.
    // If index > initializedLength, call a stub. Note that this relies on the
    // condition flags sticking from the incoming branch.
    Label callStub;
    masm.j(Assembler::NotEqual, &callStub);

    Int32Key key = ToInt32Key(index);

    // Check array capacity.
    masm.branchKey(Assembler::BelowOrEqual, Address(elements, ObjectElements::offsetOfCapacity()),
                   key, &callStub);

    // Update initialized length. The capacity guard above ensures this won't overflow,
    // due to NELEMENTS_LIMIT.
    masm.bumpKey(&key, 1);
    masm.storeKey(key, Address(elements, ObjectElements::offsetOfInitializedLength()));

    // Update length if length < initializedLength.
    Label dontUpdate;
    masm.branchKey(Assembler::AboveOrEqual, Address(elements, ObjectElements::offsetOfLength()),
                   key, &dontUpdate);
    masm.storeKey(key, Address(elements, ObjectElements::offsetOfLength()));
    masm.bind(&dontUpdate);

    masm.bumpKey(&key, -1);

    if (ins->isStoreElementHoleT() && valueType != MIRType_Double) {
        // The inline path for StoreElementHoleT does not always store the type tag,
        // so we do the store on the OOL path. We use MIRType_None for the element type
        // so that storeElementTyped will always store the type tag.
        storeElementTyped(ins->toStoreElementHoleT()->value(), valueType, MIRType_None, elements,
                          index);
        masm.jump(ool->rejoin());
    } else {
        // Jump to the inline path where we will store the value.
        masm.jump(ool->rejoinStore());
    }

    masm.bind(&callStub);
    saveLive(ins);

    pushArg(Imm32(current->mir()->strictModeCode()));
    pushArg(value);
    if (index->isConstant())
        pushArg(*index->toConstant());
    else
        pushArg(TypedOrValueRegister(MIRType_Int32, ToAnyRegister(index)));
    pushArg(object);
    if (!callVM(SetObjectElementInfo, ins))
        return false;

    restoreLive(ins);
    masm.jump(ool->rejoin());
    return true;
}

typedef bool (*ArrayPopShiftFn)(JSContext *, HandleObject, MutableHandleValue);
static const VMFunction ArrayPopDenseInfo = FunctionInfo<ArrayPopShiftFn>(ion::ArrayPopDense);
static const VMFunction ArrayShiftDenseInfo = FunctionInfo<ArrayPopShiftFn>(ion::ArrayShiftDense);

bool
CodeGenerator::emitArrayPopShift(LInstruction *lir, const MArrayPopShift *mir, Register obj,
                                 Register elementsTemp, Register lengthTemp, TypedOrValueRegister out)
{
    OutOfLineCode *ool;

    if (mir->mode() == MArrayPopShift::Pop) {
        ool = oolCallVM(ArrayPopDenseInfo, lir, (ArgList(), obj), StoreValueTo(out));
        if (!ool)
            return false;
    } else {
        JS_ASSERT(mir->mode() == MArrayPopShift::Shift);
        ool = oolCallVM(ArrayShiftDenseInfo, lir, (ArgList(), obj), StoreValueTo(out));
        if (!ool)
            return false;
    }

    // VM call if a write barrier is necessary.
    masm.branchTestNeedsBarrier(Assembler::NonZero, lengthTemp, ool->entry());

    // Load elements and length.
    masm.loadPtr(Address(obj, JSObject::offsetOfElements()), elementsTemp);
    masm.load32(Address(elementsTemp, ObjectElements::offsetOfLength()), lengthTemp);

    // VM call if length != initializedLength.
    Int32Key key = Int32Key(lengthTemp);
    Address initLength(elementsTemp, ObjectElements::offsetOfInitializedLength());
    masm.branchKey(Assembler::NotEqual, initLength, key, ool->entry());

    // Test for length != 0. On zero length either take a VM call or generate
    // an undefined value, depending on whether the call is known to produce
    // undefined.
    Label done;
    if (mir->maybeUndefined()) {
        Label notEmpty;
        masm.branchTest32(Assembler::NonZero, lengthTemp, lengthTemp, &notEmpty);
        masm.moveValue(UndefinedValue(), out.valueReg());
        masm.jump(&done);
        masm.bind(&notEmpty);
    } else {
        masm.branchTest32(Assembler::Zero, lengthTemp, lengthTemp, ool->entry());
    }

    masm.bumpKey(&key, -1);

    if (mir->mode() == MArrayPopShift::Pop) {
        masm.loadElementTypedOrValue(BaseIndex(elementsTemp, lengthTemp, TimesEight), out,
                                     mir->needsHoleCheck(), ool->entry());
    } else {
        JS_ASSERT(mir->mode() == MArrayPopShift::Shift);
        masm.loadElementTypedOrValue(Address(elementsTemp, 0), out, mir->needsHoleCheck(),
                                     ool->entry());
    }

    masm.store32(lengthTemp, Address(elementsTemp, ObjectElements::offsetOfLength()));
    masm.store32(lengthTemp, Address(elementsTemp, ObjectElements::offsetOfInitializedLength()));

    if (mir->mode() == MArrayPopShift::Shift) {
        // Don't save the temp registers.
        RegisterSet temps;
        temps.add(elementsTemp);
        temps.add(lengthTemp);

        saveVolatile(temps);
        masm.setupUnalignedABICall(1, lengthTemp);
        masm.passABIArg(obj);
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, js::ArrayShiftMoveElements));
        restoreVolatile(temps);
    }

    masm.bind(&done);
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitArrayPopShiftV(LArrayPopShiftV *lir)
{
    Register obj = ToRegister(lir->object());
    Register elements = ToRegister(lir->temp0());
    Register length = ToRegister(lir->temp1());
    TypedOrValueRegister out(ToOutValue(lir));
    return emitArrayPopShift(lir, lir->mir(), obj, elements, length, out);
}

bool
CodeGenerator::visitArrayPopShiftT(LArrayPopShiftT *lir)
{
    Register obj = ToRegister(lir->object());
    Register elements = ToRegister(lir->temp0());
    Register length = ToRegister(lir->temp1());
    TypedOrValueRegister out(lir->mir()->type(), ToAnyRegister(lir->output()));
    return emitArrayPopShift(lir, lir->mir(), obj, elements, length, out);
}

typedef bool (*ArrayPushDenseFn)(JSContext *, HandleObject, HandleValue, uint32_t *);
static const VMFunction ArrayPushDenseInfo =
    FunctionInfo<ArrayPushDenseFn>(ion::ArrayPushDense);

bool
CodeGenerator::emitArrayPush(LInstruction *lir, const MArrayPush *mir, Register obj,
                             ConstantOrRegister value, Register elementsTemp, Register length)
{
    OutOfLineCode *ool = oolCallVM(ArrayPushDenseInfo, lir, (ArgList(), obj, value), StoreRegisterTo(length));
    if (!ool)
        return false;

    // Load elements and length.
    masm.loadPtr(Address(obj, JSObject::offsetOfElements()), elementsTemp);
    masm.load32(Address(elementsTemp, ObjectElements::offsetOfLength()), length);

    Int32Key key = Int32Key(length);
    Address initLength(elementsTemp, ObjectElements::offsetOfInitializedLength());
    Address capacity(elementsTemp, ObjectElements::offsetOfCapacity());

    // Guard length == initializedLength.
    masm.branchKey(Assembler::NotEqual, initLength, key, ool->entry());

    // Guard length < capacity.
    masm.branchKey(Assembler::BelowOrEqual, capacity, key, ool->entry());

    masm.storeConstantOrRegister(value, BaseIndex(elementsTemp, length, TimesEight));

    masm.bumpKey(&key, 1);
    masm.store32(length, Address(elementsTemp, ObjectElements::offsetOfLength()));
    masm.store32(length, Address(elementsTemp, ObjectElements::offsetOfInitializedLength()));

    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitArrayPushV(LArrayPushV *lir)
{
    Register obj = ToRegister(lir->object());
    Register elementsTemp = ToRegister(lir->temp());
    Register length = ToRegister(lir->output());
    ConstantOrRegister value = TypedOrValueRegister(ToValue(lir, LArrayPushV::Value));
    return emitArrayPush(lir, lir->mir(), obj, value, elementsTemp, length);
}

bool
CodeGenerator::visitArrayPushT(LArrayPushT *lir)
{
    Register obj = ToRegister(lir->object());
    Register elementsTemp = ToRegister(lir->temp());
    Register length = ToRegister(lir->output());
    ConstantOrRegister value;
    if (lir->value()->isConstant())
        value = ConstantOrRegister(*lir->value()->toConstant());
    else
        value = TypedOrValueRegister(lir->mir()->value()->type(), ToAnyRegister(lir->value()));
    return emitArrayPush(lir, lir->mir(), obj, value, elementsTemp, length);
}

typedef JSObject *(*ArrayConcatDenseFn)(JSContext *, HandleObject, HandleObject, HandleObject);
static const VMFunction ArrayConcatDenseInfo = FunctionInfo<ArrayConcatDenseFn>(ArrayConcatDense);

bool
CodeGenerator::visitArrayConcat(LArrayConcat *lir)
{
    Register lhs = ToRegister(lir->lhs());
    Register rhs = ToRegister(lir->rhs());
    Register temp1 = ToRegister(lir->temp1());
    Register temp2 = ToRegister(lir->temp2());

    // If 'length == initializedLength' for both arrays we try to allocate an object
    // inline and pass it to the stub. Else, we just pass NULL and the stub falls back
    // to a slow path.
    Label fail, call;
    masm.loadPtr(Address(lhs, JSObject::offsetOfElements()), temp1);
    masm.load32(Address(temp1, ObjectElements::offsetOfInitializedLength()), temp2);
    masm.branch32(Assembler::NotEqual, Address(temp1, ObjectElements::offsetOfLength()), temp2, &fail);

    masm.loadPtr(Address(rhs, JSObject::offsetOfElements()), temp1);
    masm.load32(Address(temp1, ObjectElements::offsetOfInitializedLength()), temp2);
    masm.branch32(Assembler::NotEqual, Address(temp1, ObjectElements::offsetOfLength()), temp2, &fail);

    // Try to allocate an object.
    JSObject *templateObj = lir->mir()->templateObj();
    masm.newGCThing(temp1, templateObj, &fail);
    masm.initGCThing(temp1, templateObj);
    masm.jump(&call);
    {
        masm.bind(&fail);
        masm.movePtr(ImmWord((void *)NULL), temp1);
    }
    masm.bind(&call);

    pushArg(temp1);
    pushArg(ToRegister(lir->rhs()));
    pushArg(ToRegister(lir->lhs()));
    return callVM(ArrayConcatDenseInfo, lir);
}

typedef JSObject *(*GetIteratorObjectFn)(JSContext *, HandleObject, uint32_t);
static const VMFunction GetIteratorObjectInfo = FunctionInfo<GetIteratorObjectFn>(GetIteratorObject);

bool
CodeGenerator::visitCallIteratorStart(LCallIteratorStart *lir)
{
    pushArg(Imm32(lir->mir()->flags()));
    pushArg(ToRegister(lir->object()));
    return callVM(GetIteratorObjectInfo, lir);
}

bool
CodeGenerator::visitIteratorStart(LIteratorStart *lir)
{
    const Register obj = ToRegister(lir->object());
    const Register output = ToRegister(lir->output());

    uint32_t flags = lir->mir()->flags();

    OutOfLineCode *ool = oolCallVM(GetIteratorObjectInfo, lir,
                                   (ArgList(), obj, Imm32(flags)), StoreRegisterTo(output));
    if (!ool)
        return false;

    const Register temp1 = ToRegister(lir->temp1());
    const Register temp2 = ToRegister(lir->temp2());
    const Register niTemp = ToRegister(lir->temp3()); // Holds the NativeIterator object.

    // Iterators other than for-in should use LCallIteratorStart.
    JS_ASSERT(flags == JSITER_ENUMERATE);

    // Fetch the most recent iterator and ensure it's not NULL.
    masm.loadPtr(AbsoluteAddress(&gen->compartment->rt->nativeIterCache.last), output);
    masm.branchTestPtr(Assembler::Zero, output, output, ool->entry());

    // Load NativeIterator.
    masm.loadObjPrivate(output, JSObject::ITER_CLASS_NFIXED_SLOTS, niTemp);

    // Ensure the |active| and |unreusable| bits are not set.
    masm.branchTest32(Assembler::NonZero, Address(niTemp, offsetof(NativeIterator, flags)),
                      Imm32(JSITER_ACTIVE|JSITER_UNREUSABLE), ool->entry());

    // Load the iterator's shape array.
    masm.loadPtr(Address(niTemp, offsetof(NativeIterator, shapes_array)), temp2);

    // Compare shape of object with the first shape.
    masm.loadObjShape(obj, temp1);
    masm.branchPtr(Assembler::NotEqual, Address(temp2, 0), temp1, ool->entry());

    // Compare shape of object's prototype with the second shape.
    masm.loadObjProto(obj, temp1);
    masm.loadObjShape(temp1, temp1);
    masm.branchPtr(Assembler::NotEqual, Address(temp2, sizeof(Shape *)), temp1, ool->entry());

    // Ensure the object's prototype's prototype is NULL. The last native iterator
    // will always have a prototype chain length of one (i.e. it must be a plain
    // object), so we do not need to generate a loop here.
    masm.loadObjProto(obj, temp1);
    masm.loadObjProto(temp1, temp1);
    masm.branchTestPtr(Assembler::NonZero, temp1, temp1, ool->entry());

    // Write barrier for stores to the iterator. We only need to take a write
    // barrier if NativeIterator::obj is actually going to change.
    {
        Label noBarrier;
        masm.branchTestNeedsBarrier(Assembler::Zero, temp1, &noBarrier);

        Address objAddr(niTemp, offsetof(NativeIterator, obj));
        masm.branchPtr(Assembler::NotEqual, objAddr, obj, ool->entry());

        masm.bind(&noBarrier);
    }

    // Mark iterator as active.
    masm.storePtr(obj, Address(niTemp, offsetof(NativeIterator, obj)));
    masm.or32(Imm32(JSITER_ACTIVE), Address(niTemp, offsetof(NativeIterator, flags)));

    // Chain onto the active iterator stack.
    masm.loadJSContext(temp1);
    masm.loadPtr(Address(temp1, offsetof(JSContext, enumerators)), temp2);
    masm.storePtr(temp2, Address(niTemp, offsetof(NativeIterator, next)));
    masm.storePtr(output, Address(temp1, offsetof(JSContext, enumerators)));

    masm.bind(ool->rejoin());
    return true;
}

static void
LoadNativeIterator(MacroAssembler &masm, Register obj, Register dest, Label *failures)
{
    JS_ASSERT(obj != dest);

    // Test class.
    masm.branchTestObjClass(Assembler::NotEqual, obj, dest, &PropertyIteratorObject::class_, failures);

    // Load NativeIterator object.
    masm.loadObjPrivate(obj, JSObject::ITER_CLASS_NFIXED_SLOTS, dest);
}

typedef bool (*IteratorNextFn)(JSContext *, HandleObject, MutableHandleValue);
static const VMFunction IteratorNextInfo = FunctionInfo<IteratorNextFn>(js_IteratorNext);

bool
CodeGenerator::visitIteratorNext(LIteratorNext *lir)
{
    const Register obj = ToRegister(lir->object());
    const Register temp = ToRegister(lir->temp());
    const ValueOperand output = ToOutValue(lir);

    OutOfLineCode *ool = oolCallVM(IteratorNextInfo, lir, (ArgList(), obj), StoreValueTo(output));
    if (!ool)
        return false;

    LoadNativeIterator(masm, obj, temp, ool->entry());

    masm.branchTest32(Assembler::NonZero, Address(temp, offsetof(NativeIterator, flags)),
                      Imm32(JSITER_FOREACH), ool->entry());

    // Get cursor, next string.
    masm.loadPtr(Address(temp, offsetof(NativeIterator, props_cursor)), output.scratchReg());
    masm.loadPtr(Address(output.scratchReg(), 0), output.scratchReg());
    masm.tagValue(JSVAL_TYPE_STRING, output.scratchReg(), output);

    // Increase the cursor.
    masm.addPtr(Imm32(sizeof(JSString *)), Address(temp, offsetof(NativeIterator, props_cursor)));

    masm.bind(ool->rejoin());
    return true;
}

typedef bool (*IteratorMoreFn)(JSContext *, HandleObject, JSBool *);
static const VMFunction IteratorMoreInfo = FunctionInfo<IteratorMoreFn>(ion::IteratorMore);

bool
CodeGenerator::visitIteratorMore(LIteratorMore *lir)
{
    const Register obj = ToRegister(lir->object());
    const Register output = ToRegister(lir->output());
    const Register temp = ToRegister(lir->temp());

    OutOfLineCode *ool = oolCallVM(IteratorMoreInfo, lir,
                                   (ArgList(), obj), StoreRegisterTo(output));
    if (!ool)
        return false;

    LoadNativeIterator(masm, obj, output, ool->entry());

    masm.branchTest32(Assembler::NonZero, Address(output, offsetof(NativeIterator, flags)),
                      Imm32(JSITER_FOREACH), ool->entry());

    // Set output to true if props_cursor < props_end.
    masm.loadPtr(Address(output, offsetof(NativeIterator, props_end)), temp);
    masm.cmpPtr(Address(output, offsetof(NativeIterator, props_cursor)), temp);
    emitSet(Assembler::LessThan, output);

    masm.bind(ool->rejoin());
    return true;
}

typedef bool (*CloseIteratorFn)(JSContext *, HandleObject);
static const VMFunction CloseIteratorInfo = FunctionInfo<CloseIteratorFn>(CloseIterator);

bool
CodeGenerator::visitIteratorEnd(LIteratorEnd *lir)
{
    const Register obj = ToRegister(lir->object());
    const Register temp1 = ToRegister(lir->temp1());
    const Register temp2 = ToRegister(lir->temp2());

    OutOfLineCode *ool = oolCallVM(CloseIteratorInfo, lir, (ArgList(), obj), StoreNothing());
    if (!ool)
        return false;

    LoadNativeIterator(masm, obj, temp1, ool->entry());

    masm.branchTest32(Assembler::Zero, Address(temp1, offsetof(NativeIterator, flags)),
                      Imm32(JSITER_ENUMERATE), ool->entry());

    // Clear active bit.
    masm.and32(Imm32(~JSITER_ACTIVE), Address(temp1, offsetof(NativeIterator, flags)));

    // Reset property cursor.
    masm.loadPtr(Address(temp1, offsetof(NativeIterator, props_array)), temp2);
    masm.storePtr(temp2, Address(temp1, offsetof(NativeIterator, props_cursor)));

    // Advance enumerators list.
    masm.loadJSContext(temp2);
    masm.loadPtr(Address(temp1, offsetof(NativeIterator, next)), temp1);
    masm.storePtr(temp1, Address(temp2, offsetof(JSContext, enumerators)));

    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitArgumentsLength(LArgumentsLength *lir)
{
    // read number of actual arguments from the JS frame.
    Register argc = ToRegister(lir->output());
    Address ptr(StackPointer, frameSize() + IonJSFrameLayout::offsetOfNumActualArgs());

    masm.loadPtr(ptr, argc);
    return true;
}

bool
CodeGenerator::visitGetArgument(LGetArgument *lir)
{
    ValueOperand result = GetValueOutput(lir);
    const LAllocation *index = lir->index();
    size_t argvOffset = frameSize() + IonJSFrameLayout::offsetOfActualArgs();

    if (index->isConstant()) {
        int32 i = index->toConstant()->toInt32();
        Address argPtr(StackPointer, sizeof(Value) * i + argvOffset);
        masm.loadValue(argPtr, result);
    } else {
        Register i = ToRegister(index);
        BaseIndex argPtr(StackPointer, i, ScaleFromShift(sizeof(Value)), argvOffset);
        masm.loadValue(argPtr, result);
    }
    return true;
}

bool
CodeGenerator::generate()
{
    if (!safepoints_.init(graph.totalSlotCount()))
        return false;

    // Before generating any code, we generate type checks for all parameters.
    // This comes before deoptTable_, because we can't use deopt tables without
    // creating the actual frame.
    if (!generateArgumentsChecks())
        return false;

    if (frameClass_ != FrameSizeClass::None()) {
        deoptTable_ = GetIonContext()->compartment->ionCompartment()->getBailoutTable(frameClass_);
        if (!deoptTable_)
            return false;
    }

    if (!generatePrologue())
        return false;
    if (!generateBody())
        return false;
    if (!generateEpilogue())
        return false;
    if (!generateInvalidateEpilogue())
        return false;
    if (!generateOutOfLineCode())
        return false;

    return !masm.oom();
}

bool
CodeGenerator::link()
{
    AssertCanGC();
    JSContext *cx = GetIonContext()->cx;

    Linker linker(masm);
    IonCode *code = linker.newCode(cx);
    if (!code)
        return false;

    // We encode safepoints after the OSI-point offsets have been determined.
    encodeSafepoints();

    RootedScript script(cx, gen->info().script());
    ExecutionMode executionMode = gen->info().executionMode();
    JS_ASSERT(!HasIonScript(script, executionMode));

    uint32 scriptFrameSize = frameClass_ == FrameSizeClass::None()
                           ? frameDepth_
                           : FrameSizeClass::FromDepth(frameDepth_).frameSize();

    // Check to make sure we didn't have a mid-build invalidation. If so, we
    // will trickle to ion::Compile() and return Method_Skipped.
    if (cx->compartment->types.compiledInfo.compilerOutput(cx)->isInvalidated())
        return true;

    IonScript *ionScript =
      IonScript::New(cx, graph.totalSlotCount(), scriptFrameSize, snapshots_.size(),
                     bailouts_.length(), graph.numConstants(),
                     safepointIndices_.length(), osiIndices_.length(),
                     cacheList_.length(), barrierOffsets_.length(),
                     safepoints_.size(), graph.mir().numScripts());
    SetIonScript(script, executionMode, ionScript);

    if (!ionScript)
        return false;
    invalidateEpilogueData_.fixup(&masm);
    Assembler::patchDataWithValueCheck(CodeLocationLabel(code, invalidateEpilogueData_),
                                       ImmWord(uintptr_t(ionScript)),
                                       ImmWord(uintptr_t(-1)));

    IonSpew(IonSpew_Codegen, "Created IonScript %p (raw %p)",
            (void *) ionScript, (void *) code->raw());

    ionScript->setInvalidationEpilogueDataOffset(invalidateEpilogueData_.offset());
    ionScript->setOsrPc(gen->info().osrPc());
    ionScript->setOsrEntryOffset(getOsrEntryOffset());
    ptrdiff_t real_invalidate = masm.actualOffset(invalidate_.offset());
    ionScript->setInvalidationEpilogueOffset(real_invalidate);

    ionScript->setMethod(code);
    ionScript->setDeoptTable(deoptTable_);
    if (snapshots_.size())
        ionScript->copySnapshots(&snapshots_);
    if (bailouts_.length())
        ionScript->copyBailoutTable(&bailouts_[0]);
    if (graph.numConstants())
        ionScript->copyConstants(graph.constantPool());
    if (safepointIndices_.length())
        ionScript->copySafepointIndices(&safepointIndices_[0], masm);
    if (osiIndices_.length())
        ionScript->copyOsiIndices(&osiIndices_[0], masm);
    if (cacheList_.length())
        ionScript->copyCacheEntries(&cacheList_[0], masm);
    if (barrierOffsets_.length())
        ionScript->copyPrebarrierEntries(&barrierOffsets_[0], masm);
    if (safepoints_.size())
        ionScript->copySafepoints(&safepoints_);

    JS_ASSERT(graph.mir().numScripts() > 0);
    ionScript->copyScriptEntries(graph.mir().scripts());

    linkAbsoluteLabels();

    // The correct state for prebarriers is unknown until the end of compilation,
    // since a GC can occur during code generation. All barriers are emitted
    // off-by-default, and are toggled on here if necessary.
    if (cx->compartment->needsBarrier())
        ionScript->toggleBarriers(true);

    return true;
}

// An out-of-line path to convert a boxed int32 to a double.
class OutOfLineUnboxDouble : public OutOfLineCodeBase<CodeGenerator>
{
    LUnboxDouble *unboxDouble_;

  public:
    OutOfLineUnboxDouble(LUnboxDouble *unboxDouble)
      : unboxDouble_(unboxDouble)
    { }

    bool accept(CodeGenerator *codegen) {
        return codegen->visitOutOfLineUnboxDouble(this);
    }

    LUnboxDouble *unboxDouble() const {
        return unboxDouble_;
    }
};

bool
CodeGenerator::visitUnboxDouble(LUnboxDouble *lir)
{
    const ValueOperand box = ToValue(lir, LUnboxDouble::Input);
    const LDefinition *result = lir->output();

    // Out-of-line path to convert int32 to double or bailout
    // if this instruction is fallible.
    OutOfLineUnboxDouble *ool = new OutOfLineUnboxDouble(lir);
    if (!addOutOfLineCode(ool))
        return false;

    masm.branchTestDouble(Assembler::NotEqual, box, ool->entry());
    masm.unboxDouble(box, ToFloatRegister(result));
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitOutOfLineUnboxDouble(OutOfLineUnboxDouble *ool)
{
    LUnboxDouble *ins = ool->unboxDouble();
    const ValueOperand value = ToValue(ins, LUnboxDouble::Input);

    if (ins->mir()->fallible()) {
        Assembler::Condition cond = masm.testInt32(Assembler::NotEqual, value);
        if (!bailoutIf(cond, ins->snapshot()))
            return false;
    }
    masm.int32ValueToDouble(value, ToFloatRegister(ins->output()));
    masm.jump(ool->rejoin());
    return true;
}

typedef bool (*GetPropertyFn)(JSContext *, HandleValue, HandlePropertyName, MutableHandleValue);
static const VMFunction GetPropertyInfo = FunctionInfo<GetPropertyFn>(GetProperty);

bool
CodeGenerator::visitCallGetProperty(LCallGetProperty *lir)
{
    pushArg(ImmGCPtr(lir->mir()->name()));
    pushArg(ToValue(lir, LCallGetProperty::Value));
    return callVM(GetPropertyInfo, lir);
}

typedef bool (*GetOrCallElementFn)(JSContext *, HandleValue, HandleValue, MutableHandleValue);
static const VMFunction GetElementInfo = FunctionInfo<GetOrCallElementFn>(js::GetElement);
static const VMFunction CallElementInfo = FunctionInfo<GetOrCallElementFn>(js::CallElement);

bool
CodeGenerator::visitCallGetElement(LCallGetElement *lir)
{
    pushArg(ToValue(lir, LCallGetElement::RhsInput));
    pushArg(ToValue(lir, LCallGetElement::LhsInput));

    JSOp op = JSOp(*lir->mir()->resumePoint()->pc());

    if (op == JSOP_GETELEM) {
        return callVM(GetElementInfo, lir);
    } else {
        JS_ASSERT(op == JSOP_CALLELEM);
        return callVM(CallElementInfo, lir);
    }
}

bool
CodeGenerator::visitCallSetElement(LCallSetElement *lir)
{
    pushArg(Imm32(current->mir()->strictModeCode()));
    pushArg(ToValue(lir, LCallSetElement::Value));
    pushArg(ToValue(lir, LCallSetElement::Index));
    pushArg(ToRegister(lir->getOperand(0)));
    return callVM(SetObjectElementInfo, lir);
}

bool
CodeGenerator::visitLoadFixedSlotV(LLoadFixedSlotV *ins)
{
    const Register obj = ToRegister(ins->getOperand(0));
    size_t slot = ins->mir()->slot();
    ValueOperand result = GetValueOutput(ins);

    masm.loadValue(Address(obj, JSObject::getFixedSlotOffset(slot)), result);
    return true;
}

bool
CodeGenerator::visitLoadFixedSlotT(LLoadFixedSlotT *ins)
{
    const Register obj = ToRegister(ins->getOperand(0));
    size_t slot = ins->mir()->slot();
    AnyRegister result = ToAnyRegister(ins->getDef(0));
    MIRType type = ins->mir()->type();

    masm.loadUnboxedValue(Address(obj, JSObject::getFixedSlotOffset(slot)), type, result);

    return true;
}

bool
CodeGenerator::visitStoreFixedSlotV(LStoreFixedSlotV *ins)
{
    const Register obj = ToRegister(ins->getOperand(0));
    size_t slot = ins->mir()->slot();

    const ValueOperand value = ToValue(ins, LStoreFixedSlotV::Value);

    Address address(obj, JSObject::getFixedSlotOffset(slot));
    if (ins->mir()->needsBarrier())
        emitPreBarrier(address, MIRType_Value);

    masm.storeValue(value, address);

    return true;
}

bool
CodeGenerator::visitStoreFixedSlotT(LStoreFixedSlotT *ins)
{
    const Register obj = ToRegister(ins->getOperand(0));
    size_t slot = ins->mir()->slot();

    const LAllocation *value = ins->value();
    MIRType valueType = ins->mir()->value()->type();

    ConstantOrRegister nvalue = value->isConstant()
                              ? ConstantOrRegister(*value->toConstant())
                              : TypedOrValueRegister(valueType, ToAnyRegister(value));

    Address address(obj, JSObject::getFixedSlotOffset(slot));
    if (ins->mir()->needsBarrier())
        emitPreBarrier(address, MIRType_Value);

    masm.storeConstantOrRegister(nvalue, address);

    return true;
}

// An out-of-line path to call an inline cache function.
class OutOfLineCache : public OutOfLineCodeBase<CodeGenerator>
{
    LInstruction *ins;
    RepatchLabel repatchEntry_;
    CodeOffsetJump inlineJump;
    CodeOffsetLabel inlineLabel;

  public:
    OutOfLineCache(LInstruction *ins)
      : ins(ins)
    {}

    void setInlineJump(CodeOffsetJump jump, CodeOffsetLabel label) {
        inlineJump = jump;
        inlineLabel = label;
    }

    CodeOffsetJump getInlineJump() const {
        return inlineJump;
    }

    CodeOffsetLabel getInlineLabel() const {
        return inlineLabel;
    }

    bool accept(CodeGenerator *codegen) {
        switch (ins->op()) {
          case LInstruction::LOp_InstanceOfO:
          case LInstruction::LOp_InstanceOfV:
          case LInstruction::LOp_GetPropertyCacheT:
          case LInstruction::LOp_GetPropertyCacheV:
            return codegen->visitOutOfLineCacheGetProperty(this);
          case LInstruction::LOp_GetElementCacheV:
            return codegen->visitOutOfLineGetElementCache(this);
          case LInstruction::LOp_SetPropertyCacheT:
          case LInstruction::LOp_SetPropertyCacheV:
            return codegen->visitOutOfLineSetPropertyCache(this);
          case LInstruction::LOp_BindNameCache:
            return codegen->visitOutOfLineBindNameCache(this);
          case LInstruction::LOp_GetNameCache:
            return codegen->visitOutOfLineGetNameCache(this);
          default:
            JS_NOT_REACHED("Bad instruction");
            return false;
        }
    }

    LInstruction *cache() {
        return ins;
    }
    void bind(MacroAssembler *masm) {
        masm->bind(&repatchEntry_);
    }
    RepatchLabel *repatchEntry() {
        return &repatchEntry_;
    }
};

bool
CodeGenerator::visitCache(LInstruction *ins)
{
    OutOfLineCache *ool = new OutOfLineCache(ins);
    if (!addOutOfLineCode(ool))
        return false;

#if 0
    // NOTE: This is currently disabled. OSI and IC interaction are  protected
    // through other means in ICs, since the nops incur significant overhead.
    //
    // ensureOsiSpace();
#endif

    CodeOffsetJump jump = masm.jumpWithPatch(ool->repatchEntry());
    CodeOffsetLabel label = masm.labelForPatch();
    masm.bind(ool->rejoin());

    ool->setInlineJump(jump, label);
    return true;
}

typedef bool (*GetNameCacheFn)(JSContext *, size_t, HandleObject, MutableHandleValue);
static const VMFunction GetNameCacheInfo =
    FunctionInfo<GetNameCacheFn>(GetNameCache);

bool
CodeGenerator::visitOutOfLineGetNameCache(OutOfLineCache *ool)
{
    LGetNameCache *lir = ool->cache()->toGetNameCache();
    const MGetNameCache *mir = lir->mir();
    Register scopeChain = ToRegister(lir->scopeObj());
    RegisterSet liveRegs = lir->safepoint()->liveRegs();
    TypedOrValueRegister output(GetValueOutput(lir));

    IonCache::Kind kind = (mir->accessKind() == MGetNameCache::NAME)
                          ? IonCache::Name
                          : IonCache::NameTypeOf;
    IonCacheName cache(kind, ool->getInlineJump(), ool->getInlineLabel(),
                       masm.labelForPatch(), liveRegs,
                       scopeChain, mir->name(), output);

    cache.setScriptedLocation(mir->block()->info().script(), mir->resumePoint()->pc());
    size_t cacheIndex = allocateCache(cache);

    saveLive(lir);

    pushArg(scopeChain);
    pushArg(Imm32(cacheIndex));
    if (!callVM(GetNameCacheInfo, lir))
        return false;

    masm.storeCallResultValue(output);
    restoreLive(lir);

    masm.jump(ool->rejoin());
    return true;
}

typedef bool (*GetPropertyCacheFn)(JSContext *, size_t, HandleObject, MutableHandleValue);
static const VMFunction GetPropertyCacheInfo =
    FunctionInfo<GetPropertyCacheFn>(GetPropertyCache);

bool
CodeGenerator::visitOutOfLineCacheGetProperty(OutOfLineCache *ool)
{
    RegisterSet liveRegs = ool->cache()->safepoint()->liveRegs();

    LInstruction *ins = ool->cache();
    MInstruction *mir = ins->mirRaw()->toInstruction();

    TypedOrValueRegister output;

    Register objReg;

    // Define the input and output registers each opcode wants,
    // and which atom property it should get
    // Note: because all registers are saved, the output register should be
    //       a def register, else the result will be overriden by restoreLive(ins)
    PropertyName *name = NULL;
    bool allowGetters = false;
    switch (ins->op()) {
      case LInstruction::LOp_InstanceOfO:
      case LInstruction::LOp_InstanceOfV:
        name = gen->compartment->rt->atomState.classPrototype;
        objReg = ToRegister(ins->getTemp(1));
        output = TypedOrValueRegister(MIRType_Object, ToAnyRegister(ins->getDef(0)));
        break;
      case LInstruction::LOp_GetPropertyCacheT:
        name = ((LGetPropertyCacheT *) ins)->mir()->name();
        objReg = ToRegister(ins->getOperand(0));
        output = TypedOrValueRegister(mir->type(), ToAnyRegister(ins->getDef(0)));
        JS_ASSERT(mir->isGetPropertyCache());
        allowGetters = mir->toGetPropertyCache()->allowGetters();
        break;
      case LInstruction::LOp_GetPropertyCacheV:
        name = ((LGetPropertyCacheV *) ins)->mir()->name();
        objReg = ToRegister(ins->getOperand(0));
        output = TypedOrValueRegister(GetValueOutput(ins));
        JS_ASSERT(mir->isGetPropertyCache());
        allowGetters = mir->toGetPropertyCache()->allowGetters();
        break;
      default:
        JS_NOT_REACHED("Bad instruction");
        return false;
    }

    IonCacheGetProperty cache(ool->getInlineJump(), ool->getInlineLabel(),
                              masm.labelForPatch(), liveRegs,
                              objReg, name, output, allowGetters);

    if (mir->resumePoint())
        cache.setScriptedLocation(mir->block()->info().script(), mir->resumePoint()->pc());
    else
        cache.setIdempotent();
    size_t cacheIndex = allocateCache(cache);

    saveLive(ins);

    pushArg(objReg);
    pushArg(Imm32(cacheIndex));
    if (!callVM(GetPropertyCacheInfo, ins))
        return false;

    masm.storeCallResultValue(output);
    restoreLive(ins);

    masm.jump(ool->rejoin());

    return true;
}

typedef bool (*GetElementCacheFn)(JSContext *, size_t, HandleObject, HandleValue, MutableHandleValue);
static const VMFunction GetElementCacheInfo = FunctionInfo<GetElementCacheFn>(GetElementCache);

bool
CodeGenerator::visitOutOfLineGetElementCache(OutOfLineCache *ool)
{
    LGetElementCacheV *ins = ool->cache()->toGetElementCacheV();
    const MGetElementCache *mir = ins->mir();

    Register obj = ToRegister(ins->object());
    ConstantOrRegister index = TypedOrValueRegister(ToValue(ins, LGetElementCacheV::Index));
    TypedOrValueRegister output = TypedOrValueRegister(GetValueOutput(ins));

    RegisterSet liveRegs = ins->safepoint()->liveRegs();

    IonCacheGetElement cache(ool->getInlineJump(), ool->getInlineLabel(),
                             masm.labelForPatch(), liveRegs,
                             obj, index, output,
                             mir->monitoredResult());

    cache.setScriptedLocation(mir->block()->info().script(), mir->resumePoint()->pc());
    size_t cacheIndex = allocateCache(cache);

    saveLive(ins);

    pushArg(index);
    pushArg(obj);
    pushArg(Imm32(cacheIndex));
    if (!callVM(GetElementCacheInfo, ins))
        return false;

    masm.storeCallResultValue(output);
    restoreLive(ins);

    masm.jump(ool->rejoin());
    return true;
}

typedef JSObject *(*BindNameCacheFn)(JSContext *, size_t, HandleObject);
static const VMFunction BindNameCacheInfo =
    FunctionInfo<BindNameCacheFn>(BindNameCache);

bool
CodeGenerator::visitOutOfLineBindNameCache(OutOfLineCache *ool)
{
    LBindNameCache *ins = ool->cache()->toBindNameCache();
    Register scopeChain = ToRegister(ins->scopeChain());
    Register output = ToRegister(ins->output());

    RegisterSet liveRegs = ins->safepoint()->liveRegs();

    const MBindNameCache *mir = ins->mir();
    IonCacheBindName cache(ool->getInlineJump(), ool->getInlineLabel(),
                           masm.labelForPatch(), liveRegs,
                           scopeChain, mir->name(), output);
    cache.setScriptedLocation(mir->script(), mir->pc());
    size_t cacheIndex = allocateCache(cache);

    saveLive(ins);

    pushArg(scopeChain);
    pushArg(Imm32(cacheIndex));
    if (!callVM(BindNameCacheInfo, ins))
        return false;

    masm.storeCallResult(output);
    restoreLive(ins);

    masm.jump(ool->rejoin());
    return true;
}

ConstantOrRegister
CodeGenerator::getSetPropertyValue(LInstruction *ins)
{
    if (ins->getOperand(1)->isConstant()) {
        JS_ASSERT(ins->isSetPropertyCacheT());
        return ConstantOrRegister(*ins->getOperand(1)->toConstant());
    }

    switch (ins->op()) {
      case LInstruction::LOp_CallSetProperty:
        return TypedOrValueRegister(ToValue(ins, LCallSetProperty::Value));
      case LInstruction::LOp_SetPropertyCacheV:
        return TypedOrValueRegister(ToValue(ins, LSetPropertyCacheV::Value));
      case LInstruction::LOp_SetPropertyCacheT: {
        LSetPropertyCacheT *ins_ = ins->toSetPropertyCacheT();
        return TypedOrValueRegister(ins_->valueType(), ToAnyRegister(ins->getOperand(1)));
      }
      default:
        JS_NOT_REACHED("Bad opcode");
        return ConstantOrRegister(UndefinedValue());
    }
}

typedef bool (*SetPropertyFn)(JSContext *, HandleObject,
                              HandlePropertyName, const HandleValue, bool, bool);
static const VMFunction SetPropertyInfo =
    FunctionInfo<SetPropertyFn>(SetProperty);

bool
CodeGenerator::visitCallSetProperty(LCallSetProperty *ins)
{
    ConstantOrRegister value = getSetPropertyValue(ins);

    const Register objReg = ToRegister(ins->getOperand(0));
    bool isSetName = JSOp(*ins->mir()->resumePoint()->pc()) == JSOP_SETNAME;

    pushArg(Imm32(isSetName));
    pushArg(Imm32(ins->mir()->strict()));

    pushArg(value);
    pushArg(ImmGCPtr(ins->mir()->name()));
    pushArg(objReg);

    return callVM(SetPropertyInfo, ins);
}

typedef bool (*DeletePropertyFn)(JSContext *, HandleValue, HandlePropertyName, JSBool *);
static const VMFunction DeletePropertyStrictInfo =
    FunctionInfo<DeletePropertyFn>(DeleteProperty<true>);
static const VMFunction DeletePropertyNonStrictInfo =
    FunctionInfo<DeletePropertyFn>(DeleteProperty<false>);

bool
CodeGenerator::visitCallDeleteProperty(LCallDeleteProperty *lir)
{
    pushArg(ImmGCPtr(lir->mir()->name()));
    pushArg(ToValue(lir, LCallDeleteProperty::Value));

    if (lir->mir()->block()->info().script()->strictModeCode)
        return callVM(DeletePropertyStrictInfo, lir);
    else
        return callVM(DeletePropertyNonStrictInfo, lir);
}

typedef bool (*SetPropertyCacheFn)(JSContext *, size_t, HandleObject, HandleValue, bool);
static const VMFunction SetPropertyCacheInfo =
    FunctionInfo<SetPropertyCacheFn>(ion::SetPropertyCache);

bool
CodeGenerator::visitOutOfLineSetPropertyCache(OutOfLineCache *ool)
{
    LInstruction *ins = ool->cache();

    Register objReg = ToRegister(ins->getOperand(0));
    RegisterSet liveRegs = ins->safepoint()->liveRegs();

    ConstantOrRegister value = getSetPropertyValue(ins);
    const MSetPropertyCache *mir = ins->mirRaw()->toSetPropertyCache();

    IonCacheSetProperty cache(ool->getInlineJump(), ool->getInlineLabel(),
                              masm.labelForPatch(), liveRegs,
                              objReg, mir->name(), value,
                              mir->strict());

    size_t cacheIndex = allocateCache(cache);
    bool isSetName = JSOp(*mir->resumePoint()->pc()) == JSOP_SETNAME;

    saveLive(ins);

    pushArg(Imm32(isSetName));
    pushArg(value);
    pushArg(objReg);
    pushArg(Imm32(cacheIndex));

    if (!callVM(SetPropertyCacheInfo, ool->cache()))
        return false;

    restoreLive(ins);

    masm.jump(ool->rejoin());

    return true;
}

typedef bool (*ThrowFn)(JSContext *, HandleValue);
static const VMFunction ThrowInfo = FunctionInfo<ThrowFn>(js::Throw);

bool
CodeGenerator::visitThrow(LThrow *lir)
{
    pushArg(ToValue(lir, LThrow::Value));
    return callVM(ThrowInfo, lir);
}

typedef bool (*BitNotFn)(JSContext *, HandleValue, int *p);
static const VMFunction BitNotInfo = FunctionInfo<BitNotFn>(BitNot);

bool
CodeGenerator::visitBitNotV(LBitNotV *lir)
{
    pushArg(ToValue(lir, LBitNotV::Input));
    return callVM(BitNotInfo, lir);
}

typedef bool (*BitopFn)(JSContext *, HandleValue, HandleValue, int *p);
static const VMFunction BitAndInfo = FunctionInfo<BitopFn>(BitAnd);
static const VMFunction BitOrInfo = FunctionInfo<BitopFn>(BitOr);
static const VMFunction BitXorInfo = FunctionInfo<BitopFn>(BitXor);
static const VMFunction BitLhsInfo = FunctionInfo<BitopFn>(BitLsh);
static const VMFunction BitRhsInfo = FunctionInfo<BitopFn>(BitRsh);

bool
CodeGenerator::visitBitOpV(LBitOpV *lir)
{
    pushArg(ToValue(lir, LBitOpV::RhsInput));
    pushArg(ToValue(lir, LBitOpV::LhsInput));

    switch (lir->jsop()) {
      case JSOP_BITAND:
        return callVM(BitAndInfo, lir);
      case JSOP_BITOR:
        return callVM(BitOrInfo, lir);
      case JSOP_BITXOR:
        return callVM(BitXorInfo, lir);
      case JSOP_LSH:
        return callVM(BitLhsInfo, lir);
      case JSOP_RSH:
        return callVM(BitRhsInfo, lir);
      default:
        break;
    }
    JS_NOT_REACHED("unexpected bitop");
    return false;
}

class OutOfLineTypeOfV : public OutOfLineCodeBase<CodeGenerator>
{
    LTypeOfV *ins_;

  public:
    OutOfLineTypeOfV(LTypeOfV *ins)
      : ins_(ins)
    { }

    bool accept(CodeGenerator *codegen) {
        return codegen->visitOutOfLineTypeOfV(this);
    }
    LTypeOfV *ins() const {
        return ins_;
    }
};

bool
CodeGenerator::visitTypeOfV(LTypeOfV *lir)
{
    const ValueOperand value = ToValue(lir, LTypeOfV::Input);
    Register output = ToRegister(lir->output());
    Register tag = masm.splitTagForTest(value);

    OutOfLineTypeOfV *ool = new OutOfLineTypeOfV(lir);
    if (!addOutOfLineCode(ool))
        return false;

    JSRuntime *rt = gen->compartment->rt;

    // Jump to the OOL path if the value is an object. Objects are complicated
    // since they may have a typeof hook.
    masm.branchTestObject(Assembler::Equal, tag, ool->entry());

    Label done;

    Label notNumber;
    masm.branchTestNumber(Assembler::NotEqual, tag, &notNumber);
    masm.movePtr(ImmGCPtr(rt->atomState.number), output);
    masm.jump(&done);
    masm.bind(&notNumber);

    Label notUndefined;
    masm.branchTestUndefined(Assembler::NotEqual, tag, &notUndefined);
    masm.movePtr(ImmGCPtr(rt->atomState.undefined), output);
    masm.jump(&done);
    masm.bind(&notUndefined);

    Label notNull;
    masm.branchTestNull(Assembler::NotEqual, tag, &notNull);
    masm.movePtr(ImmGCPtr(rt->atomState.object), output);
    masm.jump(&done);
    masm.bind(&notNull);

    Label notBoolean;
    masm.branchTestBoolean(Assembler::NotEqual, tag, &notBoolean);
    masm.movePtr(ImmGCPtr(rt->atomState.boolean), output);
    masm.jump(&done);
    masm.bind(&notBoolean);

    masm.movePtr(ImmGCPtr(rt->atomState.string), output);

    masm.bind(&done);
    masm.bind(ool->rejoin());
    return true;
}

typedef JSString *(*TypeOfFn)(JSContext *, HandleValue);
static const VMFunction TypeOfInfo = FunctionInfo<TypeOfFn>(TypeOfOperation);

bool
CodeGenerator::visitOutOfLineTypeOfV(OutOfLineTypeOfV *ool)
{
    LTypeOfV *ins = ool->ins();
    saveLive(ins);

    pushArg(ToValue(ins, LTypeOfV::Input));
    if (!callVM(TypeOfInfo, ins))
        return false;

    masm.storeCallResult(ToRegister(ins->output()));
    restoreLive(ins);

    masm.jump(ool->rejoin());
    return true;
}

typedef bool (*ToIdFn)(JSContext *, HandleScript, jsbytecode *, HandleValue, HandleValue,
                       MutableHandleValue);
static const VMFunction ToIdInfo = FunctionInfo<ToIdFn>(ToIdOperation);

bool
CodeGenerator::visitToIdV(LToIdV *lir)
{
    pushArg(ToValue(lir, LToIdV::Index));
    pushArg(ToValue(lir, LToIdV::Object));
    pushArg(ImmWord(lir->mir()->resumePoint()->pc()));
    pushArg(ImmGCPtr(current->mir()->info().script()));
    return callVM(ToIdInfo, lir);
}

bool
CodeGenerator::visitLoadElementV(LLoadElementV *load)
{
    Register elements = ToRegister(load->elements());
    const ValueOperand out = ToOutValue(load);

    if (load->index()->isConstant())
        masm.loadValue(Address(elements, ToInt32(load->index()) * sizeof(Value)), out);
    else
        masm.loadValue(BaseIndex(elements, ToRegister(load->index()), TimesEight), out);

    if (load->mir()->needsHoleCheck()) {
        Assembler::Condition cond = masm.testMagic(Assembler::Equal, out);
        if (!bailoutIf(cond, load->snapshot()))
            return false;
    }

    return true;
}

bool
CodeGenerator::visitLoadElementHole(LLoadElementHole *lir)
{
    Register elements = ToRegister(lir->elements());
    Register initLength = ToRegister(lir->initLength());
    const ValueOperand out = ToOutValue(lir);

    // If the index is out of bounds, load |undefined|. Otherwise, load the
    // value.
    Label undefined, done;
    if (lir->index()->isConstant()) {
        masm.branch32(Assembler::BelowOrEqual, initLength, Imm32(ToInt32(lir->index())), &undefined);
        masm.loadValue(Address(elements, ToInt32(lir->index()) * sizeof(Value)), out);
    } else {
        masm.branch32(Assembler::BelowOrEqual, initLength, ToRegister(lir->index()), &undefined);
        masm.loadValue(BaseIndex(elements, ToRegister(lir->index()), TimesEight), out);
    }

    // If a hole check is needed, and the value wasn't a hole, we're done.
    // Otherwise, we'll load undefined.
    if (lir->mir()->needsHoleCheck())
        masm.branchTestMagic(Assembler::NotEqual, out, &done);
    else
        masm.jump(&done);

    masm.bind(&undefined);
    masm.moveValue(UndefinedValue(), out);
    masm.bind(&done);
    return true;
}

bool
CodeGenerator::visitLoadTypedArrayElement(LLoadTypedArrayElement *lir)
{
    Register elements = ToRegister(lir->elements());
    Register temp = lir->temp()->isBogusTemp() ? InvalidReg : ToRegister(lir->temp());
    AnyRegister out = ToAnyRegister(lir->output());

    int arrayType = lir->mir()->arrayType();
    int shift = TypedArray::slotWidth(arrayType);

    Label fail;
    if (lir->index()->isConstant()) {
        Address source(elements, ToInt32(lir->index()) * shift);
        masm.loadFromTypedArray(arrayType, source, out, temp, &fail);
    } else {
        BaseIndex source(elements, ToRegister(lir->index()), ScaleFromShift(shift));
        masm.loadFromTypedArray(arrayType, source, out, temp, &fail);
    }

    if (fail.used() && !bailoutFrom(&fail, lir->snapshot()))
        return false;

    return true;
}

class OutOfLineLoadTypedArray : public OutOfLineCodeBase<CodeGenerator>
{
    LLoadTypedArrayElementHole *ins_;

  public:
    OutOfLineLoadTypedArray(LLoadTypedArrayElementHole *ins)
      : ins_(ins)
    { }

    bool accept(CodeGenerator *codegen) {
        return codegen->visitOutOfLineLoadTypedArray(this);
    }

    LLoadTypedArrayElementHole *ins() const {
        return ins_;
    }
};

bool
CodeGenerator::visitLoadTypedArrayElementHole(LLoadTypedArrayElementHole *lir)
{
    Register object = ToRegister(lir->object());
    const ValueOperand out = ToOutValue(lir);

    OutOfLineLoadTypedArray *ool = new OutOfLineLoadTypedArray(lir);
    if (!addOutOfLineCode(ool))
        return false;

    // Load the length.
    Register scratch = out.scratchReg();
    Int32Key key = ToInt32Key(lir->index());
    masm.unboxInt32(Address(object, TypedArray::lengthOffset()), scratch);

    // OOL path if index >= length.
    masm.branchKey(Assembler::BelowOrEqual, scratch, key, ool->entry());

    // Load the elements vector.
    masm.loadPtr(Address(object, TypedArray::dataOffset()), scratch);

    int arrayType = lir->mir()->arrayType();
    int shift = TypedArray::slotWidth(arrayType);

    Label fail;
    if (key.isConstant()) {
        Address source(scratch, key.constant() * shift);
        masm.loadFromTypedArray(arrayType, source, out, lir->mir()->allowDouble(), &fail);
    } else {
        BaseIndex source(scratch, key.reg(), ScaleFromShift(shift));
        masm.loadFromTypedArray(arrayType, source, out, lir->mir()->allowDouble(), &fail);
    }

    if (fail.used() && !bailoutFrom(&fail, lir->snapshot()))
        return false;

    masm.bind(ool->rejoin());
    return true;
}

typedef bool (*GetElementMonitoredFn)(JSContext *, HandleValue, HandleValue, MutableHandleValue);
static const VMFunction GetElementMonitoredInfo =
    FunctionInfo<GetElementMonitoredFn>(js::GetElementMonitored);

bool
CodeGenerator::visitOutOfLineLoadTypedArray(OutOfLineLoadTypedArray *ool)
{
    LLoadTypedArrayElementHole *ins = ool->ins();
    saveLive(ins);

    Register object = ToRegister(ins->object());
    ValueOperand out = ToOutValue(ins);

    if (ins->index()->isConstant())
        pushArg(*ins->index()->toConstant());
    else
        pushArg(TypedOrValueRegister(MIRType_Int32, ToAnyRegister(ins->index())));
    pushArg(TypedOrValueRegister(MIRType_Object, AnyRegister(object)));
    if (!callVM(GetElementMonitoredInfo, ins))
        return false;

    masm.storeCallResultValue(out);
    restoreLive(ins);

    masm.jump(ool->rejoin());
    return true;
}

template <typename T>
static inline void
StoreToTypedArray(MacroAssembler &masm, int arrayType, const LAllocation *value, const T &dest)
{
    if (arrayType == TypedArray::TYPE_FLOAT32 || arrayType == TypedArray::TYPE_FLOAT64) {
        masm.storeToTypedFloatArray(arrayType, ToFloatRegister(value), dest);
    } else {
        if (value->isConstant())
            masm.storeToTypedIntArray(arrayType, Imm32(ToInt32(value)), dest);
        else
            masm.storeToTypedIntArray(arrayType, ToRegister(value), dest);
    }
}

bool
CodeGenerator::visitStoreTypedArrayElement(LStoreTypedArrayElement *lir)
{
    Register elements = ToRegister(lir->elements());
    const LAllocation *value = lir->value();

    int arrayType = lir->mir()->arrayType();
    int shift = TypedArray::slotWidth(arrayType);

    if (lir->index()->isConstant()) {
        Address dest(elements, ToInt32(lir->index()) * shift);
        StoreToTypedArray(masm, arrayType, value, dest);
    } else {
        BaseIndex dest(elements, ToRegister(lir->index()), ScaleFromShift(shift));
        StoreToTypedArray(masm, arrayType, value, dest);
    }

    return true;
}

bool
CodeGenerator::visitClampIToUint8(LClampIToUint8 *lir)
{
    Register input = ToRegister(lir->input());
    Register output = ToRegister(lir->output());
    masm.clampIntToUint8(input, output);
    return true;
}

bool
CodeGenerator::visitClampDToUint8(LClampDToUint8 *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    Register output = ToRegister(lir->output());
    masm.clampDoubleToUint8(input, output);
    return true;
}

bool
CodeGenerator::visitClampVToUint8(LClampVToUint8 *lir)
{
    ValueOperand input = ToValue(lir, LClampVToUint8::Input);
    FloatRegister tempFloat = ToFloatRegister(lir->tempFloat());
    Register output = ToRegister(lir->output());

    Register tag = masm.splitTagForTest(input);

    Label done;
    Label isInt32, isDouble, isBoolean;
    masm.branchTestInt32(Assembler::Equal, tag, &isInt32);
    masm.branchTestDouble(Assembler::Equal, tag, &isDouble);
    masm.branchTestBoolean(Assembler::Equal, tag, &isBoolean);

    // Undefined, null and objects are always 0.
    Label isZero;
    masm.branchTestUndefined(Assembler::Equal, tag, &isZero);
    masm.branchTestNull(Assembler::Equal, tag, &isZero);
    masm.branchTestObject(Assembler::Equal, tag, &isZero);

    // Bailout for everything else (strings).
    if (!bailout(lir->snapshot()))
        return false;

    masm.bind(&isInt32);
    masm.unboxInt32(input, output);
    masm.clampIntToUint8(output, output);
    masm.jump(&done);

    masm.bind(&isDouble);
    masm.unboxDouble(input, tempFloat);
    masm.clampDoubleToUint8(tempFloat, output);
    masm.jump(&done);

    masm.bind(&isBoolean);
    masm.unboxBoolean(input, output);
    masm.jump(&done);

    masm.bind(&isZero);
    masm.move32(Imm32(0), output);

    masm.bind(&done);
    return true;
}

typedef bool (*OperatorInFn)(JSContext *, HandleValue, HandleObject, JSBool *);
static const VMFunction OperatorInInfo = FunctionInfo<OperatorInFn>(OperatorIn);

bool
CodeGenerator::visitIn(LIn *ins)
{
    pushArg(ToRegister(ins->rhs()));
    pushArg(ToValue(ins, LIn::LHS));

    return callVM(OperatorInInfo, ins);
}

bool
CodeGenerator::visitInArray(LInArray *lir)
{
    Register elements = ToRegister(lir->elements());
    Register initLength = ToRegister(lir->initLength());
    Register output = ToRegister(lir->output());

    // When the array is not packed we need to do a hole check in addition to the bounds check.
    Label falseBranch, done;
    if (lir->index()->isConstant()) {
        masm.branch32(Assembler::BelowOrEqual, initLength, Imm32(ToInt32(lir->index())), &falseBranch);
        if (lir->mir()->needsHoleCheck()) {
            masm.branchTestMagic(Assembler::Equal, Address(elements, ToInt32(lir->index()) * sizeof(Value)),
                                 &falseBranch);
        }
    } else {
        masm.branch32(Assembler::BelowOrEqual, initLength, ToRegister(lir->index()), &falseBranch);
        if (lir->mir()->needsHoleCheck()) {
            masm.branchTestMagic(Assembler::Equal, BaseIndex(elements, ToRegister(lir->index()), TimesEight),
                                 &falseBranch);
        }
    }

    masm.move32(Imm32(1), output);
    masm.jump(&done);

    masm.bind(&falseBranch);
    masm.move32(Imm32(0), output);
    masm.bind(&done);
    return true;
}

bool
CodeGenerator::visitInstanceOfTypedO(LInstanceOfTypedO *ins)
{
    return emitInstanceOfTyped(ins, ins->mir()->prototypeObject());
}

bool
CodeGenerator::visitInstanceOfTypedV(LInstanceOfTypedV *ins)
{
    return emitInstanceOfTyped(ins, ins->mir()->prototypeObject());
}

// Wrap IsDelegate, which takes a Value for the lhs of an instanceof.
static bool
IsDelegateObject(JSContext *cx, HandleObject protoObj, HandleObject obj, JSBool *res)
{
    bool nres;
    if (!IsDelegate(cx, protoObj, ObjectValue(*obj), &nres))
        return false;
    *res = nres;
    return true;
}

typedef bool (*IsDelegateObjectFn)(JSContext *, HandleObject, HandleObject, JSBool *);
static const VMFunction IsDelegateObjectInfo = FunctionInfo<IsDelegateObjectFn>(IsDelegateObject);

bool
CodeGenerator::emitInstanceOfTyped(LInstruction *ins, RawObject prototypeObject)
{
    // This path implements fun_hasInstance when the function's prototype is
    // known to be prototypeObject.

    Label done;
    Register output = ToRegister(ins->getDef(0));

    // If the lhs is a primitive, the result is false.
    Register objReg;
    if (ins->isInstanceOfTypedV()) {
        Label isObject;
        ValueOperand lhsValue = ToValue(ins, LInstanceOfTypedV::LHS);
        masm.branchTestObject(Assembler::Equal, lhsValue, &isObject);
        masm.mov(Imm32(0), output);
        masm.jump(&done);
        masm.bind(&isObject);
        objReg = masm.extractObject(lhsValue, output);
    } else {
        objReg = ToRegister(ins->toInstanceOfTypedO()->lhs());
    }

    // Crawl the lhs's prototype chain in a loop to search for prototypeObject.
    // This follows the main loop of js::IsDelegate, though additionally breaks
    // out of the loop on Proxy::LazyProto.

    // Load the lhs's prototype.
    masm.loadPtr(Address(objReg, JSObject::offsetOfType()), output);
    masm.loadPtr(Address(output, offsetof(types::TypeObject, proto)), output);

    Label testLazy;
    {
        Label loopPrototypeChain;
        masm.bind(&loopPrototypeChain);

        // Test for the target prototype object.
        Label notPrototypeObject;
        masm.branchPtr(Assembler::NotEqual, output, ImmGCPtr(prototypeObject), &notPrototypeObject);
        masm.mov(Imm32(1), output);
        masm.jump(&done);
        masm.bind(&notPrototypeObject);

        JS_ASSERT(uintptr_t(Proxy::LazyProto) == 1);

        // Test for NULL or Proxy::LazyProto
        masm.branchPtr(Assembler::BelowOrEqual, output, ImmWord(1), &testLazy);

        // Load the current object's prototype.
        masm.loadPtr(Address(output, JSObject::offsetOfType()), output);
        masm.loadPtr(Address(output, offsetof(types::TypeObject, proto)), output);

        masm.jump(&loopPrototypeChain);
    }

    // Make a VM call if an object with a lazy proto was found on the prototype
    // chain. This currently occurs only for cross compartment wrappers, which
    // we do not expect to be compared with non-wrapper functions from this
    // compartment. Otherwise, we stopped on a NULL prototype and the output
    // register is already correct.

    OutOfLineCode *ool = oolCallVM(IsDelegateObjectInfo, ins,
                                   (ArgList(), ImmGCPtr(prototypeObject), objReg),
                                   StoreRegisterTo(output));

    // Regenerate the original lhs object for the VM call.
    Label regenerate, *lazyEntry;
    if (objReg != output) {
        lazyEntry = ool->entry();
    } else {
        masm.bind(&regenerate);
        lazyEntry = &regenerate;
        if (ins->isInstanceOfTypedV()) {
            ValueOperand lhsValue = ToValue(ins, LInstanceOfTypedV::LHS);
            objReg = masm.extractObject(lhsValue, output);
        } else {
            objReg = ToRegister(ins->toInstanceOfTypedO()->lhs());
        }
        JS_ASSERT(objReg == output);
        masm.jump(ool->entry());
    }

    masm.bind(&testLazy);
    masm.branchPtr(Assembler::Equal, output, ImmWord(1), lazyEntry);

    masm.bind(&done);
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGenerator::visitInstanceOfO(LInstanceOfO *ins)
{
    Register rhs = ToRegister(ins->getOperand(1));
    return emitInstanceOf(ins, rhs);
}

bool
CodeGenerator::visitInstanceOfV(LInstanceOfV *ins)
{
    Register rhs = ToRegister(ins->getOperand(LInstanceOfV::RHS));
    return emitInstanceOf(ins, rhs);
}

typedef bool (*HasInstanceFn)(JSContext *, HandleObject, HandleValue, JSBool *);
static const VMFunction HasInstanceInfo = FunctionInfo<HasInstanceFn>(js::HasInstance);

bool
CodeGenerator::emitInstanceOf(LInstruction *ins, Register rhs)
{
    Register rhsTmp = ToRegister(ins->getTemp(1));
    Register output = ToRegister(ins->getDef(0));

    // This temporary is used in other parts of the code.
    // Different names are used so the purpose is clear.
    Register rhsFlags = ToRegister(ins->getTemp(0));
    Register lhsTmp = ToRegister(ins->getTemp(0));

    Label boundFunctionCheck;
    Label boundFunctionDone;
    Label done;
    Label loopPrototypeChain;

    JS_ASSERT(ins->isInstanceOfO() || ins->isInstanceOfV());
    bool lhsIsValue = ins->isInstanceOfV();

    // If the lhs is an object, then the ValueOperand that gets sent to
    // HasInstance must be boxed first.  If the lhs is a value, it can
    // be sent directly.  Hence the choice between ToValue and ToTempValue
    // below.  Note that the same check is done below in the generated code
    // and explicit boxing instructions emitted before calling the OOL code
    // if we're handling a LInstanceOfO.

    OutOfLineCode *call = oolCallVM(HasInstanceInfo, ins,
        (ArgList(), rhs, lhsIsValue ? ToValue(ins, 0) : ToTempValue(ins, 0)),
        StoreRegisterTo(output));
    if (!call)
        return false;

    // 1. CODE FOR HASINSTANCE_BOUND_FUNCTION

    // ASM-equivalent of following code
    //  boundFunctionCheck:
    //      if (!rhs->isFunction())
    //          goto callHasInstance
    //      if (!rhs->isBoundFunction())
    //          goto HasInstanceCunction
    //      rhs = rhs->getBoundFunction();
    //      goto boundFunctionCheck

    masm.mov(rhs, rhsTmp);

    // Check Function
    masm.bind(&boundFunctionCheck);

    masm.loadBaseShape(rhsTmp, output);
    masm.cmpPtr(Address(output, BaseShape::offsetOfClass()), ImmWord(&js::FunctionClass));
    if (lhsIsValue) {
        // If the input LHS is a value, no boxing necessary.
        masm.j(Assembler::NotEqual, call->entry());
    } else {
        // If the input LHS is raw object pointer, it must be boxed before
        // calling into js::HasInstance.
        Label dontCallHasInstance;
        masm.j(Assembler::Equal, &dontCallHasInstance);
        masm.boxNonDouble(JSVAL_TYPE_OBJECT, ToRegister(ins->getOperand(0)), ToTempValue(ins, 0));
        masm.jump(call->entry());
        masm.bind(&dontCallHasInstance);
    }

    // Check Bound Function
    masm.loadPtr(Address(output, BaseShape::offsetOfFlags()), rhsFlags);
    masm.and32(Imm32(BaseShape::BOUND_FUNCTION), rhsFlags);
    masm.j(Assembler::Zero, &boundFunctionDone);

    // Get Bound Function
    masm.loadPtr(Address(output, BaseShape::offsetOfParent()), rhsTmp);
    masm.jump(&boundFunctionCheck);

    // 2. CODE FOR HASINSTANCE_FUNCTION
    masm.bind(&boundFunctionDone);

    // ASM-equivalent of following code
    //  if (!lhs->isObject()) {
    //    output = false;
    //    goto done;
    //  }
    //  rhs = rhs->getPrototypeClass();
    //  output = false;
    //  while (1) {
    //    lhs = lhs->getType().proto;
    //    if (lhs == NULL)
    //      goto done;
    //    if (lhs != rhs) {
    //      output = true;
    //      goto done;
    //    }
    //  }

    // When lhs is a value: The HasInstance for function objects always
    // return false when lhs isn't an object. So check if
    // lhs is an object and otherwise return false
    if (lhsIsValue) {
        Label isObject;
        ValueOperand lhsValue = ToValue(ins, LInstanceOfV::LHS);
        masm.branchTestObject(Assembler::Equal, lhsValue, &isObject);
        masm.mov(Imm32(0), output);
        masm.jump(&done);

        masm.bind(&isObject);
        Register tmp = masm.extractObject(lhsValue, lhsTmp);
        masm.mov(tmp, lhsTmp);
    } else {
        masm.mov(ToRegister(ins->getOperand(0)), lhsTmp);
    }

    // Get prototype-class by using a OutOfLine GetProperty Cache
    // It will use register 'rhsTmp' as input and register 'output' as output, see r1889
    OutOfLineCache *ool = new OutOfLineCache(ins);
    if (!addOutOfLineCode(ool))
        return false;

    // If the IC code wants to patch, make sure there is enough space to that
    // the patching does not overwrite an invalidation marker.
    ensureOsiSpace();

    CodeOffsetJump jump = masm.jumpWithPatch(ool->repatchEntry());
    CodeOffsetLabel label = masm.labelForPatch();
    masm.bind(ool->rejoin());
    ool->setInlineJump(jump, label);

    // Move the OutOfLineCache return value and set the output on false
    masm.mov(output, rhsTmp);
    masm.mov(Imm32(0), output);

    // Walk the prototype chain
    masm.bind(&loopPrototypeChain);
    masm.loadPtr(Address(lhsTmp, JSObject::offsetOfType()), lhsTmp);
    masm.loadPtr(Address(lhsTmp, offsetof(types::TypeObject, proto)), lhsTmp);

    // Bail out if we hit a lazy proto
    if (lhsIsValue) {
        masm.branch32(Assembler::Equal, lhsTmp, Imm32(1), call->entry());
    } else {
        // If the input LHS is raw object pointer, it must be boxed before
        // calling into js::HasInstance.
        Label dontCallHasInstance;
        masm.branch32(Assembler::NotEqual, lhsTmp, Imm32(1), &dontCallHasInstance);
        masm.boxNonDouble(JSVAL_TYPE_OBJECT, ToRegister(ins->getOperand(0)), ToTempValue(ins, 0));
        masm.jump(call->entry());
        masm.bind(&dontCallHasInstance);
    }

    masm.testPtr(lhsTmp, lhsTmp);
    masm.j(Assembler::Zero, &done);

    // Check lhs is equal to rhsShape
    masm.cmpPtr(lhsTmp, rhsTmp);
    masm.j(Assembler::NotEqual, &loopPrototypeChain);

    // return true
    masm.mov(Imm32(1), output);

    masm.bind(call->rejoin());
    masm.bind(&done);
    return true;
}

bool
CodeGenerator::visitGetDOMProperty(LGetDOMProperty *ins)
{
    const Register JSContextReg = ToRegister(ins->getJSContextReg());
    const Register ObjectReg = ToRegister(ins->getObjectReg());
    const Register PrivateReg = ToRegister(ins->getPrivReg());
    const Register ValueReg = ToRegister(ins->getValueReg());

    DebugOnly<uint32> initialStack = masm.framePushed();

    masm.checkStackAlignment();

    /* Make Space for the outparam */
    masm.adjustStack(-int32_t(sizeof(Value)));
    masm.movePtr(StackPointer, ValueReg);

    masm.Push(ObjectReg);

    // GetReservedSlot(obj, DOM_PROTO_INSTANCE_CLASS_SLOT).toPrivate()
    masm.loadPrivate(Address(ObjectReg, JSObject::getFixedSlotOffset(0)), PrivateReg);

    // Rooting will happen at GC time.
    masm.movePtr(StackPointer, ObjectReg);

    uint32 safepointOffset;
    if (!masm.buildFakeExitFrame(JSContextReg, &safepointOffset))
        return false;
    masm.enterFakeExitFrame(ION_FRAME_DOMGETTER);

    if (!markSafepointAt(safepointOffset, ins))
        return false;

    masm.setupUnalignedABICall(4, JSContextReg);

    masm.loadJSContext(JSContextReg);

    masm.passABIArg(JSContextReg);
    masm.passABIArg(ObjectReg);
    masm.passABIArg(PrivateReg);
    masm.passABIArg(ValueReg);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, ins->mir()->fun()));

    if (ins->mir()->isInfallible()) {
        masm.loadValue(Address(StackPointer, IonDOMExitFrameLayout::offsetOfResult()),
                       JSReturnOperand);
    } else {
        Label success, exception;
        masm.branchTest32(Assembler::Zero, ReturnReg, ReturnReg, &exception);

        masm.loadValue(Address(StackPointer, IonDOMExitFrameLayout::offsetOfResult()),
                       JSReturnOperand);
        masm.jump(&success);

        {
            masm.bind(&exception);
            masm.handleException();
        }
        masm.bind(&success);
    }
    masm.adjustStack(IonDOMExitFrameLayout::Size());

    JS_ASSERT(masm.framePushed() == initialStack);
    return true;
}

bool
CodeGenerator::visitSetDOMProperty(LSetDOMProperty *ins)
{
    const Register JSContextReg = ToRegister(ins->getJSContextReg());
    const Register ObjectReg = ToRegister(ins->getObjectReg());
    const Register PrivateReg = ToRegister(ins->getPrivReg());
    const Register ValueReg = ToRegister(ins->getValueReg());

    DebugOnly<uint32> initialStack = masm.framePushed();

    masm.checkStackAlignment();

    // Push thei argument. Rooting will happen at GC time.
    ValueOperand argVal = ToValue(ins, LSetDOMProperty::Value);
    masm.Push(argVal);
    masm.movePtr(StackPointer, ValueReg);

    masm.Push(ObjectReg);

    // GetReservedSlot(obj, DOM_PROTO_INSTANCE_CLASS_SLOT).toPrivate()
    masm.loadPrivate(Address(ObjectReg, JSObject::getFixedSlotOffset(0)), PrivateReg);

    // Rooting will happen at GC time.
    masm.movePtr(StackPointer, ObjectReg);

    uint32 safepointOffset;
    if (!masm.buildFakeExitFrame(JSContextReg, &safepointOffset))
        return false;
    masm.enterFakeExitFrame(ION_FRAME_DOMSETTER);

    if (!markSafepointAt(safepointOffset, ins))
        return false;

    masm.setupUnalignedABICall(4, JSContextReg);

    masm.loadJSContext(JSContextReg);

    masm.passABIArg(JSContextReg);
    masm.passABIArg(ObjectReg);
    masm.passABIArg(PrivateReg);
    masm.passABIArg(ValueReg);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, ins->mir()->fun()));

    Label success, exception;
    masm.branchTest32(Assembler::Zero, ReturnReg, ReturnReg, &exception);

    masm.jump(&success);

    {
        masm.bind(&exception);
        masm.handleException();
    }
    masm.bind(&success);
    masm.adjustStack(IonDOMExitFrameLayout::Size());

    JS_ASSERT(masm.framePushed() == initialStack);
    return true;
}

typedef bool(*SPSFn)(JSContext *, HandleScript);
static const VMFunction SPSEnterInfo = FunctionInfo<SPSFn>(SPSEnter);
static const VMFunction SPSExitInfo = FunctionInfo<SPSFn>(SPSExit);

bool
CodeGenerator::visitFunctionBoundary(LFunctionBoundary *lir)
{
    AssertCanGC();
    Register temp = ToRegister(lir->temp()->output());

    switch (lir->type()) {
        case MFunctionBoundary::Inline_Enter:
            // Multiple scripts can be inlined at one depth, but there is only
            // one Inline_Exit node to signify this. To deal with this, if we
            // reach the entry of another inline script on the same level, then
            // just reset the sps metadata about the frame. We must balance
            // calls to leave()/reenter(), so perform the balance without
            // emitting any instrumentation. Technically the previous inline
            // call at this same depth has reentered, but the instrumentation
            // will be emitted at the common join point for all inlines at the
            // same depth.
            if (sps_.inliningDepth() == lir->inlineLevel()) {
                sps_.leaveInlineFrame();
                sps_.skipNextReenter();
                sps_.reenter(masm, temp);
            }

            sps_.leave(masm, temp);
            if (!sps_.enterInlineFrame())
                return false;
            // fallthrough

        case MFunctionBoundary::Enter:
            if (sps_.slowAssertions()) {
                saveLive(lir);
                pushArg(ImmGCPtr(lir->script()));
                if (!callVM(SPSEnterInfo, lir))
                    return false;
                restoreLive(lir);
                sps_.pushManual(lir->script(), masm, temp);
                return true;
            }

            return sps_.push(GetIonContext()->cx, lir->script(), masm, temp);

        case MFunctionBoundary::Inline_Exit:
            // all inline returns were covered with ::Exit, so we just need to
            // maintain the state of inline frames currently active and then
            // reenter the caller
            sps_.leaveInlineFrame();
            sps_.reenter(masm, temp);
            return true;

        case MFunctionBoundary::Exit:
            if (sps_.slowAssertions()) {
                saveLive(lir);
                pushArg(ImmGCPtr(lir->script()));
                // Once we've exited, then we shouldn't emit instrumentation for
                // the corresponding reenter() because we no longer have a
                // frame.
                sps_.skipNextReenter();
                if (!callVM(SPSExitInfo, lir))
                    return false;
                restoreLive(lir);
                return true;
            }

            sps_.pop(masm, temp);
            return true;

        default:
            JS_NOT_REACHED("invalid LFunctionBoundary type");
    }
}

} // namespace ion
} // namespace js

