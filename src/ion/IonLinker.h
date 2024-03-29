/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_linker_h__
#define jsion_linker_h__

#include "jscntxt.h"
#include "jscompartment.h"
#include "ion/IonCode.h"
#include "ion/IonCompartment.h"
#include "assembler/jit/ExecutableAllocator.h"
#include "ion/IonMacroAssembler.h"

namespace js {
namespace ion {

static const int CodeAlignment = 8;
class Linker
{
    MacroAssembler &masm;

    IonCode *fail(JSContext *cx) {
        js_ReportOutOfMemory(cx);
        return NULL;
    }

    IonCode *newCode(JSContext *cx, IonCompartment *comp) {
        AssertCanGC();
#if !defined(JS_CPU_ARM)
        masm.flush();
#endif
        if (masm.oom())
            return fail(cx);

        JSC::ExecutablePool *pool;
        size_t bytesNeeded = masm.bytesNeeded() + sizeof(IonCode *) + CodeAlignment;
        if (bytesNeeded >= MAX_BUFFER_SIZE)
            return fail(cx);

        uint8 *result = (uint8 *)comp->execAlloc()->alloc(bytesNeeded, &pool, JSC::ION_CODE);
        if (!result)
            return fail(cx);

        // The IonCode pointer will be stored right before the code buffer.
        uint8 *codeStart = result + sizeof(IonCode *);

        // Bump the code up to a nice alignment.
        codeStart = (uint8 *)AlignBytes((uintptr_t)codeStart, CodeAlignment);
        uint32 headerSize = codeStart - result;
        IonCode *code = IonCode::New(cx, codeStart,
                                     bytesNeeded - headerSize, pool);
        if (!code)
            return NULL;
        code->copyFrom(masm);
        masm.link(code);
        return code;
    }

  public:
    Linker(MacroAssembler &masm)
      : masm(masm)
    {
        masm.finish();
    }

    IonCode *newCode(JSContext *cx) {
        return newCode(cx, cx->compartment->ionCompartment());
    }
};

} // namespace ion
} // namespace js

#endif // jsion_linker_h__

