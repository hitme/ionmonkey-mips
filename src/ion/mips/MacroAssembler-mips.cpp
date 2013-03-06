/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "MacroAssembler-mips.h"
#include "ion/MoveEmitter.h"
#include "ion/IonFrames.h"

#include "jsscriptinlines.h"

using namespace js;
using namespace js::ion;

void
MacroAssemblerMIPS::setupABICall(uint32 args)
{
    JS_ASSERT(!inCall_);
    inCall_ = true;

    args_ = args;
    passedArgs_ = 0;
    stackForCall_ = 0;
//    subl(Imm32(16), sp);
}

void
MacroAssemblerMIPS::setupAlignedABICall(uint32 args)
{
    setupABICall(args);
    dynamicAlignment_ = false;
}

void
MacroAssemblerMIPS::setupUnalignedABICall(uint32 args, const Register &scratch)
{
    setupABICall(args);
    dynamicAlignment_ = true;

    movl(sp, scratch);
    andl(Imm32(~(StackAlignment - 1)), sp);
    push(scratch);
}

void
MacroAssemblerMIPS::passABIArg(const MoveOperand &from)
{
    MoveOperand to;

    ++passedArgs_;

    if(passedArgs_ <= 4){
        Register destReg;
        FloatRegister destFloatReg;
    
        if (from.isDouble() && GetArgFloatReg(passedArgs_, &destFloatReg)) {
            to = MoveOperand(destFloatReg);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
        }else {
            GetArgReg(passedArgs_, &destReg); 
            to = MoveOperand(destReg);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
        }
    }else{
#if 1
        to = MoveOperand(StackPointer, stackForCall_);
        if (from.isDouble()) {
            stackForCall_ += sizeof(double);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
        } else {
            stackForCall_ += sizeof(int32_t);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
        }
#endif
    }
}

void
MacroAssemblerMIPS::passABIArg(const Register &reg)
{
    passABIArg(MoveOperand(reg));
}

void
MacroAssemblerMIPS::passABIArg(const FloatRegister &reg)
{
    passABIArg(MoveOperand(reg));
}

void
MacroAssemblerMIPS::callWithABI(void *fun, Result result)
{
    JS_ASSERT(inCall_);
    JS_ASSERT(args_ == passedArgs_);

    uint32 stackAdjust;
    if (dynamicAlignment_) {
        stackAdjust = stackForCall_
                    + ComputeByteAlignment(stackForCall_ + STACK_SLOT_SIZE,
                                           StackAlignment);
    } else {
        stackAdjust = stackForCall_
                    + ComputeByteAlignment(stackForCall_ + framePushed_,
                                           StackAlignment);
    }

    reserveStack(stackAdjust);
    subl(Imm32(16), StackPointer);

    // Position all arguments.
    {
        enoughMemory_ &= moveResolver_.resolve();
        if (!enoughMemory_)
            return;

        MoveEmitter emitter(*this);
        emitter.emit(moveResolver_);
        emitter.finish();
    }

#ifdef DEBUG
    {
        // Check call alignment.
        Label good;
        movl(sp, t0);
        testl(t0, Imm32(StackAlignment - 1));
        j(Equal, &good);
        breakpoint();
        bind(&good);
    }
#endif

    call(ImmWord(fun));

    addl(Imm32(16), StackPointer);
    freeStack(stackAdjust);
    if (result == DOUBLE) {
        reserveStack(sizeof(double));
        fstp(Operand(sp, 0));
        movsd(Operand(sp, 0), ReturnFloatReg);
        freeStack(sizeof(double));
    }
    if (dynamicAlignment_)
        //pop(sp);
        movl(Operand(sp, 0), sp);

    JS_ASSERT(inCall_);
    inCall_ = false;
}

void
MacroAssemblerMIPS::handleException()
{
    // Reserve space for exception information.
    subl(Imm32(sizeof(ResumeFromException)), sp);
    movl(sp, a0);

    // Ask for an exception handler.
    setupUnalignedABICall(1, v0);
    passABIArg(a0);
    callWithABI(JS_FUNC_TO_DATA_PTR(void *, ion::HandleException));
    
    // Load the error value, load the new stack pointer, and return.
    moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
    movl(Operand(sp, offsetof(ResumeFromException, stackPointer)), sp);
    ret();
}

void
MacroAssemblerMIPS::branchTestValue(Condition cond, const ValueOperand &value, const Value &v, Label *label)
{
    jsval_layout jv = JSVAL_TO_IMPL(v);
    if (v.isMarkable())
        cmpl(value.payloadReg(), ImmGCPtr(reinterpret_cast<gc::Cell *>(v.toGCThing())));
    else
        cmpl(value.payloadReg(), Imm32(jv.s.payload.i32));

    if (cond == Equal) {
        Label done;
        j(NotEqual, &done);
        {
            cmpl(value.typeReg(), Imm32(jv.s.tag));
            j(Equal, label);
        }
        bind(&done);
    } else {
        JS_ASSERT(cond == NotEqual);
        j(NotEqual, label);

        cmpl(value.typeReg(), Imm32(jv.s.tag));
        j(NotEqual, label);
    }
}

Assembler::Condition
MacroAssemblerMIPS::testNegativeZero(const FloatRegister &reg, const Register &scratch)
{
    // Determines whether the single double contained in the XMM register reg
    // is equal to double-precision -0.

    Label nonZero;

    // Compare to zero. Lets through {0, -0}.
    xorpd(ScratchFloatReg, ScratchFloatReg);
    // If reg is non-zero, then a test of Zero is false.
    branchDouble(DoubleNotEqual, reg, ScratchFloatReg, &nonZero);

    // Input register is either zero or negative zero. Test sign bit.
    movmskpd(reg, scratch);
    // If reg is -0, then a test of Zero is true.
    cmpl(scratch, Imm32(1));

    bind(&nonZero);
    return Zero;
}

void 
MacroAssemblerMIPS::callWithExitFrame(IonCode *target, Register dynStack) {
    addPtr(Imm32(framePushed()), dynStack);
    makeFrameDescriptor(dynStack, IonFrame_OptimizedJS);
    Push(dynStack);
    //arm : ma_callIonHalfPush
    call(target);
}

void 
MacroAssemblerMIPS::callWithExitFrame(IonCode *target) {
    uint32 descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);
//cause failure when descriptor==0x4e0
    Push(Imm32(descriptor));
    //arm : ma_callIonHalfPush
    call(target);
}

void 
MacroAssemblerMIPS::callIon(const Register &callee) {
/*arm :
    JS_ASSERT((framePushed() & 3) == 0);
    if ((framePushed() & 7) == 4) {
        ma_callIonHalfPush(callee);
    } else {
        adjustFrame(sizeof(void*));
        ma_callIon(callee);
    }
*/
    call(callee);
}


void 
MacroAssemblerMIPS::enterOsr(Register calleeToken, Register code) {
    push(Imm32(0)); // num actual args.
    push(calleeToken);
    push(Imm32(MakeFrameDescriptor(0, IonFrame_Osr)));
    //arm : ma_callIonHalfPush
    call(code);
    addl(Imm32(sizeof(uintptr_t) * 2), sp);
}
