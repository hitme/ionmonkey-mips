/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jscompartment.h"
#include "assembler/assembler/MacroAssembler.h"
#include "ion/IonCompartment.h"
#include "ion/IonLinker.h"
#include "ion/IonFrames.h"
#include "ion/IonSpewer.h"
#include "ion/Bailouts.h"
#include "ion/VMFunctions.h"

using namespace js;
using namespace js::ion;

static void
GenerateReturn(MacroAssembler &masm, int returnCode)
{
    // Restore non-volatile registers
    masm.ma_mov(Imm32(returnCode), r0);
    masm.startDataTransferM(IsLoad, sp, IA, WriteBack);
    masm.transferReg(r4);
    masm.transferReg(r5);
    masm.transferReg(r6);
    masm.transferReg(r7);
    masm.transferReg(r8);
    masm.transferReg(r9);
    masm.transferReg(r10);
    masm.transferReg(r11);
    // r12 isn't saved, so it shouldn't be restored.
    masm.transferReg(pc);
    masm.finishDataTransfer();
    masm.dumpPool();
}

struct EnterJITStack
{
    void *r0; // alignment.

    // non-volatile registers.
    void *r4;
    void *r5;
    void *r6;
    void *r7;
    void *r8;
    void *r9;
    void *r10;
    void *r11;
    // The abi does not expect r12 (ip) to be preserved
    void *lr;

    // Arguments.
    // code == r0
    // argc == r1
    // argv == r2
    // frame == r3
    CalleeToken token;
    Value *vp;
};

/*
 * This method generates a trampoline on x86 for a c++ function with
 * the following signature:
 *   void enter(void *code, int argc, Value *argv, StackFrame *fp, CalleeToken
 *              calleeToken, Value *vp)
 *   ...using standard EABI calling convention
 */
IonCode *
IonRuntime::generateEnterJIT(JSContext *cx)
{

    const Register reg_code  = r0;
    const Register reg_argc  = r1;
    const Register reg_argv  = r2;
    const Register reg_frame = r3;

    const Address slot_token(sp, offsetof(EnterJITStack, token));
    const Address slot_vp(sp, offsetof(EnterJITStack, vp));

    JS_ASSERT(OsrFrameReg == reg_frame);

    MacroAssembler masm(cx);
    AutoFlushCache afc("GenerateEnterJIT", cx->compartment->ionCompartment());
    Assembler *aasm = &masm;

    // Save non-volatile registers. These must be saved by the trampoline,
    // rather than the JIT'd code, because they are scanned by the conservative
    // scanner.
    masm.startDataTransferM(IsStore, sp, DB, WriteBack);
    masm.transferReg(r0); // [sp,0]
    masm.transferReg(r4); // [sp,4]
    masm.transferReg(r5); // [sp,8]
    masm.transferReg(r6); // [sp,12]
    masm.transferReg(r7); // [sp,16]
    masm.transferReg(r8); // [sp,20]
    masm.transferReg(r9); // [sp,24]
    masm.transferReg(r10); // [sp,28]
    masm.transferReg(r11); // [sp,32]
    // The abi does not expect r12 (ip) to be preserved
    masm.transferReg(lr);  // [sp,36]
    // The 5th argument is located at [sp, 40]
    masm.finishDataTransfer();

    // Save stack pointer into r8
    masm.movePtr(sp, r8);

    // Load calleeToken into r9.
    masm.loadPtr(slot_token, r9);

    // Load the number of actual arguments into r10.
    masm.loadPtr(slot_vp, r10);
    masm.unboxInt32(Address(r10, 0), r10);

#if 0
    // This is in case we want to go back to using frames that
    // aren't 8 byte alinged
    // there are r1 2-word arguments to the js code
    // we want 2 word alignment, so this shouldn't matter.
    // After the arguments have been pushed, we want to push an additional 3 words of
    // data, so in all, we want to decrease sp by 4 if it is currently aligned to
    // 8, and not touch it otherwise
    aasm->as_sub(sp, sp, Imm8(4));
    aasm->as_orr(sp, sp, Imm8(4));
#endif

    // Subtract off the size of the arguments from the stack pointer, store elsewhere
    aasm->as_sub(r4, sp, O2RegImmShift(r1, LSL, 3)); //r4 = sp - argc*8
    // Get the final position of the stack pointer into the stack pointer
    aasm->as_sub(sp, r4, Imm8(16)); // sp' = sp - argc*8 - 16
    // Get a copy of the number of args to use as a decrement counter, also
    // Set the zero condition code
    aasm->as_mov(r5, O2Reg(r1), SetCond);

    // Loop over arguments, copying them from an unknown buffer onto the Ion
    // stack so they can be accessed from JIT'ed code.
    {
        Label header, footer;
        // If there aren't any arguments, don't do anything
        aasm->as_b(&footer, Assembler::Zero);
        // Get the top of the loop
        masm.bind(&header);
        aasm->as_sub(r5, r5, Imm8(1), SetCond);
        // We could be more awesome, and unroll this, using a loadm
        // (particularly since the offset is effectively 0)
        // but that seems more error prone, and complex.
        // BIG FAT WARNING: this loads both r6 and r7.
        aasm->as_extdtr(IsLoad,  64, true, PostIndex, r6, EDtrAddr(r2, EDtrOffImm(8)));
        aasm->as_extdtr(IsStore, 64, true, PostIndex, r6, EDtrAddr(r4, EDtrOffImm(8)));
        aasm->as_b(&header, Assembler::NonZero);
        masm.bind(&footer);
    }

    masm.ma_sub(r8, sp, r8);
    masm.makeFrameDescriptor(r8, IonFrame_Entry);

    masm.startDataTransferM(IsStore, sp, IB, NoWriteBack);
                           // [sp]    = return address (written later)
    masm.transferReg(r8);  // [sp',4] = descriptor, argc*8+20
    masm.transferReg(r9);  // [sp',8]  = callee token
    masm.transferReg(r10); // [sp',12]  = actual arguments
    masm.finishDataTransfer();

    // Call the function.
    masm.ma_callIonNoPush(r0);

    // The top of the stack now points to the address of the field following
    // the return address because the return address is popped for the
    // return, so we need to remove the size of the return address field.
    aasm->as_sub(sp, sp, Imm8(4));

    // Load off of the stack the size of our local stack
    masm.loadPtr(Address(sp, IonJSFrameLayout::offsetOfDescriptor()), r5);
    aasm->as_add(sp, sp, lsr(r5, FRAMESIZE_SHIFT));

    // Store the returned value into the slot_vp
    masm.loadPtr(slot_vp, r5);
    masm.storeValue(JSReturnOperand, Address(r5, 0));

    // :TODO: Optimize storeValue with:
    // We're using a load-double here.  In order for that to work,
    // the data needs to be stored in two consecutive registers,
    // make sure this is the case
    //   ASSERT(JSReturnReg_Type.code() == JSReturnReg_Data.code()+1);
    //   aasm->as_extdtr(IsStore, 64, true, Offset,
    //                   JSReturnReg_Data, EDtrAddr(r5, EDtrOffImm(0)));

    // Get rid of the bogus r0 push.
    aasm->as_add(sp, sp, Imm8(4));

    // Restore non-volatile registers and return.
    GenerateReturn(masm, JS_TRUE);

    Linker linker(masm);
    return linker.newCode(cx);
}

IonCode *
IonRuntime::generateInvalidator(JSContext *cx)
{
    // See large comment in x86's IonRuntime::generateInvalidator.
    AutoIonContextAlloc aica(cx);
    MacroAssembler masm(cx);
    //masm.as_bkpt();
    // At this point, one of two things has happened.
    // 1) Execution has just returned from C code, which left the stack aligned
    // 2) Execution has just returned from Ion code, which left the stack unaligned.
    // The old return address should not matter, but we still want the
    // stack to be aligned, and there is no good reason to automatically align it with
    // a call to setupUnalignedABICall.
    masm.ma_and(Imm32(~7), sp, sp);
    masm.startDataTransferM(IsStore, sp, DB, WriteBack);
    // We don't have to push everything, but this is likely easier.
    // setting regs_
    for (uint32 i = 0; i < Registers::Total; i++)
        masm.transferReg(Register::FromCode(i));
    masm.finishDataTransfer();

    masm.startFloatTransferM(IsStore, sp, DB, WriteBack);
    for (uint32 i = 0; i < FloatRegisters::Total; i++)
        masm.transferFloatReg(FloatRegister::FromCode(i));
    masm.finishFloatTransfer();

    masm.ma_mov(sp, r0);
    const int sizeOfRetval = sizeof(size_t)*2;
    masm.reserveStack(sizeOfRetval);
    masm.mov(sp, r1);
    masm.setupAlignedABICall(2);
    masm.passABIArg(r0);
    masm.passABIArg(r1);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, InvalidationBailout));

    masm.ma_ldr(Address(sp, 0), r1);
    // Remove the return address, the IonScript, the register state
    // (InvaliationBailoutStack) and the space that was allocated for the return value
    masm.ma_add(sp, Imm32(sizeof(InvalidationBailoutStack) + sizeOfRetval), sp);
    // remove the space that this frame was using before the bailout
    // (computed by InvalidationBailout)
    masm.ma_add(sp, r1, sp);
    masm.generateBailoutTail(r1);
    Linker linker(masm);
    IonCode *code = linker.newCode(cx);
    IonSpew(IonSpew_Invalidate, "   invalidation thunk created at %p", (void *) code->raw());
    return code;
}

IonCode *
IonRuntime::generateArgumentsRectifier(JSContext *cx)
{
    MacroAssembler masm(cx);
    // ArgumentsRectifierReg contains the |nargs| pushed onto the current frame.
    // Including |this|, there are (|nargs| + 1) arguments to copy.
    JS_ASSERT(ArgumentsRectifierReg == r8);

    // Copy number of actual arguments into r0
    masm.ma_ldr(DTRAddr(sp, DtrOffImm(IonRectifierFrameLayout::offsetOfNumActualArgs())), r0);

    // Load the number of |undefined|s to push into r6.
    masm.ma_ldr(DTRAddr(sp, DtrOffImm(IonRectifierFrameLayout::offsetOfCalleeToken())), r1);
    masm.ma_ldrh(EDtrAddr(r1, EDtrOffImm(offsetof(JSFunction, nargs))), r6);

    masm.ma_sub(r6, r8, r2);

    masm.moveValue(UndefinedValue(), r5, r4);

    masm.ma_mov(sp, r3); // Save %sp.
    masm.ma_mov(sp, r7); // Save %sp again.

    // Push undefined.
    {
        Label undefLoopTop;
        masm.bind(&undefLoopTop);
        masm.ma_dataTransferN(IsStore, 64, true, sp, Imm32(-8), r4, PreIndex);
        masm.ma_sub(r2, Imm32(1), r2, SetCond);

        masm.ma_b(&undefLoopTop, Assembler::NonZero);
    }

    // Get the topmost argument.

    masm.ma_alu(r3, lsl(r8, 3), r3, op_add); // r3 <- r3 + nargs * 8
    masm.ma_add(r3, Imm32(sizeof(IonRectifierFrameLayout)), r3);

    // Push arguments, |nargs| + 1 times (to include |this|).
    {
        Label copyLoopTop;
        masm.bind(&copyLoopTop);
        masm.ma_dataTransferN(IsLoad, 64, true, r3, Imm32(-8), r4, PostIndex);
        masm.ma_dataTransferN(IsStore, 64, true, sp, Imm32(-8), r4, PreIndex);

        masm.ma_sub(r8, Imm32(1), r8, SetCond);
        masm.ma_b(&copyLoopTop, Assembler::Unsigned);
    }

    // translate the framesize from values into bytes
    masm.ma_add(r6, Imm32(1), r6);
    masm.ma_lsl(Imm32(3), r6, r6);

    // Construct sizeDescriptor.
    masm.makeFrameDescriptor(r6, IonFrame_Rectifier);

    // Construct IonJSFrameLayout.
    masm.ma_push(r0); // actual arguments.
    masm.ma_push(r1); // calleeToken.
    masm.ma_push(r6); // frame descriptor.

    // Call the target function.
    // Note that this code assumes the function is JITted.
    masm.ma_ldr(DTRAddr(r1, DtrOffImm(offsetof(JSFunction, u.i.script_))), r3);
    masm.ma_ldr(DTRAddr(r3, DtrOffImm(offsetof(JSScript, ion))), r3);
    masm.ma_ldr(DTRAddr(r3, DtrOffImm(IonScript::offsetOfMethod())), r3);
    masm.ma_ldr(DTRAddr(r3, DtrOffImm(IonCode::offsetOfCode())), r3);
    masm.ma_callIonHalfPush(r3);

    // arg1
    //  ...
    // argN
    // num actual args
    // callee token
    // sizeDescriptor     <- sp now
    // return address

    // Remove the rectifier frame.
    masm.ma_dtr(IsLoad, sp, Imm32(12), r4, PostIndex);

    // arg1
    //  ...
    // argN               <- sp now; r4 <- frame descriptor
    // num actual args
    // callee token
    // sizeDescriptor
    // return address

    // Discard pushed arguments.
    masm.ma_alu(sp, lsr(r4, FRAMESIZE_SHIFT), sp, op_add);

    masm.ret();
    Linker linker(masm);
    return linker.newCode(cx);
}

static void
GenerateBailoutThunk(MacroAssembler &masm, uint32 frameClass)
{
    // the stack should look like:
    // [IonFrame]
    // bailoutFrame.registersnapshot
    // bailoutFrame.fpsnapshot
    // bailoutFrame.snapshotOffset
    // bailoutFrame.frameSize

    // STEP 1a: save our register sets to the stack so Bailout() can
    // read everything.
    // sp % 8 == 0
    masm.startDataTransferM(IsStore, sp, DB, WriteBack);
    // We don't have to push everything, but this is likely easier.
    // setting regs_
    for (uint32 i = 0; i < Registers::Total; i++)
        masm.transferReg(Register::FromCode(i));
    masm.finishDataTransfer();

    masm.startFloatTransferM(IsStore, sp, DB, WriteBack);
    for (uint32 i = 0; i < FloatRegisters::Total; i++)
        masm.transferFloatReg(FloatRegister::FromCode(i));
    masm.finishFloatTransfer();

    // STEP 1b: Push both the "return address" of the function call (the
    //          address of the instruction after the call that we used to get
    //          here) as well as the callee token onto the stack.  The return
    //          address is currently in r14.  We will proceed by loading the
    //          callee token into a sacrificial register <= r14, then pushing
    //          both onto the stack

    // now place the frameClass onto the stack, via a register
    masm.ma_mov(Imm32(frameClass), r4);
    // And onto the stack.  Since the stack is full, we need to put this
    // one past the end of the current stack. Sadly, the ABI says that we need
    // to always point to the lowest place that has been written.  the OS is
    // free to do whatever it wants below sp.
    masm.startDataTransferM(IsStore, sp, DB, WriteBack);
    // set frameClassId_
    masm.transferReg(r4);
    // Set tableOffset_; higher registers are stored at higher locations on
    // the stack.
    masm.transferReg(lr);
    masm.finishDataTransfer();

    // SP % 8 == 4
    // STEP 1c: Call the bailout function, giving a pointer to the
    //          structure we just blitted onto the stack
    masm.ma_mov(sp, r0);
    masm.setupAlignedABICall(1);

    // Copy the present stack pointer into a temp register (it happens to be the
    // argument register)
    //masm.as_mov(r0, O2Reg(sp));

    // Decrement sp by another 4, so we keep alignment
    // Not Anymore!  pushing both the snapshotoffset as well as the
    // masm.as_sub(sp, sp, Imm8(4));

    // Set the old (4-byte aligned) value of the sp as the first argument
    masm.passABIArg(r0);

    // Sp % 8 == 0
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, Bailout));
    // Common size of a bailout frame.
    uint32 bailoutFrameSize = sizeof(void *) + // frameClass
                              sizeof(double) * FloatRegisters::Total +
                              sizeof(void *) * Registers::Total;

    if (frameClass == NO_FRAME_SIZE_CLASS_ID) {
        // Make sure the bailout frame size fits into the offset for a load
        masm.as_dtr(IsLoad, 32, Offset,
                    r4, DTRAddr(sp, DtrOffImm(4)));
        // used to be: offsetof(BailoutStack, frameSize_)
        // this structure is no longer available to us :(
        // We add 12 to the bailoutFrameSize because:
        // sizeof(uint32) for the tableOffset that was pushed onto the stack
        // sizeof(uintptr_t) for the snapshotOffset;
        // alignment to round the uintptr_t up to a multiple of 8 bytes.
        masm.ma_add(sp, Imm32(bailoutFrameSize+12), sp);
        masm.as_add(sp, sp, O2Reg(r4));
    } else {
        uint32 frameSize = FrameSizeClass::FromClass(frameClass).frameSize();
        masm.ma_add(Imm32(frameSize // the frame that was added when we entered the most recent function
                          + sizeof(void*) // the size of the "return address" that was dumped on the stack
                          + bailoutFrameSize) // everything else that was pushed on the stack
                    , sp);
    }
    masm.generateBailoutTail(r1);
}

IonCode *
IonRuntime::generateBailoutTable(JSContext *cx, uint32 frameClass)
{
    MacroAssembler masm;

    Label bailout;
    for (size_t i = 0; i < BAILOUT_TABLE_SIZE; i++)
        masm.ma_bl(&bailout);
    masm.bind(&bailout);

    GenerateBailoutThunk(masm, frameClass);

    Linker linker(masm);
    return linker.newCode(cx);
}

IonCode *
IonRuntime::generateBailoutHandler(JSContext *cx)
{
    MacroAssembler masm;
    GenerateBailoutThunk(masm, NO_FRAME_SIZE_CLASS_ID);

    Linker linker(masm);
    return linker.newCode(cx);
}

IonCode *
IonRuntime::generateVMWrapper(JSContext *cx, const VMFunction &f)
{
    typedef MoveResolver::MoveOperand MoveOperand;

    JS_ASSERT(functionWrappers_);
    JS_ASSERT(functionWrappers_->initialized());
    VMWrapperMap::AddPtr p = functionWrappers_->lookupForAdd(&f);
    if (p)
        return p->value;

    // Generate a separated code for the wrapper.
    MacroAssembler masm;
    GeneralRegisterSet regs = GeneralRegisterSet(Register::Codes::WrapperMask);

    // Wrapper register set is a superset of Volatile register set.
    JS_STATIC_ASSERT((Register::Codes::VolatileMask & ~Register::Codes::WrapperMask) == 0);

    // Stack is:
    //    ... frame ...
    //  +8  [args] + argPadding
    //  +0  ExitFrame
    //
    // We're aligned to an exit frame, so link it up.
    masm.enterExitFrame(&f);

    // Save the base of the argument set stored on the stack.
    Register argsBase = InvalidReg;
    if (f.explicitArgs) {
        argsBase = r5;
        regs.take(argsBase);
        masm.ma_add(sp, Imm32(IonExitFrameLayout::SizeWithFooter()), argsBase);
    }

    // Reserve space for the outparameter.
    Register outReg = InvalidReg;
    switch (f.outParam) {
      case Type_Value:
        outReg = r4;
        regs.take(outReg);
        masm.reserveStack(sizeof(Value));
        masm.ma_mov(sp, outReg);
        break;

      case Type_Handle:
        outReg = r4;
        regs.take(outReg);
        masm.Push(UndefinedValue());
        masm.ma_mov(sp, outReg);
        break;

      case Type_Int32:
        outReg = r4;
        regs.take(outReg);
        masm.reserveStack(sizeof(int32));
        masm.ma_mov(sp, outReg);
        break;

      default:
        JS_ASSERT(f.outParam == Type_Void);
        break;
    }

    Register temp = regs.getAny();
    masm.setupUnalignedABICall(f.argc(), temp);

    // Initialize and set the context parameter.
    // r0 is the first argument register.
    Register cxreg = r0;
    masm.loadJSContext(cxreg);
    masm.passABIArg(cxreg);

    size_t argDisp = 0;

    // Copy any arguments.
    for (uint32 explicitArg = 0; explicitArg < f.explicitArgs; explicitArg++) {
        MoveOperand from;
        switch (f.argProperties(explicitArg)) {
          case VMFunction::WordByValue:
            masm.passABIArg(MoveOperand(argsBase, argDisp));
            argDisp += sizeof(void *);
            break;
          case VMFunction::DoubleByValue:
            JS_NOT_REACHED("VMCalls with double-size value arguments is not supported.");
            masm.passABIArg(MoveOperand(argsBase, argDisp));
            argDisp += sizeof(void *);
            masm.passABIArg(MoveOperand(argsBase, argDisp));
            argDisp += sizeof(void *);
            break;
          case VMFunction::WordByRef:
            masm.passABIArg(MoveOperand(argsBase, argDisp, MoveOperand::EFFECTIVE));
            argDisp += sizeof(void *);
            break;
          case VMFunction::DoubleByRef:
            masm.passABIArg(MoveOperand(argsBase, argDisp, MoveOperand::EFFECTIVE));
            argDisp += 2 * sizeof(void *);
            break;
        }
    }

    // Copy the implicit outparam, if any.
    if (outReg != InvalidReg)
        masm.passABIArg(outReg);

    masm.callWithABI(f.wrapped);

    // Test for failure.
    Label exception;
    // Called functions return bools, which are 0/false and non-zero/true
    masm.ma_cmp(r0, Imm32(0));
    masm.ma_b(&exception, Assembler::Zero);

    // Load the outparam and free any allocated stack.
    switch (f.outParam) {
      case Type_Handle:
      case Type_Value:
        masm.loadValue(Address(sp, 0), JSReturnOperand);
        masm.freeStack(sizeof(Value));
        break;

      case Type_Int32:
        masm.load32(Address(sp, 0), ReturnReg);
        masm.freeStack(sizeof(int32));
        break;

      default:
        JS_ASSERT(f.outParam == Type_Void);
        break;
    }
    masm.leaveExitFrame();
    masm.retn(Imm32(sizeof(IonExitFrameLayout) + f.explicitStackSlots() * sizeof(void *)));

    masm.bind(&exception);
    masm.handleException();

    Linker linker(masm);
    IonCode *wrapper = linker.newCode(cx);
    if (!wrapper)
        return NULL;

    // linker.newCode may trigger a GC and sweep functionWrappers_ so we have to
    // use relookupOrAdd instead of add.
    if (!functionWrappers_->relookupOrAdd(p, &f, wrapper))
        return NULL;

    return wrapper;
}

IonCode *
IonRuntime::generatePreBarrier(JSContext *cx)
{
    MacroAssembler masm;

    RegisterSet save = RegisterSet(GeneralRegisterSet(Registers::VolatileMask),
                                   FloatRegisterSet(FloatRegisters::VolatileMask));
    masm.PushRegsInMask(save);

    JS_ASSERT(PreBarrierReg == r1);
    masm.movePtr(ImmWord(cx->runtime), r0);

    masm.setupUnalignedABICall(2, r2);
    masm.passABIArg(r0);
    masm.passABIArg(r1);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, MarkFromIon));

    masm.PopRegsInMask(save);
    masm.ret();

    Linker linker(masm);
    return linker.newCode(cx);
}

