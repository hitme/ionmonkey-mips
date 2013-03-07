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

#include "jsscriptinlines.h"

using namespace js;
using namespace js::ion;

enum EnterJitEbpArgumentOffset {
    ARG_JITCODE     = 2 * sizeof(void *),
    ARG_ARGC        = 3 * sizeof(void *),
    ARG_ARGV        = 4 * sizeof(void *),
    ARG_STACKFRAME  = 5 * sizeof(void *),
    ARG_CALLEETOKEN = 6 * sizeof(void *),
    ARG_RESULT      = 7 * sizeof(void *)
};

/*
 * Generates a trampoline for a C++ function with the EnterIonCode signature,
 * using the standard cdecl calling convention.
 */
IonCode *
IonRuntime::generateEnterJIT(JSContext *cx)
{
    MacroAssembler masm(cx);

    // Save return address, on mips ra saved mannually.
    masm.push(ra);
    // Save old stack frame pointer, set new stack frame pointer.
    masm.push(fp);
    masm.movl(sp, fp);

    // Save non-volatile registers. These must be saved by the trampoline,
    // rather than the JIT'd code, because they are scanned by the conservative
    // scanner.
    masm.push(a2);
    masm.push(a3);
    masm.push(t0);
    masm.push(t1);
    masm.push(t2);
    masm.push(t3);
    masm.push(t4);
    masm.push(t5);
    masm.push(t6);
    masm.push(t7);
    masm.push(s0);
    masm.push(s1);
    masm.push(s2);
    masm.push(s3);
    masm.push(s4);
    masm.push(s5);
    masm.push(s6);
    masm.push(s7);
    masm.push(t8);

    //save args to reserved slots
    masm.movl(a0, Operand(fp, ARG_JITCODE));
    masm.movl(a1, Operand(fp, ARG_ARGC));
    masm.movl(a2, Operand(fp, ARG_ARGV));
    masm.movl(a3, Operand(fp, ARG_STACKFRAME));

    masm.movl(sp, s0);
    // eax <- 8*argc, eax is now the offset betwen argv and the last
    masm.movl(Operand(fp, ARG_ARGC), t6);
    masm.shll(Imm32(3), t6);

    // We need to ensure that the stack is aligned on a 12-byte boundary, so
    // inside the JIT function the stack is 16-byte aligned. Our stack right
    // now might not be aligned on some platforms (win32, gcc) so we factor
    // this possibility in, and simulate what the new stack address would be.
    //   +argc * 8 for arguments
    //   +4 for pushing alignment
    //   +4 for pushing the callee token
    //   +4 for pushing the return address
    masm.movl(sp, t8);
    masm.subl(t6, t8);
    masm.subl(Imm32(12), t8);

    // ecx = ecx & 15, holds alignment.
    masm.andl(Imm32(15), t8);
    masm.subl(t8, sp);

    /***************************************************************
    Loop over argv vector, push arguments onto stack in reverse order
    ***************************************************************/

    // ebx = argv   --argv pointer is in ebp + 16
    masm.movl(Operand(fp, ARG_ARGV), s1);

    // eax = argv[8(argc)]  --eax now points one value past the last argument
    masm.addl(s1, t6);

    // while (eax > ebx)  --while still looping through arguments
    {
        Label header, footer;
        masm.bind(&header);

        masm.cmpl(t6, s1);
        masm.j(Assembler::BelowOrEqual, &footer);

        // eax -= 8  --move to previous argument
        masm.subl(Imm32(8), t6);

        // Push what eax points to on stack, a Value is 2 words
// Asse.. buf grow invalidates labels(256*4): JSC::AssemblerBuffer::grow(int) at ../../assembler/assembler/AssemblerBuffer.h:240
        masm.push(Operand(t6, 4));
        masm.push(Operand(t6, 0));

        masm.jmp(&header);
        masm.bind(&footer);
    }


    // Push the number of actual arguments.  |result| is used to store the
    // actual number of arguments without adding an extra argument to the enter
    // JIT.
    masm.mov(Operand(fp, ARG_RESULT), t6);
    masm.unboxInt32(Address(t6, 0x0), t6);
    masm.push(t6);

    // Push the callee token.
    masm.push(Operand(fp, ARG_CALLEETOKEN));

    // Load the StackFrame address into the OsrFrameReg.
    // This address is also used for setting the constructing bit on all paths.
    masm.movl(Operand(fp, ARG_STACKFRAME), OsrFrameReg);

    /*****************************************************************
    Push the number of bytes we've pushed so far on the stack and call
    *****************************************************************/
    // Create a frame descriptor.
    masm.subl(sp, s0);
    masm.makeFrameDescriptor(s0, IonFrame_Entry);
    masm.push(s0);

    /***************************************************************
        Call passed-in code, get return value and fill in the
        passed in return value pointer
    ***************************************************************/
//ok    //arm : masm.ma_callIonNoPush
//ok    masm.call(Operand(fp, ARG_JITCODE));
    masm.ma_callIonHalfPush(a0);//ok

    // Pop arguments off the stack.
    // eax <- 8*argc (size of all arguments we pushed on the stack)
    masm.pop(t6);
    masm.shrl(Imm32(FRAMESIZE_SHIFT), t6); // Unmark EntryFrame.
    masm.addl(t6, sp);

    // |ebp| could have been clobbered by the inner function.
    // Grab the address for the Value result from the argument stack.
    //  +18 ... arguments ...
    //  +14 <return>
    //  +10 ebp <- original %ebp pointing here.
    //  +8  ebx
    //  +4  esi
    //  +0  edi
    // 19 regs saved on mips
    masm.movl(Operand(sp, ARG_RESULT + 19 * sizeof(void *)), t6);
    masm.storeValue(JSReturnOperand, Operand(t6, 0));

    /**************************************************************
        Return stack and registers to correct state
    **************************************************************/
    // Restore non-volatile registers
    masm.pop(t8);
    masm.pop(s7);
    masm.pop(s6);
    masm.pop(s5);
    masm.pop(s4);
    masm.pop(s3);
    masm.pop(s2);
    masm.pop(s1);
    masm.pop(s0);
    masm.pop(t7);
    masm.pop(t6);
    masm.pop(t5);
    masm.pop(t4);
    masm.pop(t3);
    masm.pop(t2);
    masm.pop(t1);
    masm.pop(t0);
    masm.pop(a3);
    masm.pop(a2);

    // Restore old stack frame pointer
    masm.pop(fp);
    masm.ret();

    Linker linker(masm);
    return linker.newCode(cx);
}

IonCode *
IonRuntime::generateInvalidator(JSContext *cx)
{
    AutoIonContextAlloc aica(cx);
    MacroAssembler masm(cx);
    //masm.push(ra);

    // We do the minimum amount of work in assembly and shunt the rest
    // off to InvalidationBailout. Assembly does:
    //
    // - Pop the return address from the invalidation epilogue call.
    // - Push the machine state onto the stack.
    // - Call the InvalidationBailout routine with the stack pointer.
    // - Now that the frame has been bailed out, convert the invalidated
    //   frame into an exit frame.
    // - Do the normal check-return-code-and-thunk-to-the-interpreter dance.

    masm.addl(Imm32(sizeof(uintptr_t)), sp);

    masm.reserveStack(Registers::Total * sizeof(void *));
    for (uint32 i = 0; i < Registers::Total; i++)
        masm.movl(Register::FromCode(i), Operand(sp, i * sizeof(void *)));

    masm.reserveStack(FloatRegisters::Total * sizeof(double));
    for (uint32 i = 0; i < FloatRegisters::Total; i++)
        masm.movsd(FloatRegister::FromCode(i), Operand(sp, i * sizeof(double)));

    masm.movl(sp, s1); // Argument to ion::InvalidationBailout.

    // Make space for InvalidationBailout's frameSize outparam.
    masm.reserveStack(sizeof(size_t));
    masm.movl(sp, t8);

    masm.setupUnalignedABICall(2, t7);
    masm.passABIArg(s1);
    masm.passABIArg(t8);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, InvalidationBailout));

    masm.pop(s1); // Get the frameSize outparam.

    // Pop the machine state and the dead frame.
    masm.lea(Operand(sp, s1, TimesOne, sizeof(InvalidationBailoutStack)), sp);

    masm.generateBailoutTail(t7);

    Linker linker(masm);
    IonCode *code = linker.newCode(cx);
    IonSpew(IonSpew_Invalidate, "   invalidation thunk created at %p", (void *) code->raw());
    return code;
}

IonCode *
IonRuntime::generateArgumentsRectifier(JSContext *cx)
{
    MacroAssembler masm(cx);
    //masm.push(ra);

    // ArgumentsRectifierReg contains the |nargs| pushed onto the current frame.
    // Including |this|, there are (|nargs| + 1) arguments to copy.
    JS_ASSERT(ArgumentsRectifierReg == s0);

    // Load the number of |undefined|s to push into %ecx.
    masm.movl(Operand(sp, IonRectifierFrameLayout::offsetOfCalleeToken()), t6);
    masm.movzwl(Operand(t6, offsetof(JSFunction, nargs)), t8);
    masm.subl(s0, t8);

    // Copy the number of actual arguments.
    masm.movl(Operand(sp, IonRectifierFrameLayout::offsetOfNumActualArgs()), t7);

    masm.moveValue(UndefinedValue(), s1, s2);

    masm.push(FramePointer);
    masm.movl(sp, FramePointer); // Save %esp.

    // Push undefined.
    {
        Label undefLoopTop;
        masm.bind(&undefLoopTop);

        masm.push(s1); // type(undefined);
        masm.push(s2); // payload(undefined);
        masm.subl(Imm32(1), t8);

        masm.testl(t8, t8);
        masm.j(Assembler::NonZero, &undefLoopTop);
    }

    // Get the topmost argument. We did a push of %ebp earlier, so be sure to
    // account for this in the offset
    BaseIndex b = BaseIndex(FramePointer, s0, TimesEight,//TBD BaseIndex special treat
                            sizeof(IonRectifierFrameLayout) + sizeof(void*));
    masm.lea(Operand(b), t8);

    // Push arguments, |nargs| + 1 times (to include |this|).
    {
        Label copyLoopTop, initialSkip;

        masm.jump(&initialSkip);

        masm.bind(&copyLoopTop);
        masm.subl(Imm32(sizeof(Value)), t8);
        masm.subl(Imm32(1), s0);
        masm.bind(&initialSkip);

        masm.push(Operand(t8, sizeof(Value)/2));
        masm.push(Operand(t8, 0x0));

        masm.testl(s0, s0);
        masm.j(Assembler::NonZero, &copyLoopTop);
    }

    // Construct descriptor, accounting for pushed frame pointer above
    masm.lea(Operand(FramePointer, sizeof(void*)), s1);
    masm.subl(sp, s1);
    masm.makeFrameDescriptor(s1, IonFrame_Rectifier);

    // Construct IonJSFrameLayout.
    masm.push(t7); // number of actual arguments
    masm.push(t6); // calleeToken
    masm.push(s1); // descriptor

    // Call the target function.
    // Note that this assumes the function is JITted.
    masm.movl(Operand(t6, offsetof(JSFunction, u.i.script_)), t6);
    masm.movl(Operand(t6, offsetof(JSScript, ion)), t6);
    masm.movl(Operand(t6, IonScript::offsetOfMethod()), t6);
    masm.movl(Operand(t6, IonCode::offsetOfCode()), t6);
//ok    //arm : masm.ma_callIonHalfPush
//ok    masm.call(t6);
    masm.ma_callIonHalfPush(t6);

    // Remove the rectifier frame.
    masm.pop(s1);            // ebx <- descriptor with FrameType.
    masm.shrl(Imm32(FRAMESIZE_SHIFT), s1); // ebx <- descriptor.
    masm.pop(s2);            // Discard calleeToken.
    masm.pop(s2);            // Discard number of actual arguments.

    // Discard pushed arguments, but not the pushed frame pointer.
    BaseIndex unwind = BaseIndex(sp, s1, TimesOne, -int32_t(sizeof(void*)));//TBD BaseIndex special treat
    masm.lea(Operand(unwind), sp);

    masm.pop(FramePointer);
    masm.ret();

    Linker linker(masm);
    return linker.newCode(cx);
}

static void
GenerateBailoutThunk(JSContext *cx, MacroAssembler &masm, uint32 frameClass)
{
    // Push registers such that we can access them from [base + code].
    masm.reserveStack(Registers::Total * sizeof(void *));
    for (uint32 i = 0; i < Registers::Total; i++)
        masm.movl(Register::FromCode(i), Operand(sp, i * sizeof(void *)));

    // Push xmm registers, such that we can access them from [base + code].
    masm.reserveStack(FloatRegisters::Total * sizeof(double));
    for (uint32 i = 0; i < FloatRegisters::Total; i++)
        masm.movsd(FloatRegister::FromCode(i), Operand(sp, i * sizeof(double)));

    // Push the bailout table number.
    masm.push(Imm32(frameClass));

    // The current stack pointer is the first argument to ion::Bailout.
    masm.movl(sp, t6);

    // Call the bailout function. This will correct the size of the bailout.
    masm.setupUnalignedABICall(1, t8);
    masm.passABIArg(t6);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, Bailout));

    // Common size of stuff we've pushed.
    const uint32 BailoutDataSize = sizeof(void *) + // frameClass
                                   sizeof(double) * FloatRegisters::Total +
                                   sizeof(void *) * Registers::Total;

    // Remove both the bailout frame and the topmost Ion frame's stack.
    if (frameClass == NO_FRAME_SIZE_CLASS_ID) {
        // We want the frameSize. Stack is:
        //    ... frame ...
        //    snapshotOffset
        //    frameSize
        //    ... bailoutFrame ...
        masm.addl(Imm32(BailoutDataSize), sp);
        masm.pop(t8);
        masm.addl(Imm32(sizeof(uint32)), sp);
        masm.addl(t8, sp);
    } else {
        // Stack is:
        //    ... frame ...
        //    bailoutId
        //    ... bailoutFrame ...
        uint32 frameSize = FrameSizeClass::FromClass(frameClass).frameSize();
        masm.addl(Imm32(BailoutDataSize + sizeof(void *) + frameSize), sp);
    }

    masm.generateBailoutTail(t7);
}

IonCode *
IonRuntime::generateBailoutTable(JSContext *cx, uint32 frameClass)
{
    MacroAssembler masm;

    Label bailout;
    for (size_t i = 0; i < BAILOUT_TABLE_SIZE; i++)
        masm.call(&bailout);
    masm.bind(&bailout);
    //masm.push(ra);

    GenerateBailoutThunk(cx, masm, frameClass);

    Linker linker(masm);
    return linker.newCode(cx);
}

IonCode *
IonRuntime::generateBailoutHandler(JSContext *cx)
{
    MacroAssembler masm;

    //masm.push(ra);
    GenerateBailoutThunk(cx, masm, NO_FRAME_SIZE_CLASS_ID);

    Linker linker(masm);
    return linker.newCode(cx);
}

IonCode *
IonRuntime::generateVMWrapper(JSContext *cx, const VMFunction &f)
{
    AssertCanGC();
    typedef MoveResolver::MoveOperand MoveOperand;

    JS_ASSERT(!StackKeptAligned);
    JS_ASSERT(functionWrappers_);
    JS_ASSERT(functionWrappers_->initialized());
    VMWrapperMap::AddPtr p = functionWrappers_->lookupForAdd(&f);
    if (p)
        return p->value;

    // Generate a separated code for the wrapper.
    MacroAssembler masm;

    //masm.push(ra);
    // Avoid conflicts with argument registers while discarding the result after
    // the function call.
    GeneralRegisterSet regs = GeneralRegisterSet(Register::Codes::WrapperMask);

    // Wrapper register set is a superset of Volatile register set.
    JS_STATIC_ASSERT((Register::Codes::VolatileMask & ~Register::Codes::WrapperMask) == 0);

    // Stack is:
    //    ... frame ...
    //  +8  [args]
    //  +4  descriptor
    //  +0  returnAddress
    //
    // We're aligned to an exit frame, so link it up.
    masm.enterExitFrame(&f);

    // Save the current stack pointer as the base for copying arguments.
    Register argsBase = InvalidReg;
    if (f.explicitArgs) {
        argsBase = regs.takeAny();
        masm.lea(Operand(sp, IonExitFrameLayout::SizeWithFooter()), argsBase);
    }

    // Reserve space for the outparameter.
    Register outReg = InvalidReg;
    switch (f.outParam) {
      case Type_Value:
        outReg = regs.takeAny();
        masm.reserveStack(sizeof(Value));
        masm.movl(sp, outReg);
        break;

      case Type_Handle:
        outReg = regs.takeAny();
        masm.Push(UndefinedValue());
        masm.movl(sp, outReg);
        break;

      case Type_Int32:
        outReg = regs.takeAny();
        masm.reserveStack(sizeof(int32));
        masm.movl(sp, outReg);
        break;

      default:
        JS_ASSERT(f.outParam == Type_Void);
        break;
    }

    Register temp = regs.getAny();
    masm.setupUnalignedABICall(f.argc(), temp);

    // Initialize the context parameter.
    Register cxreg = regs.takeAny();
    masm.loadJSContext(cxreg);
    masm.passABIArg(cxreg);

#if 0
    masm.breakpoint();
#endif

    size_t argDisp = 0;

    // Copy arguments.
    if (f.explicitArgs) {
        for (uint32 explicitArg = 0; explicitArg < f.explicitArgs; explicitArg++) {
            MoveOperand from;
            switch (f.argProperties(explicitArg)) {
              case VMFunction::WordByValue:
                masm.passABIArg(MoveOperand(argsBase, argDisp));
#if 0
    masm.breakpoint();
#endif
                argDisp += sizeof(void *);
                break;
              case VMFunction::DoubleByValue:
                masm.passABIArg(MoveOperand(argsBase, argDisp));
#if 0
    masm.breakpoint();
#endif
                argDisp += sizeof(void *);
                masm.passABIArg(MoveOperand(argsBase, argDisp));
#if 0
    masm.breakpoint();
#endif
                argDisp += sizeof(void *);
                break;
              case VMFunction::WordByRef:
                masm.passABIArg(MoveOperand(argsBase, argDisp, MoveOperand::EFFECTIVE));
#if 0
    masm.breakpoint();
#endif
                argDisp += sizeof(void *);
                break;
              case VMFunction::DoubleByRef:
                masm.passABIArg(MoveOperand(argsBase, argDisp, MoveOperand::EFFECTIVE));
#if 0
    masm.breakpoint();
#endif
                argDisp += 2 * sizeof(void *);
                break;
            }
        }
    }

    // Copy the implicit outparam, if any.
    if (outReg != InvalidReg)
        masm.passABIArg(outReg);

    masm.callWithABI(f.wrapped);

    // Test for failure.
    Label exception;
    switch (f.failType()) {
      case Type_Object:
        masm.testl(v0, v0);
        masm.j(Assembler::Zero, &exception);
        break;
      case Type_Bool:
        masm.testb(v0, v0);
        masm.j(Assembler::Zero, &exception);
        break;
      default:
        JS_NOT_REACHED("unknown failure kind");
        break;
    }

    // Load the outparam and free any allocated stack.
    switch (f.outParam) {
      case Type_Handle:
      case Type_Value:
        masm.loadValue(Address(sp, 0), JSReturnOperand);
        masm.freeStack(sizeof(Value));
        break;

      case Type_Int32:
        masm.load32(Address(sp, 0), ReturnReg);
        masm.freeStack(sizeof(JSBool));
        break;

      default:
        JS_ASSERT(f.outParam == Type_Void);
        break;
    }
    masm.leaveExitFrame();
#if 0 && defined(JS_CPU_MIPS)
    masm.retn(Imm32(f.explicitStackSlots() * sizeof(void *)));
#else
    masm.retn(Imm32(sizeof(IonExitFrameLayout) + f.explicitStackSlots() * sizeof(void *)));
#endif

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
//    masm.push(ra);

    RegisterSet save = RegisterSet(GeneralRegisterSet(Registers::VolatileMask),
                                   FloatRegisterSet(FloatRegisters::VolatileMask));
    masm.PushRegsInMask(save);

    JS_ASSERT(PreBarrierReg == t7);
    masm.movl(ImmWord(cx->runtime), t8);

    masm.setupUnalignedABICall(2, t6);
    masm.passABIArg(t8);
    masm.passABIArg(t7);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, MarkFromIon));

    masm.PopRegsInMask(save);
    masm.ret();

    Linker linker(masm);
    return linker.newCode(cx);
}


