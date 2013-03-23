/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jscntxt.h"
#include "jscompartment.h"
#include "jsmath.h"
#include "CodeGenerator-mips.h"
#include "ion/IonFrames.h"
#include "ion/MoveEmitter.h"
#include "ion/IonCompartment.h"
#include "ion/shared/CodeGenerator-shared-inl.h"
#include "ion/MIR.h"
#include "ion/MIRGraph.h"
#include "jsnum.h"
#include "jsscope.h"
#include "jsscriptinlines.h"

using namespace js;
using namespace js::ion;

namespace js {
namespace ion {

class DeferredJumpTable : public DeferredData
{
    MTableSwitch *mswitch;

  public:
    DeferredJumpTable(MTableSwitch *mswitch)
      : mswitch(mswitch)
    { }

    void copy(IonCode *code, uint8 *buffer) const {
        void **jumpData = (void **)buffer;

        // For every case write the pointer to the start in the table
        for (size_t j = 0; j < mswitch->numCases(); j++) {
            LBlock *caseblock = mswitch->getCase(j)->lir();
            Label *caseheader = caseblock->label();

            uint32 offset = caseheader->offset();
            *jumpData = (void *)(code->raw() + offset);
            jumpData++;
        }
    }
};

CodeGeneratorMIPS::CodeGeneratorMIPS(MIRGenerator *gen, LIRGraph *graph)
  : CodeGeneratorShared(gen, graph),
    deoptLabel_(NULL)
{
}

double
test(double x, double y)
{
    return x + y;
}

bool
CodeGeneratorMIPS::generatePrologue()
{
    // Note that this automatically sets MacroAssembler::framePushed().
    masm.reserveStack(frameSize());

    // Allocate returnLabel_ on the heap, so we don't run its destructor and
    // assert-not-bound in debug mode on compilation failure.
    returnLabel_ = new HeapLabel();

    return true;
}

bool
CodeGeneratorMIPS::generateEpilogue()
{
    masm.bind(returnLabel_);

    // Pop the stack we allocated at the start of the function.
    masm.freeStack(frameSize());
    JS_ASSERT(masm.framePushed() == 0);

    masm.ret();
    return true;
}

bool
OutOfLineBailout::accept(CodeGeneratorMIPS *codegen)
{
    return codegen->visitOutOfLineBailout(this);
}

static inline NaNCond
NaNCondFromDoubleCondition(Assembler::DoubleCondition cond)
{
    switch (cond) {
      case Assembler::DoubleOrdered:
      case Assembler::DoubleEqual:
      case Assembler::DoubleNotEqual:
      case Assembler::DoubleGreaterThan:
      case Assembler::DoubleGreaterThanOrEqual:
      case Assembler::DoubleLessThan:
      case Assembler::DoubleLessThanOrEqual:
        return NaN_IsFalse;
      case Assembler::DoubleUnordered:
      case Assembler::DoubleEqualOrUnordered:
      case Assembler::DoubleNotEqualOrUnordered:
      case Assembler::DoubleGreaterThanOrUnordered:
      case Assembler::DoubleGreaterThanOrEqualOrUnordered:
      case Assembler::DoubleLessThanOrUnordered:
      case Assembler::DoubleLessThanOrEqualOrUnordered:
        return NaN_IsTrue;
    }

    JS_NOT_REACHED("Unknown double condition");
    return NaN_Unexpected;
}

#if 0
void
CodeGeneratorMIPS::emitBranch(Assembler::DoubleCondition cond, MBasicBlock *mirTrue,
                                   MBasicBlock *mirFalse, NaNCond ifNaN)
{
    LBlock *ifTrue = mirTrue->lir();
    LBlock *ifFalse = mirFalse->lir();

    if (ifNaN == NaN_IsFalse)
        masm.j(Assembler::DoubleUnordered, ifFalse->label());
    else if (ifNaN == NaN_IsTrue)
        masm.j(Assembler::DoubleUnordered, ifTrue->label());

    if (isNextBlock(ifFalse)) {
        masm.j(cond, ifTrue->label());
    } else {
        masm.j(Assembler::InvertCondition(cond), ifFalse->label());
        if (!isNextBlock(ifTrue))
            masm.jmp(ifTrue->label());
    }
}
#endif

void
CodeGeneratorMIPS::emitBranch(Assembler::Condition cond, MBasicBlock *mirTrue,
                                   MBasicBlock *mirFalse, NaNCond ifNaN)
{
    LBlock *ifTrue = mirTrue->lir();
    LBlock *ifFalse = mirFalse->lir();

    if (ifNaN == NaN_IsFalse)
        masm.j(Assembler::Parity, ifFalse->label());
    else if (ifNaN == NaN_IsTrue)
        masm.j(Assembler::Parity, ifTrue->label());

    if (isNextBlock(ifFalse)) {
        masm.j(cond, ifTrue->label());
    } else {
        masm.j(Assembler::InvertCondition(cond), ifFalse->label());
        if (!isNextBlock(ifTrue))
            masm.jmp(ifTrue->label());
    }
}

bool
CodeGeneratorMIPS::visitTestIAndBranch(LTestIAndBranch *test)
{
    const LAllocation *opd = test->input();

    // Test the operand
    masm.testl(ToRegister(opd), ToRegister(opd));
    emitBranch(Assembler::NonZero, test->ifTrue(), test->ifFalse());
    return true;
}

bool
CodeGeneratorMIPS::visitTestDAndBranch(LTestDAndBranch *test)
{
    const LAllocation *opd = test->input();

    // ucomisd flags:
    //             Z  P  C
    //            --------- 
    //      NaN    1  1  1
    //        >    0  0  0
    //        <    0  0  1
    //        =    1  0  0
    //
    // NaN is falsey, so comparing against 0 and then using the Z flag is
    // enough to determine which branch to take.
    masm.xorpd(ScratchFloatReg, ScratchFloatReg);
    masm.ucomisd(ToFloatRegister(opd), ScratchFloatReg);
    emitBranch(Assembler::NotEqual, test->ifTrue(), test->ifFalse());
    return true;
}

void
CodeGeneratorMIPS::emitSet(Assembler::Condition cond, const Register &dest, NaNCond ifNaN)
{
    if (GeneralRegisterSet(Registers::SingleByteRegs).has(dest)) {
        // If the register we're defining is a single byte register,
        // take advantage of the setCC instruction
        masm.setCC(cond, dest);
        masm.movzxbl(dest, dest);

        if (ifNaN != NaN_Unexpected) {
            Label noNaN;
            masm.j(Assembler::NoParity, &noNaN);
            if (ifNaN == NaN_IsTrue)
                masm.movl(Imm32(1), dest);
            else
                masm.xorl(dest, dest);
            masm.bind(&noNaN);
        }
    } else {
        Label end;
        Label ifFalse;

        if (ifNaN == NaN_IsFalse)
            masm.j(Assembler::Parity, &ifFalse);
        masm.movl(Imm32(1), dest);
        masm.j(cond, &end);
        if (ifNaN == NaN_IsTrue)
            masm.j(Assembler::Parity, &end);
        masm.bind(&ifFalse);
        masm.xorl(dest, dest);

        masm.bind(&end);
    }
}

void
CodeGeneratorMIPS::emitCompare(MIRType type, const LAllocation *left, const LAllocation *right)
{
#ifdef JS_CPU_X64
    if (type == MIRType_Object) {
        masm.cmpq(ToRegister(left), ToOperand(right));
        return;
    }
#endif

    if (right->isConstant())
        masm.cmpl(ToRegister(left), Imm32(ToInt32(right)));
    else
        masm.cmpl(ToRegister(left), ToOperand(right));
}

bool
CodeGeneratorMIPS::visitCompare(LCompare *comp)
{
    emitCompare(comp->mir()->specialization(), comp->left(), comp->right());
    emitSet(JSOpToCondition(comp->jsop()), ToRegister(comp->output()));
    return true;
}

bool
CodeGeneratorMIPS::visitCompareAndBranch(LCompareAndBranch *comp)
{
    emitCompare(comp->mir()->specialization(), comp->left(), comp->right());
    Assembler::Condition cond = JSOpToCondition(comp->jsop());
    emitBranch(cond, comp->ifTrue(), comp->ifFalse());
    return true;
}

bool
CodeGeneratorMIPS::visitCompareD(LCompareD *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->mir()->jsop());
    masm.compareDouble(cond, lhs, rhs);
    emitSet(Assembler::ConditionFromDoubleCondition(cond), ToRegister(comp->output()),
            NaNCondFromDoubleCondition(cond));
    return true;
}

bool
CodeGeneratorMIPS::visitNotI(LNotI *ins)
{
    masm.cmpl(ToRegister(ins->input()), Imm32(0));
    emitSet(Assembler::Equal, ToRegister(ins->output()));
    return true;
}

bool
CodeGeneratorMIPS::visitNotD(LNotD *ins)
{
    FloatRegister opd = ToFloatRegister(ins->input());

    masm.xorpd(ScratchFloatReg, ScratchFloatReg);
    masm.compareDouble(Assembler::DoubleEqualOrUnordered, opd, ScratchFloatReg);
    emitSet(Assembler::Equal, ToRegister(ins->output()), NaN_IsTrue);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareDAndBranch(LCompareDAndBranch *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->mir()->jsop());
    masm.compareDouble(cond, lhs, rhs);
    emitBranch(Assembler::ConditionFromDoubleCondition(cond), comp->ifTrue(), comp->ifFalse(),
               NaNCondFromDoubleCondition(cond));
    return true;
}

bool
CodeGeneratorMIPS::generateOutOfLineCode()
{
    if (!CodeGeneratorShared::generateOutOfLineCode())
        return false;

    if (deoptLabel_) {
        // All non-table-based bailouts will go here.
        masm.bind(deoptLabel_);
        
        // Push the frame size, so the handler can recover the IonScript.
        masm.push(Imm32(frameSize()));

        IonCompartment *ion = GetIonContext()->compartment->ionCompartment();
        IonCode *handler = ion->getGenericBailoutHandler();

        masm.jmp(handler->raw(), Relocation::IONCODE);
    }

    return true;
}

class BailoutJump {
    Assembler::Condition cond_;

  public:
    BailoutJump(Assembler::Condition cond) : cond_(cond)
    { }
#ifdef JS_CPU_X86
    void operator()(MacroAssembler &masm, uint8 *code) const {
        masm.j(cond_, code, Relocation::HARDCODED);
    }
#endif
    void operator()(MacroAssembler &masm, Label *label) const {
        masm.j(cond_, label);
    }
};

class BailoutLabel {
    Label *label_;

  public:
    BailoutLabel(Label *label) : label_(label)
    { }
#ifdef JS_CPU_X86
    void operator()(MacroAssembler &masm, uint8 *code) const {
        masm.retarget(label_, code, Relocation::HARDCODED);
    }
#endif
    void operator()(MacroAssembler &masm, Label *label) const {
        masm.retarget(label_, label);
    }
};

template <typename T> bool
CodeGeneratorMIPS::bailout(const T &binder, LSnapshot *snapshot)
{
    if (!encode(snapshot))
        return false;

    // Though the assembler doesn't track all frame pushes, at least make sure
    // the known value makes sense. We can't use bailout tables if the stack
    // isn't properly aligned to the static frame size.
    JS_ASSERT_IF(frameClass_ != FrameSizeClass::None() && deoptTable_,
                 frameClass_.frameSize() == masm.framePushed());

#ifdef JS_CPU_X86
    // On x64, bailout tables are pointless, because 16 extra bytes are
    // reserved per external jump, whereas it takes only 10 bytes to encode a
    // a non-table based bailout.
    if (assignBailoutId(snapshot)) {
        binder(masm, deoptTable_->raw() + snapshot->bailoutId() * BAILOUT_TABLE_ENTRY_SIZE);
        return true;
    }
#endif

    // We could not use a jump table, either because all bailout IDs were
    // reserved, or a jump table is not optimal for this frame size or
    // platform. Whatever, we will generate a lazy bailout.
    OutOfLineBailout *ool = new OutOfLineBailout(snapshot);
    if (!addOutOfLineCode(ool))
        return false;

    binder(masm, ool->entry());
    return true;
}

bool
CodeGeneratorMIPS::bailoutIf(Assembler::Condition condition, LSnapshot *snapshot)
{
    return bailout(BailoutJump(condition), snapshot);
}

bool
CodeGeneratorMIPS::bailoutFrom(Label *label, LSnapshot *snapshot)
{
    JS_ASSERT(label->used() && !label->bound());
    return bailout(BailoutLabel(label), snapshot);
}

bool
CodeGeneratorMIPS::bailout(LSnapshot *snapshot)
{
    Label label;
    masm.jump(&label);
    return bailoutFrom(&label, snapshot);
}

bool
CodeGeneratorMIPS::visitOutOfLineBailout(OutOfLineBailout *ool)
{
    if (!deoptLabel_)
        deoptLabel_ = new HeapLabel();

    masm.push(Imm32(ool->snapshot()->snapshotOffset()));
    masm.jmp(deoptLabel_);
    return true;
}


bool
CodeGeneratorMIPS::visitMinMaxD(LMinMaxD *ins)
{
    FloatRegister first = ToFloatRegister(ins->first());
    FloatRegister second = ToFloatRegister(ins->second());
    FloatRegister output = ToFloatRegister(ins->output());

    JS_ASSERT(first == output);

    Assembler::Condition cond = ins->mir()->isMax()
                               ? Assembler::Above
                               : Assembler::Below;
    Label nan, equal, returnSecond, done;

    masm.ucomisd(second, first);
    masm.j(Assembler::DoubleUnordered, &nan); // first or second is NaN, result is NaN.
    masm.j(Assembler::DoubleEqual, &equal); // make sure we handle -0 and 0 right.
    masm.j(cond, &returnSecond);
    masm.jmp(&done);

    // Check for zero.
    masm.bind(&equal);
    masm.xorpd(ScratchFloatReg, ScratchFloatReg);
    masm.ucomisd(first, ScratchFloatReg);
    masm.j(Assembler::DoubleNotEqual, &done); // first wasn't 0 or -0, so just return it.
    // So now both operands are either -0 or 0.
    if (ins->mir()->isMax())
        masm.addsd(second, first); // -0 + -0 = -0 and -0 + 0 = 0.
    else {
    //    masm.orpd(second, first); // This just ors the sign bit.
        masm.zerod(first);
        masm.negd(first, first);
    }
    masm.jmp(&done);

    masm.bind(&nan);
    masm.loadStaticDouble(&js_NaN, output);
    masm.jmp(&done);

    masm.bind(&returnSecond);
    masm.movsd(second, output);

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitNegD(LNegD *ins)
{
    // XOR the float in a float register with -0.0.
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));

    // From MacroAssemblerX86Shared::maybeInlineDouble
//    masm.pcmpeqw(ScratchFloatReg, ScratchFloatReg);
//    masm.psllq(Imm32(63), ScratchFloatReg);
//
//    masm.xorpd(ScratchFloatReg, input); // s ^ 0x80000000000000
    masm.negd(input, input);
    return true;
}

bool
CodeGeneratorMIPS::visitAbsD(LAbsD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));
//    masm.xorpd(ScratchFloatReg, ScratchFloatReg);
//    masm.subsd(input, ScratchFloatReg); // negate the sign bit.
//    masm.andpd(ScratchFloatReg, input); // s & ~s
    masm.absd(input);
    return true;
}

bool
CodeGeneratorMIPS::visitSqrtD(LSqrtD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));
    masm.sqrtsd(input, input);
    return true;
}

bool
CodeGeneratorMIPS::visitPowHalfD(LPowHalfD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    Register scratch = ToRegister(ins->temp());
    JS_ASSERT(input == ToFloatRegister(ins->output()));

    const uint32_t NegInfinityFloatBits = 0xFF800000;
    Label done, sqrt;

    // Branch if not -Infinity.
    masm.move32(Imm32(NegInfinityFloatBits), scratch);
    masm.loadFloatAsDouble(scratch, ScratchFloatReg);
    masm.branchDouble(Assembler::DoubleNotEqualOrUnordered, input, ScratchFloatReg, &sqrt);

    // Math.pow(-Infinity, 0.5) == Infinity.
    masm.xorpd(input, input);
    masm.subsd(ScratchFloatReg, input);
    masm.jump(&done);

    // Math.pow(-0, 0.5) == 0 == Math.pow(0, 0.5). Adding 0 converts any -0 to 0.
    masm.bind(&sqrt);
    masm.xorpd(ScratchFloatReg, ScratchFloatReg);
    masm.addsd(ScratchFloatReg, input);
    masm.sqrtsd(input, input);

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitAddI(LAddI *ins)
{
    if (ins->rhs()->isConstant())
        masm.addl(Imm32(ToInt32(ins->rhs())), ToOperand(ins->lhs()));
    else
        masm.addl(ToOperand(ins->rhs()), ToRegister(ins->lhs()));

    if (ins->snapshot() && !bailoutIf(Assembler::Overflow, ins->snapshot()))
        return false;
    return true;
}

bool
CodeGeneratorMIPS::visitSubI(LSubI *ins)
{
    if (ins->rhs()->isConstant())
        masm.subl(Imm32(ToInt32(ins->rhs())), ToOperand(ins->lhs()));
    else
        masm.subl(ToOperand(ins->rhs()), ToRegister(ins->lhs()));

    if (ins->snapshot() && !bailoutIf(Assembler::Overflow, ins->snapshot()))
        return false;
    return true;
}

class MulNegativeZeroCheck : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    LMulI *ins_;

  public:
    MulNegativeZeroCheck(LMulI *ins)
      : ins_(ins)
    { }

    virtual bool accept(CodeGeneratorMIPS *codegen) {
        return codegen->visitMulNegativeZeroCheck(this);
    }
    LMulI *ins() const {
        return ins_;
    }
};

bool
CodeGeneratorMIPS::visitMulI(LMulI *ins)
{
    const LAllocation *lhs = ins->lhs();
    const LAllocation *rhs = ins->rhs();
    MMul *mul = ins->mir();

    if (rhs->isConstant()) {
        // Bailout on -0.0
        int32 constant = ToInt32(rhs);
        if (mul->canBeNegativeZero() && constant <= 0) {
            Assembler::Condition bailoutCond = (constant == 0) ? Assembler::Signed : Assembler::Equal;
            masm.testl(ToRegister(lhs), ToRegister(lhs));
            if (!bailoutIf(bailoutCond, ins->snapshot()))
                    return false;
        }

        switch (constant) {
          case -1:
            masm.negl(ToOperand(lhs));
            break;
          case 0:
            masm.xorl(ToOperand(lhs), ToRegister(lhs));
            return true; // escape overflow check;
          case 1:
            // nop
            return true; // escape overflow check;
          case 2:
            masm.addl(ToOperand(lhs), ToRegister(lhs));
            break;
          default:
            if (!mul->canOverflow() && constant > 0) {
                // Use shift if cannot overflow and constant is power of 2
                int32 shift;
                JS_FLOOR_LOG2(shift, constant);
                if ((1 << shift) == constant) {
                    masm.shll(Imm32(shift), ToRegister(lhs));
                    return true;
                }
            }
            masm.imull(Imm32(ToInt32(rhs)), ToRegister(lhs));
        }

        // Bailout on overflow
        if (mul->canOverflow() && !bailoutIf(Assembler::Overflow, ins->snapshot()))
            return false;
    } else {
        masm.imull(ToOperand(rhs), ToRegister(lhs));

        // Bailout on overflow
        if (mul->canOverflow() && !bailoutIf(Assembler::Overflow, ins->snapshot()))
            return false;

        if (mul->canBeNegativeZero()) {
            // Jump to an OOL path if the result is 0.
            MulNegativeZeroCheck *ool = new MulNegativeZeroCheck(ins);
            if (!addOutOfLineCode(ool))
                return false;

            masm.testl(ToRegister(lhs), ToRegister(lhs));
            masm.j(Assembler::Zero, ool->entry());
            masm.bind(ool->rejoin());
        }
    }

    return true;
}

bool
CodeGeneratorMIPS::visitMulNegativeZeroCheck(MulNegativeZeroCheck *ool)
{
    LMulI *ins = ool->ins();
    Register result = ToRegister(ins->output());
    Operand lhsCopy = ToOperand(ins->lhsCopy());
    Operand rhs = ToOperand(ins->rhs());
    JS_ASSERT_IF(lhsCopy.kind() == Operand::REG, lhsCopy.reg() != result.code());

    // Result is -0 if lhs or rhs is negative.
    masm.movl(lhsCopy, result);
    masm.orl(rhs, result);
    if (!bailoutIf(Assembler::Signed, ins->snapshot()))
        return false;

    masm.xorl(result, result);
    masm.jmp(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitDivI(LDivI *ins)
{
    Register remainder = ToRegister(ins->remainder());
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());

    MDiv *mir = ins->mir();

    JS_ASSERT(remainder == t7);
    JS_ASSERT(lhs == t6);

    // Prevent divide by zero.
    if (mir->canBeDivideByZero()) {
        masm.testl(rhs, rhs);
        if (!bailoutIf(Assembler::Zero, ins->snapshot()))
            return false;
    }

    // Prevent an integer overflow exception from -2147483648 / -1.
    if (mir->canBeNegativeOverflow()) {
        Label notmin;
        masm.cmpl(lhs, Imm32(INT_MIN));
        masm.j(Assembler::NotEqual, &notmin);
        masm.cmpl(rhs, Imm32(-1));
        if (!bailoutIf(Assembler::Equal, ins->snapshot()))
            return false;
        masm.bind(&notmin);
    }

    // Prevent negative 0.
    if (mir->canBeNegativeZero()) {
        Label nonzero;
        masm.testl(lhs, lhs);
        masm.j(Assembler::NonZero, &nonzero);
        masm.cmpl(rhs, Imm32(0));
        if (!bailoutIf(Assembler::LessThan, ins->snapshot()))
            return false;
        masm.bind(&nonzero);
    }

    // Sign extend eax into edx to make (edx:eax), since idiv is 64-bit.
    masm.cdq();
    masm.idiv(rhs);

    if (!mir->isTruncated()) {
        // If the remainder is > 0, bailout since this must be a double.
        masm.testl(remainder, remainder);
        if (!bailoutIf(Assembler::NonZero, ins->snapshot()))
            return false;
    }

    return true;
}

bool
CodeGeneratorMIPS::visitModPowTwoI(LModPowTwoI *ins)
{
    Register lhs = ToRegister(ins->getOperand(0));
    int32 shift = ins->shift();
    Label negative, join;
    // Switch based on sign of the lhs.
    // Positive numbers are just a bitmask
    masm.branchTest32(Assembler::Signed, lhs, lhs, &negative);
    {
        masm.andl(Imm32((1 << shift) - 1), lhs);
        masm.jump(&join);
    }
    // Negative numbers need a negate, bitmask, negate
    {
        masm.bind(&negative);
        // visitModI has an overflow check here to catch INT_MIN % -1, but
        // here the rhs is a power of 2, and cannot be -1, so the check is not generated.
        masm.negl(lhs);
        masm.andl(Imm32((1 << shift) - 1), lhs);
        masm.negl(lhs);
        if (!bailoutIf(Assembler::Zero, ins->snapshot()))
            return false;
    }
    masm.bind(&join);
    return true;

}

bool
CodeGeneratorMIPS::visitModI(LModI *ins)
{
    Register remainder = ToRegister(ins->remainder());
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());

    // Required to use idiv.
    JS_ASSERT(remainder == t7);
    JS_ASSERT(lhs == t6);

    // If rhs == 0, bailout, since result must be a double (NaN).
    masm.testl(rhs, rhs);
    if (!bailoutIf(Assembler::Zero, ins->snapshot()))
        return false;

    Label negative, join;

    // Since the lhs will be made positive before reaching an idiv, instead of
    // sign extending eax into edx to 64-bit (edx:eax), we can simply zero it.
    masm.xorl(t7, t7);

    // Switch based on sign of the lhs.
    masm.branchTest32(Assembler::Signed, lhs, lhs, &negative);
    // If lhs >= 0 then remainder = lhs % rhs. The remainder must be positive.
    {
        masm.idiv(rhs);
        masm.jump(&join);
    }

    // If lhs < 0 then remainder = -(-lhs % rhs).
    {
        masm.bind(&negative);
        masm.negl(lhs);
        if (!bailoutIf(Assembler::Overflow, ins->snapshot()))
            return false;

        masm.idiv(rhs);

        // A remainder of 0 means that the rval must be -0, which is a double.
        masm.testl(remainder, remainder);
        if (!bailoutIf(Assembler::Zero, ins->snapshot()))
            return false; 

        // Cannot overflow.
        masm.negl(remainder);
    }

    masm.bind(&join);
    return true;
}

bool
CodeGeneratorMIPS::visitBitNotI(LBitNotI *ins)
{
    const LAllocation *input = ins->getOperand(0);
    JS_ASSERT(!input->isConstant());

    masm.notl(ToOperand(input));
    return true;
}

bool
CodeGeneratorMIPS::visitBitOpI(LBitOpI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);

    switch (ins->bitop()) {
        case JSOP_BITOR:
            if (rhs->isConstant())
                masm.orl(Imm32(ToInt32(rhs)), ToOperand(lhs));
            else
                masm.orl(ToOperand(rhs), ToRegister(lhs));
            break;
        case JSOP_BITXOR:
            if (rhs->isConstant())
                masm.xorl(Imm32(ToInt32(rhs)), ToOperand(lhs));
            else
                masm.xorl(ToOperand(rhs), ToRegister(lhs));
            break;
        case JSOP_BITAND:
            if (rhs->isConstant())
                masm.andl(Imm32(ToInt32(rhs)), ToOperand(lhs));
            else
                masm.andl(ToOperand(rhs), ToRegister(lhs));
            break;
        default:
            JS_NOT_REACHED("unexpected binary opcode");
    }

    return true;
}

bool
CodeGeneratorMIPS::visitShiftI(LShiftI *ins)
{
    Register lhs = ToRegister(ins->lhs());
    const LAllocation *rhs = ins->rhs();

    if (rhs->isConstant()) {
        int32_t shift = ToInt32(rhs) & 0x1F;
        switch (ins->bitop()) {
          case JSOP_LSH:
            if (shift)
                masm.shll(Imm32(shift), lhs);
            break;
          case JSOP_RSH:
            if (shift)
                masm.sarl(Imm32(shift), lhs);
            break;
          case JSOP_URSH:
            if (shift) {
                masm.shrl(Imm32(shift), lhs);
            } else if (ins->mir()->toUrsh()->canOverflow()) {
                // x >>> 0 can overflow.
                masm.testl(lhs, lhs);
                if (!bailoutIf(Assembler::Signed, ins->snapshot()))
                    return false;
            }
            break;
          default:
            JS_NOT_REACHED("Unexpected shift op");
        }
    } else {
        JS_ASSERT(ToRegister(rhs) == t8);
        switch (ins->bitop()) {
          case JSOP_LSH:
            masm.shll_cl(lhs);
            break;
          case JSOP_RSH:
            masm.sarl_cl(lhs);
            break;
          case JSOP_URSH:
            masm.shrl_cl(lhs);
            if (ins->mir()->toUrsh()->canOverflow()) {
                // x >>> 0 can overflow.
                masm.testl(lhs, lhs);
                if (!bailoutIf(Assembler::Signed, ins->snapshot()))
                    return false;
            }
            break;
          default:
            JS_NOT_REACHED("Unexpected shift op");
        }
    }

    return true;
}

bool
CodeGeneratorMIPS::visitUrshD(LUrshD *ins)
{
    Register lhs = ToRegister(ins->lhs());
    JS_ASSERT(ToRegister(ins->temp()) == lhs);

    const LAllocation *rhs = ins->rhs();
    FloatRegister out = ToFloatRegister(ins->output());

    if (rhs->isConstant()) {
        int32_t shift = ToInt32(rhs) & 0x1F;
        if (shift)
            masm.shrl(Imm32(shift), lhs);
    } else {
        JS_ASSERT(ToRegister(rhs) == t8);
        masm.shrl_cl(lhs);
    }

    masm.convertUInt32ToDouble(lhs, out);
    return true;
}

typedef MoveResolver::MoveOperand MoveOperand;

MoveOperand
CodeGeneratorMIPS::toMoveOperand(const LAllocation *a) const
{
    if (a->isGeneralReg())
        return MoveOperand(ToRegister(a));
    if (a->isFloatReg())
        return MoveOperand(ToFloatRegister(a));
    return MoveOperand(StackPointer, ToStackOffset(a));
}

bool
CodeGeneratorMIPS::visitMoveGroup(LMoveGroup *group)
{
    if (!group->numMoves())
        return true;

    MoveResolver &resolver = masm.moveResolver();

    for (size_t i = 0; i < group->numMoves(); i++) {
        const LMove &move = group->getMove(i);

        const LAllocation *from = move.from();
        const LAllocation *to = move.to();

        // No bogus moves.
        JS_ASSERT(*from != *to);
        JS_ASSERT(!from->isConstant());
        JS_ASSERT(from->isDouble() == to->isDouble());

        MoveResolver::Move::Kind kind = from->isDouble()
                                        ? MoveResolver::Move::DOUBLE
                                        : MoveResolver::Move::GENERAL;

        if (!resolver.addMove(toMoveOperand(from), toMoveOperand(to), kind))
            return false;
    }

    if (!resolver.resolve())
        return false;

    MoveEmitter emitter(masm);
    emitter.emit(resolver);
    emitter.finish();

    return true;
}

bool
CodeGeneratorMIPS::emitTableSwitchDispatch(MTableSwitch *mir, const Register &index,
                                                const Register &base)
{
    Label *defaultcase = mir->getDefault()->lir()->label();

    // Lower value with low value
    if (mir->low() != 0)
        masm.subl(Imm32(mir->low()), index);

    // Jump to default case if input is out of range
    int32 cases = mir->numCases();
    masm.cmpl(index, Imm32(cases));
    masm.j(Assembler::AboveOrEqual, defaultcase);

    // Create a JumpTable that during linking will get written.
    DeferredJumpTable *d = new DeferredJumpTable(mir);
    if (!masm.addDeferredData(d, (1 << ScalePointer) * cases))
        return false;

    // Compute the position where a pointer to the right case stands.
    masm.mov(d->label(), base);
    Operand pointer = Operand(base, index, ScalePointer);

    // Jump to the right case
    masm.jmp(pointer);

    return true;
}

bool
CodeGeneratorMIPS::visitMathD(LMathD *math)
{
    FloatRegister lhs = ToFloatRegister(math->lhs());
    Operand rhs = ToOperand(math->rhs());

    JS_ASSERT(ToFloatRegister(math->output()) == lhs);

    switch (math->jsop()) {
      case JSOP_ADD:
        masm.addsd(rhs, lhs);
        break;
      case JSOP_SUB:
        masm.subsd(rhs, lhs);
        break;
      case JSOP_MUL:
        masm.mulsd(rhs, lhs);
        break;
      case JSOP_DIV:
        masm.divsd(rhs, lhs);
        break;
      default:
        JS_NOT_REACHED("unexpected opcode");
        return false;
    }
    return true;
}

bool
CodeGeneratorMIPS::visitFloor(LFloor *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    FloatRegister scratch = ScratchFloatReg;
    Register output = ToRegister(lir->output());

//    if (Assembler::HasSSE41()) {
    if (true) {
        // Bail on negative-zero.
        Assembler::Condition bailCond = masm.testNegativeZero(input, output);
        if (!bailoutIf(bailCond, lir->snapshot()))
            return false;

        // Round toward -Infinity.
        masm.roundsd(input, scratch);

        masm.cvttsd2si(scratch, output);
        masm.cmp32(output, Imm32(INT_MIN));
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;
    } else {
        Label negative, end;

        // Branch to a slow path for negative inputs. Doesn't catch NaN or -0.
        masm.xorpd(scratch, scratch);
        masm.branchDouble(Assembler::DoubleLessThan, input, scratch, &negative);

        // Bail on negative-zero.
        Assembler::Condition bailCond = masm.testNegativeZero(input, output);
        if (!bailoutIf(bailCond, lir->snapshot()))
            return false;

        // Input is non-negative, so truncation correctly rounds.
        masm.cvttsd2si(input, output);
        masm.cmp32(output, Imm32(INT_MIN));
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;

        masm.jump(&end);

        // Input is negative, but isn't -0.
        // Negative values go on a comparatively expensive path, since no
        // native rounding mode matches JS semantics. Still better than callVM.
        masm.bind(&negative);
        {
            // Truncate and round toward zero.
            // This is off-by-one for everything but integer-valued inputs.
            masm.cvttsd2si(input, output);
            masm.cmp32(output, Imm32(INT_MIN));
            if (!bailoutIf(Assembler::Equal, lir->snapshot()))
                return false;
        
            // Test whether the input double was integer-valued.
            masm.cvtsi2sd(output, scratch);
            masm.branchDouble(Assembler::DoubleEqualOrUnordered, input, scratch, &end);

            // Input is not integer-valued, so we rounded off-by-one in the
            // wrong direction. Correct by subtraction.
            masm.subl(Imm32(1), output);
            // Cannot overflow: output was already checked against INT_MIN.
        }

        masm.bind(&end);
    }
    return true;
}

bool
CodeGeneratorMIPS::visitRound(LRound *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    FloatRegister temp = ToFloatRegister(lir->temp());
    FloatRegister scratch = ScratchFloatReg;
    Register output = ToRegister(lir->output());

    Label negative, end;

    // Load 0.5 in the temp register.
    static const double PointFive = 0.5;
    masm.loadStaticDouble(&PointFive, temp);

    // Branch to a slow path for negative inputs. Doesn't catch NaN or -0.
    masm.xorpd(scratch, scratch);
    masm.branchDouble(Assembler::DoubleLessThan, input, scratch, &negative);

    // Bail on negative-zero.
    Assembler::Condition bailCond = masm.testNegativeZero(input, output);
    if (!bailoutIf(bailCond, lir->snapshot()))
        return false;

    // Input is non-negative. Add 0.5 and truncate, rounding down. Note that we
    // have to add the input to the temp register (which contains 0.5) because
    // we're not allowed to modify the input register.
    masm.addsd(input, temp);

    masm.cvttsd2si(temp, output);
    masm.cmp32(output, Imm32(INT_MIN));
    if (!bailoutIf(Assembler::Equal, lir->snapshot()))
        return false;

    masm.jump(&end);


    // Input is negative, but isn't -0.
    masm.bind(&negative);

//    if (Assembler::HasSSE41()) {
    if (true) {
        // Add 0.5 and round toward -Infinity. The result is stored in the temp
        // register (currently contains 0.5).
        masm.addsd(input, temp);
        masm.roundsd(temp, scratch);

        // Truncate.
        masm.cvttsd2si(scratch, output);
        masm.cmp32(output, Imm32(INT_MIN));
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;

        // If the result is positive zero, then the actual result is -0. Bail.
        // Otherwise, the truncation will have produced the correct negative integer.
        masm.testl(output, output);
        if (!bailoutIf(Assembler::Zero, lir->snapshot()))
            return false;

    } else {
        masm.addsd(input, temp);

        // Round toward -Infinity without the benefit of ROUNDSD.
        Label testZero;
        {
            // Truncate and round toward zero.
            // This is off-by-one for everything but integer-valued inputs.
            masm.cvttsd2si(temp, output);
            masm.cmp32(output, Imm32(INT_MIN));
            if (!bailoutIf(Assembler::Equal, lir->snapshot()))
                return false;

            // Test whether the truncated double was integer-valued.
            masm.cvtsi2sd(output, scratch);
            masm.branchDouble(Assembler::DoubleEqualOrUnordered, temp, scratch, &testZero);

            // Input is not integer-valued, so we rounded off-by-one in the
            // wrong direction. Correct by subtraction.
            masm.subl(Imm32(1), output);
            // Cannot overflow: output was already checked against INT_MIN.

            // Fall through to testZero.
        }

        masm.bind(&testZero);
        if (!bailoutIf(Assembler::Zero, lir->snapshot()))
            return false;
    }

    masm.bind(&end);
    return true;
}

bool
CodeGeneratorMIPS::visitGuardShape(LGuardShape *guard)
{
    Register obj = ToRegister(guard->input());
    masm.cmpPtr(Operand(obj, JSObject::offsetOfShape()), ImmGCPtr(guard->mir()->shape()));
    if (!bailoutIf(Assembler::NotEqual, guard->snapshot()))
        return false;
    return true;
}

bool
CodeGeneratorMIPS::visitGuardClass(LGuardClass *guard)
{
    Register obj = ToRegister(guard->input());
    Register tmp = ToRegister(guard->tempInt());

    masm.loadBaseShape(obj, tmp);
    masm.cmpPtr(Operand(tmp, BaseShape::offsetOfClass()), ImmWord(guard->mir()->getClass()));
    if (!bailoutIf(Assembler::NotEqual, guard->snapshot()))
        return false;
    return true;
}

// Checks whether a double is representable as a 32-bit integer. If so, the
// integer is written to the output register. Otherwise, a bailout is taken to
// the given snapshot. This function overwrites the scratch float register.
void
CodeGeneratorMIPS::emitDoubleToInt32(const FloatRegister &src, const Register &dest, Label *fail, bool negativeZeroCheck)
{
    // Note that we don't specify the destination width for the truncated
    // conversion to integer. x64 will use the native width (quadword) which
    // sign-extends the top bits, preserving a little sanity.
    masm.cvttsd2s(src, dest);
    masm.cvtsi2sd(dest, ScratchFloatReg);
    masm.ucomisd(src, ScratchFloatReg);
    masm.j(Assembler::DoubleUnordered, fail);
    masm.j(Assembler::DoubleNotEqual, fail);

    // Check for -0
    if (negativeZeroCheck) {
        Label notZero;
        masm.testl(dest, dest);
        masm.j(Assembler::NonZero, &notZero);

//        if (Assembler::HasSSE41()) {
        if (false) {
            masm.ptest(src, src);
            masm.j(Assembler::NonZero, fail);
        } else {
            // bit 0 = sign of low double
            // bit 1 = sign of high double
            masm.movmskpd(src, dest);
            masm.andl(Imm32(1), dest);
            masm.j(Assembler::NonZero, fail);
        }

        masm.bind(&notZero);
    }
}

class OutOfLineTruncate : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    LTruncateDToInt32 *ins_;

  public:
    OutOfLineTruncate(LTruncateDToInt32 *ins)
      : ins_(ins)
    { }

    bool accept(CodeGeneratorMIPS *codegen) {
        return codegen->visitOutOfLineTruncate(this);
    }
    LTruncateDToInt32 *ins() const {
        return ins_;
    }
};

bool
CodeGeneratorMIPS::visitTruncateDToInt32(LTruncateDToInt32 *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    Register output = ToRegister(ins->output());

    OutOfLineTruncate *ool = new OutOfLineTruncate(ins);
    if (!addOutOfLineCode(ool))
        return false;

    masm.branchTruncateDouble(input, output, ool->entry());
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitOutOfLineTruncate(OutOfLineTruncate *ool)
{
    LTruncateDToInt32 *ins = ool->ins();
    FloatRegister input = ToFloatRegister(ins->input());
    FloatRegister temp = ToFloatRegister(ins->tempFloat());
    Register output = ToRegister(ins->output());

    // Try to convert doubles representing integers within 2^32 of a signed
    // integer, by adding/subtracting 2^32 and then trying to convert to int32.
    // This has to be an exact conversion, as otherwise the truncation works
    // incorrectly on the modified value.
    Label fail;
    masm.xorpd(ScratchFloatReg, ScratchFloatReg);
    masm.ucomisd(input, ScratchFloatReg);
    masm.j(Assembler::DoubleUnordered, &fail);

    {
        Label positive;
        masm.j(Assembler::Above, &positive);

        static const double shiftNeg = 4294967296.0;
        masm.loadStaticDouble(&shiftNeg, temp);
        Label skip;
        masm.jmp(&skip);

        masm.bind(&positive);
        static const double shiftPos = -4294967296.0;
        masm.loadStaticDouble(&shiftPos, temp);
        masm.bind(&skip);
    }

    masm.addsd(input, temp);
    masm.cvttsd2si(temp, output);
    masm.cvtsi2sd(output, ScratchFloatReg);

    masm.ucomisd(temp, ScratchFloatReg);
    masm.j(Assembler::DoubleUnordered, &fail);
    masm.j(Assembler::DoubleEqual, ool->rejoin());

    masm.bind(&fail);
    {
        saveVolatile(output);

        masm.setupUnalignedABICall(1, output);
        masm.passABIArg(input);
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, js::ToInt32));
        masm.storeCallResult(output);

        restoreVolatile(output);
    }

    masm.jump(ool->rejoin());
    return true;
}

Operand
CodeGeneratorMIPS::createArrayElementOperand(Register elements, const LAllocation *index)
{
    if (index->isConstant())
        return Operand(elements, ToInt32(index) * sizeof(js::Value));

    return Operand(elements, ToRegister(index), TimesEight);
}
bool
CodeGeneratorMIPS::generateInvalidateEpilogue()
{
    // Ensure that there is enough space in the buffer for the OsiPoint
    // patching to occur. Otherwise, we could overwrite the invalidation
    // epilogue.
    for (size_t i = 0; i < sizeof(void *); i+= Assembler::nopSize())
        masm.nop();

    masm.bind(&invalidate_);

    // Push the Ion script onto the stack (when we determine what that pointer is).
    invalidateEpilogueData_ = masm.pushWithPatch(ImmWord(uintptr_t(-1)));
    IonCode *thunk = GetIonContext()->compartment->ionCompartment()->getInvalidationThunk();

    masm.call(thunk);

    // We should never reach this point in JIT code -- the invalidation thunk should
    // pop the invalidated JS frame and return directly to its caller.
    masm.breakpoint();
    return true;
}

//x86
using mozilla::DebugOnly;

static const uint32 FrameSizes[] = { 128, 256, 512, 1024 };

FrameSizeClass
FrameSizeClass::FromDepth(uint32 frameDepth)
{
    for (uint32 i = 0; i < JS_ARRAY_LENGTH(FrameSizes); i++) {
        if (frameDepth < FrameSizes[i])
            return FrameSizeClass(i);
    }

    return FrameSizeClass::None();
}

FrameSizeClass
FrameSizeClass::ClassLimit()
{
    return FrameSizeClass(JS_ARRAY_LENGTH(FrameSizes));
}

uint32
FrameSizeClass::frameSize() const
{
    JS_ASSERT(class_ != NO_FRAME_SIZE_CLASS_ID);
    JS_ASSERT(class_ < JS_ARRAY_LENGTH(FrameSizes));

    return FrameSizes[class_];
}

ValueOperand
CodeGeneratorMIPS::ToValue(LInstruction *ins, size_t pos)
{
    Register typeReg = ToRegister(ins->getOperand(pos + TYPE_INDEX));
    Register payloadReg = ToRegister(ins->getOperand(pos + PAYLOAD_INDEX));
    return ValueOperand(typeReg, payloadReg);
}

ValueOperand
CodeGeneratorMIPS::ToOutValue(LInstruction *ins)
{
    Register typeReg = ToRegister(ins->getDef(TYPE_INDEX));
    Register payloadReg = ToRegister(ins->getDef(PAYLOAD_INDEX));
    return ValueOperand(typeReg, payloadReg);
}

ValueOperand
CodeGeneratorMIPS::ToTempValue(LInstruction *ins, size_t pos)
{
    Register typeReg = ToRegister(ins->getTemp(pos + TYPE_INDEX));
    Register payloadReg = ToRegister(ins->getTemp(pos + PAYLOAD_INDEX));
    return ValueOperand(typeReg, payloadReg);
}

bool
CodeGeneratorMIPS::visitValue(LValue *value)
{
    const ValueOperand out = ToOutValue(value);
    masm.moveValue(value->value(), out);
    return true;
}

bool
CodeGeneratorMIPS::visitOsrValue(LOsrValue *value)
{
    const LAllocation *frame   = value->getOperand(0);
    const ValueOperand out     = ToOutValue(value);
    const ptrdiff_t frameOffset = value->mir()->frameOffset();

    masm.loadValue(Operand(ToRegister(frame), frameOffset), out);
    return true;
}

bool
CodeGeneratorMIPS::visitBox(LBox *box)
{
    const LDefinition *type = box->getDef(TYPE_INDEX);

    DebugOnly<const LAllocation *> a = box->getOperand(0);
    JS_ASSERT(!a->isConstant());

    // On x86, the input operand and the output payload have the same
    // virtual register. All that needs to be written is the type tag for
    // the type definition.
    masm.movl(Imm32(MIRTypeToTag(box->type())), ToRegister(type));
    return true;
}

bool
CodeGeneratorMIPS::visitBoxDouble(LBoxDouble *box)
{
    const LAllocation *in = box->getOperand(0);
    const ValueOperand out = ToOutValue(box);

    masm.boxDouble(ToFloatRegister(in), out);
    return true;
}

bool
CodeGeneratorMIPS::visitUnbox(LUnbox *unbox)
{
    // Note that for unbox, the type and payload indexes are switched on the
    // inputs.
    MUnbox *mir = unbox->mir();

    if (mir->fallible()) {
        masm.cmpl(ToOperand(unbox->type()), Imm32(MIRTypeToTag(mir->type())));
        if (!bailoutIf(Assembler::NotEqual, unbox->snapshot()))
            return false;
    }
    return true;
}

void
CodeGeneratorMIPS::linkAbsoluteLabels()
{
    JSScript *script = gen->info().script();
    IonCode *method = script->ion->method();

    for (size_t i = 0; i < deferredDoubles_.length(); i++) {
        DeferredDouble *d = deferredDoubles_[i];
        const Value &v = script->ion->getConstant(d->index());
        MacroAssembler::Bind(method, d->label(), &v);
    }
}

bool
CodeGeneratorMIPS::visitDouble(LDouble *ins)
{
    const LDefinition *out = ins->getDef(0);
    const LConstantIndex *cindex = ins->getOperand(0)->toConstantIndex();
    const Value &v = graph.getConstant(cindex->index());

    union DoublePun {
        uint64_t u;
        double d;
    } dpun;
    dpun.d = v.toDouble();

    if (0 && masm.maybeInlineDouble(dpun.u, ToFloatRegister(out)))
        return true;

    DeferredDouble *d = new DeferredDouble(cindex->index());
    if (!deferredDoubles_.append(d))
        return false;

    masm.movsd(d->label(), ToFloatRegister(out));
    return true;
}

bool
CodeGeneratorMIPS::visitLoadSlotV(LLoadSlotV *load)
{
    const ValueOperand out = ToOutValue(load);
    Register base = ToRegister(load->input());
    int32 offset = load->mir()->slot() * sizeof(js::Value);

    masm.loadValue(Operand(base, offset), out);
    return true;
}

bool
CodeGeneratorMIPS::visitLoadSlotT(LLoadSlotT *load)
{
    Register base = ToRegister(load->input());
    int32 offset = load->mir()->slot() * sizeof(js::Value);

    if (load->mir()->type() == MIRType_Double)
        masm.loadInt32OrDouble(Operand(base, offset), ToFloatRegister(load->output()));
    else
        masm.movl(Operand(base, offset + NUNBOX32_PAYLOAD_OFFSET), ToRegister(load->output()));
    return true;
}

bool
CodeGeneratorMIPS::visitStoreSlotT(LStoreSlotT *store)
{
    Register base = ToRegister(store->slots());
    int32 offset = store->mir()->slot() * sizeof(js::Value);

    const LAllocation *value = store->value();
    MIRType valueType = store->mir()->value()->type();

    if (store->mir()->needsBarrier())
        emitPreBarrier(Address(base, offset), store->mir()->slotType());

    if (valueType == MIRType_Double) {
        masm.movsd(ToFloatRegister(value), Operand(base, offset));
        return true;
    }

    // Store the type tag if needed.
    if (valueType != store->mir()->slotType())
        masm.storeTypeTag(ImmType(ValueTypeFromMIRType(valueType)), Operand(base, offset));

    // Store the payload.
    if (value->isConstant())
        masm.storePayload(*value->toConstant(), Operand(base, offset));
    else
        masm.storePayload(ToRegister(value), Operand(base, offset));

    return true;
}

bool
CodeGeneratorMIPS::visitLoadElementT(LLoadElementT *load)
{
    Operand source = createArrayElementOperand(ToRegister(load->elements()), load->index());

    if (load->mir()->type() == MIRType_Double)
        masm.loadInt32OrDouble(source, ToFloatRegister(load->output()));
    else
        masm.movl(masm.ToPayload(source), ToRegister(load->output()));

    JS_ASSERT(!load->mir()->needsHoleCheck());
    return true;
}

void
CodeGeneratorMIPS::storeElementTyped(const LAllocation *value, MIRType valueType, MIRType elementType,
                                    const Register &elements, const LAllocation *index)
{
    Operand dest = createArrayElementOperand(elements, index);

    if (valueType == MIRType_Double) {
        masm.movsd(ToFloatRegister(value), dest);
        return;
    }

    // Store the type tag if needed.
    if (valueType != elementType)
        masm.storeTypeTag(ImmType(ValueTypeFromMIRType(valueType)), dest);

    // Store the payload.
    if (value->isConstant())
        masm.storePayload(*value->toConstant(), dest);
    else
        masm.storePayload(ToRegister(value), dest);
}

bool
CodeGeneratorMIPS::visitImplicitThis(LImplicitThis *lir)
{
    Register callee = ToRegister(lir->callee());
    const ValueOperand out = ToOutValue(lir);

    // The implicit |this| is always |undefined| if the function's environment
    // is the current global.
    GlobalObject *global = &gen->info().script()->global();
    masm.cmpPtr(Operand(callee, JSFunction::offsetOfEnvironment()), ImmGCPtr(global));

    // TODO: OOL stub path.
    if (!bailoutIf(Assembler::NotEqual, lir->snapshot()))
        return false;

    masm.moveValue(UndefinedValue(), out);
    return true;
}

bool
CodeGeneratorMIPS::visitRecompileCheck(LRecompileCheck *lir)
{
    // Bump the script's use count and bailout if the script is hot. Note that
    // it's safe to bake in this pointer since scripts are never nursery
    // allocated and jitcode will be purged before doing a compacting GC.
    // Without this assumption we'd need a temp register here.
    Operand addr(gen->info().script()->addressOfUseCount());
    masm.addl(Imm32(1), addr);
    masm.cmpl(addr, Imm32(lir->mir()->minUses()));
    if (!bailoutIf(Assembler::AboveOrEqual, lir->snapshot()))
        return false;
    return true;
}

typedef bool (*InterruptCheckFn)(JSContext *);
static const VMFunction InterruptCheckInfo = FunctionInfo<InterruptCheckFn>(InterruptCheck);

bool
CodeGeneratorMIPS::visitInterruptCheck(LInterruptCheck *lir)
{
    OutOfLineCode *ool = oolCallVM(InterruptCheckInfo, lir, (ArgList()), StoreNothing());
    if (!ool)
        return false;

    void *interrupt = (void*)&gen->compartment->rt->interrupt;
    masm.cmpl(Operand(interrupt), Imm32(0));
    masm.j(Assembler::NonZero, ool->entry());
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitCompareB(LCompareB *lir)
{
    MCompare *mir = lir->mir();

    const ValueOperand lhs = ToValue(lir, LCompareB::Lhs);
    const LAllocation *rhs = lir->rhs();
    const Register output = ToRegister(lir->output());

    JS_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    Label notBoolean, done;
    masm.branchTestBoolean(Assembler::NotEqual, lhs, &notBoolean);
    {
        if (rhs->isConstant())
            masm.cmp32(lhs.payloadReg(), Imm32(rhs->toConstant()->toBoolean()));
        else
            masm.cmp32(lhs.payloadReg(), ToRegister(rhs));
        emitSet(JSOpToCondition(mir->jsop()), output);
        masm.jump(&done);
    }
    masm.bind(&notBoolean);
    {
        masm.move32(Imm32(mir->jsop() == JSOP_STRICTNE), output);
    }

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareBAndBranch(LCompareBAndBranch *lir)
{
    MCompare *mir = lir->mir();
    const ValueOperand lhs = ToValue(lir, LCompareBAndBranch::Lhs);
    const LAllocation *rhs = lir->rhs();

    JS_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    if (mir->jsop() == JSOP_STRICTEQ)
        masm.branchTestBoolean(Assembler::NotEqual, lhs, lir->ifFalse()->lir()->label());
    else
        masm.branchTestBoolean(Assembler::NotEqual, lhs, lir->ifTrue()->lir()->label());

    if (rhs->isConstant())
        masm.cmp32(lhs.payloadReg(), Imm32(rhs->toConstant()->toBoolean()));
    else
        masm.cmp32(lhs.payloadReg(), ToRegister(rhs));
    emitBranch(JSOpToCondition(mir->jsop()), lir->ifTrue(), lir->ifFalse());
    return true;
}
} // namespace ion
} // namespace js
