/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_codegen_arm_h__
#define jsion_codegen_arm_h__

#include "Assembler-arm.h"
#include "ion/shared/CodeGenerator-shared.h"

namespace js {
namespace ion {

class OutOfLineBailout;

class CodeGeneratorARM : public CodeGeneratorShared
{
    friend class MoveResolverARM;

    CodeGeneratorARM *thisFromCtor() {return this;}

  protected:
    // Label for the common return path.
    HeapLabel *returnLabel_;
    HeapLabel *deoptLabel_;
    // ugh.  this is not going to be pretty to move over.
    // stack slotted variables are not useful on arm.
    // it looks like this will need to return one of two types.
    inline Operand ToOperand(const LAllocation &a) {
        if (a.isGeneralReg())
            return Operand(a.toGeneralReg()->reg());
        if (a.isFloatReg())
            return Operand(a.toFloatReg()->reg());
        return Operand(StackPointer, ToStackOffset(&a));
    }
    inline Operand ToOperand(const LAllocation *a) {
        return ToOperand(*a);
    }
    inline Operand ToOperand(const LDefinition *def) {
        return ToOperand(def->output());
    }

    MoveResolver::MoveOperand toMoveOperand(const LAllocation *a) const;

    bool bailoutIf(Assembler::Condition condition, LSnapshot *snapshot);
    bool bailoutFrom(Label *label, LSnapshot *snapshot);
    bool bailout(LSnapshot *snapshot);

  protected:
    bool generatePrologue();
    bool generateEpilogue();
    bool generateOutOfLineCode();

    void emitDoubleToInt32(const FloatRegister &src, const Register &dest, Label *fail, bool negativeZeroCheck = true);
    void emitRoundDouble(const FloatRegister &src, const Register &dest, Label *fail);

    // Emits a conditional set.
    void emitSet(Assembler::Condition cond, const Register &dest);

    // Emits a branch that directs control flow to the true block if |cond| is
    // true, and the false block if |cond| is false.
    void emitBranch(Assembler::Condition cond, MBasicBlock *ifTrue, MBasicBlock *ifFalse);

    bool emitTableSwitchDispatch(MTableSwitch *mir, const Register &index, const Register &base);

  public:
    CodeGeneratorARM(MIRGenerator *gen, LIRGraph *graph);

  public:
    // Instruction visitors.
    virtual bool visitMinMaxD(LMinMaxD *ins);
    virtual bool visitNegD(LNegD *ins);
    virtual bool visitAbsD(LAbsD *ins);
    virtual bool visitSqrtD(LSqrtD *ins);
    virtual bool visitPowHalfD(LPowHalfD *ins);
    virtual bool visitAddI(LAddI *ins);
    virtual bool visitSubI(LSubI *ins);
    virtual bool visitBitNotI(LBitNotI *ins);
    virtual bool visitBitOpI(LBitOpI *ins);

    virtual bool visitMulI(LMulI *ins);

    virtual bool visitDivI(LDivI *ins);
    virtual bool visitModI(LModI *ins);
    virtual bool visitModPowTwoI(LModPowTwoI *ins);
    virtual bool visitModMaskI(LModMaskI *ins);
    virtual bool visitShiftI(LShiftI *ins);
    virtual bool visitUrshD(LUrshD *ins);
    virtual bool visitMoveGroup(LMoveGroup *group);

    virtual bool visitTestIAndBranch(LTestIAndBranch *test);
    virtual bool visitTestDAndBranch(LTestDAndBranch *test);
    virtual bool visitCompare(LCompare *comp);
    virtual bool visitCompareAndBranch(LCompareAndBranch *comp);
    virtual bool visitCompareD(LCompareD *comp);
    virtual bool visitCompareDAndBranch(LCompareDAndBranch *comp);
    virtual bool visitNotI(LNotI *ins);
    virtual bool visitNotD(LNotD *ins);
    virtual bool visitCompareB(LCompareB *lir);
    virtual bool visitCompareBAndBranch(LCompareBAndBranch *lir);

    virtual bool visitMathD(LMathD *math);
    virtual bool visitFloor(LFloor *lir);
    virtual bool visitRound(LRound *lir);
    bool visitGuardShape(LGuardShape *guard);
    bool visitGuardClass(LGuardClass *guard);
    virtual bool visitTruncateDToInt32(LTruncateDToInt32 *ins);

    // Out of line visitors.
    bool visitOutOfLineBailout(OutOfLineBailout *ool);

  protected:
    ValueOperand ToValue(LInstruction *ins, size_t pos);
    ValueOperand ToOutValue(LInstruction *ins);
    ValueOperand ToTempValue(LInstruction *ins, size_t pos);

    // Functions for LTestVAndBranch.
    Register splitTagForTest(const ValueOperand &value);

    void storeElementTyped(const LAllocation *value, MIRType valueType, MIRType elementType,
                           const Register &elements, const LAllocation *index);

  protected:
    void linkAbsoluteLabels();

  public:
    bool visitBox(LBox *box);
    bool visitBoxDouble(LBoxDouble *box);
    bool visitUnbox(LUnbox *unbox);
    bool visitValue(LValue *value);
    bool visitOsrValue(LOsrValue *value);
    bool visitDouble(LDouble *ins);

    bool visitLoadSlotV(LLoadSlotV *load);
    bool visitLoadSlotT(LLoadSlotT *load);
    bool visitStoreSlotT(LStoreSlotT *load);

    bool visitLoadElementT(LLoadElementT *load);

    bool visitImplicitThis(LImplicitThis *lir);

    bool visitRecompileCheck(LRecompileCheck *lir);
    bool visitInterruptCheck(LInterruptCheck *lir);

    bool generateInvalidateEpilogue();
};

typedef CodeGeneratorARM CodeGeneratorSpecific;

// An out-of-line bailout thunk.
class OutOfLineBailout : public OutOfLineCodeBase<CodeGeneratorARM>
{
    LSnapshot *snapshot_;
    uint32 frameSize_;

  public:
    OutOfLineBailout(LSnapshot *snapshot, uint32 frameSize)
      : snapshot_(snapshot),
        frameSize_(frameSize)
    { }

    bool accept(CodeGeneratorARM *codegen);

    LSnapshot *snapshot() const {
        return snapshot_;
    }
};

} // namespace ion
} // namespace js

#endif // jsion_codegen_arm_h__

