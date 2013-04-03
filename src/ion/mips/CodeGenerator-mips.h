/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_codegen_mips_h__
#define jsion_codegen_mips_h__

#include "Assembler-mips.h"
#include "ion/shared/CodeGenerator-shared.h"

namespace js {
namespace ion {

class OutOfLineBailout;
class MulNegativeZeroCheck;
class OutOfLineTruncate;

enum NaNCond {
    NaN_Unexpected,
    NaN_IsTrue,
    NaN_IsFalse
};

class CodeGeneratorMIPS : public CodeGeneratorShared
{
    class DeferredDouble : public TempObject
    {
        AbsoluteLabel label_;
        uint32 index_;

      public:
        DeferredDouble(uint32 index) : index_(index)
        { }

        AbsoluteLabel *label() {
            return &label_;
        }
        uint32 index() const {
            return index_;
        }
    };

    friend class MoveResolverX86;

    CodeGeneratorMIPS *thisFromCtor() {
        return this;
    }

    template <typename T>
    bool bailout(const T &t, LSnapshot *snapshot);

  private: //x86
    js::Vector<DeferredDouble *, 0, SystemAllocPolicy> deferredDoubles_;

  protected: //x86
    ValueOperand ToValue(LInstruction *ins, size_t pos);
    ValueOperand ToOutValue(LInstruction *ins);
    ValueOperand ToTempValue(LInstruction *ins, size_t pos);

    void storeElementTyped(const LAllocation *value, MIRType valueType, MIRType elementType,
                           const Register &elements, const LAllocation *index);

  protected: //x86
    void linkAbsoluteLabels();

  public: //x86
    bool visitBox(LBox *box);
    bool visitBoxDouble(LBoxDouble *box);
    bool visitUnbox(LUnbox *unbox);
    bool visitValue(LValue *value);
    bool visitOsrValue(LOsrValue *value);
    bool visitDouble(LDouble *ins);
    bool visitLoadSlotV(LLoadSlotV *load);
    bool visitLoadSlotT(LLoadSlotT *load);
    bool visitStoreSlotT(LStoreSlotT *store);
    bool visitLoadElementT(LLoadElementT *load);
    bool visitImplicitThis(LImplicitThis *lir);
    bool visitRecompileCheck(LRecompileCheck *lir);
    bool visitInterruptCheck(LInterruptCheck *lir);
    bool visitCompareB(LCompareB *lir);
    bool visitCompareBAndBranch(LCompareBAndBranch *lir);

  protected:
    // Label for the common return path.
    HeapLabel *returnLabel_;
    HeapLabel *deoptLabel_;

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

    Operand createArrayElementOperand(Register elements, const LAllocation *index);

    void emitCompare(MIRType type, const LAllocation *left, const LAllocation *right);

    // Emits a conditional set.
    void emitSet(Assembler::Condition cond, const Register &dest, NaNCond ifNaN = NaN_Unexpected);
    void emitSet(Assembler::DoubleCondition cond, const Register &dest);

    // Emits a branch that directs control flow to the true block if |cond| is
    // true, and the false block if |cond| is false.
    void emitBranch(Assembler::Condition cond, MBasicBlock *ifTrue, MBasicBlock *ifFalse,
                    NaNCond ifNaN = NaN_Unexpected);
    void emitBranch(Assembler::DoubleCondition cond, MBasicBlock *ifTrue, MBasicBlock *ifFalse);

    bool emitTableSwitchDispatch(MTableSwitch *mir, const Register &index, const Register &base);

  public:
    CodeGeneratorMIPS(MIRGenerator *gen, LIRGraph *graph);

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
    virtual bool visitShiftI(LShiftI *ins);
    virtual bool visitUrshD(LUrshD *ins);
    virtual bool visitMoveGroup(LMoveGroup *group);
    virtual bool visitTestIAndBranch(LTestIAndBranch *test);
    virtual bool visitTestDAndBranch(LTestDAndBranch *test);
    virtual bool visitCompare(LCompare *comp);
    virtual bool visitCompareAndBranch(LCompareAndBranch *comp);
    virtual bool visitCompareD(LCompareD *comp);
    virtual bool visitCompareDAndBranch(LCompareDAndBranch *comp);
    virtual bool visitNotI(LNotI *comp);
    virtual bool visitNotD(LNotD *comp);
    virtual bool visitMathD(LMathD *math);
    virtual bool visitFloor(LFloor *lir);
    virtual bool visitRound(LRound *lir);
    virtual bool visitGuardShape(LGuardShape *guard);
    virtual bool visitGuardClass(LGuardClass *guard);
    virtual bool visitTruncateDToInt32(LTruncateDToInt32 *ins);

    // Out of line visitors.
    bool visitOutOfLineBailout(OutOfLineBailout *ool);
    bool visitMulNegativeZeroCheck(MulNegativeZeroCheck *ool);
    bool visitOutOfLineTruncate(OutOfLineTruncate *ool);
    bool generateInvalidateEpilogue();
};

// An out-of-line bailout thunk.
class OutOfLineBailout : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    LSnapshot *snapshot_;
    uint32 frameSize_;

  public:
    //OutOfLineBailout(LSnapshot *snapshot)
    OutOfLineBailout(LSnapshot *snapshot, uint32 frameSize)
      : snapshot_(snapshot),
        frameSize_(frameSize)
    { }

    bool accept(CodeGeneratorMIPS *codegen);

    LSnapshot *snapshot() const {
        return snapshot_;
    }
};

typedef CodeGeneratorMIPS CodeGeneratorSpecific;

} // namespace ion
} // namespace js

#endif // jsion_codegen_x86_shared_h__

