/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_lir_mips_h__
#define jsion_lir_mips_h__

namespace js {
namespace ion {

class LBox : public LInstructionHelper<2, 1, 0>
{
    MIRType type_;

  public:
    LIR_HEADER(Box);

    LBox(const LAllocation &in_payload, MIRType type)
      : type_(type)
    {
        setOperand(0, in_payload);
    }

    MIRType type() const {
        return type_;
    }
};

class LBoxDouble : public LInstructionHelper<2, 1, 1>
{
  public:
    LIR_HEADER(BoxDouble);

    LBoxDouble(const LAllocation &in, const LDefinition &temp) {
        setOperand(0, in);
        setTemp(0, temp);
    }
};

class LUnbox : public LInstructionHelper<1, 2, 0>
{
  public:
    LIR_HEADER(Unbox);

    MUnbox *mir() const {
        return mir_->toUnbox();
    }
    const LAllocation *payload() {
        return getOperand(0);
    }
    const LAllocation *type() {
        return getOperand(1);
    }
};

class LUnboxDouble : public LInstructionHelper<1, 2, 0>
{
  public:
    LIR_HEADER(UnboxDouble);

    static const size_t Input = 0;

    MUnbox *mir() const {
        return mir_->toUnbox();
    }
};

// Constant double.
class LDouble : public LInstructionHelper<1, 1, 0>
{
  public:
    LIR_HEADER(Double);

    LDouble(const LConstantIndex &cindex) {
        setOperand(0, cindex);
    }
};

class LDivI : public LBinaryMath<1>
{
  public:
    LIR_HEADER(DivI);

    LDivI(const LAllocation &lhs, const LAllocation &rhs, const LDefinition &temp) {
        setOperand(0, lhs);
        setOperand(1, rhs);
        setTemp(0, temp);
    }

    const LDefinition *remainder() {
        return getTemp(0);
    }
    MDiv *mir() const {
        return mir_->toDiv();
    }
};

class LModI : public LBinaryMath<1>
{
  public:
    LIR_HEADER(ModI);

    LModI(const LAllocation &lhs, const LAllocation &rhs) {
        setOperand(0, lhs);
        setOperand(1, rhs);
    }

    const LDefinition *remainder() {
        return getDef(0);
    }
};

class LModPowTwoI : public LInstructionHelper<1,1,0>
{
    const int32 shift_;

  public:
    LIR_HEADER(ModPowTwoI);

    LModPowTwoI(const LAllocation &lhs, int32 shift)
      : shift_(shift)
    {
        setOperand(0, lhs);
    }

    int32 shift() const {
        return shift_;
    }
    const LDefinition *remainder() {
        return getDef(0);
    }
};

// Double raised to a half power.
class LPowHalfD : public LInstructionHelper<1, 1, 1>
{
  public:
    LIR_HEADER(PowHalfD);
    LPowHalfD(const LAllocation &input, const LDefinition &temp) {
        setOperand(0, input);
        setTemp(0, temp);
    }

    const LAllocation *input() {
        return getOperand(0);
    }
    const LDefinition *temp() {
        return getTemp(0);
    }
    const LDefinition *output() {
        return getDef(0);
    }
};

// Takes a tableswitch with an integer to decide
class LTableSwitch : public LInstructionHelper<0, 1, 2>
{
  public:
    LIR_HEADER(TableSwitch);

    LTableSwitch(const LAllocation &in, const LDefinition &inputCopy,
                 const LDefinition &jumpTablePointer, MTableSwitch *ins)
    {
        setOperand(0, in);
        setTemp(0, inputCopy);
        setTemp(1, jumpTablePointer);
        setMir(ins);
    }

    MTableSwitch *mir() const {
        return mir_->toTableSwitch();
    }

    const LAllocation *index() {
        return getOperand(0);
    }
    const LAllocation *tempInt() {
        return getTemp(0)->output();
    }
    const LAllocation *tempPointer() {
        return getTemp(1)->output();
    }
};

// Takes a tableswitch with a value to decide
class LTableSwitchV : public LInstructionHelper<0, BOX_PIECES, 3>
{
  public:
    LIR_HEADER(TableSwitchV);

    LTableSwitchV(const LDefinition &inputCopy, const LDefinition &floatCopy,
                  const LDefinition &jumpTablePointer, MTableSwitch *ins)
    {
        setTemp(0, inputCopy);
        setTemp(1, floatCopy);
        setTemp(2, jumpTablePointer);
        setMir(ins);
    }

    MTableSwitch *mir() const {
        return mir_->toTableSwitch();
    }

    static const size_t InputValue = 0;

    const LAllocation *tempInt() {
        return getTemp(0)->output();
    }
    const LAllocation *tempFloat() {
        return getTemp(1)->output();
    }
    const LAllocation *tempPointer() {
        return getTemp(2)->output();
    }
};

// Guard against an object's shape.
class LGuardShape : public LInstructionHelper<0, 1, 0>
{
  public:
    LIR_HEADER(GuardShape);

    LGuardShape(const LAllocation &in) {
        setOperand(0, in);
    }
    const MGuardShape *mir() const {
        return mir_->toGuardShape();
    }
};

class LRecompileCheck : public LInstructionHelper<0, 0, 0>
{
  public:
    LIR_HEADER(RecompileCheck);

    const MRecompileCheck *mir() const {
        return mir_->toRecompileCheck();
    }
};

class LInterruptCheck : public LInstructionHelper<0, 0, 0>
{
  public:
    LIR_HEADER(InterruptCheck);
};

class LMulI : public LBinaryMath<0, 1>
{
  public:
    LIR_HEADER(MulI);

    LMulI(const LAllocation &lhs, const LAllocation &rhs, const LAllocation &lhsCopy) {
        setOperand(0, lhs);
        setOperand(1, rhs);
        setOperand(2, lhsCopy);
    }

    MMul *mir() {
        return mir_->toMul();
    }
    const LAllocation *lhsCopy() {
        return this->getOperand(2);
    }
};

} // namespace ion
} // namespace js

#endif // jsion_lir_x86_shared_h__


