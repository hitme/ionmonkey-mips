/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_macro_assembler_mips_h__
#define jsion_macro_assembler_mips_h__

#include "ion/mips/Assembler-mips.h"
#include "ion/IonCaches.h"
#include "jsopcode.h"

#include "ion/IonFrames.h"
#include "ion/MoveResolver.h"

#include "jscompartment.h"

namespace js {
namespace ion {

class MacroAssemblerMIPS : public Assembler
{
    // Number of bytes the stack is adjusted inside a call to C. Calls to C may
    // not be nested.
    bool inCall_;
    uint32 args_;
    uint32 passedArgs_;
    uint32 stackForCall_;
    bool dynamicAlignment_;
    bool enoughMemory_;

  protected:
    MoveResolver moveResolver_;

  private:
    Operand payloadOf(const Address &address) {
        return Operand(address.base, address.offset);
    }
    Operand tagOf(const Address &address) {
        return Operand(address.base, address.offset + 4);
    }
    Operand tagOf(const BaseIndex &address) {//TBD BaseIndex special treat
        return Operand(address.base, address.index, address.scale, address.offset + 4);
    }

    void setupABICall(uint32 args);

  public:

    enum Result {
        GENERAL,
        DOUBLE
    };

    typedef MoveResolver::MoveOperand MoveOperand;
    typedef MoveResolver::Move Move;

    bool oom() const {
        return Assembler::oom() || !enoughMemory_;
    }

    /////////////////////////////////////////////////////////////////
    // X86-specific interface.
    /////////////////////////////////////////////////////////////////

    Operand ToPayload(Operand base) {
        return base;
    }
    Operand ToType(Operand base) {
        switch (base.kind()) {
          case Operand::REG_DISP:
            return Operand(Register::FromCode(base.base()), base.disp() + sizeof(void *));

          case Operand::SCALE:
            return Operand(Register::FromCode(base.base()), Register::FromCode(base.index()),
                           base.scale(), base.disp() + sizeof(void *));

          default:
            JS_NOT_REACHED("unexpected operand kind");
            return base; // Silence GCC warning.
        }
    }
    void moveValue(const Value &val, Register type, Register data) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        movl(Imm32(jv.s.tag), type);
        if (val.isMarkable())
            movl(ImmGCPtr(reinterpret_cast<gc::Cell *>(val.toGCThing())), data);
        else
            movl(Imm32(jv.s.payload.i32), data);
    }
    void moveValue(const Value &val, const ValueOperand &dest) {
        moveValue(val, dest.typeReg(), dest.payloadReg());
    }

    /////////////////////////////////////////////////////////////////
    // X86/X64-common interface.
    /////////////////////////////////////////////////////////////////
    void storeValue(ValueOperand val, Operand dest) {
        movl(val.payloadReg(), ToPayload(dest));
        movl(val.typeReg(), ToType(dest));
    }
    void storeValue(ValueOperand val, const Address &dest) {
        storeValue(val, Operand(dest));
    }
    template <typename T>
    void storeValue(JSValueType type, Register reg, const T &dest) {
        storeTypeTag(ImmTag(JSVAL_TYPE_TO_TAG(type)), Operand(dest));
        storePayload(reg, Operand(dest));
    }
    template <typename T>
    void storeValue(const Value &val, const T &dest) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        storeTypeTag(ImmTag(jv.s.tag), Operand(dest));
        storePayload(val, Operand(dest));
    }
    void storeValue(ValueOperand val, BaseIndex dest) {//TBD BaseIndex special treat
        storeValue(val, Operand(dest));
    }
    void loadValue(Operand src, ValueOperand val) {
        Operand payload = ToPayload(src);
        Operand type = ToType(src);

        // Ensure that loading the payload does not erase the pointer to the
        // Value in memory or the index.
        Register baseReg = Register::FromCode(src.base());
        Register indexReg = (src.kind() == Operand::SCALE) ? Register::FromCode(src.index()) : InvalidReg;

        if (baseReg == val.payloadReg() || indexReg == val.payloadReg()) {
            JS_ASSERT(baseReg != val.typeReg());
            JS_ASSERT(indexReg != val.typeReg());

            movl(type, val.typeReg());
            movl(payload, val.payloadReg());
        } else {
            JS_ASSERT(baseReg != val.payloadReg());
            JS_ASSERT(indexReg != val.payloadReg());

            movl(payload, val.payloadReg());
            movl(type, val.typeReg());
        }
    }
    void loadValue(Address src, ValueOperand val) {
        loadValue(Operand(src), val);
    }
    void loadValue(const BaseIndex &src, ValueOperand val) {//TBD BaseIndex special treat
        loadValue(Operand(src), val);
    }
    void tagValue(JSValueType type, Register payload, ValueOperand dest) {
        JS_ASSERT(payload != dest.typeReg());
        movl(ImmType(type), dest.typeReg());
        if (payload != dest.payloadReg())
            movl(payload, dest.payloadReg());
    }
    void pushValue(ValueOperand val) {
        push(val.typeReg());
        push(val.payloadReg());
    }
    void popValue(ValueOperand val) {
        pop(val.payloadReg());
        pop(val.typeReg());
    }
    void pushValue(const Value &val) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        push(Imm32(jv.s.tag));
        if (val.isMarkable())
            push(ImmGCPtr(reinterpret_cast<gc::Cell *>(val.toGCThing())));
        else
            push(Imm32(jv.s.payload.i32));
    }
    void pushValue(JSValueType type, Register reg) {
        push(ImmTag(JSVAL_TYPE_TO_TAG(type)));
        push(reg);
    }
    void storePayload(const Value &val, Operand dest) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        if (val.isMarkable())
            movl(ImmGCPtr((gc::Cell *)jv.s.payload.ptr), ToPayload(dest));
        else
            movl(Imm32(jv.s.payload.i32), ToPayload(dest));
    }
    void storePayload(Register src, Operand dest) {
        movl(src, ToPayload(dest));
    }
    void storeTypeTag(ImmTag tag, Operand dest) {
        movl(tag, ToType(dest));
    }

    void movePtr(const Register &src, const Register &dest) {
        movl(src, dest);
    }

    // Returns the register containing the type tag.
    Register splitTagForTest(const ValueOperand &value) {
        return value.typeReg();
    }

    Condition testUndefined(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_UNDEFINED));
        return cond;
    }
    Condition testBoolean(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_BOOLEAN));
        return cond;
    }
    Condition testInt32(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_INT32));
        return cond;
    }
    Condition testDouble(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
        Condition actual = (cond == Equal) ? Below : AboveOrEqual;
        cmpl(tag, ImmTag(JSVAL_TAG_CLEAR));
        return actual;
    }
    Condition testNull(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_NULL));
        return cond;
    }
    Condition testString(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_STRING));
        return cond;
    }
    Condition testObject(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_OBJECT));
        return cond;
    }
    Condition testNumber(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_UPPER_INCL_TAG_OF_NUMBER_SET));
        return cond == Equal ? BelowOrEqual : Above;
    }
    Condition testGCThing(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }
    Condition testGCThing(Condition cond, const Address &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }
    Condition testGCThing(Condition cond, const BaseIndex &address) {//TBD BaseIndex special treat
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }
    Condition testMagic(Condition cond, const Address &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testMagic(Condition cond, const BaseIndex &address) {//TBD BaseIndex special treat
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testMagic(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testPrimitive(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_UPPER_EXCL_TAG_OF_PRIMITIVE_SET));
        return cond == Equal ? Below : AboveOrEqual;
    }
    Condition testError(Condition cond, const Register &tag) {
        return testMagic(cond, tag);
    }
    Condition testInt32(Condition cond, const Operand &operand) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(ToType(operand), ImmTag(JSVAL_TAG_INT32));
        return cond;
    }
    Condition testUndefined(Condition cond, const ValueOperand &value) {
        return testUndefined(cond, value.typeReg());
    }
    Condition testBoolean(Condition cond, const ValueOperand &value) {
        return testBoolean(cond, value.typeReg());
    }
    Condition testInt32(Condition cond, const ValueOperand &value) {
        return testInt32(cond, value.typeReg());
    }
    Condition testDouble(Condition cond, const ValueOperand &value) {
        return testDouble(cond, value.typeReg());
    }
    Condition testNull(Condition cond, const ValueOperand &value) {
        return testNull(cond, value.typeReg());
    }
    Condition testString(Condition cond, const ValueOperand &value) {
        return testString(cond, value.typeReg());
    }
    Condition testObject(Condition cond, const ValueOperand &value) {
        return testObject(cond, value.typeReg());
    }
    Condition testMagic(Condition cond, const ValueOperand &value) {
        return testMagic(cond, value.typeReg());
    }
    Condition testError(Condition cond, const ValueOperand &value) {
        return testMagic(cond, value);
    }
    Condition testNumber(Condition cond, const ValueOperand &value) {
        return testNumber(cond, value.typeReg());
    }
    Condition testGCThing(Condition cond, const ValueOperand &value) {
        return testGCThing(cond, value.typeReg());
    }
    Condition testPrimitive(Condition cond, const ValueOperand &value) {
        return testPrimitive(cond, value.typeReg());
    }

    void cmpPtr(Register lhs, const ImmGCPtr rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Operand &lhs, const ImmWord rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Operand &lhs, const ImmGCPtr rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Address &lhs, Register rhs) {
        cmpl(Operand(lhs), rhs);
    }
    void cmpPtr(const Address &lhs, const ImmWord rhs) {
        cmpl(Operand(lhs), rhs);
    }
    void cmpPtr(Register lhs, Register rhs) {
        cmpl(lhs, rhs);
    }
    void testPtr(Register lhs, Register rhs) {
        testl(lhs, rhs);
    }

    Condition testNegativeZero(const FloatRegister &reg, const Register &scratch);

    /////////////////////////////////////////////////////////////////
    // Common interface.
    /////////////////////////////////////////////////////////////////
    void reserveStack(uint32 amount) {
        if (amount)
            subl(Imm32(amount), StackPointer);
        framePushed_ += amount;
    }
    void freeStack(uint32 amount) {
        JS_ASSERT(amount <= framePushed_);
        if (amount)
            addl(Imm32(amount), StackPointer);
        framePushed_ -= amount;
    }
    void freeStack(Register amount) {
        addl(amount, StackPointer);
    }

    void addPtr(const Register &src, const Register &dest) {
        addl(src, dest);
    }
    void addPtr(Imm32 imm, const Register &dest) {
        addl(imm, dest);
    }
    void addPtr(ImmWord imm, const Register &dest) {
        addl(Imm32(imm.value), dest);
    }
    void addPtr(Imm32 imm, const Address &dest) {
        addl(imm, Operand(dest));
    }
    void subPtr(Imm32 imm, const Register &dest) {
        subl(imm, dest);
    }

    template <typename T, typename S>
    void branchPtr(Condition cond, T lhs, S ptr, Label *label) {
        cmpl(Operand(lhs), ptr);
        j(cond, label);
    }

    template <typename T>
    void branchPrivatePtr(Condition cond, T lhs, ImmWord ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }

    template <typename T, typename S>
    void branchPtr(Condition cond, T lhs, S ptr, RepatchLabel *label) {
        cmpl(Operand(lhs), ptr);
        j(cond, label);
    }

    CodeOffsetJump jumpWithPatch(RepatchLabel *label) {
        jump(label);
        return CodeOffsetJump(size());
    }
    template <typename S, typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, S lhs, T ptr, RepatchLabel *label) {
        branchPtr(cond, lhs, ptr, label);
        return CodeOffsetJump(size());
    }
    void branchPtr(Condition cond, Register lhs, Register rhs, RepatchLabel *label) {
        cmpl(lhs, rhs);
        j(cond, label);
    }
    void branchPtr(Condition cond, Register lhs, Register rhs, Label *label) {
        cmpl(lhs, rhs);
        j(cond, label);
    }
    void branchTestPtr(Condition cond, Register lhs, Register rhs, Label *label) {
        testl(lhs, rhs);
        j(cond, label);
    }
    void decBranchPtr(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        subPtr(imm, lhs);
        j(cond, label);
    }

    void movePtr(ImmWord imm, Register dest) {
        movl(Imm32(imm.value), dest);
    }
    void movePtr(ImmGCPtr imm, Register dest) {
        movl(imm, dest);
    }
    void loadPtr(const Address &address, Register dest) {
        movl(Operand(address), dest);
    }
    void loadPtr(const BaseIndex &src, Register dest) {//TBD BaseIndex special treat
        movl(Operand(src), dest);
    }
    void loadPtr(const AbsoluteAddress &address, Register dest) {
        movl(Operand(address), dest);
    }
    void loadPrivate(const Address &src, Register dest) {
        movl(payloadOf(src), dest);
    }
    void storePtr(ImmWord imm, const Address &address) {
        movl(Imm32(imm.value), Operand(address));
    }
    void storePtr(ImmGCPtr imm, const Address &address) {
        movl(imm, Operand(address));
    }
    void storePtr(Register src, const Address &address) {
        movl(src, Operand(address));
    }
    void storePtr(Register src, const AbsoluteAddress &address) {
        movl(src, Operand(address));
    }

    void setStackArg(const Register &reg, uint32 arg) {
        movl(reg, Operand(sp, arg * STACK_SLOT_SIZE));
    }

    // Type testing instructions can take a tag in a register or a
    // ValueOperand.
    template <typename T>
    void branchTestUndefined(Condition cond, const T &t, Label *label) {
        cond = testUndefined(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestInt32(Condition cond, const T &t, Label *label) {
        cond = testInt32(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestBoolean(Condition cond, const T &t, Label *label) {
        cond = testBoolean(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestDouble(Condition cond, const T &t, Label *label) {
        cond = testDouble(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestNull(Condition cond, const T &t, Label *label) {
        cond = testNull(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestString(Condition cond, const T &t, Label *label) {
        cond = testString(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestObject(Condition cond, const T &t, Label *label) {
        cond = testObject(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestNumber(Condition cond, const T &t, Label *label) {
        cond = testNumber(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestGCThing(Condition cond, const T &t, Label *label) {
        cond = testGCThing(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestPrimitive(Condition cond, const T &t, Label *label) {
        cond = testPrimitive(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestMagic(Condition cond, const T &t, Label *label) {
        cond = testMagic(cond, t);
        j(cond, label);
    }
    void branchTestValue(Condition cond, const ValueOperand &value, const Value &v, Label *label);

    void cmpPtr(Register lhs, const ImmWord rhs) {
        cmpl(lhs, Imm32(rhs.value));
    }

    // Note: this function clobbers the source register.
    void boxDouble(const FloatRegister &src, const ValueOperand &dest) {
//        movd(src, dest.payloadReg());
//        psrldq(Imm32(4), src);
//        movd(src, dest.typeReg());
        fastStoreDouble(src, dest.payloadReg(), dest.typeReg());
    }
    void boxNonDouble(JSValueType type, const Register &src, const ValueOperand &dest) {
        if (src != dest.payloadReg())
            movl(src, dest.payloadReg());
        movl(ImmType(type), dest.typeReg());
    }
    void unboxInt32(const ValueOperand &src, const Register &dest) {
        movl(src.payloadReg(), dest);
    }
    void unboxInt32(const Address &src, const Register &dest) {
        movl(payloadOf(src), dest);
    }
    void unboxBoolean(const ValueOperand &src, const Register &dest) {
        movl(src.payloadReg(), dest);
    }
    void unboxBoolean(const Address &src, const Register &dest) {
        movl(payloadOf(src), dest);
    }
    void unboxDouble(const ValueOperand &src, const FloatRegister &dest) {
        JS_ASSERT(dest != ScratchFloatReg);
#if 0
//        if (0/*Assembler::HasSSE41()*/) {
//            movd(src.payloadReg(), dest);
//            pinsrd(src.typeReg(), dest);
//        } else {
//            movd(src.payloadReg(), dest);
//            movd(src.typeReg(), ScratchFloatReg);
//            unpcklps(ScratchFloatReg, dest);
//        }
#endif
        fastLoadDouble(src.payloadReg(), src.typeReg(), dest);
    }
    void unboxDouble(const Operand &payload, const Operand &type,
                     const Register &scratch, const FloatRegister &dest) {
        JS_ASSERT(dest != ScratchFloatReg);
        JS_ASSERT(0);
//        if (0/*Assembler::HasSSE41()*/) {
//            movl(payload, scratch);
//            movd(scratch, dest);
//            movl(type, scratch);
//            pinsrd(scratch, dest);
//        } else {
//            movl(payload, scratch);
//            movd(scratch, dest);
//            movl(type, scratch);
//            movd(scratch, ScratchFloatReg);
//            unpcklps(ScratchFloatReg, dest);
//        }
    }
    void unboxValue(const ValueOperand &src, AnyRegister dest) {
        if (dest.isFloat()) {
            Label notInt32, end;
            branchTestInt32(Assembler::NotEqual, src, &notInt32);
            cvtsi2sd(Operand(src.payloadReg()), dest.fpu());
            jump(&end);
            bind(&notInt32);
            unboxDouble(src, dest.fpu());
            bind(&end);
        } else {
            if (src.payloadReg() != dest.gpr())
                movl(src.payloadReg(), dest.gpr());
        }
    }
    void unboxPrivate(const ValueOperand &src, Register dest) {
        if (src.payloadReg() != dest)
            movl(src.payloadReg(), dest);
    }

    // Extended unboxing API. If the payload is already in a register, returns
    // that register. Otherwise, provides a move to the given scratch register,
    // and returns that.
    Register extractObject(const Address &address, Register scratch) {
        movl(payloadOf(address), scratch);
        return scratch;
    }
    Register extractObject(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
    Register extractTag(const Address &address, Register scratch) {
        movl(tagOf(address), scratch);
        return scratch;
    }
    Register extractTag(const ValueOperand &value, Register scratch) {
        return value.typeReg();
    }

    void boolValueToDouble(const ValueOperand &operand, const FloatRegister &dest) {
        cvtsi2sd(operand.payloadReg(), dest);
    }
    void int32ValueToDouble(const ValueOperand &operand, const FloatRegister &dest) {
        cvtsi2sd(operand.payloadReg(), dest);
    }

    void loadStaticDouble(const double *dp, const FloatRegister &dest) {
        movsd(dp, dest);
    }

    Condition testInt32Truthy(bool truthy, const ValueOperand &operand) {
        testl(operand.payloadReg(), operand.payloadReg());
        return truthy ? NonZero : Zero;
    }
    void branchTestBooleanTruthy(bool truthy, const ValueOperand &operand, Label *label) {
        testl(operand.payloadReg(), operand.payloadReg());
        j(truthy ? NonZero : Zero, label);
    }
    Condition testStringTruthy(bool truthy, const ValueOperand &value) {
        Register string = value.payloadReg();
        Operand lengthAndFlags(string, JSString::offsetOfLengthAndFlags());

        size_t mask = (0xFFFFFFFF << JSString::LENGTH_SHIFT);
        testl(lengthAndFlags, Imm32(mask));
        return truthy ? Assembler::NonZero : Assembler::Zero;
    }


    void loadInt32OrDouble(const Operand &operand, const FloatRegister &dest) {
        Label notInt32, end;
        branchTestInt32(Assembler::NotEqual, operand, &notInt32);
        cvtsi2sd(ToPayload(operand), dest);
        jump(&end);
        bind(&notInt32);
        movsd(operand, dest);
        bind(&end);
    }

    template <typename T>
    void loadUnboxedValue(const T &src, MIRType type, AnyRegister dest) {
        if (dest.isFloat())
            loadInt32OrDouble(Operand(src), dest.fpu());
        else
            movl(Operand(src), dest.gpr());
    }

    void rshiftPtr(Imm32 imm, Register dest) {
        shrl(imm, dest);
    }
    void lshiftPtr(Imm32 imm, Register dest) {
        shll(imm, dest);
    }
    void orPtr(Imm32 imm, Register dest) {
        orl(imm, dest);
    }

    void loadInstructionPointerAfterCall(const Register &dest) {
        movl(Operand(StackPointer, 0x0), dest);
    }

    // Note: this function clobbers the source register.
    void convertUInt32ToDouble(const Register &src, const FloatRegister &dest) {
        // src is [0, 2^32-1]
        subl(Imm32(0x80000000), src);

        // Now src is [-2^31, 2^31-1] - int range, but not the same value.
        cvtsi2sd(src, dest);

        // dest is now a double with the int range.
        // correct the double value by adding 0x80000000.
        static const double NegativeOne = 2147483648.0;
        addsd(Operand(&NegativeOne), dest);
    }

    void inc64(AbsoluteAddress dest) {
        addl(Imm32(1), Operand(dest));
        Label noOverflow;
        j(NonZero, &noOverflow);
        addl(Imm32(1), Operand(dest.offset(4)));
        bind(&noOverflow);
    }

    // Setup a call to C/C++ code, given the number of general arguments it
    // takes. Note that this only supports cdecl.
    //
    // In order for alignment to work correctly, the MacroAssembler must have a
    // consistent view of the stack displacement. It is okay to call "push"
    // manually, however, if the stack alignment were to change, the macro
    // assembler should be notified before starting a call.
    void setupAlignedABICall(uint32 args);

    // Sets up an ABI call for when the alignment is not known. This may need a
    // scratch register.
    void setupUnalignedABICall(uint32 args, const Register &scratch);

    // Arguments must be assigned to a C/C++ call in order. They are moved
    // in parallel immediately before performing the call. This process may
    // temporarily use more stack, in which case esp-relative addresses will be
    // automatically adjusted. It is extremely important that esp-relative
    // addresses are computed *after* setupABICall(). Furthermore, no
    // operations should be emitted while setting arguments.
    void passABIArg(const MoveOperand &from);
    void passABIArg(const Register &reg);
    void passABIArg(const FloatRegister &reg);

    // Emits a call to a C/C++ function, resolving all argument moves.
    void callWithABI(void *fun, Result result = GENERAL);

    // Used from within an Exit frame to handle a pending exception.
    void handleException();

    void makeFrameDescriptor(Register frameSizeReg, FrameType type) {
        shll(Imm32(FRAMESIZE_SHIFT), frameSizeReg);
        orl(Imm32(type), frameSizeReg);
    }

    // Save an exit frame (which must be aligned to the stack pointer) to
    // ThreadData::ionTop.
    void linkExitFrame() {
        JSCompartment *compartment = GetIonContext()->compartment;
        movl(StackPointer, Operand(&compartment->rt->ionTop));
    }

    void callWithExitFrame(IonCode *target, Register dynStack);

    void enterOsr(Register calleeToken, Register code);

  protected://x86
    // Bytes pushed onto the frame by the callee; includes frameDepth_. This is
    // needed to compute offsets to stack slots while temporary space has been
    // reserved for unexpected spills or C++ function calls. It is maintained
    // by functions which track stack alignment, which for clear distinction
    // use StudlyCaps (for example, Push, Pop).
    uint32 framePushed_;

  public://x86
    MacroAssemblerMIPS()
      : inCall_(false),
        framePushed_(0),
        enoughMemory_(true)
    {
    }

    void compareDouble(DoubleCondition cond, const FloatRegister &lhs, const FloatRegister &rhs) {
        if (cond & DoubleConditionBitInvert)
            ucomisd(rhs, lhs);
        else
            ucomisd(lhs, rhs);
    }
    void branchDouble(DoubleCondition cond, const FloatRegister &lhs,
                      const FloatRegister &rhs, Label *label)
    {
        compareDouble(cond, lhs, rhs);

        if (cond == DoubleEqual) {
            Label unordered;
            j(Parity, &unordered);
            j(Equal, label);
            bind(&unordered);
            return;
        }
        if (cond == DoubleNotEqualOrUnordered) {
            j(NotEqual, label);
            j(Parity, label);
            return;
        }

        JS_ASSERT(!(cond & DoubleConditionBitSpecial));
        j(ConditionFromDoubleCondition(cond), label);
    }

    void move32(const Imm32 &imm, const Register &dest) {
        if (imm.value == 0)
            xorl(dest, dest);
        else
            movl(imm, dest);
    }
    void and32(const Imm32 &imm, const Register &dest) {
        andl(imm, dest);
    }
    void and32(const Imm32 &imm, const Address &dest) {
        andl(imm, Operand(dest));
    }
    void or32(const Imm32 &imm, const Register &dest) {
        orl(imm, dest);
    }
    void or32(const Imm32 &imm, const Address &dest) {
        orl(imm, Operand(dest));
    }
    void neg32(const Register &reg) {
        negl(reg);
    }
    void cmp32(const Register &lhs, const Imm32 &rhs) {
        cmpl(lhs, rhs);
    }
    void test32(const Register &lhs, const Register &rhs) {
        testl(lhs, rhs);
    }
    void cmp32(Register a, Register b) {
        cmpl(a, b);
    }
    void cmp32(const Operand &lhs, const Imm32 &rhs) {
        cmpl(lhs, rhs);
    }
    void cmp32(const Operand &lhs, const Register &rhs) {
        cmpl(lhs, rhs);
    }
    void add32(Imm32 imm, Register dest) {
        addl(imm, dest);
    }
    void add32(Imm32 imm, const Address &dest) {
        addl(imm, Operand(dest));
    }
    void sub32(Imm32 imm, Register dest) {
        subl(imm, dest);
    }

    void branch32(Condition cond, const Address &lhs, const Register &rhs, Label *label) {
        cmpl(Operand(lhs), rhs);
        j(cond, label);
    }
    void branch32(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        cmpl(Operand(lhs), imm);
        j(cond, label);
    }
    void branch32(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        cmpl(lhs, imm);
        j(cond, label);
    }
    void branch32(Condition cond, const Register &lhs, const Register &rhs, Label *label) {
        cmpl(lhs, rhs);
        j(cond, label);
    }
    void branchTest32(Condition cond, const Register &lhs, const Register &rhs, Label *label) {
        testl(lhs, rhs);
        j(cond, label);
    }
    void branchTest32(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        testl(lhs, imm);
        j(cond, label);
    }
    void branchTest32(Condition cond, const Address &address, Imm32 imm, Label *label) {
        testl(Operand(address), imm);
        j(cond, label);
    }

    // The following functions are exposed for use in platform-shared code.
    template <typename T>
    void Push(const T &t) {
        push(t);
        framePushed_ += STACK_SLOT_SIZE;
    }
    void Push(const FloatRegister &t) {
        push(t);
        framePushed_ += sizeof(double);
    }
    CodeOffsetLabel PushWithPatch(const ImmWord &word) {
        framePushed_ += sizeof(word.value);
        return pushWithPatch(word);
    }

    void Pop(const Register &reg) {
        pop(reg);
        framePushed_ -= STACK_SLOT_SIZE;
    }
    void implicitPop(uint32 args) {
        JS_ASSERT(args % STACK_SLOT_SIZE == 0);
        framePushed_ -= args;
    }
    uint32 framePushed() const {
        return framePushed_;
    }
    void setFramePushed(uint32 framePushed) {
        framePushed_ = framePushed;
    }

    void jump(Label *label) {
        jmp(label);
    }
    void jump(RepatchLabel *label) {
        jmp(label);
    }
    void jump(Register reg) {
        jmp(Operand(reg));
    }

    void convertInt32ToDouble(const Register &src, const FloatRegister &dest) {
        cvtsi2sd(Operand(src), dest);
    }
    DoubleCondition testDoubleTruthy(bool truthy, const FloatRegister &reg) {
        xorpd(ScratchFloatReg, ScratchFloatReg);
        ucomisd(ScratchFloatReg, reg);
        return truthy ? Assembler::DoubleNotEqual : Assembler::DoubleEqual;
    }
    void branchTruncateDouble(const FloatRegister &src, const Register &dest, Label *fail) {
        JS_STATIC_ASSERT(INT_MIN == int(0x80000000));
        cvttsd2si(src, dest);
        cmpl(dest, Imm32(INT_MIN));
        j(Assembler::Equal, fail);
    }
    void load8ZeroExtend(const Address &src, const Register &dest) {
        movzbl(Operand(src), dest);
    }
    void load8ZeroExtend(const BaseIndex &src, const Register &dest) {//TBD BaseIndex special treat
        movzbl(Operand(src), dest);
    }
    void load8SignExtend(const Address &src, const Register &dest) {
        movxbl(Operand(src), dest);
    }
    void load8SignExtend(const BaseIndex &src, const Register &dest) {//TBD BaseIndex special treat
        movxbl(Operand(src), dest);
    }
    template <typename S, typename T>
    void store8(const S &src, const T &dest) {
        movb(src, Operand(dest));
    }
    void load16ZeroExtend(const Address &src, const Register &dest) {
        movzwl(Operand(src), dest);
    }
    void load16ZeroExtend(const BaseIndex &src, const Register &dest) {//TBD BaseIndex special treat
        movzwl(Operand(src), dest);
    }
    template <typename S, typename T>
    void store16(const S &src, const T &dest) {
        movw(src, Operand(dest));
    }
    void load16SignExtend(const Address &src, const Register &dest) {
        movxwl(Operand(src), dest);
    }
    void load16SignExtend(const BaseIndex &src, const Register &dest) {//TBD BaseIndex special treat
        movxwl(Operand(src), dest);
    }
    void load32(const Address &address, Register dest) {
        movl(Operand(address), dest);
    }
    void load32(const BaseIndex &src, Register dest) {//TBD BaseIndex special treat
        movl(Operand(src), dest);
    }
    template <typename S, typename T>
    void store32(const S &src, const T &dest) {
        movl(src, Operand(dest));
    }
    void loadDouble(const Address &src, FloatRegister dest) {
        movsd(Operand(src), dest);
    }
    void loadDouble(const BaseIndex &src, FloatRegister dest) {//TBD BaseIndex special treat
        movsd(Operand(src), dest);
    }
    void storeDouble(FloatRegister src, const Address &dest) {
        movsd(src, Operand(dest));
    }
    void storeDouble(FloatRegister src, const BaseIndex &dest) {//TBD BaseIndex special treat
        movsd(src, Operand(dest));
    }
    void zeroDouble(FloatRegister reg) {
        zerod(reg);
    }
    void negDouble(FloatRegister src, FloatRegister dest) {
        negd(src, dest);
    }
    void addDouble(FloatRegister src, FloatRegister dest) {
        addsd(src, dest);
    }
    void convertDoubleToFloat(const FloatRegister &src, const FloatRegister &dest) {
        cvtsd2ss(src, dest);
    }
    void loadFloatAsDouble(const Register &src, FloatRegister dest) {
        movd(src, dest);
        cvtss2sd(dest, dest);
    }
    void loadFloatAsDouble(const Address &src, FloatRegister dest) {
        movss(Operand(src), dest);
        cvtss2sd(dest, dest);
    }
    void loadFloatAsDouble(const BaseIndex &src, FloatRegister dest) {//TBD BaseIndex special treat
        movss(Operand(src), dest);
        cvtss2sd(dest, dest);
    }
    void storeFloat(FloatRegister src, const Address &dest) {
        movss(src, Operand(dest));
    }
    void storeFloat(FloatRegister src, const BaseIndex &dest) {//TBD BaseIndex special treat
        movss(src, Operand(dest));
    }

    void clampIntToUint8(Register src, Register dest) {
        Label inRange, done;
        branchTest32(Assembler::Zero, src, Imm32(0xffffff00), &inRange);
        {
            Label negative;
            branchTest32(Assembler::Signed, src, src, &negative);
            {
                movl(Imm32(255), dest);
                jump(&done);
            }
            bind(&negative);
            {
                xorl(dest, dest);
                jump(&done);
            }
        }
        bind(&inRange);
        if (src != dest)
            movl(src, dest);
        bind(&done);
    }

    bool maybeInlineDouble(uint64_t u, const FloatRegister &dest) {
        // This implements parts of "13.4 Generating constants" of 
        // "2. Optimizing subroutines in assembly language" by Agner Fog.
        switch (u) {
          case 0x0000000000000000ULL: // 0.0
            xorpd(dest, dest);
            break;
          case 0x8000000000000000ULL: // -0.0
            pcmpeqw(dest, dest);
            psllq(Imm32(63), dest);
            break;
          case 0x3fe0000000000000ULL: // 0.5
            pcmpeqw(dest, dest);
            psllq(Imm32(55), dest);
            psrlq(Imm32(2), dest);
            break;
          case 0x3ff0000000000000ULL: // 1.0
            pcmpeqw(dest, dest);
            psllq(Imm32(54), dest);
            psrlq(Imm32(2), dest);
            break;
          case 0x3ff8000000000000ULL: // 1.5
            pcmpeqw(dest, dest);
            psllq(Imm32(53), dest);
            psrlq(Imm32(2), dest);
            break;
          case 0x4000000000000000ULL: // 2.0
            pcmpeqw(dest, dest);
            psllq(Imm32(63), dest);
            psrlq(Imm32(1), dest);
            break;
          case 0xc000000000000000ULL: // -2.0
            pcmpeqw(dest, dest);
            psllq(Imm32(62), dest);
            break;
          default:
            return false;
        }
        return true;
    }

    // Emit a JMP that can be toggled to a CMP. See ToggleToJmp(), ToggleToCmp().
    CodeOffsetLabel toggledJump(Label *label) {
        CodeOffsetLabel offset(size());
        jump(label);
        return offset;
    }

    template <typename T>
    void computeEffectiveAddress(const T &address, Register dest) {
        lea(Operand(address), dest);
    }

    // Builds an exit frame on the stack, with a return address to an internal
    // non-function. Returns offset to be passed to markSafepointAt().
    bool buildFakeExitFrame(const Register &scratch, uint32 *offset) {
        mozilla::DebugOnly<uint32> initialDepth = framePushed();

        CodeLabel *cl = new CodeLabel();
        if (!addCodeLabel(cl))
            return false;
        mov(cl->dest(), scratch);

        uint32 descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);
        Push(Imm32(descriptor));
        Push(scratch);

        bind(cl->src());
        *offset = currentOffset();

        JS_ASSERT(framePushed() == initialDepth + IonExitFrameLayout::Size());
        return true;
    }

    bool buildOOLFakeExitFrame(void *fakeReturnAddr) {
        uint32 descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);
        Push(Imm32(descriptor));
        Push(ImmWord(fakeReturnAddr));
        return true;
    }

    void callWithExitFrame(IonCode *target);

    void callIon(const Register &callee);

    void checkStackAlignment() {
        // Exists for ARM compatibility.
    }

    CodeOffsetLabel labelForPatch() {
        return CodeOffsetLabel(size());
    }
};

typedef MacroAssemblerMIPS MacroAssemblerSpecific;

} // namespace ion
} // namespace js

#endif // jsion_macro_assembler_x86_h__


