/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_macro_assembler_arm_h__
#define jsion_macro_assembler_arm_h__

#include "ion/arm/Assembler-arm.h"
#include "ion/IonCaches.h"
#include "ion/IonFrames.h"
#include "ion/MoveResolver.h"
#include "jsopcode.h"

using mozilla::DebugOnly;

namespace js {
namespace ion {

static Register CallReg = ip;
static const int defaultShift = 3;
JS_STATIC_ASSERT(1 << defaultShift == sizeof(jsval));

// MacroAssemblerARM is inheriting form Assembler defined in Assembler-arm.{h,cpp}
class MacroAssemblerARM : public Assembler
{
  public:
    MacroAssemblerARM()
    { }

    void convertInt32ToDouble(const Register &src, const FloatRegister &dest);
    void convertUInt32ToDouble(const Register &src, const FloatRegister &dest);
    void convertDoubleToFloat(const FloatRegister &src, const FloatRegister &dest);
    void branchTruncateDouble(const FloatRegister &src, const Register &dest, Label *fail);

    void inc64(AbsoluteAddress dest);

    // somewhat direct wrappers for the low-level assembler funcitons
    // bitops
    // attempt to encode a virtual alu instruction using
    // two real instructions.
  private:
    bool alu_dbl(Register src1, Imm32 imm, Register dest, ALUOp op,
                 SetCond_ sc, Condition c);
  public:
    void ma_alu(Register src1, Operand2 op2, Register dest, ALUOp op,
                SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_alu(Register src1, Imm32 imm, Register dest,
                ALUOp op,
                SetCond_ sc =  NoSetCond, Condition c = Always);

    void ma_alu(Register src1, Operand op2, Register dest, ALUOp op,
                SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_nop();
    void ma_movPatchable(Imm32 imm, Register dest, Assembler::Condition c,
                         RelocStyle rs, Instruction *i = NULL);
    // These should likely be wrapped up as a set of macros
    // or something like that.  I cannot think of a good reason
    // to explicitly have all of this code.
    // ALU based ops
    // mov
    void ma_mov(Register src, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_mov(Imm32 imm, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_mov(const ImmGCPtr &ptr, Register dest);

    // Shifts (just a move with a shifting op2)
    void ma_lsl(Imm32 shift, Register src, Register dst);
    void ma_lsr(Imm32 shift, Register src, Register dst);
    void ma_asr(Imm32 shift, Register src, Register dst);
    void ma_ror(Imm32 shift, Register src, Register dst);
    void ma_rol(Imm32 shift, Register src, Register dst);
    // Shifts (just a move with a shifting op2)
    void ma_lsl(Register shift, Register src, Register dst);
    void ma_lsr(Register shift, Register src, Register dst);
    void ma_asr(Register shift, Register src, Register dst);
    void ma_ror(Register shift, Register src, Register dst);
    void ma_rol(Register shift, Register src, Register dst);

    // Move not (dest <- ~src)
    void ma_mvn(Imm32 imm, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);


    void ma_mvn(Register src1, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    // Negate (dest <- -src) implemented as rsb dest, src, 0
    void ma_neg(Register src, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    // and
    void ma_and(Register src, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_and(Register src1, Register src2, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_and(Imm32 imm, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_and(Imm32 imm, Register src1, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);



    // bit clear (dest <- dest & ~imm) or (dest <- src1 & ~src2)
    void ma_bic(Imm32 imm, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    // exclusive or
    void ma_eor(Register src, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_eor(Register src1, Register src2, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_eor(Imm32 imm, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_eor(Imm32 imm, Register src1, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);


    // or
    void ma_orr(Register src, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_orr(Register src1, Register src2, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_orr(Imm32 imm, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);

    void ma_orr(Imm32 imm, Register src1, Register dest,
                SetCond_ sc = NoSetCond, Condition c = Always);


    // arithmetic based ops
    // add with carry
    void ma_adc(Imm32 imm, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_adc(Register src, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_adc(Register src1, Register src2, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);

    // add
    void ma_add(Imm32 imm, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_add(Register src1, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_add(Register src1, Register src2, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_add(Register src1, Operand op, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_add(Register src1, Imm32 op, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);

    // subtract with carry
    void ma_sbc(Imm32 imm, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_sbc(Register src1, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_sbc(Register src1, Register src2, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);

    // subtract
    void ma_sub(Imm32 imm, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_sub(Register src1, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_sub(Register src1, Register src2, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_sub(Register src1, Operand op, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_sub(Register src1, Imm32 op, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);

    // reverse subtract
    void ma_rsb(Imm32 imm, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_rsb(Register src1, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_rsb(Register src1, Register src2, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_rsb(Register src1, Imm32 op2, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);

    // reverse subtract with carry
    void ma_rsc(Imm32 imm, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_rsc(Register src1, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);
    void ma_rsc(Register src1, Register src2, Register dest, SetCond_ sc = NoSetCond, Condition c = Always);

    // compares/tests
    // compare negative (sets condition codes as src1 + src2 would)
    void ma_cmn(Register src1, Imm32 imm, Condition c = Always);
    void ma_cmn(Register src1, Register src2, Condition c = Always);
    void ma_cmn(Register src1, Operand op, Condition c = Always);

    // compare (src - src2)
    void ma_cmp(Register src1, Imm32 imm, Condition c = Always);
    void ma_cmp(Register src1, ImmWord ptr, Condition c = Always);
    void ma_cmp(Register src1, ImmGCPtr ptr, Condition c = Always);
    void ma_cmp(Register src1, Operand op, Condition c = Always);
    void ma_cmp(Register src1, Register src2, Condition c = Always);


    // test for equality, (src1^src2)
    void ma_teq(Register src1, Imm32 imm, Condition c = Always);
    void ma_teq(Register src1, Register src2, Condition c = Always);
    void ma_teq(Register src1, Operand op, Condition c = Always);


    // test (src1 & src2)
    void ma_tst(Register src1, Imm32 imm, Condition c = Always);
    void ma_tst(Register src1, Register src2, Condition c = Always);
    void ma_tst(Register src1, Operand op, Condition c = Always);

    // multiplies.  For now, there are only two that we care about.
    void ma_mul(Register src1, Register src2, Register dest);
    void ma_mul(Register src1, Imm32 imm, Register dest);
    Condition ma_check_mul(Register src1, Register src2, Register dest, Condition cond);
    Condition ma_check_mul(Register src1, Imm32 imm, Register dest, Condition cond);

    // fast mod, uses scratch registers, and thus needs to be in the assembler
    // implicitly assumes that we can overwrite dest at the beginning of the sequence
    void ma_mod_mask(Register src, Register dest, Register hold, int32 shift);

    // memory
    // shortcut for when we know we're transferring 32 bits of data
    void ma_dtr(LoadStore ls, Register rn, Imm32 offset, Register rt,
                Index mode = Offset, Condition cc = Always);

    void ma_dtr(LoadStore ls, Register rn, Register rm, Register rt,
                Index mode = Offset, Condition cc = Always);


    void ma_str(Register rt, DTRAddr addr, Index mode = Offset, Condition cc = Always);
    void ma_str(Register rt, const Operand &addr, Index mode = Offset, Condition cc = Always);
    void ma_dtr(LoadStore ls, Register rt, const Operand &addr, Index mode, Condition cc);

    void ma_ldr(DTRAddr addr, Register rt, Index mode = Offset, Condition cc = Always);
    void ma_ldr(const Operand &addr, Register rt, Index mode = Offset, Condition cc = Always);

    void ma_ldrb(DTRAddr addr, Register rt, Index mode = Offset, Condition cc = Always);
    void ma_ldrh(EDtrAddr addr, Register rt, Index mode = Offset, Condition cc = Always);
    void ma_ldrsh(EDtrAddr addr, Register rt, Index mode = Offset, Condition cc = Always);
    void ma_ldrsb(EDtrAddr addr, Register rt, Index mode = Offset, Condition cc = Always);
    void ma_ldrd(EDtrAddr addr, Register rt, DebugOnly<Register> rt2, Index mode = Offset, Condition cc = Always);
    void ma_strb(Register rt, DTRAddr addr, Index mode = Offset, Condition cc = Always);
    void ma_strh(Register rt, EDtrAddr addr, Index mode = Offset, Condition cc = Always);
    void ma_strd(Register rt, DebugOnly<Register> rt2, EDtrAddr addr, Index mode = Offset, Condition cc = Always);
    // specialty for moving N bits of data, where n == 8,16,32,64
    void ma_dataTransferN(LoadStore ls, int size, bool IsSigned,
                          Register rn, Register rm, Register rt,
                          Index mode = Offset, Condition cc = Always);

    void ma_dataTransferN(LoadStore ls, int size, bool IsSigned,
                          Register rn, Imm32 offset, Register rt,
                          Index mode = Offset, Condition cc = Always);
    void ma_pop(Register r);
    void ma_push(Register r);

    void ma_vpop(VFPRegister r);
    void ma_vpush(VFPRegister r);

    // branches when done from within arm-specific code
    void ma_b(Label *dest, Condition c = Always, bool isPatchable = false);
    void ma_bx(Register dest, Condition c = Always);

    void ma_b(void *target, Relocation::Kind reloc, Condition c = Always);

    // this is almost NEVER necessary, we'll basically never be calling a label
    // except, possibly in the crazy bailout-table case.
    void ma_bl(Label *dest, Condition c = Always);

    //VFP/ALU
    void ma_vadd(FloatRegister src1, FloatRegister src2, FloatRegister dst);
    void ma_vsub(FloatRegister src1, FloatRegister src2, FloatRegister dst);

    void ma_vmul(FloatRegister src1, FloatRegister src2, FloatRegister dst);
    void ma_vdiv(FloatRegister src1, FloatRegister src2, FloatRegister dst);

    void ma_vneg(FloatRegister src, FloatRegister dest, Condition cc = Always);
    void ma_vmov(FloatRegister src, FloatRegister dest, Condition cc = Always);
    void ma_vabs(FloatRegister src, FloatRegister dest, Condition cc = Always);

    void ma_vsqrt(FloatRegister src, FloatRegister dest, Condition cc = Always);

    void ma_vimm(double value, FloatRegister dest, Condition cc = Always);

    void ma_vcmp(FloatRegister src1, FloatRegister src2, Condition cc = Always);
    void ma_vcmpz(FloatRegister src1, Condition cc = Always);

    // source is F64, dest is I32
    void ma_vcvt_F64_I32(FloatRegister src, FloatRegister dest, Condition cc = Always);
    void ma_vcvt_F64_U32(FloatRegister src, FloatRegister dest, Condition cc = Always);

    // source is I32, dest is F64
    void ma_vcvt_I32_F64(FloatRegister src, FloatRegister dest, Condition cc = Always);
    void ma_vcvt_U32_F64(FloatRegister src, FloatRegister dest, Condition cc = Always);

    void ma_vxfer(FloatRegister src, Register dest, Condition cc = Always);
    void ma_vxfer(FloatRegister src, Register dest1, Register dest2, Condition cc = Always);

    void ma_vxfer(VFPRegister src, Register dest, Condition cc = Always);
    void ma_vxfer(VFPRegister src, Register dest1, Register dest2, Condition cc = Always);

    void ma_vdtr(LoadStore ls, const Operand &addr, VFPRegister dest, Condition cc = Always);

    void ma_vldr(VFPAddr addr, VFPRegister dest, Condition cc = Always);
    void ma_vldr(const Operand &addr, VFPRegister dest, Condition cc = Always);

    void ma_vstr(VFPRegister src, VFPAddr addr, Condition cc = Always);
    void ma_vstr(VFPRegister src, const Operand &addr, Condition cc = Always);

    void ma_vstr(VFPRegister src, Register base, Register index, int32 shift = defaultShift, Condition cc = Always);
    // calls an Ion function, assumes that the stack is untouched (8 byte alinged)
    void ma_callIon(const Register reg);
    // callso an Ion function, assuming that sp has already been decremented
    void ma_callIonNoPush(const Register reg);
    // calls an ion function, assuming that the stack is currently not 8 byte aligned
    void ma_callIonHalfPush(const Register reg);

    void ma_call(void *dest);
};

class MacroAssemblerARMCompat : public MacroAssemblerARM
{
    // Number of bytes the stack is adjusted inside a call to C. Calls to C may
    // not be nested.
    bool inCall_;
    uint32 args_;
    // The actual number of arguments that were passed, used to assert that
    // the initial number of arguments declared was correct.
    uint32 passedArgs_;

    // ARM treats arguments as a vector in registers/memory, that looks like:
    // { r0, r1, r2, r3, [sp], [sp,+4], [sp,+8] ... }
    // usedSlots_ keeps track of how many of these have been used.
    // It bears a passing resemblance to passedArgs_, but a single argument
    // can effectively use between one and three slots depending on its size and
    // alignment requirements
    uint32 usedSlots_;
    bool dynamicAlignment_;

    bool enoughMemory_;
    VFPRegister floatArgsInGPR[2];
    // Compute space needed for the function call and set the properties of the
    // callee.  It returns the space which has to be allocated for calling the
    // function.
    //
    // arg            Number of arguments of the function.
    void setupABICall(uint32 arg);

  protected:
    MoveResolver moveResolver_;

    // Extra bytes currently pushed onto the frame beyond frameDepth_. This is
    // needed to compute offsets to stack slots while temporary space has been
    // reserved for unexpected spills or C++ function calls. It is maintained
    // by functions which track stack alignment, which for clear distinction
    // use StudlyCaps (for example, Push, Pop).
    uint32 framePushed_;
    void adjustFrame(int value) {
        setFramePushed(framePushed_ + value);
    }
  public:
    typedef MoveResolver::MoveOperand MoveOperand;
    typedef MoveResolver::Move Move;

    enum Result {
        GENERAL,
        DOUBLE
    };

    MacroAssemblerARMCompat()
      : inCall_(false),
        enoughMemory_(true),
        framePushed_(0)
    { }

    bool oom() const {
        return Assembler::oom() || !enoughMemory_;
    }

  public:
    using MacroAssemblerARM::call;

    // jumps + other functions that should be called from
    // non-arm specific code...
    // basically, an x86 front end on top of the ARM code.
    void j(Condition code , Label *dest)
    {
        as_b(dest, code);
    }
    void j(Label *dest)
    {
        as_b(dest, Always);
    }

    void mov(Register src, Register dest) {
        ma_mov(src, dest);
    }
    void mov(Imm32 imm, Register dest) {
        ma_mov(imm, dest);
    }
    void mov(ImmWord imm, Register dest) {
        ma_mov(Imm32(imm.value), dest);
    }
    void mov(Register src, Address dest) {
        JS_NOT_REACHED("NYI-IC");
    }
    void mov(Address src, Register dest) {
        JS_NOT_REACHED("NYI-IC");
    }

    void call(const Register reg) {
        as_blx(reg);
    }

    void call(Label *label) {
        JS_NOT_REACHED("Feature NYI");
        /* we can blx to it if it close by, otherwise, we need to
         * set up a branch + link node.
         */
    }
    void call(ImmWord word) {
        BufferOffset bo = m_buffer.nextOffset();
        addPendingJump(bo, (void*)word.value, Relocation::HARDCODED);
        ma_call((void *) word.value);
    }
    void call(IonCode *c) {
        BufferOffset bo = m_buffer.nextOffset();
        addPendingJump(bo, c->raw(), Relocation::IONCODE);
        ma_mov(Imm32((uint32)c->raw()), ScratchRegister);
        ma_callIonHalfPush(ScratchRegister);
    }
    void branch(IonCode *c) {
        BufferOffset bo = m_buffer.nextOffset();
        addPendingJump(bo, c->raw(), Relocation::IONCODE);
        ma_mov(Imm32((uint32)c->raw()), ScratchRegister);
        ma_bx(ScratchRegister);
    }
    void nop() {
        ma_nop();
    }
    void ret() {
        ma_pop(pc);
        m_buffer.markGuard();
    }
    void retn(Imm32 n) {
        // pc <- [sp]; sp += n
        ma_dtr(IsLoad, sp, n, pc, PostIndex);
        m_buffer.markGuard();
    }
    void push(Imm32 imm) {
        ma_mov(imm, ScratchRegister);
        ma_push(ScratchRegister);
    }
    void push(ImmWord imm) {
        push(Imm32(imm.value));
    }
    void push(ImmGCPtr imm) {
        ma_mov(imm, ScratchRegister);
        ma_push(ScratchRegister);
    }
    void push(const Register &reg) {
        ma_push(reg);
    }
    void pushWithPadding(const Register &reg, const Imm32 extraSpace) {
        Imm32 totSpace = Imm32(extraSpace.value + 4);
        ma_dtr(IsStore, sp, totSpace, reg, PreIndex);
    }
    void pushWithPadding(const Imm32 &imm, const Imm32 extraSpace) {
        Imm32 totSpace = Imm32(extraSpace.value + 4);
        // need to use lr, since ma_dtr may need the scratch register to adjust the stack.
        ma_mov(imm, lr);
        ma_dtr(IsStore, sp, totSpace, lr, PreIndex);
    }

    void pop(const Register &reg) {
        ma_pop(reg);
    }

    void popN(const Register &reg, Imm32 extraSpace) {
        Imm32 totSpace = Imm32(extraSpace.value + 4);
        ma_dtr(IsLoad, sp, totSpace, reg, PostIndex);
    }

    CodeOffsetLabel toggledJump(Label *label);

    CodeOffsetLabel pushWithPatch(ImmWord imm) {
        CodeOffsetLabel label = currentOffset();
        ma_movPatchable(Imm32(imm.value), ScratchRegister, Always, L_MOVWT);
        ma_push(ScratchRegister);
        return label;
    }

    void jump(Label *label) {
        as_b(label);
    }
    void jump(Register reg) {
        ma_bx(reg);
    }

    void neg32(Register reg) {
        ma_neg(reg, reg, SetCond);
    }
    void test32(Register lhs, Register rhs) {
        ma_tst(lhs, rhs);
    }
    void testPtr(Register lhs, Register rhs) {
        test32(lhs, rhs);
    }

    // Returns the register containing the type tag.
    Register splitTagForTest(const ValueOperand &value) {
        return value.typeReg();
    }

    // higher level tag testing code
    Condition testInt32(Condition cond, const ValueOperand &value);
    Condition testBoolean(Condition cond, const ValueOperand &value);
    Condition testDouble(Condition cond, const ValueOperand &value);
    Condition testNull(Condition cond, const ValueOperand &value);
    Condition testUndefined(Condition cond, const ValueOperand &value);
    Condition testString(Condition cond, const ValueOperand &value);
    Condition testObject(Condition cond, const ValueOperand &value);
    Condition testMagic(Condition cond, const ValueOperand &value);

    Condition testPrimitive(Condition cond, const ValueOperand &value);

    // register-based tests
    Condition testInt32(Condition cond, const Register &tag);
    Condition testBoolean(Condition cond, const Register &tag);
    Condition testNull(Condition cond, const Register &tag);
    Condition testUndefined(Condition cond, const Register &tag);
    Condition testString(Condition cond, const Register &tag);
    Condition testObject(Condition cond, const Register &tag);
    Condition testDouble(Condition cond, const Register &tag);
    Condition testNumber(Condition cond, const Register &tag);
    Condition testMagic(Condition cond, const Register &tag);
    Condition testPrimitive(Condition cond, const Register &tag);

    Condition testGCThing(Condition cond, const Address &address);
    Condition testGCThing(Condition cond, const BaseIndex &address);
    Condition testMagic(Condition cond, const Address &address);
    Condition testMagic(Condition cond, const BaseIndex &address);

    template <typename T>
    void branchTestGCThing(Condition cond, const T &t, Label *label) {
        Condition c = testGCThing(cond, t);
        ma_b(label, c);
    }
    template <typename T>
    void branchTestPrimitive(Condition cond, const T &t, Label *label) {
        Condition c = testPrimitive(cond, t);
        ma_b(label, c);
    }

    void branchTestValue(Condition cond, const ValueOperand &value, const Value &v, Label *label);

    // unboxing code
    void unboxInt32(const ValueOperand &operand, const Register &dest);
    void unboxInt32(const Address &src, const Register &dest);
    void unboxBoolean(const ValueOperand &operand, const Register &dest);
    void unboxBoolean(const Address &src, const Register &dest);
    void unboxDouble(const ValueOperand &operand, const FloatRegister &dest);
    void unboxValue(const ValueOperand &src, AnyRegister dest);
    void unboxPrivate(const ValueOperand &src, Register dest);

    // boxing code
    void boxDouble(const FloatRegister &src, const ValueOperand &dest);
    void boxNonDouble(JSValueType type, const Register &src, const ValueOperand &dest);

    // Extended unboxing API. If the payload is already in a register, returns
    // that register. Otherwise, provides a move to the given scratch register,
    // and returns that.
    Register extractObject(const Address &address, Register scratch);
    Register extractObject(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
    Register extractTag(const Address &address, Register scratch);
    Register extractTag(const BaseIndex &address, Register scratch);
    Register extractTag(const ValueOperand &value, Register scratch) {
        return value.typeReg();
    }

    void boolValueToDouble(const ValueOperand &operand, const FloatRegister &dest);
    void int32ValueToDouble(const ValueOperand &operand, const FloatRegister &dest);
    void loadInt32OrDouble(const Operand &src, const FloatRegister &dest);
    void loadInt32OrDouble(Register base, Register index,
                           const FloatRegister &dest, int32 shift = defaultShift);
    void loadStaticDouble(const double *dp, const FloatRegister &dest);
    void loadConstantDouble(double dp, const FloatRegister &dest);
    // treat the value as a boolean, and set condition codes accordingly
    Condition testInt32Truthy(bool truthy, const ValueOperand &operand);
    Condition testBooleanTruthy(bool truthy, const ValueOperand &operand);
    Condition testDoubleTruthy(bool truthy, const FloatRegister &reg);
    Condition testStringTruthy(bool truthy, const ValueOperand &value);

    template<typename T>
    void branchTestInt32(Condition cond, const T & t, Label *label) {
        Condition c = testInt32(cond, t);
        ma_b(label, c);
    }
    template<typename T>
    void branchTestBoolean(Condition cond, const T & t, Label *label) {
        Condition c = testBoolean(cond, t);
        ma_b(label, c);
    }
    void branch32(Condition cond, Register lhs, Register rhs, Label *label) {
        ma_cmp(lhs, rhs);
        ma_b(label, cond);
    }
    void branch32(Condition cond, Register lhs, Imm32 imm, Label *label) {
        ma_cmp(lhs, imm);
        ma_b(label, cond);
    }
    void branch32(Condition cond, const Address &lhs, Register rhs, Label *label) {
        load32(lhs, ScratchRegister);
        branch32(cond, ScratchRegister, rhs, label);
    }
    void branch32(Condition cond, const Address &lhs, Imm32 rhs, Label *label) {
        load32(lhs, ScratchRegister);
        branch32(cond, ScratchRegister, rhs, label);
    }
    void branchPtr(Condition cond, const Address &lhs, Register rhs, Label *label) {
        branch32(cond, lhs, rhs, label);
    }

    void branchPrivatePtr(Condition cond, const Address &lhs, ImmWord ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }

    void branchPrivatePtr(Condition cond, Register lhs, ImmWord ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }

    template<typename T>
    void branchTestDouble(Condition cond, const T & t, Label *label) {
        Condition c = testDouble(cond, t);
        ma_b(label, c);
    }
    template<typename T>
    void branchTestNull(Condition cond, const T & t, Label *label) {
        Condition c = testNull(cond, t);
        ma_b(label, c);
    }
    template<typename T>
    void branchTestObject(Condition cond, const T & t, Label *label) {
        Condition c = testObject(cond, t);
        ma_b(label, c);
    }
    template<typename T>
    void branchTestString(Condition cond, const T & t, Label *label) {
        Condition c = testString(cond, t);
        ma_b(label, c);
    }
    template<typename T>
    void branchTestUndefined(Condition cond, const T & t, Label *label) {
        Condition c = testUndefined(cond, t);
        ma_b(label, c);
    }
    template <typename T>
    void branchTestNumber(Condition cond, const T &t, Label *label) {
        cond = testNumber(cond, t);
        ma_b(label, cond);
    }
    template <typename T>
    void branchTestMagic(Condition cond, const T &t, Label *label) {
        cond = testMagic(cond, t);
        ma_b(label, cond);
    }
    template<typename T>
    void branchTestBooleanTruthy(bool b, const T & t, Label *label) {
        Condition c = testBooleanTruthy(b, t);
        ma_b(label, c);
    }
    void branchTest32(Condition cond, const Register &lhs, const Register &rhs, Label *label) {
        // x86 likes test foo, foo rather than cmp foo, #0.
        // Convert the former into the latter.
        if (lhs == rhs && (cond == Zero || cond == NonZero))
            ma_cmp(lhs, Imm32(0));
        else
            ma_tst(lhs, rhs);
        ma_b(label, cond);
    }
    void branchTest32(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        ma_tst(lhs, imm);
        ma_b(label, cond);
    }
    void branchTest32(Condition cond, const Address &address, Imm32 imm, Label *label) {
        ma_ldr(Operand(address.base, address.offset), ScratchRegister);
        branchTest32(cond, ScratchRegister, imm, label);
    }
    void branchTestPtr(Condition cond, const Register &lhs, const Register &rhs, Label *label) {
        branchTest32(cond, lhs, rhs, label);
    }
    void branchPtr(Condition cond, Register lhs, Register rhs, Label *label) {
        branch32(cond, lhs, rhs, label);
    }
    void branchPtr(Condition cond, Register lhs, ImmGCPtr ptr, Label *label) {
        movePtr(ptr, ScratchRegister);
        branchPtr(cond, lhs, ScratchRegister, label);
    }
    void branchPtr(Condition cond, Register lhs, ImmWord imm, Label *label) {
        branch32(cond, lhs, Imm32(imm.value), label);
    }
    void decBranchPtr(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        subPtr(imm, lhs);
        branch32(cond, lhs, Imm32(0), label);
    }
    void moveValue(const Value &val, Register type, Register data);

    CodeOffsetJump jumpWithPatch(RepatchLabel *label, Condition cond = Always);
    template <typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, Register reg, T ptr, RepatchLabel *label) {
        ma_cmp(reg, ptr);
        return jumpWithPatch(label, cond);
    }
    template <typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, Address addr, T ptr, RepatchLabel *label) {
        // if (compare(*addr, ptr) == cond) goto label
        // with only one temp reg, this is downright PAINFUL.
        // HAXHAXHAXHAXHAX I've reserved lr for private use until I figure out how
        // to teach the register allocator that it is more volatile than most.
        // In the end, this will require us to take an extra temp register and pass it in
        // (or spend 4 instructions making the comparison :()
        // since I wish to get this tested, I'm using a register that should not be used.
        // when everything breaks, this is probably the cause
        // See also branchPtr.
        ma_ldr(addr, lr);
        ma_cmp(lr, ptr);
        return jumpWithPatch(label, cond);
    }
    void branchPtr(Condition cond, Address addr, ImmGCPtr ptr, Label *label) {
        // See the comment in branchPtrWithPatch.
        ma_ldr(addr, lr);
        ma_cmp(lr, ptr);
        ma_b(label, cond);
    }
    void branchPtr(Condition cond, Address addr, ImmWord ptr, Label *label) {
        // See the comment in branchPtrWithPatch.
        ma_ldr(addr, lr);
        ma_cmp(lr, ptr);
        ma_b(label, cond);
    }
    void branchPtr(Condition cond, const AbsoluteAddress &addr, const Register &ptr, Label *label) {
        loadPtr(addr, lr); // ma_cmp will use the scratch register.
        ma_cmp(lr, ptr);
        ma_b(label, cond);
    }

    void loadUnboxedValue(Address address, MIRType type, AnyRegister dest) {
        if (dest.isFloat())
            loadInt32OrDouble(Operand(address), dest.fpu());
        else
            ma_ldr(address, dest.gpr());
    }

    void loadUnboxedValue(BaseIndex address, MIRType type, AnyRegister dest) {
        if (dest.isFloat())
            loadInt32OrDouble(address.base, address.index, dest.fpu(), address.scale);
        else
            load32(address, dest.gpr());
    }

    void moveValue(const Value &val, const ValueOperand &dest);

    void storeValue(ValueOperand val, Operand dst);
    void storeValue(ValueOperand val, Register base, Register index, int32 shift = defaultShift);
    void storeValue(ValueOperand val, const Address &dest) {
        storeValue(val, Operand(dest));
    }
    void storeValue(JSValueType type, Register reg, Address dest) {
        ma_mov(ImmTag(JSVAL_TYPE_TO_TAG(type)), lr);
        ma_str(lr, Address(dest.base, dest.offset + 4));
        ma_str(reg, dest);
    }
    void storeValue(JSValueType type, Register reg, BaseIndex dest) {
        // Harder cases not handled yet.
        JS_ASSERT(dest.offset == 0);
        ma_alu(dest.base, lsl(dest.index, dest.scale), ScratchRegister, op_add);
        storeValue(type, reg, Address(ScratchRegister, 0));
    }
    void storeValue(ValueOperand val, const BaseIndex &dest) {
        // Harder cases not handled yet.
        JS_ASSERT(dest.offset == 0);
        storeValue(val, dest.base, dest.index);
    }
    void storeValue(const Value &val, Address dest) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        ma_mov(Imm32(jv.s.tag), lr);
        ma_str(lr, Address(dest.base, dest.offset + 4));
        if (val.isMarkable())
            ma_mov(ImmGCPtr(reinterpret_cast<gc::Cell *>(val.toGCThing())), lr);
        else
            ma_mov(Imm32(jv.s.payload.i32), lr);
        ma_str(lr, dest);
    }
    void storeValue(const Value &val, BaseIndex dest) {
        // Harder cases not handled yet.
        JS_ASSERT(dest.offset == 0);
        ma_alu(dest.base, lsl(dest.index, dest.scale), ScratchRegister, op_add);
        storeValue(val, Address(ScratchRegister, 0));
    }

    void loadValue(Address src, ValueOperand val);
    void loadValue(Operand dest, ValueOperand val) {
        loadValue(dest.toAddress(), val);
    }
    void loadValue(Register base, Register index, ValueOperand val, Imm32 of);
    void loadValue(const BaseIndex &addr, ValueOperand val) {
        loadValue(addr.base, addr.index, val, Imm32(addr.offset));
    }
    void tagValue(JSValueType type, Register payload, ValueOperand dest);

    void pushValue(ValueOperand val);
    void popValue(ValueOperand val);
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
        ma_push(reg);
    }
    void storePayload(const Value &val, Operand dest);
    void storePayload(Register src, Operand dest);
    void storePayload(const Value &val, Register base, Register index, int32 shift = defaultShift);
    void storePayload(Register src, Register base, Register index, int32 shift = defaultShift);
    void storeTypeTag(ImmTag tag, Operand dest);
    void storeTypeTag(ImmTag tag, Register base, Register index, int32 shift = defaultShift);

    void makeFrameDescriptor(Register frameSizeReg, FrameType type) {
        ma_lsl(Imm32(FRAMESIZE_SHIFT), frameSizeReg, frameSizeReg);
        ma_orr(Imm32(type), frameSizeReg);
    }

    void linkExitFrame();
    void handleException();

    /////////////////////////////////////////////////////////////////
    // Common interface.
    /////////////////////////////////////////////////////////////////
  public:
    // The following functions are exposed for use in platform-shared code.
    void Push(const Register &reg) {
        ma_push(reg);
        adjustFrame(STACK_SLOT_SIZE);
    }
    void Push(const Imm32 imm) {
        push(imm);
        adjustFrame(STACK_SLOT_SIZE);
    }
    void Push(const ImmWord imm) {
        push(imm);
        adjustFrame(STACK_SLOT_SIZE);
    }
    void Push(const ImmGCPtr ptr) {
        push(ptr);
        adjustFrame(STACK_SLOT_SIZE);
    }
    void Push(const FloatRegister &t) {
        VFPRegister r = VFPRegister(t);
        ma_vpush(VFPRegister(t));
        adjustFrame(r.size());
    }

    CodeOffsetLabel PushWithPatch(const ImmWord &word) {
        framePushed_ += sizeof(word.value);
        return pushWithPatch(word);
    }


    void PushWithPadding(const Register &reg, const Imm32 extraSpace) {
        pushWithPadding(reg, extraSpace);
        adjustFrame(STACK_SLOT_SIZE + extraSpace.value);
    }
    void PushWithPadding(const Imm32 imm, const Imm32 extraSpace) {
        pushWithPadding(imm, extraSpace);
        adjustFrame(STACK_SLOT_SIZE + extraSpace.value);
    }

    void Pop(const Register &reg) {
        ma_pop(reg);
        adjustFrame(-STACK_SLOT_SIZE);
    }
    void implicitPop(uint32 args) {
        JS_ASSERT(args % STACK_SLOT_SIZE == 0);
        adjustFrame(-args);
    }
    uint32 framePushed() const {
        return framePushed_;
    }
    void setFramePushed(uint32 framePushed) {
        framePushed_ = framePushed;
    }

    // Builds an exit frame on the stack, with a return address to an internal
    // non-function. Returns offset to be passed to markSafepointAt().
    bool buildFakeExitFrame(const Register &scratch, uint32 *offset);
    bool buildOOLFakeExitFrame(void *fakeReturnAddr);

    void callWithExitFrame(IonCode *target);
    void callWithExitFrame(IonCode *target, Register dynStack);

    // Makes an Ion call using the only two methods that it is sane for
    // indep code to make a call
    void callIon(const Register &callee);

    void reserveStack(uint32 amount);
    void freeStack(uint32 amount);
    void freeStack(Register amount);

    void add32(Imm32 imm, Register dest);
    void add32(Imm32 imm, const Address &dest);
    void sub32(Imm32 imm, Register dest);

    void and32(Imm32 imm, Register dest);
    void and32(Imm32 imm, const Address &dest);
    void or32(Imm32 imm, const Address &dest);
    void orPtr(Imm32 imm, Register dest);
    void addPtr(Register src, Register dest);

    void move32(const Imm32 &imm, const Register &dest);

    void movePtr(const Register &src, const Register &dest);
    void movePtr(const ImmWord &imm, const Register &dest);
    void movePtr(const ImmGCPtr &imm, const Register &dest);

    void load8SignExtend(const Address &address, const Register &dest);
    void load8SignExtend(const BaseIndex &src, const Register &dest);

    void load8ZeroExtend(const Address &address, const Register &dest);
    void load8ZeroExtend(const BaseIndex &src, const Register &dest);

    void load16SignExtend(const Address &address, const Register &dest);
    void load16SignExtend(const BaseIndex &src, const Register &dest);

    void load16ZeroExtend(const Address &address, const Register &dest);
    void load16ZeroExtend(const BaseIndex &src, const Register &dest);

    void load32(const Address &address, const Register &dest);
    void load32(const BaseIndex &address, const Register &dest);
    void load32(const AbsoluteAddress &address, const Register &dest);

    void loadPtr(const Address &address, const Register &dest);
    void loadPtr(const BaseIndex &src, const Register &dest);
    void loadPtr(const AbsoluteAddress &address, const Register &dest);

    void loadPrivate(const Address &address, const Register &dest);

    void loadDouble(const Address &addr, const FloatRegister &dest);
    void loadDouble(const BaseIndex &src, const FloatRegister &dest);

    // Load a float value into a register, then expand it to a double.
    void loadFloatAsDouble(const Address &addr, const FloatRegister &dest);
    void loadFloatAsDouble(const BaseIndex &src, const FloatRegister &dest);

    void store8(const Register &src, const Address &address);
    void store8(const Imm32 &imm, const Address &address);
    void store8(const Register &src, const BaseIndex &address);
    void store8(const Imm32 &imm, const BaseIndex &address);

    void store16(const Register &src, const Address &address);
    void store16(const Imm32 &imm, const Address &address);
    void store16(const Register &src, const BaseIndex &address);
    void store16(const Imm32 &imm, const BaseIndex &address);

    void store32(const Register &src, const AbsoluteAddress &address);
    void store32(const Register &src, const Address &address);
    void store32(const Register &src, const BaseIndex &address);
    void store32(const Imm32 &src, const Address &address);
    void store32(const Imm32 &src, const BaseIndex &address);

    void storePtr(ImmWord imm, const Address &address);
    void storePtr(ImmGCPtr imm, const Address &address);
    void storePtr(Register src, const Address &address);
    void storePtr(const Register &src, const AbsoluteAddress &dest);
    void storeDouble(FloatRegister src, Address addr) {
        ma_vstr(src, Operand(addr));
    }
    void storeDouble(FloatRegister src, BaseIndex addr) {
        // Harder cases not handled yet.
        JS_ASSERT(addr.offset == 0);
        uint32 scale = Imm32::ShiftOf(addr.scale).value;
        ma_vstr(src, addr.base, addr.index, scale);
    }

    void storeFloat(FloatRegister src, Address addr) {
        ma_vstr(VFPRegister(src).singleOverlay(), Operand(addr));
    }
    void storeFloat(FloatRegister src, BaseIndex addr) {
        // Harder cases not handled yet.
        JS_ASSERT(addr.offset == 0);
        uint32 scale = Imm32::ShiftOf(addr.scale).value;
        ma_vstr(VFPRegister(src).singleOverlay(), addr.base, addr.index, scale);
    }

    void clampIntToUint8(Register src, Register dest) {
        // look at (src >> 8) if it is 0, then src shouldn't be clamped
        // if it is <0, then we want to clamp to 0, otherwise, we wish to clamp to 255
        as_mov(ScratchRegister, asr(src, 8), SetCond);
        ma_mov(src, dest);
        ma_mov(Imm32(0xff), dest, NoSetCond, NotEqual);
        ma_mov(Imm32(0), dest, NoSetCond, Signed);
    }

    void cmp32(const Register &lhs, const Imm32 &rhs);
    void cmp32(const Register &lhs, const Register &rhs);
    void cmp32(const Operand &lhs, const Imm32 &rhs);
    void cmp32(const Operand &lhs, const Register &rhs);
    void cmpPtr(const Register &lhs, const ImmWord &rhs);
    void cmpPtr(const Register &lhs, const Register &rhs);
    void cmpPtr(const Register &lhs, const ImmGCPtr &rhs);
    void cmpPtr(const Address &lhs, const Register &rhs);
    void cmpPtr(const Address &lhs, const ImmWord &rhs);

    void subPtr(Imm32 imm, const Register dest);
    void addPtr(Imm32 imm, const Register dest);
    void addPtr(Imm32 imm, const Address &dest);
    void addPtr(ImmWord imm, const Register dest) {
        addPtr(Imm32(imm.value), dest);
    }

    void setStackArg(const Register &reg, uint32 arg);

    void breakpoint();

    void compareDouble(FloatRegister lhs, FloatRegister rhs);
    void branchDouble(DoubleCondition cond, const FloatRegister &lhs, const FloatRegister &rhs,
                      Label *label);

    void checkStackAlignment();

    void rshiftPtr(Imm32 imm, Register dest) {
        ma_lsr(imm, dest, dest);
    }
    void lshiftPtr(Imm32 imm, Register dest) {
        ma_lsl(imm, dest, dest);
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

    // Arguments must be assigned in a left-to-right order. This process may
    // temporarily use more stack, in which case esp-relative addresses will be
    // automatically adjusted. It is extremely important that esp-relative
    // addresses are computed *after* setupABICall(). Furthermore, no
    // operations should be emitted while setting arguments.
    void passABIArg(const MoveOperand &from);
    void passABIArg(const Register &reg);
    void passABIArg(const FloatRegister &reg);
    void passABIArg(const ValueOperand &regs);

    // Emits a call to a C/C++ function, resolving all argument moves.
    void callWithABI(void *fun, Result result = GENERAL);

    CodeOffsetLabel labelForPatch() {
        return CodeOffsetLabel(nextOffset().getOffset());
    }

    void computeEffectiveAddress(const Address &address, Register dest) {
        ma_add(address.base, Imm32(address.offset), dest, NoSetCond);
    }
    void computeEffectiveAddress(const BaseIndex &address, Register dest) {
        ma_alu(address.base, lsl(address.index, address.scale), dest, op_add, NoSetCond);
        if (address.offset)
            ma_add(dest, Imm32(address.offset), dest, NoSetCond);
    }
    void floor(FloatRegister input, Register output, Label *handleNotAnInt);
    void round(FloatRegister input, Register output, Label *handleNotAnInt, FloatRegister tmp);

    void clampCheck(Register r, Label *handleNotAnInt) {
        // check explicitly for r == INT_MIN || r == INT_MAX
        // this is the instruction sequence that gcc generated for this
        // operation.
        ma_sub(r, Imm32(0x80000001), ScratchRegister);
        ma_cmn(ScratchRegister, Imm32(3));
        ma_b(handleNotAnInt, Above);
    }

    void enterOsr(Register calleeToken, Register code);
};

typedef MacroAssemblerARMCompat MacroAssemblerSpecific;

} // namespace ion
} // namespace js

#endif // jsion_macro_assembler_arm_h__
