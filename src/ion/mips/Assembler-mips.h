/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_assembler_mips__
#define jsion_assembler_mips__
#include <cstddef>
#include "ion/shared/Assembler-shared.h"
#include "assembler/assembler/MIPSAssembler.h"
#include "assembler/assembler/MacroAssemblerMIPS.h"
#include "ion/CompactBuffer.h"
#include "ion/IonCode.h"

#include "jsscriptinlines.h"

namespace js {
namespace ion {

#if 0
static const Register eax = {JSC::MIPSRegisters::at};//{ JSC::X86Registers::eax };
static const Register ecx = {JSC::MIPSRegisters::at};//{ JSC::X86Registers::ecx };
static const Register edx = {JSC::MIPSRegisters::at};//{ JSC::X86Registers::edx };
static const Register ebx = {JSC::MIPSRegisters::at};//{ JSC::X86Registers::ebx };
static const Register esp = {JSC::MIPSRegisters::at};//{ JSC::X86Registers::esp };
static const Register ebp = {JSC::MIPSRegisters::at};//{ JSC::X86Registers::ebp };
static const Register esi = {JSC::MIPSRegisters::at};//{ JSC::X86Registers::esi };
static const Register edi = {JSC::MIPSRegisters::at};//{ JSC::X86Registers::edi };

static const FloatRegister xmm0 = {JSC::MIPSRegisters::f0};//{ JSC::X86Registers::xmm0 };
static const FloatRegister xmm1 = {JSC::MIPSRegisters::f0};//{ JSC::X86Registers::xmm1 };
static const FloatRegister xmm2 = {JSC::MIPSRegisters::f0};//{ JSC::X86Registers::xmm2 };
static const FloatRegister xmm3 = {JSC::MIPSRegisters::f0};//{ JSC::X86Registers::xmm3 };
static const FloatRegister xmm4 = {JSC::MIPSRegisters::f0};//{ JSC::X86Registers::xmm4 };
static const FloatRegister xmm5 = {JSC::MIPSRegisters::f0};//{ JSC::X86Registers::xmm5 };
static const FloatRegister xmm6 = {JSC::MIPSRegisters::f0};//{ JSC::X86Registers::xmm6 };
static const FloatRegister xmm7 = {JSC::MIPSRegisters::f0};//{ JSC::X86Registers::xmm7 };
#endif

//NOTE: there are duplicates in this list!
// sometimes we want to specifically refer to the
// link register as a link register (bl lr is much
// clearer than bl r14).  HOWEVER, this register can
// easily be a gpr when it is not busy holding the return
// address.
static const Register zero = { JSC::MIPSRegisters::zero };
static const Register at = { JSC::MIPSRegisters::at };
static const Register v0 = { JSC::MIPSRegisters::v0 };
static const Register v1 = { JSC::MIPSRegisters::v1 };
static const Register a0 = { JSC::MIPSRegisters::a0 };
static const Register a1 = { JSC::MIPSRegisters::a1 };
static const Register a2 = { JSC::MIPSRegisters::a2 };
static const Register a3 = { JSC::MIPSRegisters::a3 };
static const Register t0 = { JSC::MIPSRegisters::t0 };
static const Register t1 = { JSC::MIPSRegisters::t1 };
static const Register t2 = { JSC::MIPSRegisters::t2 };
static const Register t3 = { JSC::MIPSRegisters::t3 };
static const Register t4 = { JSC::MIPSRegisters::t4 };
static const Register t5 = { JSC::MIPSRegisters::t5 };
static const Register t6 = { JSC::MIPSRegisters::t6 };
static const Register t7 = { JSC::MIPSRegisters::t7 };
static const Register s0 = { JSC::MIPSRegisters::s0 };
static const Register s1 = { JSC::MIPSRegisters::s1 };
static const Register s2 = { JSC::MIPSRegisters::s2 };
static const Register s3 = { JSC::MIPSRegisters::s3 };
static const Register s4 = { JSC::MIPSRegisters::s4 };
static const Register s5 = { JSC::MIPSRegisters::s5 };
static const Register s6 = { JSC::MIPSRegisters::s6 };
static const Register s7 = { JSC::MIPSRegisters::s7 };
static const Register t8 = { JSC::MIPSRegisters::t8 };
static const Register t9 = { JSC::MIPSRegisters::t9 };
static const Register k0 = { JSC::MIPSRegisters::k0 };
static const Register k1 = { JSC::MIPSRegisters::k1 };
static const Register gp = { JSC::MIPSRegisters::gp };
static const Register sp = { JSC::MIPSRegisters::sp };
static const Register fp = { JSC::MIPSRegisters::fp };
static const Register ra = { JSC::MIPSRegisters::ra };

static const FloatRegister f0  = {JSC::MIPSRegisters::f0};
static const FloatRegister f1  = {JSC::MIPSRegisters::f1};
static const FloatRegister f2  = {JSC::MIPSRegisters::f2};
static const FloatRegister f3  = {JSC::MIPSRegisters::f3};
static const FloatRegister f4  = {JSC::MIPSRegisters::f4};
static const FloatRegister f5  = {JSC::MIPSRegisters::f5};
static const FloatRegister f6  = {JSC::MIPSRegisters::f6};
static const FloatRegister f7  = {JSC::MIPSRegisters::f7};
static const FloatRegister f8  = {JSC::MIPSRegisters::f8};
static const FloatRegister f9  = {JSC::MIPSRegisters::f9};
static const FloatRegister f10 = {JSC::MIPSRegisters::f10};
static const FloatRegister f11 = {JSC::MIPSRegisters::f11};
static const FloatRegister f12 = {JSC::MIPSRegisters::f12};
static const FloatRegister f13 = {JSC::MIPSRegisters::f13};
static const FloatRegister f14 = {JSC::MIPSRegisters::f14};
static const FloatRegister f15 = {JSC::MIPSRegisters::f15};
static const FloatRegister f16 = {JSC::MIPSRegisters::f16};
static const FloatRegister f17 = {JSC::MIPSRegisters::f17};
static const FloatRegister f18 = {JSC::MIPSRegisters::f18};
static const FloatRegister f19 = {JSC::MIPSRegisters::f19};
static const FloatRegister f20 = {JSC::MIPSRegisters::f20};
static const FloatRegister f21 = {JSC::MIPSRegisters::f21};
static const FloatRegister f22 = {JSC::MIPSRegisters::f22};
static const FloatRegister f23 = {JSC::MIPSRegisters::f23};
static const FloatRegister f24 = {JSC::MIPSRegisters::f24};
static const FloatRegister f25 = {JSC::MIPSRegisters::f25};
static const FloatRegister f26 = {JSC::MIPSRegisters::f26};
static const FloatRegister f27 = {JSC::MIPSRegisters::f27};
static const FloatRegister f28 = {JSC::MIPSRegisters::f28};
static const FloatRegister f29 = {JSC::MIPSRegisters::f29};
static const FloatRegister f30 = {JSC::MIPSRegisters::f30};
static const FloatRegister f31 = {JSC::MIPSRegisters::f31};

static const Register ArgumentsRectifierReg = a2;
static const Register CallTempReg0 = t0;
static const Register CallTempReg1 = t1;
static const Register CallTempReg2 = t2;
static const Register CallTempReg3 = t3;
static const Register CallTempReg4 = v0;
static const Register CallTempReg5 = v1;

static const Register OsrFrameReg = a1;
static const Register PreBarrierReg = a1;

static const Register InvalidReg = { JSC::MIPSRegisters::invalid_reg };
static const FloatRegister InvalidFloatReg = { JSC::MIPSRegisters::invalid_freg };

static const Register JSReturnReg_Type = a0;
static const Register JSReturnReg_Data = a1;
static const Register ReturnReg = v0;
static const Register StackPointer = sp;
static const Register FramePointer = InvalidReg;
static const FloatRegister ReturnFloatReg = { JSC::MIPSRegisters::f1 };
static const FloatRegister ScratchFloatReg = { JSC::MIPSRegisters::f0 };

static const Register ScratchRegister = t9;

static const Register immTempRegister  = t0;
static const Register dataTempRegister = t1;
static const Register addrTempRegister = t2;
static const Register cmpTempRegister  = t3;
static const Register dataTemp2Register = t4;
static const Register cmpTemp2Register  = t5;

static const FloatRegister fpTempRegister = f30;
static const FloatRegister fpTemp2Register = f31;

// For maximal awesomeness, 8 should be sufficent.
// ldrd/strd (dual-register load/store) operate in a single cycle
// when the address they are dealing with is 8 byte aligned.
// Also, the ARM abi wants the stack to be 8 byte aligned at
// function boundaries.  I'm trying to make sure this is always true.
static const uint32 StackAlignment = 8;
static const bool StackKeptAligned = true;

static const Scale ScalePointer = TimesFour;
struct ImmTag : public Imm32
{
    ImmTag(JSValueTag mask)
      : Imm32(int32(mask))
    { }
};

struct ImmType : public ImmTag
{
    ImmType(JSValueType type)
      : ImmTag(JSVAL_TYPE_TO_TAG(type))
    { }
};


class Operand
{
  public:
    enum Kind {
        REG,
        REG_DISP,
        FPREG,
        SCALE,
        ADDRESS
    };

    Kind kind_ : 4;
    int32 index_ : 5;
    Scale scale_ : 3;
    int32 base_;
    int32 disp_;

  public:
    explicit Operand(Register reg)
      : kind_(REG),
        base_(reg.code())
    { }
    explicit Operand(FloatRegister reg)
      : kind_(FPREG),
        base_(reg.code())
    { }
    explicit Operand(const Address &address)
      : kind_(REG_DISP),
        base_(address.base.code()),
        disp_(address.offset)
    { }
    explicit Operand(const BaseIndex &address)//TBD BaseIndex special treat
      : kind_(SCALE),
        index_(address.index.code()),
        scale_(address.scale),
        base_(address.base.code()),
        disp_(address.offset)
    { }
    Operand(Register base, Register index, Scale scale, int32 disp = 0)
      : kind_(SCALE),
        index_(index.code()),
        scale_(scale),
        base_(base.code()),
        disp_(disp)
    { }
    Operand(Register reg, int32 disp)
      : kind_(REG_DISP),
        base_(reg.code()),
        disp_(disp)
    { }
    explicit Operand(const AbsoluteAddress &address)
      : kind_(ADDRESS),
        base_(reinterpret_cast<int32>(address.addr))
    { }
    explicit Operand(const void *address)
      : kind_(ADDRESS),
        base_(reinterpret_cast<int32>(address))
    { }

    Kind kind() const {
        return kind_;
    }
    Registers::Code reg() const {
        JS_ASSERT(kind() == REG);
        return (Registers::Code)base_;
    }
    Registers::Code base() const {
        JS_ASSERT(kind() == REG_DISP || kind() == SCALE);
        return (Registers::Code)base_;
    }
    Registers::Code index() const {
        JS_ASSERT(kind() == SCALE);
        return (Registers::Code)index_;
    }
    Scale scale() const {
        JS_ASSERT(kind() == SCALE);
        return scale_;
    }
    FloatRegisters::Code fpu() const {
        JS_ASSERT(kind() == FPREG);
        return (FloatRegisters::Code)base_;
    }
    int32 disp() const {
        JS_ASSERT(kind() == REG_DISP || kind() == SCALE);
        return disp_;
    }
    void *address() const {
        JS_ASSERT(kind() == ADDRESS);
        return reinterpret_cast<void *>(base_);
    }
};


static inline void
PatchJump(CodeLocationJump jump, CodeLocationLabel label)
{
#ifdef DEBUG
    // Assert that we're overwriting a jump instruction, either:
    //   0F 80+cc <imm32>, or
    //   E9 <imm32>
    unsigned char *x = (unsigned char *)jump.raw() - 5;
//    JS_ASSERT(((*x >= 0x80 && *x <= 0x8F) && *(x - 1) == 0x0F) || (*x == 0xE9));
#endif
//ok    JSC::MIPSAssembler::setRel32(jump.raw(), label.raw());
    JSC::MacroAssemblerMIPS::repatchJump(JSC::CodeLocationJump(jump.raw()), JSC::CodeLocationLabel(label.raw()));
}

// Return operand from a JS -> JS call.
static const ValueOperand JSReturnOperand = ValueOperand(JSReturnReg_Type, JSReturnReg_Data);

class Assembler
{
  protected:
    struct RelativePatch {
        int32 offset;
        void *target;
        Relocation::Kind kind;

        RelativePatch(int32 offset, void *target, Relocation::Kind kind)
          : offset(offset),
            target(target),
            kind(kind)
        { }
    };

    js::Vector<DeferredData *, 0, SystemAllocPolicy> data_;
    js::Vector<CodeLabel *, 0, SystemAllocPolicy> codeLabels_;
    js::Vector<RelativePatch, 8, SystemAllocPolicy> jumps_;
    CompactBufferWriter jumpRelocations_;
    CompactBufferWriter dataRelocations_;
    size_t dataBytesNeeded_;
    bool enoughMemory_;

    void writeDataRelocation(const Value &val) {
        if (val.isMarkable())
            dataRelocations_.writeUnsigned(masm.currentOffset());
    }
    void writeDataRelocation(const ImmGCPtr &ptr) {
        if (ptr.value)
            dataRelocations_.writeUnsigned(masm.currentOffset());
    }

  protected:
    JSC::MacroAssemblerMIPS mcss;
    JSC::MIPSAssembler& masm;
    JSC::MIPSAssembler& m_assembler;
    typedef JSC::MacroAssemblerMIPS::Address mAddress ;
    typedef JSC::MacroAssemblerMIPS::ExtendedAddress mExtendedAddress;
    typedef JSC::MacroAssemblerMIPS::ImplicitAddress mImplicitAddress;
    typedef JSC::MacroAssemblerMIPS::BaseIndex mBaseIndex;
    typedef JSC::MacroAssemblerMIPS::AbsoluteAddress mAbsoluteAddress;
    typedef JSC::MacroAssemblerMIPS::TrustedImmPtr mTrustedImmPtr;
    typedef JSC::MacroAssemblerMIPS::TrustedImm32 mTrustedImm32;
    typedef JSC::MacroAssemblerMIPS::Scale mScale;
    typedef JSC::MacroAssemblerMIPS::ImmPtr mImmPtr;
    typedef JSC::MacroAssemblerMIPS::Imm32 mImm32;
    typedef JSC::MacroAssemblerMIPS::ImmDouble mImmDouble;
    typedef JSC::MacroAssemblerMIPS::RegisterID mRegisterID;

    typedef JSC::MIPSAssembler::JmpSrc JmpSrc;
    typedef JSC::MIPSAssembler::JmpDst JmpDst;

  protected://x86
    void writeRelocation(JmpSrc src) {
        jumpRelocations_.writeUnsigned(src.offset());
    }
    void addPendingJump(JmpSrc src, void *target, Relocation::Kind kind) {
        enoughMemory_ &= jumps_.append(RelativePatch(src.offset(), target, kind));
        if (kind == Relocation::IONCODE)
            writeRelocation(src);
    }

  public:
    enum Condition {
        Equal = JSC::MacroAssemblerMIPS::Equal,
        NotEqual = JSC::MacroAssemblerMIPS::NotEqual,
        Above = JSC::MacroAssemblerMIPS::Above,
        AboveOrEqual = JSC::MacroAssemblerMIPS::AboveOrEqual,
        Below = JSC::MacroAssemblerMIPS::Below,
        BelowOrEqual = JSC::MacroAssemblerMIPS::BelowOrEqual,
        GreaterThan = JSC::MacroAssemblerMIPS::GreaterThan,
        GreaterThanOrEqual = JSC::MacroAssemblerMIPS::GreaterThanOrEqual,
        LessThan = JSC::MacroAssemblerMIPS::LessThan,
        LessThanOrEqual = JSC::MacroAssemblerMIPS::LessThanOrEqual,
        Overflow = JSC::MacroAssemblerMIPS::Overflow,
        Signed = JSC::MacroAssemblerMIPS::Signed,
        Zero = JSC::MacroAssemblerMIPS::Zero,
        NonZero = JSC::MacroAssemblerMIPS::NonZero,
        Parity,
        NoParity
    };

    // If this bit is set, the ucomisd operands have to be inverted.
    static const int DoubleConditionBitInvert = 0x10;

    // Bit set when a DoubleCondition does not map to a single x86 condition.
    // The macro assembler has to special-case these conditions.
    static const int DoubleConditionBitSpecial = 0x20;
    static const int DoubleConditionBits = DoubleConditionBitInvert | DoubleConditionBitSpecial;

    enum DoubleCondition {
        // These conditions will only evaluate to true if the comparison is ordered - i.e. neither operand is NaN.
        DoubleOrdered = NoParity,
        DoubleEqual = Equal | DoubleConditionBitSpecial,
        DoubleNotEqual = NotEqual,
        DoubleGreaterThan = Above,
        DoubleGreaterThanOrEqual = AboveOrEqual,
        DoubleLessThan = Above | DoubleConditionBitInvert,
        DoubleLessThanOrEqual = AboveOrEqual | DoubleConditionBitInvert,
        // If either operand is NaN, these conditions always evaluate to true.
        DoubleUnordered = Parity,
        DoubleEqualOrUnordered = Equal,
        DoubleNotEqualOrUnordered = NotEqual | DoubleConditionBitSpecial,
        DoubleGreaterThanOrUnordered = Below | DoubleConditionBitInvert,
        DoubleGreaterThanOrEqualOrUnordered = BelowOrEqual | DoubleConditionBitInvert,
        DoubleLessThanOrUnordered = Below,
        DoubleLessThanOrEqualOrUnordered = BelowOrEqual
    };

    static void staticAsserts() {
        // DoubleConditionBits should not interfere with x86 condition codes.
        JS_STATIC_ASSERT(!((Equal | NotEqual | Above | AboveOrEqual | Below |
                            BelowOrEqual | Parity | NoParity) & DoubleConditionBits));
    }

    Assembler()
      : masm(mcss.assembler()),
        m_assembler(mcss.assembler()),
        dataBytesNeeded_(0),
        enoughMemory_(true)
    {
    }

    static Condition InvertCondition(Condition cond);

    static inline Condition ConditionFromDoubleCondition(DoubleCondition cond) {
        return static_cast<Condition>(cond & ~DoubleConditionBits);
    }

    static void TraceDataRelocations(JSTracer *trc, IonCode *code, CompactBufferReader &reader);

    // MacroAssemblers hold onto gcthings, so they are traced by the GC.
    void trace(JSTracer *trc);

    bool oom() const {
        return masm.oom() ||
               !enoughMemory_ ||
               jumpRelocations_.oom() ||
               dataRelocations_.oom();
    }

    void setPrinter(Sprinter *sp) {
        masm.setPrinter(sp);
    }

    void executableCopy(void *buffer);
    void processDeferredData(IonCode *code, uint8 *data);
    void processCodeLabels(IonCode *code);
    void copyJumpRelocationTable(uint8 *buffer);
    void copyDataRelocationTable(uint8 *buffer);

    bool addDeferredData(DeferredData *data, size_t bytes) {
        data->setOffset(dataBytesNeeded_);
        dataBytesNeeded_ += bytes;
        if (dataBytesNeeded_ >= MAX_BUFFER_SIZE)
            return false;
        return data_.append(data);
    }
    
    bool addCodeLabel(CodeLabel *label) {
        return codeLabels_.append(label);
    }

    // Size of the instruction stream, in bytes.
    size_t size() const {
        return masm.size();
    }
    // Size of the jump relocation table, in bytes.
    size_t jumpRelocationTableBytes() const {
        return jumpRelocations_.length();
    }
    size_t dataRelocationTableBytes() const {
        return dataRelocations_.length();
    }
    // Size of the data table, in bytes.
    size_t dataSize() const {
        return dataBytesNeeded_;
    }
    size_t bytesNeeded() const {
        return size() +
               dataSize() +
               jumpRelocationTableBytes() +
               dataRelocationTableBytes();
    }

  public:
    static void TraceJumpRelocations(JSTracer *trc, IonCode *code, CompactBufferReader &reader);

    // The buffer is about to be linked, make sure any constant pools or excess
    // bookkeeping has been flushed to the instruction stream.
    void flush() { }

    // Copy the assembly code to the given buffer, and perform any pending
    // relocations relying on the target address.
    void executableCopy(uint8 *buffer);

    // Actual assembly emitting functions.

    void push(const ImmGCPtr &ptr) {
        push(Imm32(ptr.value));
        writeDataRelocation(ptr);
    }
    void push(const ImmWord imm) {
        push(Imm32(imm.value));
    }
    void push(const FloatRegister &src) {
        subl(Imm32(sizeof(double)), StackPointer);
        movsd(src, Operand(StackPointer, 0));
    }

    CodeOffsetLabel pushWithPatch(const ImmWord &word) {
        push(Imm32(word.value));
        return masm.currentOffset();
    }

    void movl(const ImmGCPtr &ptr, const Register &dest) {
//ok        masm.movl_i32r(ptr.value, dest.code());
        mcss.move(mTrustedImmPtr(reinterpret_cast<const void*>(ptr.value)), dest.code());
        writeDataRelocation(ptr);
    }
    void movl(const ImmGCPtr &ptr, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG:
//ok            masm.movl_i32r(ptr.value, dest.reg());
            mcss.move(mTrustedImm32(ptr.value), dest.reg());
            writeDataRelocation(ptr);
            break;
          case Operand::REG_DISP:
//ok            masm.movl_i32m(ptr.value, dest.disp(), dest.base());
            mcss.store32(mTrustedImm32(ptr.value), mImplicitAddress(mAddress(dest.base(), dest.disp())));
            writeDataRelocation(ptr);
            break;
          case Operand::SCALE:
//ok            masm.movl_i32m(ptr.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store32(mTrustedImm32(ptr.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            writeDataRelocation(ptr);
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movl(ImmWord imm, Register dest) {
//ok        masm.movl_i32r(imm.value, dest.code());
        mcss.move(mTrustedImm32(imm.value), dest.code());
    }
    void mov(ImmWord imm, Register dest) {
        movl(imm, dest);
    }
    void mov(Imm32 imm, Register dest) {
        movl(imm, dest);
    }
    void mov(const Operand &src, const Register &dest) {
        movl(src, dest);
    }
    void mov(const Register &src, const Operand &dest) {
        movl(src, dest);
    }
    void mov(AbsoluteLabel *label, const Register &dest) {
        JS_ASSERT(!label->bound());
        // Thread the patch list through the unpatched address word in the
        // instruction stream.
//ok        masm.movl_i32r(label->prev(), dest.code());
        mcss.move(mTrustedImmPtr(reinterpret_cast<const void*>(label->prev())), dest.code());
        label->setPrev(masm.size());
    }
    void mov(const Register &src, const Register &dest) {
        movl(src, dest);
    }
    void lea(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.leal_mr(src.disp(), src.base(), dest.code());
            mcss.lea(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.leal_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.lea(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void cvttsd2s(const FloatRegister &src, const Register &dest) {
        cvttsd2si(src, dest);
    }

    void cmpl(const Register src, ImmWord ptr) {
//okm        masm.cmpl_ir(ptr.value, src.code());
        mcss.move(src.code(), cmpTempRegister.code());
        mcss.move(mTrustedImm32(ptr.value), cmpTemp2Register.code());
    }
    void cmpl(const Register src, ImmGCPtr ptr) {
//okm        masm.cmpl_ir(ptr.value, src.code());
        mcss.move(src.code(), cmpTempRegister.code());
        mcss.move(mTrustedImmPtr(reinterpret_cast<const void*>(ptr.value)), cmpTemp2Register.code());
        writeDataRelocation(ptr);
    }
    void cmpl(const Operand &op, ImmGCPtr imm) {
        switch (op.kind()) {
          case Operand::REG:
//okm            masm.cmpl_ir_force32(imm.value, op.reg());
            mcss.move(op.reg(), cmpTempRegister.code());
            mcss.move(mTrustedImmPtr(reinterpret_cast<const void*>(imm.value)), cmpTemp2Register.code());
            writeDataRelocation(imm);
            break;
          case Operand::REG_DISP:
//okm            masm.cmpl_im_force32(imm.value, op.disp(), op.base());
            mcss.load32(mAddress(op.base(), op.disp()), cmpTempRegister.code());
            mcss.move(mTrustedImmPtr(reinterpret_cast<const void*>(imm.value)), cmpTemp2Register.code());
            writeDataRelocation(imm);
            break;
          case Operand::ADDRESS:
//okm            masm.cmpl_im(imm.value, op.address());
            mcss.load32(op.address(), cmpTempRegister.code());
            mcss.move(mTrustedImmPtr(reinterpret_cast<const void*>(imm.value)), cmpTemp2Register.code());
            writeDataRelocation(imm);
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }

    void jmp(void *target, Relocation::Kind reloc = Relocation::HARDCODED) {
//ok        JmpSrc src = masm.jmp();
        JmpSrc src = mcss.jump().m_jmp;
        addPendingJump(src, target, reloc);
    }
    void j(Condition cond, void *target,
           Relocation::Kind reloc = Relocation::HARDCODED) {
//okm        JmpSrc src = masm.jCC(static_cast<JSC::X86Assembler::Condition>(cond));
        JmpSrc src = mcss.branch32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond), cmpTempRegister.code(), cmpTemp2Register.code()).m_jmp;
        addPendingJump(src, target, reloc);
    }

    void jmp(IonCode *target) {
        jmp(target->raw(), Relocation::IONCODE);
    }
    void j(Condition cond, IonCode *target) {
        j(cond, target->raw(), Relocation::IONCODE);
    }
    void j(DoubleCondition cond, void *target,
           Relocation::Kind reloc = Relocation::HARDCODED) {
//okm        JmpSrc src = masm.jCC(static_cast<JSC::X86Assembler::Condition>(cond));
        JmpSrc src = mcss.branchDouble(static_cast<JSC::MacroAssemblerMIPS::DoubleCondition>(cond), fpTempRegister.code(), fpTemp2Register.code()).m_jmp;
        addPendingJump(src, target, reloc);
    }

    void j(DoubleCondition cond, IonCode *target) {
        j(cond, target->raw(), Relocation::IONCODE);
    }
    void call(IonCode *target) {
//ok        JmpSrc src = masm.call();
        JmpSrc src = mcss.call().m_jmp;
        addPendingJump(src, target->raw(), Relocation::IONCODE);
    }
    void call(ImmWord target) {
//ok        JmpSrc src = masm.call();
        JmpSrc src = mcss.call().m_jmp;
        addPendingJump(src, target.asPointer(), Relocation::HARDCODED);
    }

    // Re-routes pending jumps to an external target, flushing the label in the
    // process.
    void retarget(Label *label, void *target, Relocation::Kind reloc) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            bool more;
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::MIPSAssembler::JmpSrc next;
                more = masm.nextJump(jmp, &next);
                addPendingJump(jmp, target, reloc);
                jmp = next;
            } while (more);
        }
        label->reset();
    }

    void movsd(const double *dp, const FloatRegister &dest) {
//ok        masm.movsd_mr((const void *)dp, dest.code());
        mcss.loadDouble(reinterpret_cast<const void *>(dp), dest.code());
    }
    void movsd(AbsoluteLabel *label, const FloatRegister &dest) {
        JS_ASSERT(!label->bound());
        // Thread the patch list through the unpatched address word in the
        // instruction stream.
//ok        masm.movsd_mr(reinterpret_cast<void *>(label->prev()), dest.code());
        mcss.loadDouble(reinterpret_cast<void *>(label->prev()), dest.code());
        label->setPrev(masm.size());
    }
  public:
    void align(int alignment) {
        masm.align(alignment);
    }
    void movl(const Imm32 &imm32, const Register &dest) {
//ok        masm.movl_i32r(imm32.value, dest.code());
        mcss.move(mTrustedImm32(imm32.value), dest.code());
    }
    void movl(const Register &src, const Register &dest) {
//ok        masm.movl_rr(src.code(), dest.code());
        mcss.move(src.code(), dest.code());
    }
    void movl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.movl_rr(src.reg(), dest.code());
            mcss.move(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.movl_mr(src.disp(), src.base(), dest.code());
            mcss.load32(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load32(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
//ok            masm.movl_mr(src.address(), dest.code());
            mcss.load32(src.address(), dest.code());
            break;
//endif
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movl(const Register &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG:
//ok            masm.movl_rr(src.code(), dest.reg());
            mcss.move(src.code(),dest.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.movl_rm(src.code(), dest.disp(), dest.base());
            mcss.store32(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movl_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store32(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
//ok            masm.movl_rm(src.code(), dest.address());
            mcss.store32(src.code(), dest.address());
            break;
//#endif
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movl(const Imm32 &imm32, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG:
//ok            masm.movl_i32r(imm32.value, dest.reg());
            mcss.move(mTrustedImm32(imm32.value), dest.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.movl_i32m(imm32.value, dest.disp(), dest.base());
            mcss.store32(mTrustedImm32(imm32.value), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movl_i32m(imm32.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store32(mTrustedImm32(imm32.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }

    void movsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.movsd_rr(src.code(), dest.code());
            mcss.moveDouble(src.code(), dest.code());
    }
    void movsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.movsd_rr(src.fpu(), dest.code());
            mcss.moveDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.movsd_mr(src.disp(), src.base(), dest.code());
            mcss.loadDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movsd_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.loadDouble(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movsd(const FloatRegister &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::FPREG:
//ok            masm.movsd_rr(src.code(), dest.fpu());
            mcss.moveDouble(src.code(), dest.fpu());
            break;
          case Operand::REG_DISP:
//ok            masm.movsd_rm(src.code(), dest.disp(), dest.base());
            mcss.storeDouble(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movsd_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.storeDouble(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movss(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movss_mr(src.disp(), src.base(), dest.code());
            mcss.loadFloat(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movss_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.loadFloat(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movss(const FloatRegister &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movss_rm(src.code(), dest.disp(), dest.base());
            mcss.storeFloat(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movss_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.storeFloat(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
#if 0
    void movdqa(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//            masm.movdqa_mr(src.disp(), src.base(), dest.code());
            break;
          case Operand::SCALE:
//            masm.movdqa_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movdqa(const FloatRegister &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//            masm.movdqa_rm(src.code(), dest.disp(), dest.base());
            break;
          case Operand::SCALE:
//            masm.movdqa_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
#endif
    void cvtss2sd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.cvtss2sd_rr(src.code(), dest.code());
        mcss.convertFloatToDouble(src.code(), dest.code());
    }
    void cvtsd2ss(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.cvtsd2ss_rr(src.code(), dest.code());
        mcss.convertDoubleToFloat(src.code(), dest.code());
    }
    void movzbl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movzbl_mr(src.disp(), src.base(), dest.code());
            mcss.load8ZeroExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movzbl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load8ZeroExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movxbl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movxbl_mr(src.disp(), src.base(), dest.code());
            mcss.load8SignExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movxbl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load8SignExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movb(const Register &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movb_rm(src.code(), dest.disp(), dest.base());
            mcss.store8(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movb_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store8(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movb(const Imm32 &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movb_i8m(src.value, dest.disp(), dest.base());
            mcss.store8(mTrustedImm32(src.value), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movb_i8m(src.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store8(mTrustedImm32(src.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movzwl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movzwl_mr(src.disp(), src.base(), dest.code());
            mcss.load16ZeroExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movzwl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load16ZeroExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }

    void movw(const Register &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movw_rm(src.code(), dest.disp(), dest.base());
            mcss.store16(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movw_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store16(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movw(const Imm32 &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movw_i16m(src.value, dest.disp(), dest.base());
            mcss.store16(mTrustedImm32(src.value), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movw_i16m(src.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store16(mTrustedImm32(src.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void movxwl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movxwl_mr(src.disp(), src.base(), dest.code());
            mcss.load16SignExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movxwl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load16SignExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }

  protected:
    JmpSrc jSrc(Condition cond, Label *label) {
//okm        JmpSrc j = masm.jCC(static_cast<JSC::X86Assembler::Condition>(cond));
        JmpSrc j = mcss.branch32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond), cmpTempRegister.code(), cmpTemp2Register.code()).m_jmp;
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            masm.setNextJump(j, prev);
        }
        return j;
    }
    JmpSrc jmpSrc(Label *label) {
//ok        JmpSrc j = masm.jmp();
        JmpSrc j = mcss.jump().m_jmp;
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            masm.setNextJump(j, prev);
        }
        return j;
    }

    // Comparison of EAX against the address given by a Label.
    JmpSrc cmpSrc(Label *label) {
//        JmpSrc j = masm.cmp_eax();
        JmpSrc j = mcss.jump().m_jmp;
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            masm.setNextJump(j, prev);
        }
        return j;
    }
    JmpSrc jSrc(Condition cond, RepatchLabel *label) {
//okm        JmpSrc j = masm.jCC(static_cast<JSC::X86Assembler::Condition>(cond));
        JmpSrc j = mcss.branch32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond), cmpTempRegister.code(), cmpTemp2Register.code()).m_jmp;
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            label->use(j.offset());
        }
        return j;
    }
    JmpSrc jmpSrc(RepatchLabel *label) {
//ok        JmpSrc j = masm.jmp();
        JmpSrc j = mcss.jump().m_jmp;
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            label->use(j.offset());
        }
        return j;
    }

  public:
    void nop() { masm.nop(); }
    void j(Condition cond, Label *label) { jSrc(cond, label); }
    void jmp(Label *label) { jmpSrc(label); }
    void j(Condition cond, RepatchLabel *label) { jSrc(cond, label); }
    void jmp(RepatchLabel *label) { jmpSrc(label); }

    void jmp(const Operand &op){
        switch (op.kind()) {
          case Operand::SCALE:
//ok            masm.jmp_m(op.disp(), op.base(), op.index(), op.scale());
            mcss.jump(mBaseIndex(op.base(), op.index(), mScale(op.scale()), op.disp()));
            break;
          case Operand::REG:
//ok            masm.jmp_r(op.reg());
            mcss.jump(op.reg());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void cmpEAX(Label *label) { cmpSrc(label); }
    void bind(Label *label) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            bool more;
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::MIPSAssembler::JmpSrc next;
                more = masm.nextJump(jmp, &next);
                masm.linkJump(jmp, masm.label());
                jmp = next;
            } while (more);
        }
        label->bind(masm.label().offset());
    }
    void bind(RepatchLabel *label) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            masm.linkJump(jmp, masm.label());
        }
        label->bind(masm.label().offset());
    }
    uint32 currentOffset() {
        return masm.label().offset();
    }

    // Re-routes pending jumps to a new label.
    void retarget(Label *label, Label *target) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            bool more;
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::MIPSAssembler::JmpSrc next;
                more = masm.nextJump(jmp, &next);

                if (target->bound()) {
                    // The jump can be immediately patched to the correct destination.
                    masm.linkJump(jmp, JmpDst(target->offset()));
                } else {
                    // Thread the jump list through the unpatched jump targets.
                    JmpSrc prev = JmpSrc(target->use(jmp.offset()));
                    masm.setNextJump(jmp, prev);
                }

                jmp = next;
            } while (more);
        }
        label->reset();
    }

    static void Bind(IonCode *code, AbsoluteLabel *label, const void *address) {
        uint8 *raw = code->raw();
        if (label->used()) {
            intptr_t src = label->offset();
            do {
                intptr_t next = reinterpret_cast<intptr_t>(JSC::MIPSAssembler::getPointer(raw + src));
                JSC::MIPSAssembler::setPointer(raw + src, address);
                src = next;
            } while (src != AbsoluteLabel::INVALID_OFFSET);
        }
        label->bind();
    }

    void ret() {
//ok        masm.ret();
        mcss.ret();
    }
    void retn(Imm32 n) {
        // Remove the size of the return address which is included in the frame.
//okm        masm.ret(n.value - sizeof(void *));
        mcss.ret((n.value - sizeof(void *)));
    }
    void call(Label *label) {
        if (label->bound()) {
//ok            masm.linkJump(mcss.call(), JmpDst(label->offset()));
            masm.linkJump(mcss.call().m_jmp, JmpDst(label->offset()));
        } else {
//ok            JmpSrc j = mcss.call();
            JmpSrc j = mcss.call().m_jmp;
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            masm.setNextJump(j, prev);
        }
    }
    void call(const Register &reg) {
        mcss.call(reg.code());
    }
    void call(const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
            mcss.call(op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.call_m(op.disp(), op.base());
            mcss.call(mAddress(op.base(), op.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }

    void breakpoint() {
//ok        masm.int3();
        mcss.breakpoint();
    }

#if 0
    static bool HasSSE41() {
        return JSC::MacroAssembler::getSSEState() >= JSC::MacroAssembler::HasSSE4_1;
    }
#endif
    // The below cmpl methods switch the lhs and rhs when it invokes the
    // macroassembler to conform with intel standard.  When calling this
    // function put the left operand on the left as you would expect.
    void cmpl(const Register &lhs, const Register &rhs) {
//okm        masm.cmpl_rr(rhs.code(), lhs.code());
// there must be a j following this cmpl in x86
        mcss.move(lhs.code(), cmpTempRegister.code());
        mcss.move(rhs.code(), cmpTemp2Register.code());
    }
    void cmpl(const Register &lhs, const Operand &rhs) {
        switch (rhs.kind()) {
          case Operand::REG:
//okm            masm.cmpl_rr(rhs.reg(), lhs.code());
            mcss.move(lhs.code(), cmpTempRegister.code());
            mcss.move(rhs.reg(), cmpTemp2Register.code());
            break;
          case Operand::REG_DISP:
//okm            masm.cmpl_mr(rhs.disp(), rhs.base(), lhs.code());
            mcss.move(lhs.code(), cmpTempRegister.code());
            mcss.load32(mAddress(rhs.base(), rhs.disp()), cmpTemp2Register.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void cmpl(const Register &src, Imm32 imm) {
//okm        masm.cmpl_ir(imm.value, src.code());
        mcss.move(src.code(), cmpTempRegister.code());
        mcss.move(mTrustedImm32(imm.value), cmpTemp2Register.code());
    }
    void cmpl(const Operand &op, Imm32 imm) {
        switch (op.kind()) {
          case Operand::REG:
//okm            masm.cmpl_ir(imm.value, op.reg());
            mcss.move(op.reg(), cmpTempRegister.code());
            mcss.move(mTrustedImm32(imm.value), cmpTemp2Register.code());
            break;
          case Operand::REG_DISP:
//okm            masm.cmpl_im(imm.value, op.disp(), op.base());
            mcss.load32(mAddress(op.base(), op.disp()), cmpTempRegister.code());
            mcss.move(mTrustedImm32(imm.value), cmpTemp2Register.code());
            break;
          case Operand::SCALE:
//okm            masm.cmpl_im(imm.value, op.disp(), op.base(), op.index(), op.scale());
            mcss.load32(mBaseIndex(op.base(), op.index(), mScale(op.scale()), op.disp()), cmpTempRegister.code());
            mcss.move(mTrustedImm32(imm.value), cmpTemp2Register.code());
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
//okm            masm.cmpl_im(imm.value, op.address());
            mcss.load32(op.address(), cmpTempRegister.code());
            mcss.move(mTrustedImm32(imm.value), cmpTemp2Register.code());
            break;
//#endif
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void cmpl(const Operand &lhs, const Register &rhs) {
        switch (lhs.kind()) {
          case Operand::REG:
//okm            masm.cmpl_rr(rhs.code(), lhs.reg());
            mcss.move(lhs.reg(), cmpTempRegister.code());
            mcss.move(rhs.code(), cmpTemp2Register.code());
            break;
          case Operand::REG_DISP:
//okm            masm.cmpl_rm(rhs.code(), lhs.disp(), lhs.base());
            mcss.load32(mAddress(lhs.base(), lhs.disp()), cmpTempRegister.code());
            mcss.move(rhs.code(), cmpTemp2Register.code());
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
//okm            masm.cmpl_rm(rhs.code(), lhs.address());
            mcss.load32(lhs.address(), cmpTempRegister.code());
            mcss.move(rhs.code(), cmpTemp2Register.code());
            break;
//#endif
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void cmpl(const Operand &op, ImmWord imm) {
        switch (op.kind()) {
          case Operand::REG:
//okm            masm.cmpl_ir(imm.value, op.reg());
            mcss.move(op.reg(), cmpTempRegister.code());
            mcss.move(mTrustedImm32(imm.value), cmpTemp2Register.code());
            break;
          case Operand::REG_DISP:
//okm            masm.cmpl_im(imm.value, op.disp(), op.base());
            mcss.load32(mAddress(op.base(), op.disp()), cmpTempRegister.code());
            mcss.move(mTrustedImm32(imm.value), cmpTemp2Register.code());
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
//okm            masm.cmpl_im(imm.value, op.address());
            mcss.load32(op.address(), cmpTempRegister.code());
            mcss.move(mTrustedImm32(imm.value), cmpTemp2Register.code());
            break;
//#endif
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void setCC(Condition cond, const Register &r) {
//        masm.setCC_r(static_cast<JSC::X86Assembler::Condition>(cond), r.code());
    }
    void testb(const Register &lhs, const Register &rhs) {
        JS_ASSERT(GeneralRegisterSet(Registers::SingleByteRegs).has(lhs));
        JS_ASSERT(GeneralRegisterSet(Registers::SingleByteRegs).has(rhs));
//okm        masm.testb_rr(rhs.code(), lhs.code());
//okm this instruction test if lhs is zero or not
        mcss.move(rhs.code(), cmpTempRegister.code());
        mcss.move(mRegisterID(0), cmpTempRegister.code());
    }

    void testl(const Register &lhs, const Register &rhs) {
//okm        masm.testl_rr(rhs.code(), lhs.code());
//okm this instruction test if lhs is zero or not
        mcss.move(rhs.code(), cmpTempRegister.code());
        mcss.move(mRegisterID(0), cmpTempRegister.code());
    }
    void testl(const Register &lhs, Imm32 rhs) {
//okm        masm.testl_i32r(rhs.value, lhs.code());
        mcss.move(lhs.code(), cmpTempRegister.code());
        mcss.move(mTrustedImm32(rhs.value), cmpTemp2Register.code());
        mcss.and32(cmpTemp2Register.code(), cmpTempRegister.code());
        mcss.move(mRegisterID(0), cmpTempRegister.code());
    }
    void testl(const Operand &lhs, Imm32 rhs) {
        switch (lhs.kind()) {
          case Operand::REG:
//okm            masm.testl_i32r(rhs.value, lhs.reg());
            mcss.move(lhs.reg(), cmpTempRegister.code());
            mcss.move(mTrustedImm32(rhs.value), cmpTemp2Register.code());
            mcss.and32(cmpTemp2Register.code(), cmpTempRegister.code());
            mcss.move(mRegisterID(0), cmpTempRegister.code());
            break;
          case Operand::REG_DISP:
//okm            masm.testl_i32m(rhs.value, lhs.disp(), lhs.base());
            mcss.load32(mAddress(lhs.base(), lhs.disp()), cmpTempRegister.code());
            mcss.move(mTrustedImm32(rhs.value), cmpTemp2Register.code());
            mcss.and32(cmpTemp2Register.code(), cmpTempRegister.code());
            mcss.move(mRegisterID(0), cmpTempRegister.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
            break;
        }
    }
    void addl(Imm32 imm, const Register &dest) {
//ok        masm.addl_ir(imm.value, dest.code());
       mcss.add32(mTrustedImm32(imm.value), dest.code());
    }
    void addl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.addl_ir(imm.value, op.reg());
            mcss.add32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.addl_im(imm.value, op.disp(), op.base());
            mcss.add32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
//ok            masm.addl_im(imm.value, op.address());
            mcss.load32(op.address(), dataTempRegister.code());
            mcss.add32(mTrustedImm32(imm.value), dataTempRegister.code());
            mcss.store32(dataTempRegister.code(), op.address());
            break;
//#endif
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void subl(Imm32 imm, const Register &dest) {
//ok        masm.subl_ir(imm.value, dest.code());
        mcss.sub32(mTrustedImm32(imm.value), dest.code());
    }
    void subl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.subl_ir(imm.value, op.reg());
            mcss.sub32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.subl_im(imm.value, op.disp(), op.base());
            mcss.sub32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void addl(const Register &src, const Register &dest) {
//ok        masm.addl_rr(src.code(), dest.code());
        mcss.add32(src.code(), dest.code());
    }
    void subl(const Register &src, const Register &dest) {
//ok        masm.subl_rr(src.code(), dest.code());
        mcss.sub32(src.code(), dest.code());
    }
    void subl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.subl_rr(src.reg(), dest.code());
            mcss.sub32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.subl_mr(src.disp(), src.base(), dest.code());
            mcss.sub32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void orl(const Register &reg, const Register &dest) {
//ok        masm.orl_rr(reg.code(), dest.code());
        mcss.or32(reg.code(), dest.code());
    }
    void orl(Imm32 imm, const Register &reg) {
//ok        masm.orl_ir(imm.value, reg.code());
        mcss.or32(mTrustedImm32(imm.value), reg.code());
    }
    void orl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.orl_ir(imm.value, op.reg());
            mcss.or32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.orl_im(imm.value, op.disp(), op.base());
            mcss.or32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void xorl(const Register &src, const Register &dest) {
//ok        masm.xorl_rr(src.code(), dest.code());
         mcss.xor32(src.code(), dest.code());
    }
    void xorl(Imm32 imm, const Register &reg) {
//ok        masm.xorl_ir(imm.value, reg.code());
        mcss.xor32(mTrustedImm32(imm.value), reg.code());
    }
    void xorl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.xorl_ir(imm.value, op.reg());
            mcss.xor32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.xorl_im(imm.value, op.disp(), op.base());
            mcss.xor32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void andl(Imm32 imm, const Register &dest) {
//ok        masm.andl_ir(imm.value, dest.code());
        mcss.and32(mTrustedImm32(imm.value), dest.code());
    }
    void andl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.andl_ir(imm.value, op.reg());
            mcss.and32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.andl_im(imm.value, op.disp(), op.base());
            mcss.and32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void addl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.addl_rr(src.reg(), dest.code());
            mcss.add32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.addl_mr(src.disp(), src.base(), dest.code());
            mcss.add32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void orl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.orl_rr(src.reg(), dest.code());
            mcss.or32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.orl_mr(src.disp(), src.base(), dest.code());
            mcss.or32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void xorl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.xorl_rr(src.reg(), dest.code());
            mcss.xor32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.xorl_mr(src.disp(), src.base(), dest.code());
            mcss.xor32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void andl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.andl_rr(src.reg(), dest.code());
            mcss.and32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.andl_mr(src.disp(), src.base(), dest.code());
            mcss.and32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void imull(Imm32 imm, const Register &dest) {
//ok        masm.imull_i32r(dest.code(), imm.value, dest.code());
        mcss.mul32(mTrustedImm32(imm.value), dest.code());
    }
    void imull(const Register &src, const Register &dest) {
//ok        masm.imull_rr(src.code(), dest.code());
        mcss.mul32(src.code(), dest.code());
    }
    void imull(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.imull_rr(src.reg(), dest.code());
            mcss.mul32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.imull_mr(src.disp(), src.base(), dest.code());
            mcss.mul32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void negl(const Operand &src) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.negl_r(src.reg());
            mcss.neg32(src.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.negl_m(src.disp(), src.base());
            mcss.neg32(mAddress(src.base(), src.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void negl(const Register &reg) {
//ok        masm.negl_r(reg.code());
        mcss.neg32(reg.code());
    }
    void notl(const Operand &src) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.notl_r(src.reg());
            mcss.not32(src.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.notl_m(src.disp(), src.base());
            mcss.not32(mAddress(src.base(), src.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }

    void shrl(const Imm32 imm, const Register &dest) {
//ok        masm.shrl_i8r(imm.value, dest.code());
        mcss.urshift32(mTrustedImm32(imm.value), dest.code());
    }
    void shll(const Imm32 imm, const Register &dest) {
//ok        masm.shll_i8r(imm.value, dest.code());
        mcss.lshift32(mTrustedImm32(imm.value), dest.code());
    }
    void sarl(const Imm32 imm, const Register &dest) {
//ok        masm.sarl_i8r(imm.value, dest.code());
        mcss.rshift32(mTrustedImm32(imm.value), dest.code());
    }
    void shrl_cl(const Register &dest) {
//        masm.shrl_CLr(dest.code());
    }
    void shll_cl(const Register &dest) {
//        masm.shll_CLr(dest.code());
    }
    void sarl_cl(const Register &dest) {
//        masm.sarl_CLr(dest.code());
    }

    void push(const Imm32 imm) {
//ok        masm.push_i32(imm.value);
        mcss.push(mTrustedImm32(imm.value));
    }

    void push(const Operand &src) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.push_r(src.reg());
            mcss.push(src.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.push_m(src.disp(), src.base());
            mcss.push(mAddress(src.base(), src.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void push(const Register &src) {
//ok        masm.push_r(src.code());
        mcss.push(src.code());
    }

    void pop(const Operand &src) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.pop_r(src.reg());
            mcss.push(src.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.pop_m(src.disp(), src.base());
            mcss.push(mAddress(src.base(), src.disp()));
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void pop(const Register &src) {
//ok        masm.pop_r(src.code());
        mcss.pop(src.code());
    }

#ifdef JS_CPU_X86
    void pushAllRegs() {
        masm.pusha();
    }
    void popAllRegs() {
        masm.popa();
    }
#endif

    // Zero-extend byte to 32-bit integer.
    void movzxbl(const Register &src, const Register &dest) {
//        masm.movzbl_rr(src.code(), dest.code());
    }

    void cdq() {
//        masm.cdq();
    }
    void idiv(Register dest) {
//        masm.idivl_r(dest.code());
    }

    void unpcklps(const FloatRegister &src, const FloatRegister &dest) {
//        masm.unpcklps_rr(src.code(), dest.code());
    }
    void pinsrd(const Register &src, const FloatRegister &dest) {
//        masm.pinsrd_rr(src.code(), dest.code());
    }
    void pinsrd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::REG:
//            masm.pinsrd_rr(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//            masm.pinsrd_mr(src.disp(), src.base(), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void psrldq(Imm32 shift, const FloatRegister &dest) {
//        masm.psrldq_rr(dest.code(), shift.value);
    }
    void psllq(Imm32 shift, const FloatRegister &dest) {
//        masm.psllq_rr(dest.code(), shift.value);
    }
    void psrlq(Imm32 shift, const FloatRegister &dest) {
//        masm.psrlq_rr(dest.code(), shift.value);
    }

    void cvtsi2sd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.cvtsi2sd_rr(src.reg(), dest.code());
            mcss.convertInt32ToDouble(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.cvtsi2sd_mr(src.disp(), src.base(), dest.code());
            mcss.convertInt32ToDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.cvtsi2sd_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.convertInt32ToDouble(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void cvttsd2si(const FloatRegister &src, const Register &dest) {
//ok        masm.cvttsd2si_rr(src.code(), dest.code());
        mcss.truncateDoubleToInt32(src.code(), dest.code());
    }
    void cvtsi2sd(const Register &src, const FloatRegister &dest) {
//ok        masm.cvtsi2sd_rr(src.code(), dest.code());
        mcss.convertInt32ToDouble(src.code(), dest.code());
    }
    void movmskpd(const FloatRegister &src, const Register &dest) {
//        masm.movmskpd_rr(src.code(), dest.code());
    }
    void ptest(const FloatRegister &lhs, const FloatRegister &rhs) {
//        JS_ASSERT(HasSSE41());
//        masm.ptest_rr(rhs.code(), lhs.code());
    }
    void ucomisd(const FloatRegister &lhs, const FloatRegister &rhs) {
//okm        masm.ucomisd_rr(rhs.code(), lhs.code());
        mcss.moveDouble(lhs.code(), fpTempRegister.code());
        mcss.moveDouble(rhs.code(), fpTemp2Register.code());
    }
    void pcmpeqw(const FloatRegister &lhs, const FloatRegister &rhs) {
//        masm.pcmpeqw_rr(rhs.code(), lhs.code());
    }    
    void movd(const Register &src, const FloatRegister &dest) {
//ok        masm.movd_rr(src.code(), dest.code());
        mcss.convertInt32ToDouble(src.code(),dest.code());
    }
    void movd(const FloatRegister &src, const Register &dest) {
//ok        masm.movd_rr(src.code(), dest.code());
        mcss.truncateDoubleToInt32(src.code(), dest.code());
    }
    void addsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.addsd_rr(src.code(), dest.code());
        mcss.addDouble(src.code(), dest.code());
    }
    void addsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.addsd_rr(src.fpu(), dest.code());
            mcss.addDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.addsd_mr(src.disp(), src.base(), dest.code());
            mcss.addDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
//ok            masm.addsd_mr(src.address(), dest.code());
            mcss.addDouble(src.address(), dest.code());
            break;
//#endif
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void subsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.subsd_rr(src.code(), dest.code());
        mcss.subDouble(src.code(), dest.code());
    }
    void subsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.subsd_rr(src.fpu(), dest.code());
            mcss.subDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.subsd_mr(src.disp(), src.base(), dest.code());
            mcss.subDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void mulsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.mulsd_rr(src.code(), dest.code());
        mcss.mulDouble(src.code(), dest.code());
    }
    void mulsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.mulsd_rr(src.fpu(), dest.code());
            mcss.mulDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.mulsd_mr(src.disp(), src.base(), dest.code());
            mcss.mulDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
    void divsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.divsd_rr(src.code(), dest.code());
        mcss.divDouble(src.code(), dest.code());
    }
    void divsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.divsd_rr(src.fpu(), dest.code());
            mcss.divDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.divsd_mr(src.disp(), src.base(), dest.code());
            mcss.divDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            JS_NOT_REACHED("unexpected operand kind");
        }
    }
//ok xorpd mainly used to clear fpScratchRegister, on mips, use zeroDouble instead
    void xorpd(const FloatRegister &src, const FloatRegister &dest) {
//        masm.xorpd_rr(src.code(), dest.code());
    }
//ok orpd only one use case
    void orpd(const FloatRegister &src, const FloatRegister &dest) {
//        masm.orpd_rr(src.code(), dest.code());
    }
//ok andpd only one use case
    void andpd(const FloatRegister &src, const FloatRegister &dest) {
//        masm.andpd_rr(src.code(), dest.code());
    }
    void sqrtsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.sqrtsd_rr(src.code(), dest.code());
        mcss.sqrtDouble(src.code(), dest.code());
    }
//ok only RoundDown, on mips use floor double instead
    void roundsd(const FloatRegister &src, const FloatRegister &dest)
//                 JSC::X86Assembler::RoundingMode mode)
    {
//ok        masm.roundsd_rr(src.code(), dest.code(), mode);
        mcss.floorDouble(src.code(), dest.code());
    }
    void fstp(const Operand &src) {//callWithABI
         switch (src.kind()) {
           case Operand::REG_DISP:
//             masm.fstp_m(src.disp(), src.base());
             break;
           default:
             JS_NOT_REACHED("unexpected operand kind");
         }
    }

    // Defined for compatibility with ARM's assembler
    uint32 actualOffset(uint32 x) {
        return x;
    }

    uint32 actualIndex(uint32 x) {
        return x;
    }

    void flushBuffer() {
    }

    void finish() {
    }

    // Patching.

    static size_t patchWrite_NearCallSize() {
//TBD ok
        return 16;
    }
    static uintptr_t getPointer(uint8 *instPtr) {
//TBD
        uintptr_t *ptr = ((uintptr_t *) instPtr) - 1;
        return *ptr;
    }
    // Write a relative call at the start location |dataLabel|.
    // Note that this DOES NOT patch data that comes before |label|.
    static void patchWrite_NearCall(CodeLocationLabel startLabel, CodeLocationLabel target) {
//TBD ok
        uint32_t *start = (uint32_t*)startLabel.raw();
        *start = 0;
        *(start + 1) = 0;
        *(start + 3) = 0;
        start += 2;
        *start = 0x0c000000;
        ptrdiff_t offset = target - startLabel - patchWrite_NearCallSize();
        offset &= 0x03ffffff;
        JS_ASSERT(int32(offset) == offset);
        *start |= offset >> 2;
    }

    static void patchWrite_Imm32(CodeLocationLabel dataLabel, Imm32 toWrite) {
//TBD
        *((int32 *) dataLabel.raw() - 1) = toWrite.value;
    }

    static void patchDataWithValueCheck(CodeLocationLabel data, ImmWord newData,
                                        ImmWord expectedData) {
//TBDok
        // The pointer given is a pointer to *after* the data.
//        uintptr_t *ptr = ((uintptr_t *) data.raw()) - 1;
//        JS_ASSERT(*ptr == expectedData.value);
//        *ptr = newData.value;
        uint32_t old = JSC::MIPSAssembler::getInt32(data.raw());
        JS_ASSERT(old == expectedData.value);
        JSC::MIPSAssembler::setInt32(((uint8_t *)data.raw()), (newData.value));
    }
    static uint32 nopSize() {
        return 1;
    }
    static uint8 *nextInstruction(uint8 *cur, uint32 *count) {
        JS_NOT_REACHED("nextInstruction NYI on x86");
    }

    // Toggle a jmp or cmp emitted by toggledJump().
    static void ToggleToJmp(CodeLocationLabel inst) {
        uint8_t *ptr = (uint8_t *)inst.raw();
        //CMP AX,imm16
        JS_ASSERT(*ptr == 0x3D);
        //JMP rel32
        *ptr = 0xE9;
    }
    static void ToggleToCmp(CodeLocationLabel inst) {
        uint8_t *ptr = (uint8_t *)inst.raw();
        JS_ASSERT(*ptr == 0xE9);
        *ptr = 0x3D;
    }
};

} // namespace ion
} // namespace js

#endif // jsion_assembler_x86_shared__


