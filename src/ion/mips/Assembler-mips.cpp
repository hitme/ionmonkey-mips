/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "ion/IonMacroAssembler.h"
#include "gc/Marking.h"

#include "jsscriptinlines.h"

using namespace js;
using namespace js::ion;

void
Assembler::copyJumpRelocationTable(uint8 *dest)
{
    if (jumpRelocations_.length())
        memcpy(dest, jumpRelocations_.buffer(), jumpRelocations_.length());
}

void
Assembler::copyDataRelocationTable(uint8 *dest)
{
    if (dataRelocations_.length())
        memcpy(dest, dataRelocations_.buffer(), dataRelocations_.length());
}

static void
TraceDataRelocations(JSTracer *trc, uint8 *buffer, CompactBufferReader &reader)
{
    while (reader.more()) {
        size_t offset = reader.readUnsigned();
        void **ptr = JSC::MIPSAssembler::getPointerRef(buffer + offset);

#ifdef JS_PUNBOX64
        // All pointers on x64 will have the top bits cleared. If those bits
        // are not cleared, this must be a Value.
        uintptr_t *word = reinterpret_cast<uintptr_t *>(ptr);
        if (*word >> JSVAL_TAG_SHIFT) {
            jsval_layout layout;
            layout.asBits = *word;
            Value v = IMPL_TO_JSVAL(layout);
            gc::MarkValueUnbarriered(trc, &v, "ion-masm-value");
            JS_ASSERT(*word == JSVAL_TO_IMPL(v).asBits);
            continue;
        }
#endif

        // No barrier needed since these are constants.
        gc::MarkGCThingUnbarriered(trc, reinterpret_cast<void **>(ptr), "ion-masm-ptr");
    }
}

void
Assembler::TraceDataRelocations(JSTracer *trc, IonCode *code, CompactBufferReader &reader)
{
    ::TraceDataRelocations(trc, code->raw(), reader);
}

void
Assembler::trace(JSTracer *trc)
{
    for (size_t i = 0; i < jumps_.length(); i++) {
        RelativePatch &rp = jumps_[i];
        if (rp.kind == Relocation::IONCODE) {
            IonCode *code = IonCode::FromExecutable((uint8 *)rp.target);
            MarkIonCodeUnbarriered(trc, &code, "masmrel32");
            JS_ASSERT(code == IonCode::FromExecutable((uint8 *)rp.target));
        }
    }
    if (dataRelocations_.length()) {
        CompactBufferReader reader(dataRelocations_);
        ::TraceDataRelocations(trc, masm.buffer(), reader);
    }
}

void
Assembler::executableCopy(void *buffer)
{
    masm.executableCopy(buffer);
}

void
Assembler::processDeferredData(IonCode *code, uint8 *data)
{
    for (size_t i = 0; i < data_.length(); i++) {
        DeferredData *deferred = data_[i];
        Bind(code, deferred->label(), data + deferred->offset());
        deferred->copy(code, data + deferred->offset());
    }
}

void
Assembler::processCodeLabels(IonCode *code)
{
    for (size_t i = 0; i < codeLabels_.length(); i++) {
        CodeLabel *label = codeLabels_[i];
        Bind(code, label->dest(), code->raw() + label->src()->offset());
    }
}

Assembler::Condition
Assembler::InvertCondition(Condition cond)
{
    switch (cond) {
      case Zero:
        return NonZero;
      case NonZero:
        return Zero;
      case LessThan:
        return GreaterThanOrEqual;
      case LessThanOrEqual:
        return GreaterThan;
      case GreaterThan:
        return LessThanOrEqual;
      case GreaterThanOrEqual:
        return LessThan;
      case Above:
        return BelowOrEqual;
      case AboveOrEqual:
        return Below;
      case Below:
        return AboveOrEqual;
      case BelowOrEqual:
        return Above;
      default:
        JS_NOT_REACHED("unexpected condition");
        return Equal;
    }
}

void
AutoFlushCache::update(uintptr_t newStart, size_t len)
{
}

AutoFlushCache::~AutoFlushCache()
{
    if (!myCompartment_)
        return;

    if (myCompartment_->flusher() == this)
        myCompartment_->setFlusher(NULL);
}

//x86
void
Assembler::executableCopy(uint8 *buffer)
{
    masm.executableCopy(buffer);

    for (size_t i = 0; i < jumps_.length(); i++) {
        RelativePatch &rp = jumps_[i];
//ok        JSC::MIPSAssembler::setRel32(buffer + rp.offset, rp.target);
        mcss.repatchJump(JSC::CodeLocationJump(buffer + rp.offset), JSC::CodeLocationLabel(rp.target));
    }
}

class RelocationIterator
{
    CompactBufferReader reader_;
    uint32 offset_;

  public:
    RelocationIterator(CompactBufferReader &reader)
      : reader_(reader)
    { }

    bool read() {
        if (!reader_.more())
            return false;
        offset_ = reader_.readUnsigned();
        return true;
    }

    uint32 offset() const {
        return offset_;
    }
};

static inline IonCode *
CodeFromJump(uint8 *jump)
{
    uint8 *target = (uint8 *)JSC::MIPSAssembler::getRel32Target(jump);
    return IonCode::FromExecutable(target);
}

void
Assembler::TraceJumpRelocations(JSTracer *trc, IonCode *code, CompactBufferReader &reader)
{
    RelocationIterator iter(reader);
    while (iter.read()) {
        IonCode *child = CodeFromJump(code->raw() + iter.offset());
        MarkIonCodeUnbarriered(trc, &child, "rel32");
        JS_ASSERT(child == CodeFromJump(code->raw() + iter.offset()));
    }
}

//x86 like

void
Assembler::patchWrite_Imm32(CodeLocationLabel dataLabel, Imm32 toWrite) {
   // ASSERT(0);
//TBD
//ok    *((int32 *) dataLabel.raw() - 1) = toWrite.value;
    // special patching for calls like "call *%ebx"
    JSC::MIPSAssembler::setInt32((int32 *) dataLabel.raw(), toWrite.value);
}

void
Assembler::fstp(const Operand &src) {//callWithABI
    ASSERT(0);//ok
     switch (src.kind()) {
       case Operand::REG_DISP:
//             masm.fstp_m(src.disp(), src.base());
         break;
       default:
         JS_NOT_REACHED("unexpected operand kind");
     }
}

void
Assembler::andpd(const FloatRegister &src, const FloatRegister &dest) {
    ASSERT(0);//ok
	//      masm.andpd_rr(src.code(), dest.code());
}

void
Assembler::orpd(const FloatRegister &src, const FloatRegister &dest) {
    ASSERT(0);//ok
	//      masm.orpd_rr(src.code(), dest.code());
}
void
Assembler::xorpd(const FloatRegister &src, const FloatRegister &dest) {
    ASSERT(src.code() == dest.code());
    zerod(src);//ok
//        masm.xorpd_rr(src.code(), dest.code());
}
void
Assembler::pcmpeqw(const FloatRegister &lhs, const FloatRegister &rhs) {
    ASSERT(0);//ok
    //only usecase MacroAssemblerMIPS:maybeInlineDouble CodeGeneratorMIPS:visitNegD
//        masm.pcmpeqw_rr(rhs.code(), lhs.code());
}    
void
Assembler::ptest(const FloatRegister &lhs, const FloatRegister &rhs) {
    ASSERT(0);//ok
//        JS_ASSERT(HasSSE41());
//        masm.ptest_rr(rhs.code(), lhs.code());
}
void
Assembler::movmskpd(const FloatRegister &src, const Register &dest) {
    //ASSERT(0);//ok extract sign bit from src to dest
    masm.dmfc1(mRegisterID(src.code()), mFPRegisterID(dest.code()));
    masm.dsrl32(mRegisterID(dest.code()), mRegisterID(dest.code()), 31);
//        masm.movmskpd_rr(src.code(), dest.code());
}

void
Assembler::cdq() {
//ok    ASSERT(0);
//        masm.cdq();
}
void
Assembler::idiv(Register dest) {
//ok    ASSERT(0);
        masm.div(t6.code(), dest.code());
        masm.mflo(dest.code());
//        masm.idivl_r(dest.code());
}

void
Assembler::unpcklps(const FloatRegister &src, const FloatRegister &dest) {
    ASSERT(0); //ok
//        masm.unpcklps_rr(src.code(), dest.code());
}
void
Assembler::pinsrd(const Register &src, const FloatRegister &dest) {
    ASSERT(0); //ok
//        masm.pinsrd_rr(src.code(), dest.code());
}
void
Assembler::pinsrd(const Operand &src, const FloatRegister &dest) {
    ASSERT(0); //ok
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
void
Assembler::psrldq(Imm32 shift, const FloatRegister &dest) {
    ASSERT(0);//ok
//        masm.psrldq_rr(dest.code(), shift.value);
}
void
Assembler::psllq(Imm32 shift, const FloatRegister &dest) {
    ASSERT(0);//ok
    //only usecase MacroAssemblerMIPS:maybeInlineDouble CodeGeneratorMIPS:visitNegD
//        masm.psllq_rr(dest.code(), shift.value);
}
void
Assembler::psrlq(Imm32 shift, const FloatRegister &dest) {
    ASSERT(0);//ok
    //only usecase maybeInlineDouble
//        masm.psrlq_rr(dest.code(), shift.value);
}

void
Assembler::shrl_cl(const Register &dest) {
    //ASSERT(0);
//ok        masm.shrl_CLr(dest.code());
    mcss.urshift32(mRegisterID(v0.code()), dest.code());
}
void
Assembler::shll_cl(const Register &dest) {
    //ASSERT(0);
//ok       masm.shll_CLr(dest.code());
    mcss.lshift32(mRegisterID(v0.code()), dest.code());
}
void
Assembler::sarl_cl(const Register &dest) {
    //ASSERT(0);
//ok        masm.sarl_CLr(dest.code());
    mcss.rshift32(mRegisterID(v0.code()), dest.code());
}

/*x86 gen sequence with setCC followed by movzxbl
 *cmpl       $0x0, %eax
 *sete       %dl
 *movzbl      %edx, %edx
 */
void
Assembler::setCC(Condition cond, const Register &r) {
//ok        masm.setCC_r(static_cast<JSC::X86Assembler::Condition>(cond), r.code());
    //ASSERT(0);
    //mcss.set8(static_cast<JSC::MIPSAssembler::Condition>(cond), v0.code(), mImm32(0), dest.code());
    mcss.set32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond), v0.code(), mImm32(0), r.code());
}
// Zero-extend byte to 32-bit integer.
void
Assembler::movzxbl(const Register &src, const Register &dest) {
//ok        masm.movzbl_rr(src.code(), dest.code());
    //ASSERT(0);
}
// Comparison of EAX against the address given by a Label.
Assembler::JmpSrc
Assembler::cmpSrc(Label *label) {
    ASSERT(0);
//ok:no usage        JmpSrc j = masm.cmp_eax();
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
// Toggle a jmp or cmp emitted by toggledJump().
void
Assembler::ToggleToJmp(CodeLocationLabel inst) {
    uint8_t *ptr = (uint8_t *)inst.raw();
    //CMP AX,imm16
    JS_ASSERT(*ptr == 0x3D);
    //JMP rel32
    *ptr = 0xE9;
}
void
Assembler::ToggleToCmp(CodeLocationLabel inst) {
    uint8_t *ptr = (uint8_t *)inst.raw();
    JS_ASSERT(*ptr == 0xE9);
    *ptr = 0x3D;
}
void
Assembler::retn(Imm32 n) {
    // Remove the size of the return address which is included in the frame.
//okm   masm.ret(n.value - sizeof(void *));
    pop(ra);
    mcss.ret((n.value - sizeof(void *)));
    //mcss.ret((n.value));
}
void 
Assembler::call(IonCode *target) {
//ok        JmpSrc src = masm.call();
//ok    //arm : ma_callIonHalfPush from MacroAsse..-arm.h
    mcss.offsetFromPCToV0(sizeof(int*)*7);//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call().m_jmp;//4insns
    addPendingJump(src, target->raw(), Relocation::IONCODE);
}
void 
Assembler::call(ImmWord target) {
//ok        JmpSrc src = masm.call();
    //arm : ma_call((void *) word.value);
//    mcss.offsetFromPCToV0(sizeof(int*)*7);//2insns
//    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call().m_jmp;
    addPendingJump(src, target.asPointer(), Relocation::HARDCODED);
}

Assembler::JmpSrc
Assembler::callWithPush() 
{
    mcss.offsetFromPCToV0(sizeof(int*)*7);//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call().m_jmp;//4insns
    return src;
}

Assembler::JmpSrc
Assembler::callRelWithPush() 
{
    mcss.offsetFromPCToV0(sizeof(int*)*7);//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.callRel().m_jmp;//4insns
    return src;
}

void 
Assembler::call(Label *label) {
    if (label->bound()) {
//ok            masm.linkJump(mcss.call(), JmpDst(label->offset()));
//ok    masm.linkJump(mcss.call().m_jmp, JmpDst(label->offset()));
    masm.linkJump(callRelWithPush(), JmpDst(label->offset()));
} else {
//ok            JmpSrc j = mcss.call();
//ok        JmpSrc j = mcss.call().m_jmp;
        JmpSrc j = callRelWithPush();
        JmpSrc prev = JmpSrc(label->use(j.offset()));
        masm.setNextJump(j, prev);
    }
}
void 
Assembler::call(const Register &reg) {
//ok    mcss.call(reg.code());
    ma_callIonHalfPush(reg);
}
void 
Assembler::call(const Operand &op) {
    switch (op.kind()) {
      case Operand::REG:
//ok        mcss.call(op.reg());
        ma_callIonHalfPush(Register::FromCode((int)(op.reg()))); //force cast
        break;
      case Operand::REG_DISP:
//ok            masm.call_m(op.disp(), op.base());
//ok        mcss.call(mAddress(op.base(), op.disp()));
        mcss.load32(mAddress(op.base(), op.disp()), v1.code());
        ma_callIonHalfPush(v1);
        break;
      default:
        JS_NOT_REACHED("unexpected operand kind");
    }
}
// ARM says that all reads of pc will return 8 higher than the
// address of the currently executing instruction.  This means we are
// correctly storing the address of the instruction after the call
// in the register.
// Also ION is breaking the ARM EABI here (sort of). The ARM EABI
// says that a function call should move the pc into the link register,
// then branch to the function, and *sp is data that is owned by the caller,
// not the callee.  The ION ABI says *sp should be the address that
// we will return to when leaving this function
Assembler::JmpSrc
Assembler::ma_callIon(const Register r)
{
    // When the stack is 8 byte aligned,
    // we want to decrement sp by 8, and write pc+8 into the new sp.
    // when we return from this call, sp will be its present value minus 4.
    //as_dtr(IsStore, 32, PreIndex, pc, DTRAddr(sp, DtrOffImm(-8)));
    //as_blx(r);
//ok
    mcss.offsetFromPCToV0(sizeof(int*)*8);//1insns
    mcss.sub32(mTrustedImm32(4), sp.code());//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call(r.code()).m_jmp;//2insns
    return src;
}

Assembler::JmpSrc
Assembler::ma_callIonNoPush(const Register r)
{
    // Since we just write the return address into the stack, which is
    // popped on return, the net effect is removing 4 bytes from the stack
    //as_dtr(IsStore, 32, Offset, pc, DTRAddr(sp, DtrOffImm(0)));
//ok    //as_blx(r);
    mcss.offsetFromPCToV0(sizeof(int*)*8);//1insns
    mcss.add32(mTrustedImm32(4), sp.code());//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call(r.code()).m_jmp;//2insns
    return src;
}

Assembler::JmpSrc
Assembler::ma_callIonHalfPush(const Register r)
{
    // The stack is unaligned by 4 bytes.
    // We push the pc to the stack to align the stack before the call, when we
    // return the pc is poped and the stack is restored to its unaligned state.
    //ma_push(pc);
    //as_blx(r);
    mcss.offsetFromPCToV0(sizeof(int*)*7);//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call(r.code()).m_jmp;//2insns
    return src;
}

Assembler::JmpSrc
Assembler::ma_call(void *dest) // KEEP EMPTY
{
    JS_NOT_REACHED("no use!");
    //ma_mov(Imm32((uint32)dest), CallReg);
    //as_blx(CallReg);
}

size_t 
Assembler::patchWrite_NearCallSize() {
//TBD ok
    return 36;
}

static unsigned int* __getpc(void)
{
    unsigned int *rtaddr;
    
    __asm__ volatile ("move %0, $31" : "=r"(rtaddr));
    
    return rtaddr;
}

// Write a relative call at the start location |dataLabel|.
// Note that this DOES NOT patch data that comes before |label|.
void 
Assembler::patchWrite_NearCall(CodeLocationLabel startLabel, CodeLocationLabel target) {
//TBD ok
    unsigned lw, hg;
    hg = ((unsigned int)__getpc)>>16;
    lw = ((unsigned int)__getpc)&0xffff;

    uint32_t *start = (uint32_t*)startLabel.raw();
    uint32_t *to = (uint32_t*)target.raw();
//    JS_ASSERT((reinterpret_cast<intptr_t>(start)) >> 28 == (reinterpret_cast<intptr_t>(to)) >> 28);
//    JS_ASSERT((*start & 0xffffffc0) == 0x14000000);
//    JS_ASSERT(*(start + 1) == 0);
//    JS_ASSERT((*(start + 2) & 0xffe00000) == 0x3c000000);
//    JS_ASSERT((*(start + 3) & 0xfc000000) == 0x34000000);
//    JS_ASSERT(*(start + 4) == 0x03200008);
//    JS_ASSERT(*(start + 5) == 0);
//    JS_ASSERT(*(start + 6) == 0);
//    JS_ASSERT(*(start + 7) == 0);
//    JS_ASSERT(*(start + 8) == 0);

    *start = 0x3c190000 | hg;
    *(start + 1) = 0x37390000 | lw;
    *(start + 2) = 0x0320f809;
    *(start + 4) = 0x24420000 | 0x14;
    *(start + 5) = 0x27bdfffc;
    *(start + 6) = 0xafa20000;
    *(start + 7) = 0x0c000000 | (((reinterpret_cast<intptr_t>(to)) >> 2) & 0x3ffffff);
}

uintptr_t 
Assembler::getPointer(uint8 *instPtr) {
//TBD
    uintptr_t ptr = reinterpret_cast<uintptr_t>(JSC::MIPSAssembler::getPointer(instPtr));
    //uintptr_t *ptr = ((uintptr_t *) instPtr) - 1;
    return ptr;
}
uint32 
Assembler::nopSize() {
    return 4;
}
void 
Assembler::absd(const FloatRegister &src) {
    mcss.absDouble(mFPRegisterID(src.code()), mFPRegisterID(src.code()));
}
void 
Assembler::zerod(const FloatRegister &src) {
    mcss.zeroDouble(mFPRegisterID(src.code()));
}
void 
Assembler::negd(const FloatRegister &src, const FloatRegister &dest) {
    mcss.negDouble(mFPRegisterID(src.code()), mFPRegisterID(dest.code()));
}
