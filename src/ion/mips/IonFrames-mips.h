/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef js_ion_frame_layouts_mips_h__
#define js_ion_frame_layouts_mips_h__

#include "ion/shared/IonFrames-shared.h"

namespace js {
namespace ion {

class IonCommonFrameLayout
{
  private:
    uint8 *returnAddress_;
    uintptr_t descriptor_;

    static const uintptr_t FrameTypeMask = (1 << FRAMETYPE_BITS) - 1;

  public:
    static size_t offsetOfDescriptor() {
        return offsetof(IonCommonFrameLayout, descriptor_);
    }
    static size_t offsetOfReturnAddress() {
        return offsetof(IonCommonFrameLayout, returnAddress_);
    }
    FrameType prevType() const {
        return FrameType(descriptor_ & FrameTypeMask);
    }
    void changePrevType(FrameType type) {
        descriptor_ &= ~FrameTypeMask;
        descriptor_ |= type;
    }
    size_t prevFrameLocalSize() const {
        return descriptor_ >> FRAMESIZE_SHIFT;
    }
    void setFrameDescriptor(size_t size, FrameType type) {
        descriptor_ = (size << FRAMESIZE_SHIFT) | type;
    }
    uint8 *returnAddress() const {
        return returnAddress_;
    }
};

class IonJSFrameLayout : public IonCommonFrameLayout
{
    void *calleeToken_;
    uintptr_t numActualArgs_;

  public:
    CalleeToken calleeToken() const {
        return calleeToken_;
    }
    void replaceCalleeToken(void *value) {
        calleeToken_ = value;
    }

    static size_t offsetOfCalleeToken() {
        return offsetof(IonJSFrameLayout, calleeToken_);
    }
    static size_t offsetOfNumActualArgs() {
        return offsetof(IonJSFrameLayout, numActualArgs_);
    }
    static size_t offsetOfActualArgs() {
        IonJSFrameLayout *base = NULL;
        // +1 to skip |this|.
        return reinterpret_cast<size_t>(&base->argv()[1]);
    }

    Value *argv() {
        return (Value *)(this + 1);
    }
    uintptr_t numActualArgs() const {
        return numActualArgs_;
    }

    // Computes a reference to a slot, where a slot is a distance from the base
    // frame pointer (as would be used for LStackSlot).
    uintptr_t *slotRef(uint32 slot) {
        return (uintptr_t *)((uint8 *)this - (slot * STACK_SLOT_SIZE));
    }

    static inline size_t Size() {
        return sizeof(IonJSFrameLayout);
    }
};

class IonEntryFrameLayout : public IonJSFrameLayout
{
  public:
    static inline size_t Size() {
        return sizeof(IonEntryFrameLayout);
    }
};

class IonRectifierFrameLayout : public IonJSFrameLayout
{
  public:
    static inline size_t Size() {
        return sizeof(IonRectifierFrameLayout);
    }
};

// The callee token is now dead.
class IonBailedRectifierFrameLayout : public IonRectifierFrameLayout
{
  public:
    static inline size_t Size() {
        return sizeof(IonBailedRectifierFrameLayout);
    }
};

// GC related data used to keep alive data surrounding the Exit frame.
class IonExitFooterFrame
{
    const VMFunction *function_;
    IonCode *ionCode_;

  public:
    static inline size_t Size() {
        return sizeof(IonExitFooterFrame);
    }
    inline IonCode *ionCode() const {
        return ionCode_;
    }
    inline IonCode **addressOfIonCode() {
        return &ionCode_;
    }
    inline const VMFunction *function() const {
        return function_;
    }

    // This should only be called for function()->outParam == Type_Handle
    Value *outVp() {
        return reinterpret_cast<Value *>(reinterpret_cast<char *>(this) - sizeof(Value));
    }
};

class IonNativeExitFrameLayout;
class IonOOLNativeGetterExitFrameLayout;
class IonOOLPropertyOpExitFrameLayout;
class IonDOMExitFrameLayout;

class IonExitFrameLayout : public IonCommonFrameLayout
{
    inline uint8 *top() {
        return reinterpret_cast<uint8 *>(this + 1);
    }

  public:
    static inline size_t Size() {
        return sizeof(IonExitFrameLayout);
    }
    static inline size_t SizeWithFooter() {
        return Size() + IonExitFooterFrame::Size();
    }

    inline IonExitFooterFrame *footer() {
        uint8 *sp = reinterpret_cast<uint8 *>(this);
        return reinterpret_cast<IonExitFooterFrame *>(sp - IonExitFooterFrame::Size());
    }

    // argBase targets the point which precedes the exit frame. Arguments of VM
    // each wrapper are pushed before the exit frame.  This correspond exactly
    // to the value of the argBase register of the generateVMWrapper function.
    inline uint8 *argBase() {
        JS_ASSERT(footer()->ionCode() != NULL);
        return top();
    }

    inline bool isWrapperExit() {
        return footer()->function() != NULL;
    }
    inline bool isNativeExit() {
        return footer()->ionCode() == NULL;
    }
    inline bool isOOLNativeGetterExit() {
        return footer()->ionCode() == ION_FRAME_OOL_NATIVE_GETTER;
    }
    inline bool isOOLPropertyOpExit() {
        return footer()->ionCode() == ION_FRAME_OOL_PROPERTY_OP;
    }
    inline bool isDomExit() {
        IonCode *code = footer()->ionCode();
        return
            code == ION_FRAME_DOMGETTER ||
            code == ION_FRAME_DOMSETTER ||
            code == ION_FRAME_DOMMETHOD;
    }

    inline IonNativeExitFrameLayout *nativeExit() {
        // see CodeGenerator::visitCallNative
        JS_ASSERT(isNativeExit());
        return reinterpret_cast<IonNativeExitFrameLayout *>(footer());
    }
    inline IonOOLNativeGetterExitFrameLayout *oolNativeGetterExit() {
        JS_ASSERT(isOOLNativeGetterExit());
        return reinterpret_cast<IonOOLNativeGetterExitFrameLayout *>(footer());
    }
    inline IonOOLPropertyOpExitFrameLayout *oolPropertyOpExit() {
        JS_ASSERT(isOOLPropertyOpExit());
        return reinterpret_cast<IonOOLPropertyOpExitFrameLayout *>(footer());
    }
    inline IonDOMExitFrameLayout *DOMExit() {
        JS_ASSERT(isDomExit());
        return reinterpret_cast<IonDOMExitFrameLayout *>(footer());
    }
};

class IonNativeExitFrameLayout
{
  protected: // only to silence a clang warning about unused private fields
    IonExitFooterFrame footer_;
    IonExitFrameLayout exit_;
    uintptr_t argc_;

    // We need to split the Value in 2 field of 32 bits, otherwise the C++
    // compiler may add some padding between the fields.
    uint32_t loCalleeResult_;
    uint32_t hiCalleeResult_;

  public:
    static inline size_t Size() {
        return sizeof(IonNativeExitFrameLayout);
    }

    static size_t offsetOfResult() {
        return offsetof(IonNativeExitFrameLayout, loCalleeResult_);
    }
    inline Value *vp() {
        return reinterpret_cast<Value*>(&loCalleeResult_);
    }
    inline uintptr_t argc() const {
        return argc_;
    }
};

class IonOOLNativeGetterExitFrameLayout
{
  protected: // only to silence a clang warning about unused private fields
    IonExitFooterFrame footer_;
    IonExitFrameLayout exit_;

    // We need to split the Value into 2 fields of 32 bits, otherwise the C++
    // compiler may add some padding between the fields.
    uint32_t loCalleeResult_;
    uint32_t hiCalleeResult_;

    // The frame includes the object argument.
    uint32_t loThis_;
    uint32_t hiThis_;

    // pointer to root the stub's IonCode
    IonCode *stubCode_;

  public:
    static inline size_t Size() {
        return sizeof(IonOOLNativeGetterExitFrameLayout);
    }

    static size_t offsetOfResult() {
        return offsetof(IonOOLNativeGetterExitFrameLayout, loCalleeResult_);
    }

    inline IonCode **stubCode() {
        return &stubCode_;
    }
    inline Value *vp() {
        return reinterpret_cast<Value*>(&loCalleeResult_);
    }
    inline Value *thisp() {
        return reinterpret_cast<Value*>(&loThis_);
    }

    inline uintptr_t argc() const {
        return 0;
    }
};

class IonOOLPropertyOpExitFrameLayout
{
  protected: // only to silence a clang warning about unused private fields
    IonExitFooterFrame footer_;
    IonExitFrameLayout exit_;

    // Object for JSHandleObject
    JSObject *obj_;

    // id for JSHandleId
    jsid id_;

    // space for JSMutableHandleValue result
    // use two uint32_t so compiler doesn't align.
    uint32_t vp0_;
    uint32_t vp1_;

    // pointer to root the stub's IonCode
    IonCode *stubCode_;

  public:
    static inline size_t Size() {
        return sizeof(IonOOLPropertyOpExitFrameLayout);
    }

    static size_t offsetOfResult() {
        return offsetof(IonOOLPropertyOpExitFrameLayout, vp0_);
    }

    inline IonCode **stubCode() {
        return &stubCode_;
    }
    inline Value *vp() {
        return reinterpret_cast<Value*>(&vp0_);
    }
    inline jsid *id() {
        return &id_;
    }
    inline JSObject **obj() {
        return &obj_;
    }
};

class IonDOMExitFrameLayout
{
  protected: // only to silence a clang warning about unused private fields
    IonExitFooterFrame footer_;
    IonExitFrameLayout exit_;
    JSObject *thisObj;

    // We need to split the Value in 2 field of 32 bits, otherwise the C++
    // compiler may add some padding between the fields.
    uint32_t loCalleeResult_;
    uint32_t hiCalleeResult_;

  public:
    static inline size_t Size() {
        return sizeof(IonDOMExitFrameLayout);
    }

    static size_t offsetOfResult() {
        return offsetof(IonDOMExitFrameLayout, loCalleeResult_);
    }
    inline Value *vp() {
        return reinterpret_cast<Value*>(&loCalleeResult_);
    }
    inline JSObject **thisObjAddress() {
        return &thisObj;
    }
    inline bool isSetterFrame() {
        return footer_.ionCode() == ION_FRAME_DOMSETTER;
    }
    inline bool isMethodFrame() {
        return footer_.ionCode() == ION_FRAME_DOMMETHOD;
    }
};

class IonDOMMethodExitFrameLayout
{
  protected: // only to silence a clang warning about unused private fields
    IonExitFooterFrame footer_;
    IonExitFrameLayout exit_;
    // This must be the last thing pushed, so as to stay common with
    // IonDOMExitFrameLayout.
    JSObject *thisObj_;
    uintptr_t argc_;

    Value CalleeResult_;

  public:
    static inline size_t Size() {
        return sizeof(IonDOMMethodExitFrameLayout);
    }

    static size_t offsetOfResult() {
        return offsetof(IonDOMMethodExitFrameLayout, CalleeResult_);
    }
    inline Value *vp() {
        JS_STATIC_ASSERT(offsetof(IonDOMMethodExitFrameLayout, CalleeResult_) ==
                         (offsetof(IonDOMMethodExitFrameLayout, argc_) + sizeof(uintptr_t)));
        return &CalleeResult_;
    }
    inline JSObject **thisObjAddress() {
        return &thisObj_;
    }
    inline uintptr_t argc() {
        return argc_;
    }
};

class IonOsrFrameLayout : public IonJSFrameLayout
{
  public:
    static inline size_t Size() {
        return sizeof(IonOsrFrameLayout);
    }
 };

// An invalidation bailout stack is at the stack pointer for the callee frame.
class InvalidationBailoutStack
{
    double      fpregs_[FloatRegisters::Total];
    uintptr_t   regs_[Registers::Total];
    IonScript   *ionScript_;
    uint8       *osiPointReturnAddress_;

  public:
    uint8 *sp() const {
        return (uint8 *) this + sizeof(InvalidationBailoutStack);
    }
    IonJSFrameLayout *fp() const;
    MachineState machine() {
        return MachineState::FromBailout(regs_, fpregs_);
    }

    IonScript *ionScript() const {
        return ionScript_;
    }
    uint8 *osiPointReturnAddress() const {
        return osiPointReturnAddress_;
    }

    void checkInvariants() const;
};

}
}

#endif // js_ion_frame_layouts_x86_h__


