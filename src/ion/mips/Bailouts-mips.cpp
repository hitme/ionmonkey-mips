/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jscntxt.h"
#include "jscompartment.h"
#include "ion/Bailouts.h"
#include "ion/IonCompartment.h"
#include "ion/IonFrames-inl.h"

using namespace js;
using namespace js::ion;

#if defined(_WIN32)
# pragma pack(push, 1)
#endif

namespace js {
namespace ion {

class BailoutStack
{
    uintptr_t frameClassId_;
    uintptr_t padding;
    double    fpregs_[FloatRegisters::Total];
    uintptr_t regs_[Registers::Total];
    union {
        uintptr_t frameSize_;
        uintptr_t tableOffset_;
    };
    uintptr_t snapshotOffset_;//maybe unaligned

  public:
    FrameSizeClass frameClass() const {
        return FrameSizeClass::FromClass(frameClassId_);
    }
    uintptr_t tableOffset() const {
        JS_ASSERT(frameClass() != FrameSizeClass::None());
        return tableOffset_;
    }
    uint32 frameSize() const {
        if (frameClass() == FrameSizeClass::None())
            return frameSize_;
        return frameClass().frameSize();
    }
    MachineState machine() {
        return MachineState::FromBailout(regs_, fpregs_);
    }
    SnapshotOffset snapshotOffset() const {
        JS_ASSERT(frameClass() == FrameSizeClass::None());
        return snapshotOffset_;
    }
    uint8 *parentStackPointer() const {
        if (frameClass() == FrameSizeClass::None())
            return (uint8 *)this + sizeof(BailoutStack);
        return (uint8 *)this + offsetof(BailoutStack, snapshotOffset_);
    }
};

} // namespace ion
} // namespace js

#if defined(_WIN32)
# pragma pack(pop)
#endif

IonBailoutIterator::IonBailoutIterator(const IonActivationIterator &activations,
                                       BailoutStack *bailout)
  : IonFrameIterator(activations),
    machine_(bailout->machine())
{
    uint8 *sp = bailout->parentStackPointer();//off of snapshotOffset_
    uint8 *fp = sp + bailout->frameSize();

    current_ = fp;
    type_ = IonFrame_OptimizedJS;
    topFrameSize_ = current_ - sp;
    topIonScript_ = script()->ion;

    if (bailout->frameClass() == FrameSizeClass::None()) {
        snapshotOffset_ = bailout->snapshotOffset();
        return;
    }

    // Compute the snapshot offset from the bailout ID.
    IonActivation *activation = activations.activation();
    JSCompartment *jsCompartment = activation->compartment();
    IonCompartment *ionCompartment = jsCompartment->ionCompartment();
    IonCode *code = ionCompartment->getBailoutTable(bailout->frameClass());
    uintptr_t tableOffset = bailout->tableOffset();
    uintptr_t tableStart = reinterpret_cast<uintptr_t>(code->raw());

    JS_ASSERT(tableOffset >= tableStart &&
              tableOffset < tableStart + code->instructionsSize());
    JS_ASSERT((tableOffset - tableStart) % BAILOUT_TABLE_ENTRY_SIZE == 0);

    uint32 bailoutId = ((tableOffset - tableStart) / BAILOUT_TABLE_ENTRY_SIZE) - 1;
    JS_ASSERT(bailoutId < BAILOUT_TABLE_SIZE);

    snapshotOffset_ = topIonScript_->bailoutToSnapshot(bailoutId);
}

IonBailoutIterator::IonBailoutIterator(const IonActivationIterator &activations,
                                       InvalidationBailoutStack *bailout)
  : IonFrameIterator(activations),
    machine_(bailout->machine())
{
    returnAddressToFp_ = bailout->osiPointReturnAddress();
    topIonScript_ = bailout->ionScript();
    const OsiIndex *osiIndex = topIonScript_->getOsiIndex(returnAddressToFp_);

    current_ = (uint8*) bailout->fp();
    type_ = IonFrame_OptimizedJS;
    topFrameSize_ = current_ - bailout->sp();
    snapshotOffset_ = osiIndex->snapshotOffset();
}

