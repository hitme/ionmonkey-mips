/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_move_resolver_mips_h__
#define jsion_move_resolver_mips_h__

#include "ion/MoveResolver.h"
#include "ion/IonMacroAssembler.h"

namespace js {
namespace ion {

class CodeGenerator;

class MoveEmitterMIPS
{
    typedef MoveResolver::Move Move;
    typedef MoveResolver::MoveOperand MoveOperand;

    bool inCycle_;
    MacroAssemblerSpecific &masm;

    // Original stack push value.
    uint32 pushedAtStart_;

    // These store stack offsets to spill locations, snapshotting
    // codegen->framePushed_ at the time they were allocated. They are -1 if no
    // stack space has been allocated for that particular spill.
    int32 pushedAtCycle_;
    int32 pushedAtSpill_;

    // Register that is available for temporary use. It may be assigned
    // InvalidReg. If no corresponding spill space has been assigned,
    // then this register do not need to be spilled.
    Register spilledReg_;

    void assertDone();
    Register tempReg();
    Operand cycleSlot() const;
    Operand spillSlot() const;
    Operand toOperand(const MoveOperand &operand) const;

    void emitMove(const MoveOperand &from, const MoveOperand &to);
    void emitDoubleMove(const MoveOperand &from, const MoveOperand &to);
    void breakCycle(const MoveOperand &from, const MoveOperand &to, Move::Kind kind);
    void completeCycle(const MoveOperand &from, const MoveOperand &to, Move::Kind kind);
    void emit(const Move &move);

  public:
    MoveEmitterMIPS(MacroAssemblerSpecific &masm);
    ~MoveEmitterMIPS();
    void emit(const MoveResolver &moves);
    void finish();
};

typedef MoveEmitterMIPS MoveEmitter;

} // ion
} // js

#endif // jsion_move_resolver_x86_shared_h__


