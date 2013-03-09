/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "Architecture-mips.h"

namespace js {
namespace ion {

//class Registers
 
const uint32 Registers::Total = 30; //TBD:must be smaller than MIN_REG_FIELD_ESC(30), defined in Snapshots.cpp
 
const uint32 Registers::Allocatable = 20; 

const uint32 Registers::AllMask = (1 << Total) - 1;
 
const uint32 Registers::ArgRegMask = 0;

 
const uint32 Registers::VolatileMask =
#if 0
        (1 << JSC::MIPSRegisters::t0) |
        (1 << JSC::MIPSRegisters::t3) |
        (1 << JSC::MIPSRegisters::t5) |
#endif
        (1 << JSC::MIPSRegisters::t1) |
        (1 << JSC::MIPSRegisters::t2) |
        (1 << JSC::MIPSRegisters::t4) |
        (1 << JSC::MIPSRegisters::t6) |
        (1 << JSC::MIPSRegisters::t7) |
        (1 << JSC::MIPSRegisters::t8);

 
const uint32 Registers::NonVolatileMask =
        (1 << JSC::MIPSRegisters::s0) |
        (1 << JSC::MIPSRegisters::s1) |
        (1 << JSC::MIPSRegisters::s2) |
        (1 << JSC::MIPSRegisters::s3) |
        (1 << JSC::MIPSRegisters::s4) |
        (1 << JSC::MIPSRegisters::s5) |
        (1 << JSC::MIPSRegisters::s6) |
        (1 << JSC::MIPSRegisters::s7);

const uint32 Registers::WrapperMask = VolatileMask;
 
const uint32 Registers::SingleByteRegs =
        VolatileMask | NonVolatileMask |
        (1 << JSC::MIPSRegisters::v0);

 
const uint32 Registers::NonAllocatableMask =
        (1 << JSC::MIPSRegisters::zero) |
        (1 << JSC::MIPSRegisters::at) |
        (1 << JSC::MIPSRegisters::v0) |
        (1 << JSC::MIPSRegisters::v1) |
        (1 << JSC::MIPSRegisters::a0) |
        (1 << JSC::MIPSRegisters::a1) |
        (1 << JSC::MIPSRegisters::a2) |
        (1 << JSC::MIPSRegisters::a3) |
#if 1
        (1 << JSC::MIPSRegisters::t0) |
        (1 << JSC::MIPSRegisters::t3) |
        (1 << JSC::MIPSRegisters::t5) |
#endif
        (1 << JSC::MIPSRegisters::t9) | // t9 = scratch
        (1 << JSC::MIPSRegisters::k0) |
        (1 << JSC::MIPSRegisters::k1) |
        (1 << JSC::MIPSRegisters::gp) |
        (1 << JSC::MIPSRegisters::sp) |
        (1 << JSC::MIPSRegisters::fp) |
        (1 << JSC::MIPSRegisters::ra);

const uint32 Registers::TempMask = VolatileMask & ~NonAllocatableMask;
    // Registers returned from a JS -> JS call.
 
const uint32 Registers::JSCallMask =
        (1 << JSC::MIPSRegisters::a0) |
        (1 << JSC::MIPSRegisters::a1);

    // Registers returned from a JS -> C call.
 
const uint32 Registers::CallMask =
        (1 << JSC::MIPSRegisters::v0) |
        (1 << JSC::MIPSRegisters::v1);  // used for double-size returns

const uint32 Registers::AllocatableMask = AllMask & ~NonAllocatableMask;
 
const uint32 FloatRegisters::Total = 32;
 
const uint32 FloatRegisters::Allocatable = 31;

const uint32 FloatRegisters::AllMask = (1 << Total) - 1;
 
const uint32 FloatRegisters::VolatileMask = AllMask;
 
const uint32 FloatRegisters::NonVolatileMask = 0;

const uint32 FloatRegisters::WrapperMask = VolatileMask;
    // d0 is the ARM scratch float register.
 
const uint32 FloatRegisters::NonAllocatableMask = (1 << JSC::MIPSRegisters::f0);
const uint32 FloatRegisters::TempMask = VolatileMask & ~NonAllocatableMask;

const uint32 FloatRegisters::AllocatableMask = AllMask & ~NonAllocatableMask;
} // namespace ion
} // namespace js
