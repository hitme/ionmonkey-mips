/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_architecture_mips_h__
#define jsion_architecture_mips_h__

#include "assembler/assembler/MacroAssembler.h"

namespace js {
namespace ion {

static const ptrdiff_t STACK_SLOT_SIZE       = 4;
static const uint32 DOUBLE_STACK_ALIGNMENT   = 2;

// In bytes: slots needed for potential memory->memory move spills.
//   +8 for cycles
//   +4 for gpr spills
//   +8 for double spills
static const uint32 ION_FRAME_SLACK_SIZE    = 20;

// An offset that is illegal for a local variable's stack allocation.
static const int32 INVALID_STACK_SLOT       = -1;

// These offsets are specific to nunboxing, and capture offsets into the
// components of a js::Value.
static const int32 NUNBOX32_TYPE_OFFSET         = 4;
static const int32 NUNBOX32_PAYLOAD_OFFSET      = 0;

////
// These offsets are related to bailouts.
////

// Size of each bailout table entry. On arm, this is presently
// a single call (which is wrong!). the call clobbers lr.
// For now, I've dealt with this by ensuring that we never allocate to lr.
// it should probably be 8 bytes, a mov of an immediate into r12 (not
// allocated presently, or ever) followed by a branch to the apropriate code.
//ok static const uint32 BAILOUT_TABLE_ENTRY_SIZE    = 4;
static const uint32 BAILOUT_TABLE_ENTRY_SIZE    = 4*11; //see to Trampoline-mips.cpp:generateBailoutTable

class Registers
{
public:
    typedef JSC::MIPSRegisters::RegisterID Code;

    static const char *GetName(Code code) {
        static const char *Names[] = { "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
                                       "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
                                       "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
                                       "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"};
        
        return Names[code];
    }

    static const Code StackPointer = JSC::MIPSRegisters::sp; 
    static const Code Invalid = JSC::MIPSRegisters::invalid_reg;

    static const uint32 Total = 32; //TBD:must be smaller than MIN_REG_FIELD_ESC(30), defined in Snapshots.cpp
    static const uint32 Allocatable = 11; 

    //static const uint32 AllMask = (1 << Total) - 1;
    static const uint32 AllMask = 0xffffffff;
    //static const uint32 ArgRegMask = 0x1f;
    static const uint32 ArgRegMask = 0;

    static const uint32 VolatileMask =
        (1 << JSC::MIPSRegisters::t6) |
        (1 << JSC::MIPSRegisters::t7) |
        (1 << JSC::MIPSRegisters::t8);

    static const uint32 NonVolatileMask =
        (1 << JSC::MIPSRegisters::s0) |
        (1 << JSC::MIPSRegisters::s1) |
        (1 << JSC::MIPSRegisters::s2) |
        (1 << JSC::MIPSRegisters::s3) |
        (1 << JSC::MIPSRegisters::s4) |
        (1 << JSC::MIPSRegisters::s5) |
        (1 << JSC::MIPSRegisters::s6) |
        (1 << JSC::MIPSRegisters::s7);

    static const uint32 WrapperMask = VolatileMask;
#if 0
    static const uint32 WrapperMask =
        VolatileMask |         // = arguments
        (1 << JSC::MIPSRegisters::a2) | // = outReg
        (1 << JSC::MIPSRegisters::a3);  // = argBase
#endif
    static const uint32 SingleByteRegs =
        VolatileMask | NonVolatileMask |
        (1 << JSC::MIPSRegisters::v0);

    static const uint32 NonAllocatableMask =
        (1 << JSC::MIPSRegisters::zero) |
        (1 << JSC::MIPSRegisters::at) |
        (1 << JSC::MIPSRegisters::v0) |
        (1 << JSC::MIPSRegisters::v1) |
        (1 << JSC::MIPSRegisters::a0) |
        (1 << JSC::MIPSRegisters::a1) |
        (1 << JSC::MIPSRegisters::a2) |
        (1 << JSC::MIPSRegisters::a3) |
        (1 << JSC::MIPSRegisters::t0) |
        (1 << JSC::MIPSRegisters::t1) |
        (1 << JSC::MIPSRegisters::t2) |
        (1 << JSC::MIPSRegisters::t3) |
        (1 << JSC::MIPSRegisters::t4) |
        (1 << JSC::MIPSRegisters::t5) |
        (1 << JSC::MIPSRegisters::t9) | // t9 = scratch
        (1 << JSC::MIPSRegisters::k0) |
        (1 << JSC::MIPSRegisters::k1) |
        (1 << JSC::MIPSRegisters::gp) |
        (1 << JSC::MIPSRegisters::sp) |
        (1 << JSC::MIPSRegisters::fp) |
        (1 << JSC::MIPSRegisters::ra);

    // Registers that can be allocated without being saved, generally.
    static const uint32 TempMask = VolatileMask & ~NonAllocatableMask;

    // Registers returned from a JS -> JS call.
    static const uint32 JSCallMask =
        (1 << JSC::MIPSRegisters::a0) |
        (1 << JSC::MIPSRegisters::a1);

    // Registers returned from a JS -> C call.
    static const uint32 CallMask =
        (1 << JSC::MIPSRegisters::v0) |
        (1 << JSC::MIPSRegisters::v1);  // used for double-size returns

    static const uint32 AllocatableMask = AllMask & ~NonAllocatableMask;

    typedef JSC::MacroAssembler::RegisterID RegisterID;
};

// Smallest integer type that can hold a register bitmask.
typedef uint16 PackedRegisterMask;

class FloatRegisters
{
  public:
    typedef JSC::MIPSRegisters::FPRegisterID Code;

    static const char *GetName(Code code) {
        static const char *Names[] = { "f0",  "f1",  "f2",  "f3",  "f4",  "f5",  "f6",  "f7",
                                       "f8",  "f9",  "f10", "f11", "f12", "f13", "f14", "f15",
                                       "f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
                                       "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31"};
        return Names[code];
    }

    static const Code Invalid = JSC::MIPSRegisters::invalid_freg;    

    static const uint32 Total = 32;//strictly SMALLER than 32
    static const uint32 Allocatable = 10;

    static const uint32 AllMask = 0xffffffff;

    static const uint32 VolatileMask =
        (1 << JSC::MIPSRegisters::f4) |
        (1 << JSC::MIPSRegisters::f6) |
        (1 << JSC::MIPSRegisters::f8) |
        (1 << JSC::MIPSRegisters::f10) |
        (1 << JSC::MIPSRegisters::f16) |
        (1 << JSC::MIPSRegisters::f18); 

    static const uint32 NonVolatileMask =
        (1 << JSC::MIPSRegisters::f20) |
        (1 << JSC::MIPSRegisters::f22) |
        (1 << JSC::MIPSRegisters::f24) |
        (1 << JSC::MIPSRegisters::f26);

    static const uint32 WrapperMask = VolatileMask;

    // d0 is the ARM scratch float register.
    static const uint32 NonAllocatableMask = 
        (1 << JSC::MIPSRegisters::f1) |
        (1 << JSC::MIPSRegisters::f3) |
        (1 << JSC::MIPSRegisters::f5) |
        (1 << JSC::MIPSRegisters::f7) |
        (1 << JSC::MIPSRegisters::f9) |
        (1 << JSC::MIPSRegisters::f11) |
        (1 << JSC::MIPSRegisters::f13) |
        (1 << JSC::MIPSRegisters::f15) |
        (1 << JSC::MIPSRegisters::f17) |
        (1 << JSC::MIPSRegisters::f19) |
        (1 << JSC::MIPSRegisters::f21) |
        (1 << JSC::MIPSRegisters::f23) |
        (1 << JSC::MIPSRegisters::f25) |
        (1 << JSC::MIPSRegisters::f27) |
        (1 << JSC::MIPSRegisters::f29) |
        (1 << JSC::MIPSRegisters::f31) |

        (1 << JSC::MIPSRegisters::f0) |
        (1 << JSC::MIPSRegisters::f2) |
        (1 << JSC::MIPSRegisters::f12)|
        (1 << JSC::MIPSRegisters::f14)|//by JSC
        (1 << JSC::MIPSRegisters::f28)|//fpTemp
        (1 << JSC::MIPSRegisters::f30);//fpTemp2

    // Registers that can be allocated without being saved, generally.
    static const uint32 TempMask = VolatileMask & ~NonAllocatableMask;

    static const uint32 AllocatableMask = AllMask & ~NonAllocatableMask;
};

} // namespace ion
} // namespace js

#endif // jsion_architecture_arm_h__
