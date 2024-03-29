/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_move_emitter_h__
#define jsion_move_emitter_h__

#if defined(JS_CPU_X86) || defined(JS_CPU_X64)
# include "ion/shared/MoveEmitter-x86-shared.h"
#elif defined(JS_CPU_ARM)
# include "ion/arm/MoveEmitter-arm.h"
#elif defined(JS_CPU_MIPS)
# include "ion/mips/MoveEmitter-mips.h"
#else
# error "CPU Not Supported"
#endif

#endif // jsion_move_emitter_h__

