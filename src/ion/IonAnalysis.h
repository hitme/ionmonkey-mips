/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jsion_ion_analysis_h__
#define jsion_ion_analysis_h__

// This file declares various analysis passes that operate on MIR.

#include "IonAllocPolicy.h"
#include "MIR.h"

namespace js {
namespace ion {

class MIRGenerator;
class MIRGraph;

bool
SplitCriticalEdges(MIRGraph &graph);

bool
EliminatePhis(MIRGenerator *mir, MIRGraph &graph);

bool
EliminateDeadCode(MIRGenerator *mir, MIRGraph &graph);

bool
ApplyTypeInformation(MIRGenerator *mir, MIRGraph &graph);

bool
RenumberBlocks(MIRGraph &graph);

bool
BuildDominatorTree(MIRGraph &graph);

bool
BuildPhiReverseMapping(MIRGraph &graph);

void
AssertGraphCoherency(MIRGraph &graph);

bool
EliminateRedundantBoundsChecks(MIRGraph &graph);

class MDefinition;

// Simple linear sum of the form 'n' or 'x + n'.
struct SimpleLinearSum
{
    MDefinition *term;
    int32 constant;

    SimpleLinearSum(MDefinition *term, int32 constant)
        : term(term), constant(constant)
    {}
};

SimpleLinearSum
ExtractLinearSum(MDefinition *ins);

bool
ExtractLinearInequality(MTest *test, BranchDirection direction,
                        SimpleLinearSum *plhs, MDefinition **prhs, bool *plessEqual);

struct LinearTerm
{
    MDefinition *term;
    int32 scale;

    LinearTerm(MDefinition *term, int32 scale)
      : term(term), scale(scale)
    {
    }
};

// General linear sum of the form 'x1*n1 + x2*n2 + ... + n'
class LinearSum
{
  public:
    LinearSum()
      : constant_(0)
    {
    }

    LinearSum(const LinearSum &other)
      : constant_(other.constant_)
    {
        for (size_t i = 0; i < other.terms_.length(); i++)
            terms_.append(other.terms_[i]);
    }

    bool multiply(int32 scale);
    bool add(const LinearSum &other);
    bool add(MDefinition *term, int32 scale);
    bool add(int32 constant);

    int32 constant() const { return constant_; }
    size_t numTerms() const { return terms_.length(); }
    LinearTerm term(size_t i) const { return terms_[i]; }

    void print(Sprinter &sp) const;

  private:
    Vector<LinearTerm, 2, IonAllocPolicy> terms_;
    int32 constant_;
};

} // namespace ion
} // namespace js

#endif // jsion_ion_analysis_h__

