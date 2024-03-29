/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <stdarg.h>

#include "JSONSpewer.h"
#include "LIR.h"
#include "TypeOracle.h"
#include "MIR.h"
#include "MIRGraph.h"
#include "LinearScan.h"
#include "RangeAnalysis.h"
using namespace js;
using namespace js::ion;

JSONSpewer::~JSONSpewer()
{
    if (fp_)
        fclose(fp_);
}

void
JSONSpewer::indent()
{
    if (!fp_)
        return;
    JS_ASSERT(indentLevel_ >= 0);
    fprintf(fp_, "\n");
    for (int i = 0; i < indentLevel_; i++)
        fprintf(fp_, "  ");
}

void
JSONSpewer::property(const char *name)
{
    if (!fp_)
        return;

    if (!first_)
        fprintf(fp_, ",");
    indent();
    fprintf(fp_, "\"%s\":", name);
    first_ = false;
}

void
JSONSpewer::beginObject()
{
    if (!fp_)
        return;

    if (!first_) {
        fprintf(fp_, ",");
        indent();
    }
    fprintf(fp_, "{");
    indentLevel_++;
    first_ = true;
}

void
JSONSpewer::beginObjectProperty(const char *name)
{
    if (!fp_)
        return;

    property(name);
    fprintf(fp_, "{");
    indentLevel_++;
    first_ = true;
}

void
JSONSpewer::beginListProperty(const char *name)
{
    if (!fp_)
        return;

    property(name);
    fprintf(fp_, "[");
    first_ = true;
}

void
JSONSpewer::stringProperty(const char *name, const char *format, ...)
{
    if (!fp_)
        return;

    va_list ap;
    va_start(ap, format);

    property(name);
    fprintf(fp_, "\"");
    vfprintf(fp_, format, ap);
    fprintf(fp_, "\"");

    va_end(ap);
}

void
JSONSpewer::stringValue(const char *format, ...)
{
    if (!fp_)
        return;

    va_list ap;
    va_start(ap, format);

    if (!first_)
        fprintf(fp_, ",");
    fprintf(fp_, "\"");
    vfprintf(fp_, format, ap);
    fprintf(fp_, "\"");

    va_end(ap);
    first_ = false;
}

void
JSONSpewer::integerProperty(const char *name, int value)
{
    if (!fp_)
        return;

    property(name);
    fprintf(fp_, "%d", value);
}

void
JSONSpewer::integerValue(int value)
{
    if (!fp_)
        return;

    if (!first_)
        fprintf(fp_, ",");
    fprintf(fp_, "%d", value);
    first_ = false;
}

void
JSONSpewer::endObject()
{
    if (!fp_)
        return;

    indentLevel_--;
    indent();
    fprintf(fp_, "}");
    first_ = false;
}

void
JSONSpewer::endList()
{
    if (!fp_)
        return;

    fprintf(fp_, "]");
    first_ = false;
}

bool
JSONSpewer::init(const char *path)
{
    fp_ = fopen(path, "w");
    if (!fp_)
        return false;

    beginObject();
    beginListProperty("functions");
    return true;
}

void
JSONSpewer::beginFunction(JSScript *script)
{
    if (inFunction_)
        endFunction();

    beginObject();
    stringProperty("name", "%s:%d", script->filename, script->lineno);
    beginListProperty("passes");

    inFunction_ = true;
}

void
JSONSpewer::beginPass(const char *pass)
{
    beginObject();
    stringProperty("name", pass);
}

void
JSONSpewer::spewMResumePoint(MResumePoint *rp)
{
    beginObjectProperty("resumePoint");

    if (rp->caller())
        integerProperty("caller", rp->caller()->block()->id());

    property("mode");
    switch (rp->mode()) {
      case MResumePoint::ResumeAt:
        fprintf(fp_, "\"At\"");
        break;
      case MResumePoint::ResumeAfter:
        fprintf(fp_, "\"After\"");
        break;
      case MResumePoint::Outer:
        fprintf(fp_, "\"Outer\"");
        break;
    }

    beginListProperty("operands");
    for (MResumePoint *iter = rp; iter; iter = iter->caller()) {
        for (int i = iter->numOperands() - 1; i >= 0; i--)
            integerValue(iter->getOperand(i)->id());
    }
    endList();

    endObject();
}

void
JSONSpewer::spewMDef(MDefinition *def)
{
    beginObject();

    integerProperty("id", def->id());

    property("opcode");
    fprintf(fp_, "\"");
    def->printOpcode(fp_);
    fprintf(fp_, "\"");

    beginListProperty("attributes");
#define OUTPUT_ATTRIBUTE(X) do{ if(def->is##X()) stringValue(#X); } while(0);
    MIR_FLAG_LIST(OUTPUT_ATTRIBUTE);
#undef OUTPUT_ATTRIBUTE
    endList();

    beginListProperty("inputs");
    for (size_t i = 0; i < def->numOperands(); i++)
        integerValue(def->getOperand(i)->id());
    endList();

    beginListProperty("uses");
    for (MUseDefIterator use(def); use; use++)
        integerValue(use.def()->id());
    endList();

    if (def->range()) {
        Sprinter sp(GetIonContext()->cx);
        sp.init();
        def->range()->print(sp);
        stringProperty("type", "%s : %s", sp.string());
    } else {
        stringProperty("type", "%s", StringFromMIRType(def->type()));
    }

    if (def->isInstruction()) {
        if (MResumePoint *rp = def->toInstruction()->resumePoint())
            spewMResumePoint(rp);
    }

    endObject();
}

void
JSONSpewer::spewMIR(MIRGraph *mir)
{
    if (!fp_)
        return;

    beginObjectProperty("mir");
    beginListProperty("blocks");

    for (MBasicBlockIterator block(mir->begin()); block != mir->end(); block++) {
        beginObject();

        integerProperty("number", block->id());

        beginListProperty("attributes");
        if (block->isLoopBackedge())
            stringValue("backedge");
        if (block->isLoopHeader())
            stringValue("loopheader");
        if (block->isSplitEdge())
            stringValue("splitedge");
        endList();

        beginListProperty("predecessors");
        for (size_t i = 0; i < block->numPredecessors(); i++)
            integerValue(block->getPredecessor(i)->id());
        endList();

        beginListProperty("successors");
        for (size_t i = 0; i < block->numSuccessors(); i++)
            integerValue(block->getSuccessor(i)->id());
        endList();

        beginListProperty("instructions");
        for (MPhiIterator phi(block->phisBegin()); phi != block->phisEnd(); phi++)
            spewMDef(*phi);
        for (MInstructionIterator i(block->begin()); i != block->end(); i++)
            spewMDef(*i);
        endList();

        spewMResumePoint(block->entryResumePoint());

        endObject();
    }

    endList();
    endObject();
}

void
JSONSpewer::spewLIns(LInstruction *ins)
{
    if (!fp_)
        return;

    beginObject();

    integerProperty("id", ins->id());

    property("opcode");
    fprintf(fp_, "\"");
    ins->print(fp_);
    fprintf(fp_, "\"");

    beginListProperty("defs");
    for (size_t i = 0; i < ins->numDefs(); i++)
        integerValue(ins->getDef(i)->virtualRegister());
    endList();

    endObject();
}

void
JSONSpewer::spewLIR(MIRGraph *mir)
{
    if (!fp_)
        return;

    beginObjectProperty("lir");
    beginListProperty("blocks");

    for (MBasicBlockIterator i(mir->begin()); i != mir->end(); i++) {
        LBlock *block = i->lir();
        if (!block)
            continue;

        beginObject();
        integerProperty("number", i->id());

        beginListProperty("instructions");
        for (size_t p = 0; p < block->numPhis(); p++)
            spewLIns(block->getPhi(p));
        for (LInstructionIterator ins(block->begin()); ins != block->end(); ins++)
            spewLIns(*ins);
        endList();

        endObject();
    }

    endList();
    endObject();
}

void
JSONSpewer::spewIntervals(LinearScanAllocator *regalloc)
{
    if (!fp_)
        return;

    beginObjectProperty("intervals");
    beginListProperty("blocks");

    for (size_t bno = 0; bno < regalloc->graph.numBlocks(); bno++) {
        beginObject();
        integerProperty("number", bno);
        beginListProperty("vregs");

        LBlock *lir = regalloc->graph.getBlock(bno);
        for (LInstructionIterator ins = lir->begin(); ins != lir->end(); ins++) {
            for (size_t k = 0; k < ins->numDefs(); k++) {
                VirtualRegister *vreg = &regalloc->vregs[ins->getDef(k)->virtualRegister()];

                beginObject();
                integerProperty("vreg", vreg->reg());
                beginListProperty("intervals");

                for (size_t i = 0; i < vreg->numIntervals(); i++) {
                    LiveInterval *live = vreg->getInterval(i);

                    if (live->numRanges()) {
                        beginObject();
                        property("allocation");
                        fprintf(fp_, "\"");
                        LAllocation::PrintAllocation(fp_, live->getAllocation());
                        fprintf(fp_, "\"");
                        beginListProperty("ranges");

                        for (size_t j = 0; j < live->numRanges(); j++) {
                            beginObject();
                            integerProperty("start", live->getRange(j)->from.pos());
                            integerProperty("end", live->getRange(j)->to.pos());
                            endObject();
                        }

                        endList();
                        endObject();
                    }
                }

                endList();
                endObject();
            }
        }

        endList();
        endObject();
    }

    endList();
    endObject();
}

void
JSONSpewer::endPass()
{
    endObject();
    fflush(fp_);
}

void
JSONSpewer::endFunction()
{
    JS_ASSERT(inFunction_);
    endList();
    endObject();
    fflush(fp_);
    inFunction_ = false;
}

void
JSONSpewer::finish()
{
    if (!fp_)
        return;

    if (inFunction_)
        endFunction();

    endList();
    endObject();
    fprintf(fp_, "\n");

    fclose(fp_);
    fp_ = NULL;
}

