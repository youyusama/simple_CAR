#include "WLModel.h"

#include "Btor2Frontend.h"
#include "Model.h"
#include "WLArrayAbstraction.h"
#include "WLBitblastor.h"
#include "WLPackageResize.h"

#include <cstdlib>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace car {
namespace {

std::runtime_error Unsupported(const Btor2IRNode &node,
                               const std::string &reason) {
    return std::runtime_error("BTOR2 line " + std::to_string(node.line) +
                              " (id " + std::to_string(node.id) + "): " +
                              reason);
}

bool IsArray(const Btor2IR &ir, int64_t sortId) {
    return sortId && ir.Sort(sortId).tag == BTOR2_TAG_SORT_array;
}

void ValidateArraySupport(const Btor2IR &ir) {
    // Nested arrays are outside the selected-slot abstraction supported subset.
    for (const auto &[id, sort] : ir.Sorts()) {
        (void)id;
        if (sort.tag != BTOR2_TAG_SORT_array) continue;
        if (ir.Sort(sort.indexSort).tag != BTOR2_TAG_SORT_bitvec ||
            ir.Sort(sort.elementSort).tag != BTOR2_TAG_SORT_bitvec) {
            throw std::runtime_error("nested BTOR2 arrays are unsupported");
        }
    }

    if (!ir.HasArrays()) return;

    // Restrict array-valued expressions to the remodellable state/write/ite form.
    for (const Btor2IRNode &node : ir.Nodes()) {
        if (!IsArray(ir, node.sortId)) continue;
        switch (node.tag) {
        case BTOR2_TAG_state:
        case BTOR2_TAG_write:
        case BTOR2_TAG_ite:
        case BTOR2_TAG_init:
        case BTOR2_TAG_next:
            break;
        case BTOR2_TAG_input:
            throw Unsupported(node, "whole-array inputs are unsupported");
        default:
            throw Unsupported(node,
                              "array-valued operator is outside the supported "
                              "state/write/ite/next subset");
        }
    }

    // Reject scalar operators that consume arrays outside the supported boundaries.
    for (const Btor2IRNode &node : ir.Nodes()) {
        if (node.tag != BTOR2_TAG_eq && node.tag != BTOR2_TAG_neq) continue;
        if (IsArray(ir, ir.Node(node.args[0]).sortId)) {
            throw Unsupported(node, "array equality and inequality are unsupported");
        }
    }

    for (const Btor2IRNode &node : ir.Nodes()) {
        for (uint32_t i = 0; i < node.nargs; ++i) {
            const Btor2IRNode &argument = ir.Node(node.args[i]);
            if (!IsArray(ir, argument.sortId)) continue;

            bool allowed =
                (node.tag == BTOR2_TAG_read && i == 0) ||
                (node.tag == BTOR2_TAG_write && i == 0) ||
                (node.tag == BTOR2_TAG_ite && (i == 1 || i == 2)) ||
                (node.tag == BTOR2_TAG_init && (i == 0 || i == 1)) ||
                (node.tag == BTOR2_TAG_next && (i == 0 || i == 1));
            if (!allowed) {
                throw Unsupported(
                    node,
                    "array operand is used outside read/write/ite/init/next");
            }
        }
    }
}

Btor2IR ReduceToPropertyCoi(const Btor2IR &ir) {
    std::unordered_map<int64_t, int64_t> initNodes;
    std::unordered_map<int64_t, int64_t> nextNodes;
    std::vector<int64_t> worklist;
    std::unordered_set<int64_t> liveNodes;

    // Constraints are roots because dropping assumptions changes reachability.
    for (const Btor2IRNode &node : ir.Nodes()) {
        switch (node.tag) {
        case BTOR2_TAG_init:
            initNodes[std::abs(node.args[0])] = node.id;
            break;
        case BTOR2_TAG_next:
            nextNodes[std::abs(node.args[0])] = node.id;
            break;
        case BTOR2_TAG_bad:
        case BTOR2_TAG_constraint:
            worklist.push_back(node.id);
            break;
        default: break;
        }
    }

    // Traverse iteratively so deep BTOR2 expression DAGs do not use the call stack.
    while (!worklist.empty()) {
        const int64_t id = std::abs(worklist.back());
        worklist.pop_back();
        if (!liveNodes.insert(id).second) continue;

        const Btor2IRNode &node = ir.Node(id);
        for (uint32_t i = 0; i < node.nargs; ++i)
            worklist.push_back(node.args[i]);

        if (node.tag != BTOR2_TAG_state) continue;
        auto init = initNodes.find(id);
        if (init != initNodes.end()) worklist.push_back(init->second);
        auto next = nextNodes.find(id);
        if (next != nextNodes.end()) worklist.push_back(next->second);
    }

    Btor2IR output;
    std::unordered_set<int64_t> copiedSorts;
    const auto copySort = [&](auto &&self, int64_t sortId) -> void {
        if (!sortId || !copiedSorts.insert(sortId).second) return;
        const Btor2IRSort &sort = ir.Sort(sortId);
        if (sort.tag == BTOR2_TAG_SORT_array) {
            self(self, sort.indexSort);
            self(self, sort.elementSort);
        }
        output.AddSort(sort);
    };

    // Retain only sorts reachable from live nodes, including array sub-sorts.
    for (const Btor2IRNode &node : ir.Nodes()) {
        if (liveNodes.count(node.id)) copySort(copySort, node.sortId);
    }

    // Original order preserves the BTOR2 topological definition order and IDs.
    for (const Btor2IRNode &node : ir.Nodes()) {
        if (liveNodes.count(node.id)) output.AddNode(node);
    }
    return output;
}

unsigned NodeWidth(const Btor2IR &ir, int64_t id) {
    return ir.Sort(ir.Node(id).sortId).width;
}

unsigned ArrayIndexWidth(const Btor2IR &ir, int64_t memoryId) {
    const Btor2IRSort &sort = ir.Sort(ir.Node(memoryId).sortId);
    return ir.Sort(sort.indexSort).width;
}

unsigned ArrayElementWidth(const Btor2IR &ir, int64_t memoryId) {
    const Btor2IRSort &sort = ir.Sort(ir.Node(memoryId).sortId);
    return ir.Sort(sort.elementSort).width;
}

std::unordered_map<Var, bool> CubeValues(const Cube &cube) {
    std::unordered_map<Var, bool> values;
    for (Lit literal : cube) {
        const bool value = !Sign(literal);
        auto [it, inserted] = values.emplace(VarOf(literal), value);
        if (!inserted && it->second != value)
            throw std::runtime_error(
                "checker trace contains contradictory literals");
    }
    return values;
}

WLBitVector &EnsureNodeValue(
    std::unordered_map<int64_t, WLBitVector> &values,
    int64_t id,
    unsigned width) {
    auto [it, inserted] = values.emplace(id, WLBitVector::Zero(width));
    (void)inserted;
    return it->second;
}

WLBitVector &EnsurePairValue(
    std::unordered_map<size_t, WLBitVector> &values,
    size_t index,
    unsigned width) {
    auto [it, inserted] = values.emplace(index, WLBitVector::Zero(width));
    (void)inserted;
    return it->second;
}

WLBitVector &ReplayDestination(const Btor2IR &ir,
                               WLReplayStep &step,
                               const WLValueOrigin &origin) {
    switch (origin.kind) {
    case WLTraceKind::OriginalInput:
        return EnsureNodeValue(step.inputValues,
                               origin.nodeId,
                               NodeWidth(ir, origin.nodeId));
    case WLTraceKind::OriginalState:
        return EnsureNodeValue(step.stateValues,
                               origin.nodeId,
                               NodeWidth(ir, origin.nodeId));
    case WLTraceKind::AbstractReadInput:
        return EnsureNodeValue(step.abstractReadValues,
                               origin.nodeId,
                               NodeWidth(ir, origin.nodeId));
    case WLTraceKind::SelectorState:
        return EnsurePairValue(step.selectorValues,
                               origin.pairIndex,
                               ArrayIndexWidth(ir, origin.nodeId));
    case WLTraceKind::ContentState:
        return EnsurePairValue(step.contentValues,
                               origin.pairIndex,
                               ArrayElementWidth(ir, origin.nodeId));
    }
    throw std::runtime_error("unknown word-level trace origin");
}

void SetReplaySegment(const Btor2IR &ir,
                      WLReplayStep &step,
                      const WLTraceSpan &span,
                      const WLBitVector &encoded) {
    const WLValueOrigin &origin = span.origin;
    WLBitVector &destination = ReplayDestination(ir, step, origin);
    if (encoded.Width() != span.encodedWidth ||
        span.encodedWidth > origin.originalSegmentWidth ||
        origin.originalBitOffset + origin.originalSegmentWidth >
            destination.Width()) {
        throw std::runtime_error("invalid word-level replay span");
    }

    // A compact all-ones package denotes the original segment's all-ones value.
    const bool resized = span.encodedWidth < origin.originalSegmentWidth;
    const bool expandsToOnes = resized && encoded.IsOnes();
    for (uint32_t bit = 0; bit < origin.originalSegmentWidth; ++bit) {
        const bool value =
            expandsToOnes ||
            (bit < encoded.Width() && encoded.GetBit(bit));
        destination.SetBit(origin.originalBitOffset + bit, value);
    }
}

void DecodeReplayStep(const Btor2IR &ir,
                      const std::pair<Cube, Cube> &bitStep,
                      const WLTraceMap &traceMap,
                      WLReplayStep &step,
                      bool loadInitialLatches) {
    const auto inputValues = CubeValues(bitStep.first);
    const auto latchValues = CubeValues(bitStep.second);

    auto decode = [&](const std::vector<WLTraceSpan> &spans,
                      const auto &bitValues) {
        for (const WLTraceSpan &span : spans) {
            WLBitVector encoded = WLBitVector::Zero(span.encodedWidth);
            for (uint32_t bit = 0; bit < span.encodedWidth; ++bit) {
                auto found = bitValues.find(span.firstAigVar + bit);
                if (found != bitValues.end() && found->second)
                    encoded.SetBit(bit, true);
            }
            SetReplaySegment(ir, step, span, encoded);
        }
    };

    decode(traceMap.inputSpans, inputValues);
    if (loadInitialLatches) decode(traceMap.latchSpans, latchValues);
}

} // namespace

WLModel::WLModel(const Settings &settings, Log &log)
    : m_settings(settings), m_log(log), m_inputPath(settings.aigFilePath) {
    m_sourceIr =
        std::make_unique<Btor2IR>(Btor2Frontend::LoadIR(m_inputPath));
    // Replay still consumes SourceIR(), so validate before pruning its cone.
    ValidateArraySupport(*m_sourceIr);
    m_sourceHasArrays = m_sourceIr->HasArrays();
}

WLModel::~WLModel() = default;

void WLModel::PreparePropertyIR() {
    if (m_propertyIr) return;

    m_propertyIr = std::make_unique<Btor2IR>(
        m_settings.wlDisableCoi ? *m_sourceIr
                                : ReduceToPropertyCoi(*m_sourceIr));
}

const Btor2IR &WLModel::PropertyIR() {
    PreparePropertyIR();
    return *m_propertyIr;
}

WLReplayTrace WLModel::DecodeBitTrace(
    const std::vector<std::pair<Cube, Cube>> &trace) const {
    if (trace.empty())
        throw std::runtime_error("checker returned an empty bit-level trace");

    WLReplayTrace replay;
    replay.steps.resize(trace.size());
    replay.memoryPairs = m_traceMap.memoryPairs;
    for (size_t time = 0; time < trace.size(); ++time) {
        DecodeReplayStep(*m_sourceIr,
                         trace[time],
                         m_traceMap,
                         replay.steps[time],
                         time == 0);
    }
    return replay;
}

void WLModel::WriteBitblastAig() const {
    if (!m_aig || m_settings.wlBitblastOutputPath.empty()) {
        throw std::runtime_error("missing AIGER bitblast output");
    }
    if (!aiger_open_and_write_to_file(
            m_aig.get(), m_settings.wlBitblastOutputPath.c_str())) {
        throw std::runtime_error(
            "failed to write AIGER output: " +
            m_settings.wlBitblastOutputPath);
    }
}

void WLModel::Build(const std::vector<WLMemoryPair> &memoryPairs) {
    // Re-run the complete WL pipeline after each CEGAR refinement.
    WLModelBuildResult result = BuildFromBtor2(memoryPairs);
    m_sourceHasArrays = result.sourceHasArrays;
    m_traceMap = std::move(result.traceMap);
    m_aig = std::move(result.aig);

    // AIG export stops at the final bitblast, before Model simplification.
    if (!m_settings.wlBitblastOutputPath.empty()) {
        m_model.reset();
        return;
    }
    m_model = std::make_unique<Model>(m_settings, m_log, m_aig);
}

WLModelBuildResult
WLModel::BuildFromBtor2(const std::vector<WLMemoryPair> &memoryPairs) {
    // Stage 1: prepare the property cone once, on first build.
    PreparePropertyIR();
    Btor2IR ir = *m_propertyIr;
    const bool sourceHasArrays = ir.HasArrays();
    const bool bitblastOnly = !m_settings.wlBitblastOutputPath.empty();
    if (bitblastOnly && sourceHasArrays) {
        throw std::runtime_error(
            "--wl-bitblast-only accepts only array-free BTOR2 input");
    }

    Btor2IR processedIr;
    std::vector<WLMemoryPair> tracePairs;
    WLIRTraceMap traceSources;

    // Stage 2: normal checking abstracts arrays; export mode is array-free.
    if (bitblastOnly) {
        processedIr = std::move(ir);
    } else {
        WLArrayAbstractionResult abstraction =
            WLArrayAbstraction::Run(ir, memoryPairs);
        if (abstraction.ir.HasArrays()) {
            throw std::runtime_error(
                "array abstraction produced an array-valued word-level model");
        }
        processedIr = std::move(abstraction.ir);
        tracePairs = std::move(abstraction.tracePairs);
        traceSources = std::move(abstraction.traceSources);
    }

    // Stage 3: compact safe finite-domain packages in the array-free IR.
    if (!m_settings.wlDisablePackageResize)
        WLPackageResize::Run(processedIr, traceSources);

    // Stage 4: standard bitblast produces the bit-level model and trace mapping.
    WLTraceMap traceMap;
    traceMap.memoryPairs = std::move(tracePairs);
    std::shared_ptr<aiger> aig =
        GenerateWLAig(processedIr, traceSources, traceMap);

    return {std::move(aig),
            sourceHasArrays,
            std::move(traceMap)};
}

} // namespace car
