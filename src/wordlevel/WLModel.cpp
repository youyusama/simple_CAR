#include "WLModel.h"

#include "Btor2Frontend.h"
#include "Model.h"
#include "WLArrayAbstraction.h"
#include "WLBoolectorBitblast.h"
#include "WLPackageResize.h"

#include <cstdlib>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace car {
namespace {

class PropertyCoiReducer {
  public:
    explicit PropertyCoiReducer(const Btor2IR &input) : m_input(input) {}

    Btor2IR Run() {
        IndexModel();
        MarkLiveNodes();
        return BuildReducedIr();
    }

  private:
    void IndexModel() {
        // Constraints are roots because dropping assumptions changes reachability.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_init:
                m_initNodes[std::abs(node.args[0])] = node.id;
                break;
            case BTOR2_TAG_next:
                m_nextNodes[std::abs(node.args[0])] = node.id;
                break;
            case BTOR2_TAG_bad:
                m_worklist.push_back(node.id);
                break;
            case BTOR2_TAG_constraint:
                m_worklist.push_back(node.id);
                break;
            default: break;
            }
        }
    }

    void MarkLiveNodes() {
        // Traverse iteratively so deep BTOR2 expression DAGs do not use the call stack.
        while (!m_worklist.empty()) {
            const int64_t id = std::abs(m_worklist.back());
            m_worklist.pop_back();
            if (!m_liveNodes.insert(id).second) continue;

            const Btor2IRNode &node = m_input.Node(id);
            for (uint32_t i = 0; i < node.nargs; ++i)
                m_worklist.push_back(node.args[i]);

            if (node.tag != BTOR2_TAG_state) continue;
            auto init = m_initNodes.find(id);
            if (init != m_initNodes.end()) m_worklist.push_back(init->second);
            auto next = m_nextNodes.find(id);
            if (next != m_nextNodes.end()) m_worklist.push_back(next->second);
        }
    }

    void MarkSort(int64_t sortId) {
        if (!sortId || !m_liveSorts.insert(sortId).second) return;
        const Btor2IRSort &sort = m_input.Sort(sortId);
        if (sort.tag != BTOR2_TAG_SORT_array) return;
        MarkSort(sort.indexSort);
        MarkSort(sort.elementSort);
    }

    void CopySort(int64_t sortId,
                  Btor2IR &output,
                  std::unordered_set<int64_t> &copied) const {
        if (!sortId || !copied.insert(sortId).second) return;
        const Btor2IRSort &sort = m_input.Sort(sortId);
        if (sort.tag == BTOR2_TAG_SORT_array) {
            CopySort(sort.indexSort, output, copied);
            CopySort(sort.elementSort, output, copied);
        }
        output.AddSort(sort);
    }

    Btor2IR BuildReducedIr() {
        // Retain only sorts reachable from live nodes, including array sub-sorts.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            if (m_liveNodes.count(node.id)) MarkSort(node.sortId);
        }

        Btor2IR output;
        std::unordered_set<int64_t> copiedSorts;
        for (int64_t sortId : m_liveSorts)
            CopySort(sortId, output, copiedSorts);

        // Original order preserves the BTOR2 topological definition order and IDs.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            if (m_liveNodes.count(node.id)) output.AddNode(node);
        }
        return output;
    }

    const Btor2IR &m_input;
    std::vector<int64_t> m_worklist;
    std::unordered_set<int64_t> m_liveNodes;
    std::unordered_set<int64_t> m_liveSorts;
    std::unordered_map<int64_t, int64_t> m_initNodes;
    std::unordered_map<int64_t, int64_t> m_nextNodes;
};

Btor2IR ReduceToPropertyCoi(const Btor2IR &ir) {
    return PropertyCoiReducer(ir).Run();
}

} // namespace

WLModel::WLModel(const Settings &settings, Log &log)
    : m_settings(settings), m_log(log), m_inputPath(settings.aigFilePath) {
    // The initial abstraction tracks no memory/address pairs.
    Build({});
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
    m_arrayReads = std::move(result.arrayReads);
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
    // Stage 1: parse and validate the format-specific source model.
    Btor2IR ir = Btor2Frontend::LoadIR(m_inputPath);
    const bool sourceHasArrays = ir.HasArrays();
    const bool bitblastOnly = !m_settings.wlBitblastOutputPath.empty();
    if (bitblastOnly && sourceHasArrays) {
        throw std::runtime_error(
            "--wl-bitblast-only accepts only array-free BTOR2 input");
    }

    // Stage 2: discard logic outside the safety property/constraint COI.
    if (!m_settings.wlDisableCoi) ir = ReduceToPropertyCoi(ir);

    Btor2IR processedIr;
    std::vector<WLMemoryPair> tracePairs;
    std::vector<WLArrayRead> reads;
    WLIRTraceMap traceSources;

    // Stage 3: normal checking abstracts arrays; export mode is array-free.
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
        reads = std::move(abstraction.reads);
        traceSources = std::move(abstraction.traceSources);
    }

    // Stage 4: compact safe finite-domain packages in the array-free IR.
    if (!m_settings.wlDisablePackageResize)
        WLPackageResize::Run(processedIr, traceSources);

    // Stage 5: standard bitblast produces the bit-level model and trace mapping.
    WLTraceMap traceMap;
    traceMap.memoryPairs = std::move(tracePairs);
    std::shared_ptr<aiger> aig =
        GenerateWLAig(processedIr, traceSources, traceMap);

    return {std::move(aig),
            sourceHasArrays,
            std::move(reads),
            std::move(traceMap)};
}

} // namespace car
