#include "WLModel.h"

#include "Btor2Frontend.h"
#include "WLBitblastor.h"
#include "WLPackageResize.h"
#include "WLArraySimplifier.h"
#include "WLSimulator.h"
#include "Log.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace car {
namespace {

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

} // namespace

WLModel::WLModel(const Settings &settings, Log &log)
    : m_sourceIr(Btor2Frontend::LoadIR(settings.aigFilePath)),
      m_disableCoi(settings.wlDisableCoi), m_log(log) {
    m_sourceIr.ValidateSupportedArrays();
}

WLModel::~WLModel() = default;

const Btor2IR &WLModel::PropertyIR() const {
    if (!m_propertyIr) {
        auto property = m_disableCoi ? m_sourceIr : ReduceToPropertyCoi(m_sourceIr);
        if (property.HasArrays()) {
            m_arraySimplifier = std::make_unique<WLArraySimplifier>(property);
            const auto &stats = m_arraySimplifier->Stats();
            LOG_L(m_log, 1, "word-level array simplification: comparisons=", stats.comparisonsBefore,
                  " -> ", stats.comparisonsAfter, " defined_inputs=", stats.definedInputs,
                  " context=", stats.context);
            for (const auto &reason : stats.skippedRules)
                LOG_L(m_log, 2, "word-level array simplification skipped rule: ", reason);
            property = m_arraySimplifier->IR();
            if (!m_disableCoi) property = ReduceToPropertyCoi(property);
        }
        m_propertyIr = std::make_unique<Btor2IR>(std::move(property));
    }
    return *m_propertyIr;
}

void WLModel::RestoreSourceTrace(WLTrace &trace) const {
    const auto &property = PropertyIR();
    if (m_arraySimplifier) m_arraySimplifier->RestoreTrace(trace, property);
    WLSimulator::CompleteCoiChoices(m_sourceIr, property, trace);
}

void WLModel::WriteScalarAig(const std::string &path, bool resize) const {
    if (SourceHasArrays())
        throw std::runtime_error("--wl-bitblast-only accepts only array-free BTOR2 input");
    std::unique_ptr<WLPackageResize> resized;
    if (resize) resized = std::make_unique<WLPackageResize>(PropertyIR());
    const auto &ir = resized ? resized->IR() : PropertyIR();
    WLWordLayout layout;
    auto aig = GenerateWLAig(ir, layout);
    if (path.empty() || !aiger_open_and_write_to_file(aig.get(), path.c_str()))
        throw std::runtime_error("failed to write AIGER output: " + path);
}

} // namespace car
