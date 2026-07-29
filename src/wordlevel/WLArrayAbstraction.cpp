#include "WLArrayAbstraction.h"

#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace car {
namespace {

class Builder {
  public:
    Builder(const Btor2IR &input, const std::vector<WLMemoryPair> &pairs)
        : m_input(input), m_pairs(pairs) {
        // Build a fresh array-free IR instead of mutating the source BTOR2 model.
        CopyBitVectorSorts();
        IndexModel();
        CreateScalarVariables();
        CreateTrackedSlots();
        BuildScalarModel();
        BuildMemories();
        BuildProperties();
        m_output.SetHasArrays(false);
    }

    WLArrayAbstractionResult Finish() {
        return {std::move(m_output),
                std::move(m_tracePairs),
                std::move(m_reads),
                std::move(m_traceSources)};
    }

  private:
    // One tracked slot represents memory content at one selected address.
    struct TrackedSlot {
        WLMemoryPair pair;
        int64_t selectorId{0};
        int64_t contentId{0};
        size_t pairIndex{0};
    };

    void CopyBitVectorSorts() {
        // Array sorts disappear after abstraction; scalar sorts retain their IDs.
        for (const auto &[id, sort] : m_input.Sorts()) {
            (void)id;
            if (sort.tag == BTOR2_TAG_SORT_bitvec) m_output.AddSort(sort);
        }
    }

    int64_t EnsureBitVectorSort(uint32_t width) {
        // Reuse an existing width before allocating a synthetic sort.
        for (const auto &[id, sort] : m_output.Sorts()) {
            if (sort.tag == BTOR2_TAG_SORT_bitvec && sort.width == width)
                return id;
        }
        int64_t id = NewSyntheticSortId();
        m_output.AddSort(
            {id, BTOR2_TAG_SORT_bitvec, width, 0, 0});
        return id;
    }

    void IndexModel() {
        // Collect transition metadata and map every read to its originating memory.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_init: m_inits[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_next: m_next[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_bad: m_badIds.push_back(node.id); break;
            case BTOR2_TAG_constraint:
                m_constraintIds.push_back(node.id);
                break;
            case BTOR2_TAG_read: {
                int64_t memoryId = FindMemory(node.args[0]);
                m_readMemory[node.id] = memoryId;
                m_reads.push_back(
                    {node.id, memoryId, node.args[1]});
                break;
            }
            default: break;
            }
        }

        // Validate CEGAR-selected memory/address/delay pairs before rewriting.
        for (const WLMemoryPair &pair : m_pairs) {
            const Btor2IRNode &state = m_input.Node(pair.memoryStateId);
            if (!state.sortId ||
                m_input.Sort(state.sortId).tag != BTOR2_TAG_SORT_array) {
                throw std::runtime_error(
                    "memory abstraction pair references a non-array state");
            }
            m_requestedSlots[pair.memoryStateId].push_back(pair);
        }
    }

    int64_t FindMemory(int64_t expressionId) const {
        // Follow write/ite expressions back to the unique underlying array state.
        const Btor2IRNode &node = m_input.Node(expressionId);
        if (node.tag == BTOR2_TAG_state) return node.id;
        if (node.tag == BTOR2_TAG_write) return FindMemory(node.args[0]);
        if (node.tag == BTOR2_TAG_ite) {
            int64_t lhs = FindMemory(node.args[1]);
            int64_t rhs = FindMemory(node.args[2]);
            if (lhs != rhs) {
                throw std::runtime_error(
                    "BTOR2 line " + std::to_string(node.line) + " (id " +
                    std::to_string(node.id) +
                    "): array ite combines different memory states");
            }
            return lhs;
        }
        if (node.tag == BTOR2_TAG_input) return -node.id;
        throw std::runtime_error(
            "array expression is not remodellable at BTOR2 id " +
            std::to_string(node.id));
    }

    void CreateScalarVariables() {
        // Copy original scalar inputs/states and record their trace provenance.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            if (node.tag != BTOR2_TAG_input && node.tag != BTOR2_TAG_state)
                continue;
            if (!node.sortId ||
                m_input.Sort(node.sortId).tag != BTOR2_TAG_SORT_bitvec)
                continue;
            m_output.AddNode(node);
            m_scalarMap[node.id] = node.id;
            m_traceSources[node.id] = {
                node.tag == BTOR2_TAG_input
                    ? WLTraceBitKind::OriginalInput
                    : WLTraceBitKind::OriginalState,
                node.id,
                0};
        }
    }

    void CreateTrackedSlots() {
        // Replace each selected memory pair with selector/content bit-vector states.
        for (const auto &[memoryId, requested] : m_requestedSlots) {
            const Btor2IRSort &arraySort =
                m_input.Sort(m_input.Node(memoryId).sortId);
            for (const WLMemoryPair &pair : requested) {
                TrackedSlot slot;
                slot.pair = pair;
                slot.pairIndex = m_tracePairs.size();
                slot.selectorId = NewSyntheticNodeId();
                slot.contentId = NewSyntheticNodeId();

                AddNode(slot.selectorId,
                        BTOR2_TAG_state,
                        arraySort.indexSort,
                        {},
                        0,
                        "wl.mem." + std::to_string(memoryId) + ".selector");
                AddNode(slot.contentId,
                        BTOR2_TAG_state,
                        arraySort.elementSort,
                        {},
                        0,
                        "wl.mem." + std::to_string(memoryId) + ".content");
                m_traceSources[slot.selectorId] = {
                    WLTraceBitKind::SelectorState,
                    memoryId,
                    slot.pairIndex};
                m_traceSources[slot.contentId] = {
                    WLTraceBitKind::ContentState,
                    memoryId,
                    slot.pairIndex};
                m_tracePairs.push_back(pair);

                // Selectors are stable state; only the tracked content is updated.
                AddMetaNode(BTOR2_TAG_next,
                            arraySort.indexSort,
                            slot.selectorId,
                            slot.selectorId);
                m_slots[memoryId].push_back(slot);
            }
        }
    }

    void BuildScalarModel() {
        // Clone all scalar expressions, replacing array reads on demand.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_input:
            case BTOR2_TAG_state:
            case BTOR2_TAG_init:
            case BTOR2_TAG_next:
            case BTOR2_TAG_bad:
            case BTOR2_TAG_constraint:
            case BTOR2_TAG_output:
            case BTOR2_TAG_fair:
            case BTOR2_TAG_justice:
            case BTOR2_TAG_write:
                continue;
            default: break;
            }
            if (node.sortId &&
                m_input.Sort(node.sortId).tag == BTOR2_TAG_SORT_array)
                continue;
            CloneScalar(node.id);
        }

        // Recreate init/next metadata for original scalar states.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            if (node.tag != BTOR2_TAG_init && node.tag != BTOR2_TAG_next)
                continue;
            const Btor2IRNode &state = m_input.Node(node.args[0]);
            if (m_input.Sort(state.sortId).tag == BTOR2_TAG_SORT_array)
                continue;
            Btor2IRNode copy = node;
            copy.args[0] = CloneScalar(node.args[0]);
            copy.args[1] = CloneScalar(node.args[1]);
            m_output.AddNode(copy);
        }
    }

    void BuildMemories() {
        // Derive each content state's next/init expression at its selector address.
        for (auto &[memoryId, slots] : m_slots) {
            auto nextIt = m_next.find(memoryId);
            if (nextIt == m_next.end()) {
                throw std::runtime_error("array state " +
                                         std::to_string(memoryId) +
                                         " has no next function");
            }
            const Btor2IRSort &arraySort =
                m_input.Sort(m_input.Node(memoryId).sortId);
            for (TrackedSlot &slot : slots) {
                int64_t nextValue =
                    EvaluateArrayAt(nextIt->second, memoryId, slot);
                AddMetaNode(BTOR2_TAG_next,
                            arraySort.elementSort,
                            slot.contentId,
                            nextValue);

                auto initIt = m_inits.find(memoryId);
                if (initIt == m_inits.end()) continue;
                const Btor2IRNode &init = m_input.Node(initIt->second);
                if (init.sortId &&
                    m_input.Sort(init.sortId).tag == BTOR2_TAG_SORT_array) {
                    throw std::runtime_error(
                        "non-uniform array initialization is unsupported");
                }
                AddMetaNode(BTOR2_TAG_init,
                            arraySort.elementSort,
                            slot.contentId,
                            CloneScalar(initIt->second));
            }
        }
    }

    void BuildProperties() {
        // Delay address-match guards so a tracked pair constrains its requested frame.
        const int64_t boolSort = EnsureBitVectorSort(1);
        std::vector<int64_t> guards;
        for (auto &[memoryId, slots] : m_slots) {
            (void)memoryId;
            for (TrackedSlot &slot : slots) {
                int64_t guard = AddExpression(
                    BTOR2_TAG_eq,
                    boolSort,
                    {slot.selectorId,
                     CloneScalar(slot.pair.addressNodeId),
                     0},
                    2);
                for (unsigned i = 0; i < slot.pair.delay; ++i) {
                    int64_t delayId = NewSyntheticNodeId();
                    AddNode(delayId,
                            BTOR2_TAG_state,
                            boolSort,
                            {},
                            0,
                            "wl.guard." + std::to_string(delayId));
                    m_traceSources[delayId] = {
                        WLTraceBitKind::GuardState,
                        slot.pair.addressNodeId,
                        slot.pairIndex};
                    AddMetaNode(
                        BTOR2_TAG_next, boolSort, delayId, guard);
                    guard = delayId;
                }
                guards.push_back(guard);
            }
        }

        // The abstract bad property is enabled only when every tracked guard holds.
        int64_t allGuards = 0;
        for (int64_t guard : guards) {
            allGuards = allGuards
                            ? AddExpression(
                                  BTOR2_TAG_and,
                                  boolSort,
                                  {allGuards, guard, 0},
                                  2)
                            : guard;
        }

        // Rebuild bad nodes against the transformed scalar expressions.
        for (int64_t badNodeId : m_badIds) {
            const Btor2IRNode &badNode = m_input.Node(badNodeId);
            int64_t bad = CloneScalar(badNode.args[0]);
            if (allGuards) {
                bad = AddExpression(
                    BTOR2_TAG_and,
                    boolSort,
                    {bad, allGuards, 0},
                    2);
            }
            Btor2IRNode copy = badNode;
            copy.args[0] = bad;
            m_output.AddNode(copy);
        }

        // Constraints remain unconditional and are cloned without guard weakening.
        for (int64_t constraintNodeId : m_constraintIds) {
            const Btor2IRNode &constraint =
                m_input.Node(constraintNodeId);
            Btor2IRNode copy = constraint;
            copy.args[0] = CloneScalar(constraint.args[0]);
            m_output.AddNode(copy);
        }
    }

    int64_t CloneScalar(int64_t signedId) {
        // Memoized recursive clone preserves original scalar node IDs where possible.
        if (signedId < 0) return -CloneScalar(-signedId);
        auto mapped = m_scalarMap.find(signedId);
        if (mapped != m_scalarMap.end()) return mapped->second;

        const Btor2IRNode &node = m_input.Node(signedId);
        if (node.tag == BTOR2_TAG_read) return CloneRead(node);
        if (node.sortId &&
            m_input.Sort(node.sortId).tag == BTOR2_TAG_SORT_array) {
            throw std::runtime_error(
                "array-valued node reached scalar abstraction");
        }

        Btor2IRNode copy = node;
        switch (node.tag) {
        case BTOR2_TAG_slice:
            copy.args[0] = CloneScalar(node.args[0]);
            break;
        case BTOR2_TAG_uext:
        case BTOR2_TAG_sext:
            copy.args[0] = CloneScalar(node.args[0]);
            break;
        default:
            for (uint32_t i = 0; i < node.nargs; ++i)
                copy.args[i] = CloneScalar(node.args[i]);
            break;
        }
        m_output.AddNode(copy);
        m_scalarMap[signedId] = copy.id;
        return copy.id;
    }

    int64_t CloneRead(const Btor2IRNode &read) {
        // Start with a nondeterministic miss value and overlay tracked slot hits.
        int64_t memoryId = m_readMemory.at(read.id);
        const Btor2IRSort &arraySort =
            m_input.Sort(m_input.Node(std::abs(memoryId)).sortId);
        int64_t address = CloneScalar(read.args[1]);
        auto slotsIt = m_slots.find(memoryId);
        // An untracked read is represented entirely by its abstract miss input.
        if (memoryId < 0 || slotsIt == m_slots.end() ||
            slotsIt->second.empty()) {
            AddNode(read.id,
                    BTOR2_TAG_input,
                    arraySort.elementSort,
                    {},
                    0,
                    "wl.read." + std::to_string(read.id) + ".miss");
            m_traceSources[read.id] = {
                WLTraceBitKind::AbstractReadInput, read.id, 0};
            m_scalarMap[read.id] = read.id;
            return read.id;
        }

        int64_t result = NewSyntheticNodeId();
        AddNode(result,
                BTOR2_TAG_input,
                arraySort.elementSort,
                {},
                0,
                "wl.read." + std::to_string(read.id) + ".miss");
        m_traceSources[result] = {
            WLTraceBitKind::AbstractReadInput, read.id, 0};

        // Build a priority ITE chain that returns concrete content on selector hits.
        auto &slots = slotsIt->second;
        const int64_t boolSort = EnsureBitVectorSort(1);
        for (size_t index = slots.size(); index-- > 0;) {
            const TrackedSlot &slot = slots[index];
            int64_t value =
                EvaluateArrayAt(read.args[0], memoryId, slot);
            int64_t hit = AddExpression(
                BTOR2_TAG_eq,
                boolSort,
                {address, slot.selectorId, 0},
                2);
            int64_t id =
                index == 0 ? read.id : NewSyntheticNodeId();
            AddNode(id,
                    BTOR2_TAG_ite,
                    arraySort.elementSort,
                    {hit, value, result},
                    3,
                    {});
            result = id;
        }
        m_scalarMap[read.id] = read.id;
        return read.id;
    }

    int64_t EvaluateArrayAt(int64_t expressionId,
                            int64_t memoryId,
                            const TrackedSlot &slot) {
        // Evaluate an array expression at one symbolic selector without materializing it.
        const Btor2IRNode &node = m_input.Node(expressionId);
        switch (node.tag) {
        case BTOR2_TAG_state:
            if (node.id != memoryId)
                throw std::runtime_error("array expression mixes memories");
            return slot.contentId;
        case BTOR2_TAG_write: {
            // A write changes tracked content exactly when its address matches.
            int64_t old =
                EvaluateArrayAt(node.args[0], memoryId, slot);
            int64_t hit = AddExpression(
                BTOR2_TAG_eq,
                EnsureBitVectorSort(1),
                {slot.selectorId, CloneScalar(node.args[1]), 0},
                2);
            return AddExpression(
                BTOR2_TAG_ite,
                m_input.Sort(node.sortId).elementSort,
                {hit, CloneScalar(node.args[2]), old},
                3);
        }
        case BTOR2_TAG_ite:
            return AddExpression(
                BTOR2_TAG_ite,
                m_input.Sort(node.sortId).elementSort,
                {CloneScalar(node.args[0]),
                 EvaluateArrayAt(node.args[1], memoryId, slot),
                 EvaluateArrayAt(node.args[2], memoryId, slot)},
                3);
        case BTOR2_TAG_input: {
            // A nondeterministic whole-array next value becomes one scalar input per slot.
            const Btor2IRSort &arraySort = m_input.Sort(node.sortId);
            int64_t inputId = NewSyntheticNodeId();
            AddNode(inputId,
                    BTOR2_TAG_input,
                    arraySort.elementSort,
                    {},
                    0,
                    "wl.array_input." + std::to_string(node.id));
            m_traceSources[inputId] = {
                WLTraceBitKind::ArrayNextInput, node.id, 0};
            return inputId;
        }
        default:
            throw std::runtime_error(
                "array expression is outside the remodellable subset");
        }
    }

    int64_t AddExpression(Btor2Tag tag,
                          int64_t sortId,
                          std::array<int64_t, 3> args,
                          uint32_t nargs) {
        // Expression helpers allocate synthetic IDs above the BTOR2 input ID range.
        int64_t id = NewSyntheticNodeId();
        AddNode(id, tag, sortId, args, nargs, {});
        return id;
    }

    void AddMetaNode(Btor2Tag tag,
                     int64_t sortId,
                     int64_t lhs,
                     int64_t rhs) {
        AddNode(NewSyntheticNodeId(),
                tag,
                sortId,
                {lhs, rhs, 0},
                2,
                {});
    }

    void AddNode(int64_t id,
                 Btor2Tag tag,
                 int64_t sortId,
                 std::array<int64_t, 3> args,
                 uint32_t nargs,
                 std::string symbol) {
        Btor2IRNode node;
        node.id = id;
        node.tag = tag;
        node.sortId = sortId;
        node.nargs = nargs;
        node.args = args;
        node.symbol = std::move(symbol);
        m_output.AddNode(node);
    }

    int64_t NewSyntheticNodeId() { return m_nextSyntheticNodeId++; }
    int64_t NewSyntheticSortId() { return m_nextSyntheticSortId++; }

    const Btor2IR &m_input;
    const std::vector<WLMemoryPair> &m_pairs;
    Btor2IR m_output;
    std::vector<WLMemoryPair> m_tracePairs;
    std::vector<WLArrayRead> m_reads;
    WLIRTraceMap m_traceSources;
    int64_t m_nextSyntheticNodeId{INT64_C(1) << 50};
    int64_t m_nextSyntheticSortId{INT64_C(1) << 49};
    std::unordered_map<int64_t, int64_t> m_scalarMap;
    std::unordered_map<int64_t, int64_t> m_inits;
    std::unordered_map<int64_t, int64_t> m_next;
    std::vector<int64_t> m_badIds;
    std::vector<int64_t> m_constraintIds;
    std::unordered_map<int64_t, int64_t> m_readMemory;
    std::unordered_map<int64_t, std::vector<WLMemoryPair>> m_requestedSlots;
    std::unordered_map<int64_t, std::vector<TrackedSlot>> m_slots;
};

} // namespace

WLArrayAbstractionResult
WLArrayAbstraction::Run(const Btor2IR &ir,
                        const std::vector<WLMemoryPair> &pairs) {
    return Builder(ir, pairs).Finish();
}

} // namespace car
