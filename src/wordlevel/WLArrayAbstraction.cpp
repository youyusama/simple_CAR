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
        : m_input(input), m_pairs(pairs) {}

    WLArrayAbstractionResult Build() {
        // Build a fresh array-free IR instead of mutating the source BTOR2 model.
        m_output.ReserveFreshIdsAfter(m_input);
        CopyBitVectorSorts();
        IndexModel();
        CreateBitVectorVariables();
        DeclareTrackedSlotStates();
        RewriteBitVectorLogicAndReads();
        BuildTrackedSlotTransitions();
        BuildUninitializedSlotConsistency();
        BuildProperties();
        m_output.SetHasArrays(false);
        return {std::move(m_output),
                std::move(m_tracePairs),
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
        // Array sorts disappear after abstraction; bit-vector sorts retain their IDs.
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
        int64_t id = m_output.FreshId();
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
            case BTOR2_TAG_bad: m_badId = node.id; break;
            case BTOR2_TAG_constraint:
                m_constraintIds.push_back(node.id);
                break;
            case BTOR2_TAG_read: {
                int64_t memoryId = FindMemory(node.args[0]);
                m_readMemory[node.id] = memoryId;
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
        throw std::runtime_error(
            "array expression is not remodellable at BTOR2 id " +
            std::to_string(node.id));
    }

    void CreateBitVectorVariables() {
        // Copy original bit-vector inputs/states and record their trace provenance.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            if (node.tag != BTOR2_TAG_input && node.tag != BTOR2_TAG_state)
                continue;
            if (!node.sortId ||
                m_input.Sort(node.sortId).tag != BTOR2_TAG_SORT_bitvec)
                continue;
            m_output.AddNode(node);
            m_bitVectorMap[node.id] = node.id;
            m_traceSources.insert_or_assign(
                node.id,
                WLValueOrigin{
                    node.tag == BTOR2_TAG_input
                        ? WLTraceKind::OriginalInput
                        : WLTraceKind::OriginalState,
                    node.id,
                    0});
        }
    }

    void DeclareTrackedSlotStates() {
        // Declare selector/content states before read and transition rewriting.
        for (const auto &[memoryId, requested] : m_requestedSlots) {
            const Btor2IRSort &arraySort =
                m_input.Sort(m_input.Node(memoryId).sortId);
            for (const WLMemoryPair &pair : requested) {
                TrackedSlot slot;
                slot.pair = pair;
                slot.pairIndex = m_tracePairs.size();
                slot.selectorId = m_output.FreshId();
                slot.contentId = m_output.FreshId();

                AddNode(slot.selectorId,
                        BTOR2_TAG_state,
                        arraySort.indexSort,
                        {},
                        0,
                        "wl.mem." + std::to_string(memoryId) + ".slot." +
                            std::to_string(slot.pairIndex) + ".selector");
                AddNode(slot.contentId,
                        BTOR2_TAG_state,
                        arraySort.elementSort,
                        {},
                        0,
                        "wl.mem." + std::to_string(memoryId) + ".slot." +
                            std::to_string(slot.pairIndex) + ".content");
                m_traceSources.insert_or_assign(
                    slot.selectorId,
                    WLValueOrigin{
                        WLTraceKind::SelectorState,
                        memoryId,
                        slot.pairIndex});
                m_traceSources.insert_or_assign(
                    slot.contentId,
                    WLValueOrigin{
                        WLTraceKind::ContentState,
                        memoryId,
                        slot.pairIndex});
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

    void RewriteBitVectorLogicAndReads() {
        // Clone bit-vector logic and replace array reads with selected-slot muxes.
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
            CloneBitVectorNode(node.id);
        }

        // Recreate init/next metadata for original bit-vector states.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            if (node.tag != BTOR2_TAG_init && node.tag != BTOR2_TAG_next)
                continue;
            const Btor2IRNode &state = m_input.Node(node.args[0]);
            if (m_input.Sort(state.sortId).tag == BTOR2_TAG_SORT_array)
                continue;
            Btor2IRNode copy = node;
            copy.args[0] = CloneBitVectorNode(node.args[0]);
            copy.args[1] = CloneBitVectorNode(node.args[1]);
            m_output.AddNode(copy);
        }
    }

    void BuildTrackedSlotTransitions() {
        // Derive each tracked content state's init/next expression at its selector.
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
                            CloneBitVectorNode(initIt->second));
            }
        }
    }

    void BuildUninitializedSlotConsistency() {
        // An uninitialized memory is one arbitrary function: equal selected
        // addresses must therefore denote equal initial contents.
        const int64_t boolSort = EnsureBitVectorSort(1);
        for (const auto &[memoryId, slots] : m_slots) {
            if (m_inits.count(memoryId)) continue;
            for (size_t i = 0; i < slots.size(); ++i) {
                for (size_t j = i + 1; j < slots.size(); ++j) {
                    int64_t sameSelector = AddExpression(
                        BTOR2_TAG_eq,
                        boolSort,
                        {slots[i].selectorId, slots[j].selectorId, 0},
                        2);
                    int64_t sameContent = AddExpression(
                        BTOR2_TAG_eq,
                        boolSort,
                        {slots[i].contentId, slots[j].contentId, 0},
                        2);
                    int64_t consistent = AddExpression(
                        BTOR2_TAG_implies,
                        boolSort,
                        {sameSelector, sameContent, 0},
                        2);
                    AddNode(m_output.FreshId(),
                            BTOR2_TAG_constraint,
                            boolSort,
                            {consistent, 0, 0},
                            1,
                            {});
                }
            }
        }
    }

    void BuildProperties() {
        // Delay address-match guards to constrain the requested time step.
        const int64_t boolSort = EnsureBitVectorSort(1);
        std::vector<int64_t> guards;
        for (auto &[memoryId, slots] : m_slots) {
            (void)memoryId;
            for (TrackedSlot &slot : slots) {
                int64_t guard = AddExpression(
                    BTOR2_TAG_eq,
                    boolSort,
                    {slot.selectorId,
                     CloneBitVectorNode(slot.pair.addressNodeId),
                     0},
                    2);
                for (unsigned i = 0; i < slot.pair.delay; ++i) {
                    int64_t delayId = m_output.FreshId();
                    AddNode(delayId,
                            BTOR2_TAG_state,
                            boolSort,
                            {},
                            0,
                            "wl.guard." + std::to_string(delayId));
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

        // Rebuild the single bad property against the transformed expressions.
        const Btor2IRNode &badNode = m_input.Node(m_badId);
        int64_t bad = CloneBitVectorNode(badNode.args[0]);
        if (allGuards) {
            bad = AddExpression(
                BTOR2_TAG_and,
                boolSort,
                {bad, allGuards, 0},
                2);
        }
        Btor2IRNode badCopy = badNode;
        badCopy.args[0] = bad;
        m_output.AddNode(badCopy);

        // Constraints remain unconditional and are cloned without guard weakening.
        for (int64_t constraintNodeId : m_constraintIds) {
            const Btor2IRNode &constraint =
                m_input.Node(constraintNodeId);
            Btor2IRNode copy = constraint;
            copy.args[0] = CloneBitVectorNode(constraint.args[0]);
            m_output.AddNode(copy);
        }
    }

    int64_t CloneBitVectorNode(int64_t signedId) {
        // Memoized recursive clone preserves original bit-vector node IDs where possible.
        if (signedId < 0) return -CloneBitVectorNode(-signedId);
        auto mapped = m_bitVectorMap.find(signedId);
        if (mapped != m_bitVectorMap.end()) return mapped->second;

        const Btor2IRNode &node = m_input.Node(signedId);
        if (node.tag == BTOR2_TAG_read) return CloneRead(node);
        if (node.sortId &&
            m_input.Sort(node.sortId).tag == BTOR2_TAG_SORT_array) {
            throw std::runtime_error(
                "array-valued node reached bit-vector rewriting");
        }

        Btor2IRNode copy = node;
        switch (node.tag) {
        case BTOR2_TAG_slice:
            copy.args[0] = CloneBitVectorNode(node.args[0]);
            break;
        case BTOR2_TAG_uext:
        case BTOR2_TAG_sext:
            copy.args[0] = CloneBitVectorNode(node.args[0]);
            break;
        default:
            for (uint32_t i = 0; i < node.nargs; ++i)
                copy.args[i] = CloneBitVectorNode(node.args[i]);
            break;
        }
        m_output.AddNode(copy);
        m_bitVectorMap[signedId] = copy.id;
        return copy.id;
    }

    int64_t CloneRead(const Btor2IRNode &read) {
        // Start with a nondeterministic miss value and overlay tracked slot hits.
        int64_t memoryId = m_readMemory.at(read.id);
        const Btor2IRSort &arraySort =
            m_input.Sort(m_input.Node(memoryId).sortId);
        int64_t address = CloneBitVectorNode(read.args[1]);
        auto slotsIt = m_slots.find(memoryId);
        // An untracked read is represented entirely by its abstract miss input.
        if (slotsIt == m_slots.end() || slotsIt->second.empty()) {
            AddNode(read.id,
                    BTOR2_TAG_input,
                    arraySort.elementSort,
                    {},
                    0,
                    "wl.read." + std::to_string(read.id) + ".miss");
            m_traceSources.insert_or_assign(
                read.id,
                WLValueOrigin{
                    WLTraceKind::AbstractReadInput, read.id, 0});
            m_bitVectorMap[read.id] = read.id;
            return read.id;
        }

        int64_t result = m_output.FreshId();
        AddNode(result,
                BTOR2_TAG_input,
                arraySort.elementSort,
                {},
                0,
                "wl.read." + std::to_string(read.id) + ".miss");
        m_traceSources.insert_or_assign(
            result,
            WLValueOrigin{
                WLTraceKind::AbstractReadInput, read.id, 0});

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
                index == 0 ? read.id : m_output.FreshId();
            AddNode(id,
                    BTOR2_TAG_ite,
                    arraySort.elementSort,
                    {hit, value, result},
                    3,
                    {});
            result = id;
        }
        m_bitVectorMap[read.id] = read.id;
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
                {slot.selectorId, CloneBitVectorNode(node.args[1]), 0},
                2);
            return AddExpression(
                BTOR2_TAG_ite,
                m_input.Sort(node.sortId).elementSort,
                {hit, CloneBitVectorNode(node.args[2]), old},
                3);
        }
        case BTOR2_TAG_ite:
            return AddExpression(
                BTOR2_TAG_ite,
                m_input.Sort(node.sortId).elementSort,
                {CloneBitVectorNode(node.args[0]),
                 EvaluateArrayAt(node.args[1], memoryId, slot),
                 EvaluateArrayAt(node.args[2], memoryId, slot)},
                3);
        default:
            throw std::runtime_error(
                "array expression is outside the remodellable subset");
        }
    }

    int64_t AddExpression(Btor2Tag tag,
                          int64_t sortId,
                          std::array<int64_t, 3> args,
                          uint32_t nargs) {
        // Expression helpers allocate IDs above the complete source ID space.
        int64_t id = m_output.FreshId();
        AddNode(id, tag, sortId, args, nargs, {});
        return id;
    }

    void AddMetaNode(Btor2Tag tag,
                     int64_t sortId,
                     int64_t lhs,
                     int64_t rhs) {
        AddNode(m_output.FreshId(),
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

    const Btor2IR &m_input;
    const std::vector<WLMemoryPair> &m_pairs;
    Btor2IR m_output;
    std::vector<WLMemoryPair> m_tracePairs;
    WLIRTraceMap m_traceSources;
    std::unordered_map<int64_t, int64_t> m_bitVectorMap;
    std::unordered_map<int64_t, int64_t> m_inits;
    std::unordered_map<int64_t, int64_t> m_next;
    int64_t m_badId{0};
    std::vector<int64_t> m_constraintIds;
    std::unordered_map<int64_t, int64_t> m_readMemory;
    std::unordered_map<int64_t, std::vector<WLMemoryPair>> m_requestedSlots;
    std::unordered_map<int64_t, std::vector<TrackedSlot>> m_slots;
};

} // namespace

WLArrayAbstractionResult
WLArrayAbstraction::Run(const Btor2IR &ir,
                        const std::vector<WLMemoryPair> &pairs) {
    return Builder(ir, pairs).Build();
}

} // namespace car
