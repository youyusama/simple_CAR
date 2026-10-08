#include "WLArrayAbstraction.h"
#include "WLSimulator.h"

#include <algorithm>
#include <map>
#include <set>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace car {

class WLArrayAbstraction::Builder {
  public:
    Builder(const WLArrayAbstraction &abstraction,
            const Precision &precision)
        : m_input(abstraction.IR()), m_abstraction(abstraction), m_build(abstraction.IR(), precision),
          m_slots(abstraction.Slots(precision)) {
        abstraction.ValidatePrecision(precision);
    }

    BuildResult Build() {
        m_build.ir.ReserveFreshIdsAfter(m_input);
        for (const auto &[id, sort] : m_input.Sorts()) {
            (void)id;
            if (sort.tag == BTOR2_TAG_SORT_bitvec)
                m_build.ir.AddSort(sort);
        }
        IndexModel();
        CreateVariables();
        DeclareComparisons();
        DeclareSlots();
        RewriteBitVectorLogicAndReads();
        BuildSlotTransitions();
        BuildSlotConsistency();
        BuildComparisons();
        BuildProperties();
        // Export only observed omega words, then discard construction bindings.
        for (int64_t q : m_comparisons)
            m_build.comparisons.at(q).address = m_addressNodes.at(m_abstraction.WitnessAddress(q));
        m_build.ir.SetHasArrays(false);
        return std::move(m_build);
    }

  private:
    int64_t EnsureBitVectorSort(uint32_t width) {
        for (const auto &[id, sort] : m_build.ir.Sorts()) {
            if (sort.width == width)
                return id;
        }
        return m_build.ir.AddSort(
            {m_build.ir.FreshId(), BTOR2_TAG_SORT_bitvec, width, 0, 0});
    }

    bool IsArray(int64_t sort) const {
        return sort && m_input.Sort(sort).tag == BTOR2_TAG_SORT_array;
    }

    bool IsArrayComparison(const Btor2IRNode &node) const {
        return (node.tag == BTOR2_TAG_eq || node.tag == BTOR2_TAG_neq) &&
               IsArray(m_input.Node(node.args[0]).sortId);
    }

    void IndexModel() {
        for (const auto &node : m_input.Nodes()) {
            if (IsArrayComparison(node))
                m_comparisons.push_back(node.id);
            switch (node.tag) {
            case BTOR2_TAG_init:
                m_inits[node.args[0]] = node.args[1];
                break;
            case BTOR2_TAG_next:
                m_next[node.args[0]] = node.args[1];
                break;
            case BTOR2_TAG_bad:
                m_badId = node.id;
                break;
            case BTOR2_TAG_constraint:
                m_constraintIds.push_back(node.id);
                break;
            case BTOR2_TAG_state:
            case BTOR2_TAG_input:
                if (IsArray(node.sortId))
                    m_roots.push_back(node.id);
                break;
            default:
                break;
            }
        }
    }

    void CreateVariables() {
        for (const auto &node : m_input.Nodes()) {
            if ((node.tag != BTOR2_TAG_input && node.tag != BTOR2_TAG_state) ||
                IsArray(node.sortId))
                continue;
            m_build.ir.AddNode(node);
            m_bitVectorMap[node.id] = node.id;
        }
    }

    void DeclareComparisons() {
        // Declare all comparison leaves before cloning any scalar control,
        // data or address cone. Constraints defining their meaning come later.
        for (int64_t id : m_comparisons) {
            const auto &node = m_input.Node(id);
            const auto &sort = m_input.Sort(m_input.Node(node.args[0]).sortId);
            const int64_t equal = m_build.ir.FreshId();
            const int64_t address = m_build.ir.FreshId();
            AddNode(equal, BTOR2_TAG_input, EnsureBitVectorSort(1), {}, 0,
                    "wl.eq." + std::to_string(id));
            AddNode(address, BTOR2_TAG_input, sort.indexSort, {}, 0,
                    "wl.omega." + std::to_string(id));
            auto &binding = m_build.comparisons[id];
            binding.equal = equal;
            m_addressNodes.emplace(m_abstraction.WitnessAddress(id), address);
            m_bitVectorMap.emplace(id, node.tag == BTOR2_TAG_neq ? -equal : equal);
        }
    }

    int64_t AddressValue(Address address) {
        const auto found = m_addressNodes.find(address);
        if (found != m_addressNodes.end()) return found->second;
        int64_t node;
        if (const auto value = m_abstraction.ConstantAddressValue(address)) {
            Btor2IRNode constant;
            constant.id = m_build.ir.FreshId();
            constant.tag = BTOR2_TAG_const;
            constant.sortId = m_abstraction.AddressSort(address);
            constant.constant = value->ToBinary();
            m_build.ir.AddNode(constant);
            node = constant.id;
        } else {
            // All omega inputs were bound by DeclareComparisons before cloning.
            node = CloneBitVectorNode(m_abstraction.OriginalNode(address));
        }
        m_addressNodes.emplace(address, node);
        return node;
    }

    void Require(int64_t condition) {
        AddNode(m_build.ir.FreshId(), BTOR2_TAG_constraint, 0, {condition, 0, 0}, 1, {});
    }

    void BuildComparisons() {
        std::map<std::pair<int64_t, int64_t>, int64_t> sameOperands;
        for (int64_t id : m_comparisons) {
            const auto &node = m_input.Node(id);
            const int64_t lhs = node.args[0], rhs = node.args[1];
            const int64_t equal = m_build.comparisons.at(id).equal;
            const int64_t address = AddressValue(m_abstraction.WitnessAddress(id));
            const auto prefix = "wl.eq." + std::to_string(id);
            const int64_t left = BuildRead(lhs, address, ReadMiss(lhs, prefix + ".left.miss"));
            const int64_t right = BuildRead(rhs, address, ReadMiss(rhs, prefix + ".right.miss"));
            m_build.comparisons.at(id).left = left;
            m_build.comparisons.at(id).right = right;
            // e -> equal witness reads, !e -> different witness reads.
            Require(Equal(equal, Equal(left, right)));
            for (size_t slot = 0; slot < m_build.precision.targets.size(); ++slot) {
                if (!m_abstraction.SameArrayGroup(m_build.precision.targets[slot].group, lhs))
                    continue;
                const int64_t samePoint =
                    Equal(EvaluateArrayAt(lhs, slot), EvaluateArrayAt(rhs, slot));
                Require(AddExpression(BTOR2_TAG_implies, EnsureBitVectorSort(1),
                                      {equal, samePoint, 0}, 2));
            }
            // Repeated/reversed syntactic operands denote the same relation.
            // Keep each q's own omega and provenance, without a read matrix.
            auto [previous, inserted] = sameOperands.emplace(
                std::make_pair(std::min(lhs, rhs), std::max(lhs, rhs)), equal);
            if (!inserted)
                Require(Equal(equal, previous->second));
        }
    }

    void DeclareSlots() {
        m_build.selectorBindings.resize(m_build.precision.targets.size());
        // Preserve deterministic allocation order: selector, then sorted roots.
        for (size_t index = 0; index < m_build.precision.targets.size(); ++index) {
            const auto &target = m_build.precision.targets[index];
            const int64_t indexSort = m_input.Sort(m_input.Node(target.group).sortId).indexSort;
            const int64_t selector = m_build.ir.FreshId();
            AddNode(selector, BTOR2_TAG_state, indexSort, {}, 0,
                    "wl.slot." + std::to_string(index) + ".selector");
            AddMetaNode(BTOR2_TAG_next, indexSort, selector, selector);
            m_build.selectorBindings[index] = selector;
            for (int64_t root : m_abstraction.ArrayRoots(target.group)) {
                const auto &node = m_input.Node(root);
                const int64_t elementSort = m_input.Sort(node.sortId).elementSort;
                const int64_t value = m_build.ir.FreshId();
                const auto key = std::make_pair(root, index);
                AddNode(value, node.tag, elementSort, {}, 0,
                        "wl.array." + std::to_string(root) + ".slot." + std::to_string(index));
                m_build.slotValueBindings.emplace(key, value);
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
            default:
                break;
            }
            if (node.sortId && m_input.Sort(node.sortId).tag == BTOR2_TAG_SORT_array)
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
            m_build.ir.AddNode(copy);
        }
    }

    void BuildSlotTransitions() {
        // All roots' point variables exist before simultaneous copy/swap equations.
        for (const auto &slot : m_slots) {
            const int64_t root = slot.arrayNodeId;
            if (m_input.Node(root).tag != BTOR2_TAG_state) continue;
            const int64_t elementSort = m_input.Sort(m_input.Node(root).sortId).elementSort;
            const int64_t value = m_build.slotValueBindings.at({root, slot.selectorIndex});
            auto next = m_next.find(root);
            if (next != m_next.end())
                AddMetaNode(BTOR2_TAG_next, elementSort, value,
                            EvaluateArrayAt(next->second, slot.selectorIndex));
            auto init = m_inits.find(root);
            if (init != m_inits.end()) {
                const int64_t initial = IsArray(m_input.Node(init->second).sortId)
                    ? EvaluateArrayAt(init->second, slot.selectorIndex) : CloneBitVectorNode(init->second);
                AddMetaNode(BTOR2_TAG_init, elementSort, value, initial);
            }
        }
    }

    int64_t Equal(int64_t lhs, int64_t rhs) {
        return AddExpression(BTOR2_TAG_eq, EnsureBitVectorSort(1), {lhs, rhs, 0}, 2);
    }

    void RequireSamePoint(int64_t a, int64_t x, int64_t b, int64_t y) {
        const int64_t condition =
            AddExpression(BTOR2_TAG_implies, EnsureBitVectorSort(1),
                          {Equal(a, b), Equal(x, y), 0}, 2);
        AddNode(m_build.ir.FreshId(), BTOR2_TAG_constraint, 0, {condition, 0, 0}, 1, {});
    }

    void BuildSlotConsistency() {
        // Needed every frame for inputs and no-next states; also sound for
        // initialized/updated states. Selectors need not be distinct.
        for (int64_t root : m_roots) {
            for (size_t i = 0; i < m_build.selectorBindings.size(); ++i) {
                auto lhs = m_build.slotValueBindings.find({root, i});
                if (lhs == m_build.slotValueBindings.end())
                    continue;
                for (size_t j = i + 1; j < m_build.selectorBindings.size(); ++j) {
                    auto rhs = m_build.slotValueBindings.find({root, j});
                    if (rhs != m_build.slotValueBindings.end())
                        RequireSamePoint(m_build.selectorBindings[i], lhs->second,
                                         m_build.selectorBindings[j], rhs->second);
                }
            }
        }
    }

    void BuildProperties() {
        const int64_t boolSort = EnsureBitVectorSort(1);
        int64_t allGuards = 0;
        int64_t zero = 0;
        for (size_t index = 0; index < m_build.precision.targets.size(); ++index) {
            const auto &binding = m_build.precision.targets[index];
            int64_t guard = Equal(m_build.selectorBindings.at(index), AddressValue(binding.address));
            for (unsigned i = 0; i < binding.delay; ++i) {
                if (!zero)
                    zero = AddExpression(BTOR2_TAG_zero, boolSort, {}, 0);
                int64_t delay = m_build.ir.FreshId();
                AddNode(delay, BTOR2_TAG_state, boolSort, {}, 0,
                        "wl.guard." + std::to_string(delay));
                AddMetaNode(BTOR2_TAG_init, boolSort, delay, zero);
                AddMetaNode(BTOR2_TAG_next, boolSort, delay, guard);
                guard = delay;
            }
            allGuards = allGuards ? AddExpression(BTOR2_TAG_and, boolSort,
                                                  {allGuards, guard, 0}, 2)
                                  : guard;
        }
        auto bad = m_input.Node(m_badId);
        bad.args[0] = CloneBitVectorNode(bad.args[0]);
        if (allGuards)
            bad.args[0] =
                AddExpression(BTOR2_TAG_and, boolSort, {bad.args[0], allGuards, 0}, 2);
        m_build.ir.AddNode(bad);
        // Guards only restrict bad, never the original constraints.
        for (int64_t id : m_constraintIds) {
            auto constraint = m_input.Node(id);
            constraint.args[0] = CloneBitVectorNode(constraint.args[0]);
            m_build.ir.AddNode(constraint);
        }
    }

    int64_t ScalarValue(int64_t id) const {
        return id < 0 ? -m_bitVectorMap.at(-id) : m_bitVectorMap.at(id);
    }

    // Called only after the original address expression has been built.
    int64_t OriginalAddressValue(int64_t id) {
        const Address address = m_abstraction.OriginalAddress(id);
        const auto [it, inserted] = m_addressNodes.emplace(address, ScalarValue(id));
        return it->second;
    }

    // Scalar rewriting and selected-point evaluation share one explicit stack:
    // a write's data/address may contain a read of another array expression.
    // A scalar task has no selector; an array task computes (node, selector).
    void BuildValue(int64_t expression, std::optional<size_t> selector) {
        using Key = std::pair<int64_t, std::optional<size_t>>;
        struct Frame {
            Key key;
            size_t phase{0};
            std::array<int64_t, 3> values{};
            size_t nextSelector{0};
        };
        std::vector<Frame> work;
        std::set<Key> active;
        const auto push = [&](int64_t id, std::optional<size_t> slot) {
            if (!slot && id < 0) {
                if (id == std::numeric_limits<int64_t>::min())
                    throw std::runtime_error("invalid scalar node reference");
                id = -id;
            }
            if (slot ? m_pointValues.count({id, *slot}) : m_bitVectorMap.count(id))
                return;
            const Key key{id, slot};
            if (!active.insert(key).second)
                throw std::runtime_error("cyclic array/scalar expression during abstraction");
            work.push_back({key});
        };
        push(expression, selector);
        while (!work.empty()) {
            // A push can reallocate work. Set the continuation before pushing
            // and return to the loop immediately; never keep a frame reference.
            auto &frame = work.back();
            const auto [id, slot] = frame.key;
            const auto &node = m_input.Node(id);
            int64_t result;
            if (slot) {
                if (node.tag == BTOR2_TAG_input || node.tag == BTOR2_TAG_state) {
                    result = m_build.slotValueBindings.at({node.id, *slot});
                } else if (node.tag == BTOR2_TAG_write || node.tag == BTOR2_TAG_ite) {
                    const bool write = node.tag == BTOR2_TAG_write;
                    if (frame.phase == 0) {
                        frame.phase = 1;
                        push(node.args[write ? 1 : 0], std::nullopt);
                        continue;
                    }
                    if (frame.phase == 1) {
                        frame.values[0] = write
                            ? Equal(m_build.selectorBindings[*slot], OriginalAddressValue(node.args[1]))
                            : ScalarValue(node.args[0]);
                        frame.phase = 2;
                        if (write) push(node.args[2], std::nullopt);
                        else push(node.args[1], slot);
                        continue;
                    }
                    if (frame.phase == 2) {
                        frame.values[1] = write ? ScalarValue(node.args[2])
                            : m_pointValues.at({node.args[1], *slot});
                        frame.phase = 3;
                        push(node.args[write ? 0 : 2], slot);
                        continue;
                    }
                    frame.values[2] = m_pointValues.at({node.args[write ? 0 : 2], *slot});
                    result = AddExpression(BTOR2_TAG_ite,
                        m_input.Sort(node.sortId).elementSort, frame.values, 3);
                } else {
                    throw std::runtime_error("unsupported array point expression");
                }
            } else if (node.tag == BTOR2_TAG_read) {
                if (frame.phase == 0) {
                    frame.phase = 1;
                    push(node.args[1], std::nullopt);
                    continue;
                }
                if (frame.phase == 1) {
                    frame.values[0] = OriginalAddressValue(node.args[1]);
                    frame.values[2] = ReadMiss(node.args[0], "wl.read." + std::to_string(node.id) + ".miss");
                    frame.nextSelector = m_build.precision.targets.size();
                    frame.phase = 2;
                }
                if (frame.phase == 2) {
                    while (frame.nextSelector && !m_abstraction.SameArrayGroup(
                        node.args[0], m_build.precision.targets[frame.nextSelector - 1].group))
                        --frame.nextSelector;
                    if (frame.nextSelector) {
                        --frame.nextSelector;
                        frame.values[1] = Equal(frame.values[0], m_build.selectorBindings[frame.nextSelector]);
                        frame.phase = 3;
                        push(node.args[0], frame.nextSelector);
                        continue;
                    }
                    result = frame.values[2];
                    m_build.semanticReads.emplace(id, result);
                } else {
                    frame.values[2] = AddExpression(BTOR2_TAG_ite, node.sortId,
                        {frame.values[1], m_pointValues.at({node.args[0], frame.nextSelector}), frame.values[2]}, 3);
                    frame.phase = 2;
                    continue;
                }
            } else {
                if (IsArray(node.sortId))
                    throw std::runtime_error("array-valued node reached bit-vector rewriting");
                // Slice bounds and extension widths are immediate arguments.
                const uint32_t nargs = node.tag == BTOR2_TAG_slice ||
                    node.tag == BTOR2_TAG_uext || node.tag == BTOR2_TAG_sext ? 1 : node.nargs;
                if (frame.phase < nargs) {
                    const int64_t argument = node.args[frame.phase++];
                    push(argument, std::nullopt);
                    continue;
                }
                Btor2IRNode copy = node;
                for (uint32_t i = 0; i < nargs; ++i) copy.args[i] = ScalarValue(node.args[i]);
                m_build.ir.AddNode(copy);
                result = copy.id;
            }
            if (slot) m_pointValues.emplace(std::make_pair(id, *slot), result);
            else m_bitVectorMap.emplace(id, result);
            active.erase(frame.key);
            work.pop_back();
        }
    }

    int64_t CloneBitVectorNode(int64_t signedId) {
        BuildValue(signedId, std::nullopt);
        return ScalarValue(signedId);
    }

    int64_t EvaluateArrayAt(int64_t expression, size_t index) {
        BuildValue(expression, index);
        return m_pointValues.at({expression, index});
    }

    int64_t ReadMiss(int64_t expression, const std::string &name) {
        const int64_t miss = m_build.ir.FreshId();
        AddNode(miss, BTOR2_TAG_input, m_input.Sort(m_input.Node(expression).sortId).elementSort,
                {}, 0, name);
        return miss;
    }

    int64_t BuildRead(int64_t expression, int64_t address, int64_t miss) {
        // Only selected points preserve array semantics. Each read owns its
        // miss, including witness reads and syntactically identical reads.
        const int64_t sort = m_input.Sort(m_input.Node(expression).sortId).elementSort;
        int64_t result = miss;
        for (size_t index = m_build.precision.targets.size(); index-- > 0;) {
            if (!m_abstraction.SameArrayGroup(expression, m_build.precision.targets[index].group))
                continue;
            result = AddExpression(BTOR2_TAG_ite, sort,
                                   {Equal(address, m_build.selectorBindings[index]),
                                    EvaluateArrayAt(expression, index), result}, 3);
        }
        return result;
    }

    int64_t AddExpression(Btor2Tag tag, int64_t sortId, std::array<int64_t, 3> args,
                          uint32_t nargs) {
        // Expression helpers allocate IDs above the complete source ID space.
        int64_t id = m_build.ir.FreshId();
        AddNode(id, tag, sortId, args, nargs, {});
        return id;
    }

    void AddMetaNode(Btor2Tag tag, int64_t sortId, int64_t lhs, int64_t rhs) {
        AddNode(m_build.ir.FreshId(), tag, sortId, {lhs, rhs, 0}, 2, {});
    }

    void AddNode(int64_t id, Btor2Tag tag, int64_t sortId, std::array<int64_t, 3> args,
                 uint32_t nargs, std::string symbol) {
        Btor2IRNode node;
        node.id = id;
        node.tag = tag;
        node.sortId = sortId;
        node.nargs = nargs;
        node.args = args;
        node.symbol = std::move(symbol);
        m_build.ir.AddNode(node);
    }

    const Btor2IR &m_input;
    const WLArrayAbstraction &m_abstraction;
    BuildResult m_build;
    const std::vector<Slot> m_slots;
    std::vector<int64_t> m_roots;
    std::unordered_map<Address, int64_t> m_addressNodes;
    std::unordered_map<int64_t, int64_t> m_bitVectorMap;
    std::unordered_map<int64_t, int64_t> m_inits;
    std::unordered_map<int64_t, int64_t> m_next;
    std::map<std::pair<int64_t, size_t>, int64_t> m_pointValues;
    std::vector<int64_t> m_comparisons;
    int64_t m_badId{0};
    std::vector<int64_t> m_constraintIds;
};

namespace {
using Values = std::unordered_map<int64_t, WLBitVector>;
using TargetKey = std::tuple<int64_t, WLArrayAbstraction::Address, unsigned>;
TargetKey Key(const WLArrayAbstraction::TrackingTarget &target) {
    return {target.group, target.address, target.delay};
}
bool Array(const Btor2IR &ir, int64_t id) {
    const auto sort = ir.Node(id).sortId;
    return sort && ir.Sort(sort).tag == BTOR2_TAG_SORT_array;
}
WLBitVector Invert(WLBitVector value) {
    for (unsigned i = 0; i < value.Width(); ++i) value.SetBit(i, !value.GetBit(i));
    return value;
}
void Patch(Values &values, int64_t id, WLBitVector value) {
    if (id < 0) { id = -id; value = Invert(std::move(value)); }
    auto [it, added] = values.emplace(id, value);
    if (!added && it->second != value)
        throw std::runtime_error("incompatible corrections for one abstract expression");
}
struct Index {
    std::vector<int64_t> inputs, states, reads, addressExpressions;
    std::map<int64_t, int64_t> init, next;
    std::map<std::pair<int64_t, int64_t>, std::vector<int64_t>> comparisons;
    explicit Index(const Btor2IR &ir) {
        std::set<int64_t> observe;
        for (const auto &n : ir.Nodes()) {
            if (n.tag == BTOR2_TAG_input) inputs.push_back(n.id);
            if (n.tag == BTOR2_TAG_state) states.push_back(n.id);
            if (n.tag == BTOR2_TAG_init) init[n.args[0]] = n.args[1];
            if (n.tag == BTOR2_TAG_next) next[n.args[0]] = n.args[1];
            if (n.tag == BTOR2_TAG_read) reads.push_back(n.id);
            if (n.tag == BTOR2_TAG_read || n.tag == BTOR2_TAG_write) observe.insert(n.args[1]);
            if (n.tag == BTOR2_TAG_ite && Array(ir, n.id)) observe.insert(n.args[0]);
            if ((n.tag == BTOR2_TAG_eq || n.tag == BTOR2_TAG_neq) && Array(ir, n.args[0]))
                comparisons[std::minmax(n.args[0], n.args[1])].push_back(n.id);
        }
        std::sort(reads.begin(), reads.end());
        for (auto &[operands, ids] : comparisons) std::sort(ids.begin(), ids.end());
        addressExpressions.assign(observe.begin(), observe.end());
    }
};

// Only nondeterministic choices survive a new simulation. An initialized or
// updated state is an observation, not a value to pin after a correction.
WLTrace FreeChoices(const Btor2IR &ir, const WLTrace &trace) {
    const Index index(ir);
    auto result = trace;
    for (size_t t = 0; t < result.steps.size(); ++t) {
        auto &step = result.steps[t];
        const auto &derived = t ? index.next : index.init;
        for (const auto &[id, expression] : derived) {
            step.stateValues.erase(id);
            step.arrayStateValues.erase(id);
        }
    }
    return result;
}

bool Survives(const WLSimulator::Execution &execution) {
    return !execution.bad.empty() && execution.bad.back() &&
        std::all_of(execution.constraintsHold.begin(), execution.constraintsHold.end(), [](bool b) { return b; });
}

} // namespace

class WLArrayAbstraction::Analyzer {
    struct Correction {
        size_t time;
        int64_t source;
        Values overrides;
        std::vector<TrackingTarget> targets;
        // A comparison's unequal reference operands supply this concrete address.
        // Address selection happens after the single reference execution is complete.
        std::optional<WLBitVector> difference;
        std::vector<int64_t> comparisons;
    };

  public:
    Analyzer(WLArrayAbstraction &abstraction, const BuildResult &build, const WLTrace &choices)
        : abstraction(abstraction), build(build), ir(abstraction.IR()), abstractIR(build.IR()),
          precision(abstraction.GetPrecision(build)), index(ir), abstractChoices(FreeChoices(abstractIR, choices)) {
        if (choices.steps.empty() || choices.steps.size() - 1 > std::numeric_limits<unsigned>::max())
            throw std::runtime_error("refinement needs a nonempty representable trace");
        end = choices.steps.size() - 1;
        for (const auto &target : precision.targets) existing.insert(Key(target));
    }

    AnalysisResult Run() {
        abstraction.ValidatePrecision(precision);
        const auto expressions = abstraction.ObservationExpressions(build);
        auto abstractExecution = WLSimulator(abstractIR).Simulate(abstractChoices, expressions);
        if (!Survives(abstractExecution))
            throw std::runtime_error("abstract IR replay does not reproduce the endpoint counterexample");
        observations = std::move(abstractExecution.observations);
        reference = ReferenceChoices();
        referenceValues.resize(end + 1);
        WLSimulator::SimulationOptions options;
        options.onFrame = [&](size_t time, const WLSimulator::Frame &frame) {
            for (auto id : index.addressExpressions) referenceValues[time].emplace(id, frame.Scalar(id));
            CollectCorrections(time, frame);
        };
        const auto concrete = WLSimulator(ir).Simulate(reference, {}, WLSimulator::MissingChoices::Reject, options);
        bool valid = true;
        for (size_t time = 0; time <= end; ++time) {
            valid &= concrete.constraintsHold.at(time);
            if (valid && concrete.bad.at(time)) {
                reference.steps.resize(time + 1);
                result.kind = AnalysisResult::Kind::ConcreteCandidate;
                result.trace = std::move(reference);
                return std::move(result);
            }
        }
        for (auto &correction : corrections) {
            if (correction.difference) {
                const auto &q = ir.Node(correction.source);
                correction.targets.push_back(DifferenceTarget(q, correction.time, *correction.difference));
            }
            auto &targets = correction.targets;
            targets.erase(std::remove_if(targets.begin(), targets.end(), [&](const auto &t) {
                return existing.count(Key(t));
            }), targets.end());
        }
        corrections.erase(std::remove_if(corrections.begin(), corrections.end(), [](const auto &c) {
            return c.targets.empty();
        }), corrections.end());
        std::sort(corrections.begin(), corrections.end(), [](const auto &a, const auto &b) {
            return std::tie(a.time, a.source) < std::tie(b.time, b.source);
        });
        for (const auto &c : corrections) {
            if (c.comparisons.empty()) ++result.readCorrections;
            else ++result.comparisonCorrections;
        }
        if (corrections.empty()) return Unknown("no correction with a new tracking target");
        std::vector<bool> retained(corrections.size(), true);
        if (Trial(retained)) return Unknown("full correction retains the endpoint counterexample");
        bool changed;
        do {
            changed = false;
            for (size_t i = 0; i < retained.size(); ++i) {
                if (!retained[i]) continue;
                retained[i] = false;
                if (Trial(retained)) retained[i] = true;
                else changed = true;
            }
        } while (changed);
        auto keys = existing;
        for (size_t i = 0; i < retained.size(); ++i)
            if (retained[i]) for (const auto &target : corrections[i].targets) {
                abstraction.ValidateTarget(target);
                if (keys.insert(Key(target)).second) result.targets.push_back(target);
            }
        if (result.targets.empty()) return Unknown("no new target after greedy reduction");
        result.kind = AnalysisResult::Kind::Refined;
        return std::move(result);
    }

  private:
    AnalysisResult Unknown(const std::string &reason) {
        result.kind = AnalysisResult::Kind::Unknown;
        result.reason = reason;
        result.targets.clear();
        return std::move(result);
    }
    WLBitVector Observed(size_t time, int64_t expression) const {
        if (expression < 0) return Invert(Observed(time, -expression));
        return observations.at(time).at(expression);
    }
    unsigned Delay(size_t time) const { return static_cast<unsigned>(end - time); }

    WLArrayValue FreeArray(int64_t root, size_t time) const {
        const auto &sort = ir.Sort(ir.Node(root).sortId);
        WLArrayValue value;
        value.defaultValue = WLBitVector::Zero(ir.Sort(sort.elementSort).width);
        std::map<std::string, WLArrayEntry> cells;
        for (size_t j = 0; j < precision.targets.size(); ++j) {
            if (!abstraction.SameArrayGroup(root, precision.targets[j].group)) continue;
            auto address = Observed(time, abstraction.SelectorWord(build, j));
            auto data = Observed(time, abstraction.SlotWord(build, root, j));
            auto [it, added] = cells.emplace(address.ToBinary(), WLArrayEntry{address, data});
            if (!added && it->second.value != data)
                throw std::runtime_error("inconsistent free array slot collision");
        }
        for (auto &[address, cell] : cells) value.entries.push_back(std::move(cell));
        return value;
    }
    WLTrace ReferenceChoices() const {
        WLTrace trace;
        trace.steps.resize(end + 1);
        for (size_t time = 0; time <= end; ++time) {
            auto &step = trace.steps[time];
            for (auto id : index.inputs) {
                if (Array(ir, id)) step.arrayInputValues.emplace(id, FreeArray(id, time));
                else step.inputValues.emplace(id, Observed(time, id));
            }
            for (auto id : index.states) {
                if (!(time ? index.next : index.init).count(id)) {
                    if (Array(ir, id)) step.arrayStateValues.emplace(id, FreeArray(id, time));
                    else step.stateValues.emplace(id, Observed(time, id));
                }
            }
        }
        return trace;
    }

    void CollectCorrections(size_t time, const WLSimulator::Frame &frame) {
        for (auto id : index.reads) {
            const auto &node = ir.Node(id);
            const auto expression = abstraction.ReadExpression(build, id);
            auto value = frame.Scalar(id);
            if (value == Observed(time, expression)) continue;
            Correction correction{time, id, {}, {}, {}, {}};
            Patch(correction.overrides, expression, std::move(value));
            correction.targets.push_back(abstraction.MakeTarget(node.args[0],
                abstraction.OriginalAddress(node.args[1]), Delay(time)));
            corrections.push_back(std::move(correction));
        }
        for (const auto &[operands, ids] : index.comparisons) {
            const auto &first = ir.Node(ids.front());
            auto equality = frame.Scalar(first.id);
            if (first.tag == BTOR2_TAG_neq) equality = Invert(std::move(equality));
            if (equality == Observed(time, abstraction.ComparisonExpressions(build, first.id).equal)) continue;
            Correction correction{time, first.id, {}, {}, {}, ids};
            if (!equality.IsOne()) {
                correction.difference = frame.FindArrayDifference(first.args[0], first.args[1]);
                if (!correction.difference)
                    throw std::runtime_error("unequal reference arrays have no difference address");
            }
            for (auto id : ids) {
                const auto &q = ir.Node(id);
                const auto binding = abstraction.ComparisonExpressions(build, id);
                const auto address = correction.difference ? *correction.difference : Observed(time, binding.address);
                Patch(correction.overrides, binding.equal, equality);
                Patch(correction.overrides, binding.address, address);
                Patch(correction.overrides, binding.left, frame.ReadArray(q.args[0], address));
                Patch(correction.overrides, binding.right, frame.ReadArray(q.args[1], address));
                if (equality.IsOne()) correction.targets.push_back(abstraction.MakeTarget(q.args[0],
                    abstraction.WitnessAddress(id), Delay(time)));
            }
            corrections.push_back(std::move(correction));
        }
    }

    // Follow only the two reference array histories at the differing cell.
    // Native symbolic addresses are preferred to freezing the numerical value.
    // This is target selection, not a conflict certificate or an exclusion proof.
    TrackingTarget DifferenceTarget(const Btor2IRNode &comparison, size_t time,
                                     const WLBitVector &address) {
        std::set<std::pair<size_t, int64_t>> visited;
        std::vector<std::pair<size_t, int64_t>> todo{{time, comparison.args[0]}, {time, comparison.args[1]}};
        std::vector<TrackingTarget> candidates;
        while (!todo.empty()) {
            const auto [t, id] = todo.back(); todo.pop_back();
            if (!visited.emplace(t, id).second) continue;
            const auto &n = ir.Node(id);
            for (auto read : index.reads) {
                const auto &r = ir.Node(read);
                if (r.args[0] == id && referenceValues[t].at(r.args[1]) == address)
                    candidates.push_back(abstraction.MakeTarget(id, abstraction.OriginalAddress(r.args[1]), Delay(t)));
            }
            if (n.tag == BTOR2_TAG_write) {
                if (referenceValues[t].at(n.args[1]) == address)
                    candidates.push_back(abstraction.MakeTarget(id, abstraction.OriginalAddress(n.args[1]), Delay(t)));
                else todo.emplace_back(t, n.args[0]);
            } else if (n.tag == BTOR2_TAG_ite) {
                todo.emplace_back(t, n.args[referenceValues[t].at(n.args[0]).IsOne() ? 1 : 2]);
            } else if (n.tag == BTOR2_TAG_state) {
                const auto &relations = t ? index.next : index.init;
                auto relation = relations.find(id);
                if (relation != relations.end() && Array(ir, relation->second))
                    todo.emplace_back(t ? t - 1 : 0, relation->second);
            } else if (n.tag != BTOR2_TAG_input) {
                throw std::runtime_error("invalid reference array history");
            }
        }
        std::sort(candidates.begin(), candidates.end(), [&](const auto &a, const auto &b) {
            return std::make_tuple(a.delay, abstraction.AddressOrder(a.address)) <
                   std::make_tuple(b.delay, abstraction.AddressOrder(b.address));
        });
        for (const auto &candidate : candidates)
            if (!existing.count(Key(candidate))) return candidate;
        const auto sort = ir.Sort(ir.Node(comparison.args[0]).sortId).indexSort;
        return abstraction.MakeTarget(comparison.args[0], abstraction.RegisterConstantAddress(sort, address), Delay(time));
    }

    bool Trial(const std::vector<bool> &retained) {
        WLSimulator::SimulationOptions options;
        options.overrides.resize(end + 1);
        for (size_t i = 0; i < retained.size(); ++i)
            if (retained[i]) for (const auto &[id, value] : corrections[i].overrides)
                Patch(options.overrides[corrections[i].time], id, value);
        ++result.trials;
        return Survives(WLSimulator(abstractIR).Simulate(abstractChoices, {},
            WLSimulator::MissingChoices::Reject, options));
    }

    WLArrayAbstraction &abstraction;
    const BuildResult &build;
    const Btor2IR &ir, &abstractIR;
    const Precision &precision;
    Index index;
    WLTrace abstractChoices, reference;
    size_t end{0};
    std::vector<Values> observations, referenceValues;
    std::set<TargetKey> existing;
    std::vector<Correction> corrections;
    AnalysisResult result;
};

WLArrayAbstraction::WLArrayAbstraction(const Btor2IR &ir) : m_ir(ir) {
    ir.ValidateSupportedArrays();
    std::vector<int64_t> comparisons;
    // Temporary union-find over array values, including cross-time init/next.
    // Scalar dependencies (addresses, conditions and data) do not merge groups.
    std::unordered_map<int64_t, int64_t> parent;
    std::unordered_map<int64_t, unsigned> rank;
    auto array = [&](int64_t id) {
        const auto &node = ir.Node(id);
        return node.sortId && ir.Sort(node.sortId).tag == BTOR2_TAG_SORT_array;
    };
    auto find = [&](int64_t id) {
        parent.emplace(id, id);
        int64_t root = id;
        while (parent.at(root) != root) root = parent.at(root);
        while (parent.at(id) != id) {
            const int64_t nextParent = parent.at(id);
            parent.at(id) = root;
            id = nextParent;
        }
        return root;
    };
    auto merge = [&](int64_t lhs, int64_t rhs) {
        if (ir.Node(lhs).sortId != ir.Node(rhs).sortId)
            throw std::runtime_error("array dependency requires matching array sorts");
        lhs = find(lhs);
        rhs = find(rhs);
        if (lhs == rhs) return;
        if (rank[lhs] < rank[rhs]) std::swap(lhs, rhs);
        parent.at(rhs) = lhs;
        if (rank[lhs] == rank[rhs]) ++rank[lhs];
    };
    for (const auto &node : ir.Nodes()) {
        if ((node.tag == BTOR2_TAG_eq || node.tag == BTOR2_TAG_neq) &&
            array(node.args[0])) {
            comparisons.push_back(node.id);
            merge(node.args[0], node.args[1]);
        }
        switch (node.tag) {
        case BTOR2_TAG_input:
        case BTOR2_TAG_state:
            if (array(node.id)) find(node.id);
            break;
        case BTOR2_TAG_write:
            merge(node.id, node.args[0]);
            break;
        case BTOR2_TAG_ite:
            if (array(node.id)) {
                merge(node.id, node.args[1]);
                merge(node.id, node.args[2]);
            }
            break;
        case BTOR2_TAG_init:
        case BTOR2_TAG_next:
            if (array(node.args[0]) && array(node.args[1]))
                merge(node.args[0], node.args[1]);
            break;
        default:
            break;
        }
    }
    std::sort(comparisons.begin(), comparisons.end());
    // Public representatives are original minimum IDs, independent of union
    // order, rank and unordered-map iteration.
    std::unordered_map<int64_t, int64_t> minimum;
    for (const auto &[id, unused] : parent) {
        (void)unused;
        const int64_t root = find(id);
        auto [it, inserted] = minimum.emplace(root, id);
        if (!inserted) it->second = std::min(it->second, id);
    }
    for (const auto &[id, unused] : parent) {
        (void)unused;
        const int64_t group = minimum.at(find(id));
        m_arrayGroups.emplace(id, group);
        auto &roots = m_arrayRoots[group];
        const auto tag = ir.Node(id).tag;
        if (tag == BTOR2_TAG_input || tag == BTOR2_TAG_state) roots.push_back(id);
    }
    for (auto &[group, roots] : m_arrayRoots) {
        (void)group;
        std::sort(roots.begin(), roots.end());
    }
    std::set<int64_t> indexSorts, originalAddresses;
    for (const auto &[id, sort] : ir.Sorts())
        if (sort.tag == BTOR2_TAG_SORT_array) indexSorts.insert(sort.indexSort);
    for (const auto &node : ir.Nodes()) {
        if (node.tag == BTOR2_TAG_read || node.tag == BTOR2_TAG_write)
            originalAddresses.insert(node.args[1]);
        if (node.tag == BTOR2_TAG_state && indexSorts.count(node.sortId)) {
            originalAddresses.insert(node.id);
            originalAddresses.insert(-node.id);
        }
    }
    for (int64_t id : originalAddresses) RegisterOriginalAddress(id);
    for (int64_t q : comparisons) {
        const Address address = m_addressSources.size();
        m_addressSources.push_back(WLArrayAbstraction::WitnessSource{q});
        m_witnessAddresses.emplace(q, address);
    }
}

int64_t WLArrayAbstraction::ArrayGroup(int64_t arrayNodeId) const {
    return m_arrayGroups.at(arrayNodeId);
}

const std::vector<int64_t> &WLArrayAbstraction::ArrayRoots(int64_t arrayNodeId) const {
    return m_arrayRoots.at(ArrayGroup(arrayNodeId));
}

bool WLArrayAbstraction::SameArrayGroup(int64_t lhs, int64_t rhs) const {
    return ArrayGroup(lhs) == ArrayGroup(rhs);
}

const WLArrayAbstraction::AddressSource &WLArrayAbstraction::Source(Address address) const {
    if (!address || address >= m_addressSources.size())
        throw std::runtime_error("undeclared abstraction address index");
    return m_addressSources[address];
}

WLArrayAbstraction::Address WLArrayAbstraction::RegisterOriginalAddress(int64_t signedNodeId) {
    const auto &ir = m_ir;
    if (!signedNodeId || signedNodeId == std::numeric_limits<int64_t>::min())
        throw std::runtime_error("address requires an original BV node reference");
    const auto &node = ir.Node(signedNodeId);
    if (!node.sortId || ir.Sort(node.sortId).tag != BTOR2_TAG_SORT_bitvec ||
        node.tag == BTOR2_TAG_init || node.tag == BTOR2_TAG_next)
        throw std::runtime_error("address requires an original BV value");
    const auto found = m_originalAddresses.find(signedNodeId);
    if (found != m_originalAddresses.end()) return found->second;
    const Address address = m_addressSources.size();
    m_addressSources.push_back(OriginalSource{signedNodeId});
    m_originalAddresses.emplace(signedNodeId, address);
    return address;
}

WLArrayAbstraction::Address WLArrayAbstraction::RegisterConstantAddress(
    int64_t indexSort, const WLBitVector &value) {
    const auto &sort = m_ir.Sort(indexSort);
    if (sort.tag != BTOR2_TAG_SORT_bitvec || sort.width != value.Width())
        throw std::runtime_error("constant address requires a matching BV index sort");
    indexSort = sort.id;
    const auto key = std::make_pair(indexSort, value.ToBinary());
    const auto found = m_constantAddresses.find(key);
    if (found != m_constantAddresses.end()) return found->second;
    const Address address = m_addressSources.size();
    m_addressSources.push_back(ConstantSource{indexSort, value});
    m_constantAddresses.emplace(key, address);
    return address;
}

WLArrayAbstraction::Address WLArrayAbstraction::OriginalAddress(int64_t signedNodeId) const {
    const auto found = m_originalAddresses.find(signedNodeId);
    if (found == m_originalAddresses.end())
        throw std::runtime_error("undeclared original address node " + std::to_string(signedNodeId));
    return found->second;
}

WLArrayAbstraction::Address WLArrayAbstraction::WitnessAddress(int64_t comparison) const {
    return m_witnessAddresses.at(comparison);
}

std::optional<int64_t> WLArrayAbstraction::WitnessComparison(Address address) const {
    if (const auto *witness = std::get_if<WitnessSource>(&Source(address)))
        return witness->comparisonNodeId;
    return std::nullopt;
}

std::optional<WLBitVector> WLArrayAbstraction::ConstantAddressValue(Address address) const {
    if (const auto *constant = std::get_if<ConstantSource>(&Source(address)))
        return constant->value;
    return std::nullopt;
}

int64_t WLArrayAbstraction::OriginalNode(Address address) const {
    if (const auto *original = std::get_if<OriginalSource>(&Source(address)))
        return original->signedNodeId;
    throw std::runtime_error("abstraction address is not an original BV node");
}

int64_t WLArrayAbstraction::AddressSort(Address address) const {
    const auto &ir = m_ir;
    if (const auto *constant = std::get_if<ConstantSource>(&Source(address)))
        return constant->indexSort;
    if (const auto q = WitnessComparison(address)) {
        const auto &node = ir.Node(*q);
        if ((node.tag != BTOR2_TAG_eq && node.tag != BTOR2_TAG_neq) ||
            ir.Sort(ir.Node(node.args[0]).sortId).tag != BTOR2_TAG_SORT_array)
            throw std::runtime_error("witness declaration requires an array comparison");
        return ir.Sort(ir.Node(node.args[0]).sortId).indexSort;
    }
    return ir.Node(OriginalNode(address)).sortId;
}

WLArrayAbstraction::TrackingTarget WLArrayAbstraction::MakeTarget(
    int64_t arrayNodeId, Address address, unsigned delay) const {
    const auto group = ArrayGroup(arrayNodeId);
    const auto &sort = m_ir.Sort(m_ir.Node(arrayNodeId).sortId);
    if (sort.tag != BTOR2_TAG_SORT_array || AddressSort(address) != sort.indexSort)
        throw std::runtime_error("tracking target requires an address of its array index sort");
    if (const auto q = WitnessComparison(address))
        if (!SameArrayGroup(m_ir.Node(*q).args[0], arrayNodeId))
            throw std::runtime_error("witness address requires the same array dependency group");
    if (ArrayRoots(group).empty())
        throw std::runtime_error("tracking target requires a nonempty array group");
    return {group, address, delay};
}

void WLArrayAbstraction::ValidateTarget(const TrackingTarget &target) const {
    if (ArrayGroup(target.group) != target.group)
        throw std::runtime_error("tracking target requires a canonical array group");
    MakeTarget(target.group, target.address, target.delay);
}

std::pair<int, int64_t> WLArrayAbstraction::AddressOrder(
    Address address) const {
    if (const auto q = WitnessComparison(address)) return {1, *q};
    if (std::holds_alternative<ConstantSource>(Source(address)))
        return {2, static_cast<int64_t>(address)};
    return {0, OriginalNode(address)};
}

std::vector<int64_t> WLArrayAbstraction::ObservationExpressions(const BuildResult &build) const {
    CheckBuild(build);
    std::set<int64_t> expressions;
    // Scalar ports retain their source IDs; only rewritten array semantics
    // need persistent bindings in the build.
    for (const auto &node : m_ir.Nodes())
        if ((node.tag == BTOR2_TAG_input || node.tag == BTOR2_TAG_state) &&
            !Array(m_ir, node.id))
            expressions.insert(node.id);
    for (auto [id, expression] : build.semanticReads) expressions.insert(expression);
    for (auto word : build.selectorBindings) expressions.insert(word);
    for (auto [slot, word] : build.slotValueBindings) expressions.insert(word);
    for (auto [id, b] : build.comparisons)
        for (auto expression : {b.equal, b.address, b.left, b.right}) expressions.insert(expression);
    return {expressions.begin(), expressions.end()};
}

unsigned WLArrayAbstraction::Precision::MaxDelay() const {
    unsigned result = 0;
    for (const auto &target : targets) result = std::max(result, target.delay);
    return result;
}

std::vector<WLArrayAbstraction::Slot> WLArrayAbstraction::Slots(const Precision &precision) const {
    std::vector<Slot> result;
    for (size_t j = 0; j < precision.targets.size(); ++j)
        for (auto root : ArrayRoots(precision.targets[j].group)) result.push_back({root, j});
    return result;
}

unsigned WLArrayAbstraction::MaxDelay(const BuildResult &build) const {
    return GetPrecision(build).MaxDelay();
}

size_t WLArrayAbstraction::SelectorCount(const BuildResult &build) const {
    return GetPrecision(build).targets.size();
}

size_t WLArrayAbstraction::SlotCount(const BuildResult &build) const {
    size_t count = 0;
    for (const auto &target : GetPrecision(build).targets) count += ArrayRoots(target.group).size();
    return count;
}

void WLArrayAbstraction::ValidatePrecision(const Precision &precision) const {
    for (const auto &target : precision.targets) ValidateTarget(target);
}

std::optional<WLArrayAbstraction::Precision> WLArrayAbstraction::ExtendPrecision(
    const BuildResult &build, const std::vector<TrackingTarget> &targets) const {
    const auto &precision = GetPrecision(build);
    ValidatePrecision(precision);
    auto next = precision;
    std::set<std::tuple<int64_t, Address, unsigned>> keys;
    for (const auto &target : next.targets) keys.emplace(target.group, target.address, target.delay);
    for (const auto &target : targets) {
        ValidateTarget(target);
        if (keys.emplace(target.group, target.address, target.delay).second)
            next.targets.push_back(target);
    }
    if (next.targets.size() == precision.targets.size()) return std::nullopt;
    return next;
}

WLArrayAbstraction::BuildResult WLArrayAbstraction::Build(
    const Precision &precision) const {
    return Builder(*this, precision).Build();
}

void WLArrayAbstraction::CheckBuild(const BuildResult &build) const {
    if (build.source != &m_ir)
        throw std::runtime_error("array build belongs to a different source IR");
}

const WLArrayAbstraction::Precision &WLArrayAbstraction::GetPrecision(const BuildResult &build) const {
    CheckBuild(build);
    return build.precision;
}
int64_t WLArrayAbstraction::ReadExpression(const BuildResult &build, int64_t read) const {
    CheckBuild(build);
    return build.semanticReads.at(read);
}
int64_t WLArrayAbstraction::SelectorWord(const BuildResult &build, size_t selector) const {
    CheckBuild(build);
    return build.selectorBindings.at(selector);
}
int64_t WLArrayAbstraction::SlotWord(const BuildResult &build, int64_t arrayNodeId, size_t selector) const {
    CheckBuild(build);
    return build.slotValueBindings.at({arrayNodeId, selector});
}
WLArrayAbstraction::ComparisonBinding WLArrayAbstraction::ComparisonExpressions(
    const BuildResult &build, int64_t comparison) const {
    CheckBuild(build);
    return build.comparisons.at(comparison);
}

WLArrayAbstraction::AnalysisResult WLArrayAbstraction::AnalyzeCounterexample(
    const BuildResult &build, const WLTrace &abstractChoices) {
    try {
        return Analyzer(*this, build, abstractChoices).Run();
    } catch (const std::exception &error) {
        AnalysisResult result;
        result.reason = std::string("array greedy: ") + error.what();
        return result;
    }
}

} // namespace car
