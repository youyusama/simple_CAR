#include "WLBoundedUnroller.h"

#include <set>
#include <stdexcept>

namespace car {

WLBoundedUnroller::WLBoundedUnroller(const Btor2IR &source, unsigned bound)
    : m_source(source), m_bound(bound) {
    m_output.CopySortsFrom(source);
    for (const auto &[id, sort] : source.Sorts()) {
        if (sort.tag == BTOR2_TAG_SORT_bitvec && sort.width == 1) m_boolSort = id;
    }
    if (!m_boolSort) {
        m_boolSort = m_output.AddSort(
            {m_output.FreshId(), BTOR2_TAG_SORT_bitvec, 1, 0, 0});
    }
    int64_t bad = 0;
    for (const auto &node : source.Nodes()) {
        if (node.tag == BTOR2_TAG_init) m_init[node.args[0]] = node.args[1];
        if (node.tag == BTOR2_TAG_next) m_next[node.args[0]] = node.args[1];
        if (node.tag == BTOR2_TAG_bad) bad = node.args[0];
    }
    if (!bad) throw std::runtime_error("bounded unrolling requires a bad property");
    for (unsigned time = 0;; ++time) {
        for (const auto &node : source.Nodes()) {
            const bool state = node.tag == BTOR2_TAG_state;
            if (!state && node.tag != BTOR2_TAG_input) continue;
            const bool array = source.Sort(node.sortId).tag == BTOR2_TAG_SORT_array;
            const int64_t lowered = Term(node.id, time);
            const bool free = !state || (time == 0 ? !m_init.count(node.id) : !m_next.count(node.id));
            if (!array || free) m_interface.push_back({node.id, lowered, time, array, state});
            if (!state || array) continue;
            if (time == 0 && m_init.count(node.id))
                RequireEqual(lowered, Term(m_init.at(node.id), 0));
            if (time > 0 && m_next.count(node.id))
                RequireEqual(lowered, Term(m_next.at(node.id), time - 1));
        }
        for (const auto &node : source.Nodes())
            if (node.tag == BTOR2_TAG_constraint) Constraint(Term(node.args[0], time));
        if (time == bound) break;
    }
    const int64_t condition = Term(bad, bound);
    Btor2IRNode property;
    property.id = m_output.FreshId();
    property.tag = BTOR2_TAG_bad;
    property.nargs = 1;
    property.args[0] = condition;
    m_output.AddNode(property);
}

int64_t WLBoundedUnroller::Term(int64_t id, unsigned time) {
    using Key = std::pair<int64_t, unsigned>;
    struct Frame {
        enum class Kind { Expression, Alias, Uniform };
        Key key;
        Btor2IRNode node;
        Kind kind{Kind::Expression};
        unsigned operand{0};
        int64_t dependency{0};
        unsigned dependencyTime{0};
    };
    auto keyOf = [](int64_t value, unsigned frame) {
        return Key{value < 0 ? -value : value, frame};
    };
    auto lookup = [&](int64_t value, unsigned frame) {
        const int64_t lowered = m_terms.at(keyOf(value, frame));
        return value < 0 ? -lowered : lowered;
    };
    const Key requested = keyOf(id, time);
    if (m_terms.count(requested)) return lookup(id, time);

    std::vector<Frame> stack;
    std::set<Key> building;
    auto push = [&](int64_t value, unsigned frame) {
        const Key key = keyOf(value, frame);
        if (!building.insert(key).second)
            throw std::runtime_error("cyclic bounded array initialization");
        Frame work{key, m_source.Node(key.first)};
        auto &node = work.node;
        if (!node.sortId) throw std::runtime_error("metadata used as bounded value");
        if (node.tag == BTOR2_TAG_state &&
            m_source.Sort(node.sortId).tag == BTOR2_TAG_SORT_array) {
            if (frame > 0 && m_next.count(key.first)) {
                work.kind = Frame::Kind::Alias;
                work.dependency = m_next.at(key.first);
                work.dependencyTime = frame - 1;
            } else if (frame == 0 && m_init.count(key.first)) {
                work.dependency = m_init.at(key.first);
                work.kind = m_source.Sort(m_source.Node(work.dependency).sortId).tag ==
                                    BTOR2_TAG_SORT_array
                                ? Frame::Kind::Alias : Frame::Kind::Uniform;
            }
        }
        if (work.kind != Frame::Kind::Alias) {
            node.id = m_output.FreshId();
            node.symbol = "wl.unroll." + std::to_string(key.first) + "." +
                          std::to_string(frame);
            if (work.kind == Frame::Kind::Uniform) {
                // Its scalar initializer may refer back to this array. Publish
                // the declaration first, then finish its defining init equation.
                m_output.AddNode(node);
                m_terms.emplace(key, node.id);
            } else if (node.tag == BTOR2_TAG_input || node.tag == BTOR2_TAG_state) {
                node.tag = BTOR2_TAG_input;
                node.nargs = 0;
                node.args = {};
            }
        }
        stack.push_back(std::move(work));
    };
    push(id, time);
    while (!stack.empty()) {
        auto &work = stack.back();
        if (work.kind == Frame::Kind::Expression && work.operand < work.node.nargs) {
            // Only nargs operands are node references; the remaining slice and
            // extension arguments are immediates and retain their original values.
            const int64_t operand = work.node.args[work.operand];
            const unsigned frame = work.key.second;
            if (!m_terms.count(keyOf(operand, frame))) {
                push(operand, frame);
                continue;
            }
            work.node.args[work.operand++] = lookup(operand, frame);
            continue;
        }
        if (work.kind != Frame::Kind::Expression) {
            const int64_t dependency = work.dependency;
            const unsigned frame = work.dependencyTime;
            if (!m_terms.count(keyOf(dependency, frame))) {
                push(dependency, frame);
                continue;
            }
            const int64_t value = lookup(dependency, frame);
            if (work.kind == Frame::Kind::Alias) {
                m_terms.emplace(work.key, value);
            } else {
                Btor2IRNode initialization;
                initialization.id = m_output.FreshId();
                initialization.tag = BTOR2_TAG_init;
                initialization.sortId = work.node.sortId;
                initialization.nargs = 2;
                initialization.args = {work.node.id, value, 0};
                m_output.AddNode(initialization);
            }
        } else {
            m_output.AddNode(work.node);
            m_terms.emplace(work.key, work.node.id);
        }
        building.erase(work.key);
        stack.pop_back();
    }
    return lookup(id, time);
}

void WLBoundedUnroller::Constraint(int64_t condition) {
    Btor2IRNode node;
    node.id = m_output.FreshId();
    node.tag = BTOR2_TAG_constraint;
    node.nargs = 1;
    node.args[0] = condition;
    m_output.AddNode(node);
}

void WLBoundedUnroller::RequireEqual(int64_t lhs, int64_t rhs) {
    Btor2IRNode node;
    node.id = m_output.FreshId();
    node.tag = BTOR2_TAG_eq;
    node.sortId = m_boolSort;
    node.nargs = 2;
    node.args = {lhs, rhs, 0};
    m_output.AddNode(node);
    Constraint(node.id);
}

WLTrace WLBoundedUnroller::DecodeTrace(const WLTraceStep &flat) const {
    WLTrace result;
    result.steps.resize(static_cast<size_t>(m_bound) + 1);
    for (const auto &port : m_interface) {
        auto &step = result.steps.at(port.time);
        if (port.array) {
            auto &values = port.state ? step.arrayStateValues : step.arrayInputValues;
            values.emplace(port.original, flat.arrayInputValues.at(port.lowered));
        } else {
            auto &values = port.state ? step.stateValues : step.inputValues;
            values.emplace(port.original, flat.inputValues.at(port.lowered));
        }
    }
    return result;
}

} // namespace car
