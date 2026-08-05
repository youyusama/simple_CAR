#include "Btor2Frontend.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <unordered_set>

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

} // namespace

Btor2IR Btor2Frontend::LoadIR(const std::string &path) {
    // Every frontend consumer receives a parsed and supported-subset-validated IR.
    Btor2IR ir = Btor2IR::Parse(path);
    Validate(ir);
    return ir;
}

Btor2IR Btor2IR::Parse(const std::string &path) {
    // Delegate syntax parsing to btor2tools, then copy data into owned C++ objects.
    FILE *file = fopen(path.c_str(), "r");
    if (!file) throw std::runtime_error("cannot open BTOR2 input: " + path);

    Btor2Parser *parser = btor2parser_new();
    if (!btor2parser_read_lines(parser, file)) {
        std::string error = btor2parser_error(parser);
        btor2parser_delete(parser);
        fclose(file);
        throw std::runtime_error(error);
    }
    fclose(file);

    Btor2IR result;
    Btor2LineIterator iterator = btor2parser_iter_init(parser);
    Btor2Line *line;
    while ((line = btor2parser_iter_next(&iterator))) {
        // Sort declarations live in a separate table and are not value nodes.
        if (line->tag == BTOR2_TAG_sort) {
            Btor2IRSort sort;
            sort.id = line->id;
            sort.tag = line->sort.tag;
            if (sort.tag == BTOR2_TAG_SORT_bitvec) {
                sort.width = line->sort.bitvec.width;
            } else {
                sort.indexSort = line->sort.array.index;
                sort.elementSort = line->sort.array.element;
            }
            result.AddSort(sort);
            continue;
        }

        // Preserve signed argument IDs because BTOR2 uses negative IDs for inversion.
        Btor2IRNode node;
        node.id = line->id;
        node.line = line->lineno;
        node.tag = line->tag;
        node.sortId = line->sort.id;
        node.nargs = line->nargs;
        for (size_t i = 0; i < node.args.size(); ++i) {
            node.args[i] = line->args ? line->args[i] : 0;
        }
        if (line->constant) node.constant = line->constant;
        if (line->symbol) node.symbol = line->symbol;
        result.AddNode(node);
    }

    btor2parser_delete(parser);
    return result;
}

const Btor2IRSort &Btor2IR::Sort(int64_t id) const {
    auto it = m_sorts.find(id);
    if (it == m_sorts.end()) {
        throw std::runtime_error("BTOR2 references unknown sort id " +
                                 std::to_string(id));
    }
    return it->second;
}

const Btor2IRNode &Btor2IR::Node(int64_t id) const {
    auto it = m_nodeIndex.find(std::abs(id));
    if (it == m_nodeIndex.end()) {
        throw std::runtime_error("BTOR2 references unknown node id " +
                                 std::to_string(id));
    }
    return m_nodes[it->second];
}

Btor2IRNode &Btor2IR::MutableNode(int64_t id) {
    auto it = m_nodeIndex.find(std::abs(id));
    if (it == m_nodeIndex.end()) {
        throw std::runtime_error("BTOR2 references unknown node id " +
                                 std::to_string(id));
    }
    return m_nodes[it->second];
}

void Btor2IR::AddSort(const Btor2IRSort &sort) {
    // BTOR2 sort and node declarations share one global positive ID space.
    ObserveId(sort.id);
    if (m_nodeIndex.count(sort.id) ||
        !m_sorts.emplace(sort.id, sort).second) {
        throw std::runtime_error("duplicate word-level id " +
                                 std::to_string(sort.id));
    }
    if (sort.tag == BTOR2_TAG_SORT_array) m_hasArrays = true;
}

void Btor2IR::AddNode(const Btor2IRNode &node) {
    // Keep the vector order for traversal and an ID index for constant-time lookup.
    ObserveId(node.id);
    if (m_sorts.count(node.id) ||
        !m_nodeIndex.emplace(node.id, m_nodes.size()).second) {
        throw std::runtime_error("duplicate word-level id " +
                                 std::to_string(node.id));
    }
    m_nodes.push_back(node);
}

void Btor2IR::ObserveId(int64_t id) {
    if (id <= 0)
        throw std::runtime_error("word-level IDs must be positive");
    m_nextFreshId = std::max(
        m_nextFreshId, static_cast<uint64_t>(id) + UINT64_C(1));
}

int64_t Btor2IR::FreshId() {
    if (m_nextFreshId >
        static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
        throw std::runtime_error("word-level ID space exhausted");
    }
    return static_cast<int64_t>(m_nextFreshId++);
}

void Btor2IR::ReserveFreshIdsAfter(const Btor2IR &source) {
    m_nextFreshId = std::max(m_nextFreshId, source.m_nextFreshId);
}

void Btor2Frontend::Validate(const Btor2IR &ir) {
    // Safety and fairness metadata always consumes a one-bit condition.
    for (const Btor2IRNode &node : ir.Nodes()) {
        if (node.tag != BTOR2_TAG_bad &&
            node.tag != BTOR2_TAG_constraint &&
            node.tag != BTOR2_TAG_fair) {
            continue;
        }
        const Btor2IRNode &condition = ir.Node(node.args[0]);
        if (!condition.sortId ||
            ir.Sort(condition.sortId).tag != BTOR2_TAG_SORT_bitvec ||
            ir.Sort(condition.sortId).width != 1) {
            throw Unsupported(node,
                              "property condition must be a one-bit bitvector");
        }
    }

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

    // Direct array next-state inputs are the only accepted nondeterministic arrays.
    std::unordered_set<int64_t> allowedArrayInputs;

    // Restrict array-valued expressions to the remodellable state/write/ite form.
    for (const Btor2IRNode &node : ir.Nodes()) {
        if (node.tag == BTOR2_TAG_next) {
            const Btor2IRNode &state = ir.Node(node.args[0]);
            const Btor2IRNode &value = ir.Node(node.args[1]);
            if (IsArray(ir, state.sortId) && value.tag == BTOR2_TAG_input) {
                allowedArrayInputs.insert(value.id);
            }
        }
    }

    // Array equality requires reasoning not provided by the selected-slot model.
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
            if (!allowedArrayInputs.count(node.id)) {
                throw Unsupported(
                    node,
                    "general array inputs are unsupported; only an array "
                    "state's direct nondeterministic next value is allowed");
            }
            break;
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

            if (argument.tag == BTOR2_TAG_input &&
                !(node.tag == BTOR2_TAG_next && i == 1)) {
                throw Unsupported(
                    node,
                    "array input is used outside a direct state next value");
            }
        }
    }
}

} // namespace car
