#include "Btor2IR.h"

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <utility>

namespace car {
namespace {

bool IsArray(const Btor2IR &ir, int64_t sort) {
    return sort && ir.Sort(sort).tag == BTOR2_TAG_SORT_array;
}

Btor2Unsupported UnsupportedArrayUsage(const Btor2IRNode &node,
                               const std::string &reason) {
    return Btor2Unsupported("BTOR2 line " + std::to_string(node.line) +
                              " (id " + std::to_string(node.id) + "): " + reason);
}

void ValidateFlatArraySort(const Btor2IR &ir, int64_t sortId) {
    const auto &sort = ir.Sort(sortId);
    if (sort.tag != BTOR2_TAG_SORT_array ||
        ir.Sort(sort.indexSort).tag != BTOR2_TAG_SORT_bitvec ||
        ir.Sort(sort.elementSort).tag != BTOR2_TAG_SORT_bitvec)
        throw Btor2Unsupported("nested or non-flat BTOR2 arrays are unsupported");
}

} // namespace

const Btor2IRSort &Btor2IR::Sort(int64_t id) const {
    auto it = m_sortAliases.find(id);
    if (it == m_sortAliases.end()) {
        throw std::runtime_error("BTOR2 references unknown sort id " +
                                 std::to_string(id));
    }
    return m_sorts.at(it->second);
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

int64_t Btor2IR::AddSort(const Btor2IRSort &declaration) {
    // BTOR2 sort and node declarations share one global positive ID space.
    ObserveId(declaration.id);
    if (m_nodeIndex.count(declaration.id) || m_sortAliases.count(declaration.id)) {
        throw std::runtime_error("duplicate word-level id " +
                                 std::to_string(declaration.id));
    }
    Btor2IRSort sort = declaration;
    if (sort.tag == BTOR2_TAG_SORT_array) {
        sort.width = 0;
        sort.indexSort = Sort(sort.indexSort).id;
        sort.elementSort = Sort(sort.elementSort).id;
    } else if (sort.tag == BTOR2_TAG_SORT_bitvec && sort.width > 0) {
        sort.indexSort = sort.elementSort = 0;
    } else {
        throw std::runtime_error("invalid word-level sort " + std::to_string(sort.id));
    }
    const auto key = std::make_tuple(sort.tag, sort.width, sort.indexSort, sort.elementSort);
    auto [canonical, inserted] = m_sortIds.emplace(key, sort.id);
    m_sortAliases.emplace(sort.id, canonical->second);
    if (inserted) m_sorts.emplace(sort.id, sort);
    if (sort.tag == BTOR2_TAG_SORT_array) m_hasArrays = true;
    return canonical->second;
}

void Btor2IR::CopySortsFrom(const Btor2IR &source) {
    if (!m_nodes.empty() || !m_sortAliases.empty())
        throw std::runtime_error("type-table copy requires an empty IR");
    m_sorts = source.m_sorts;
    m_sortAliases = source.m_sortAliases;
    m_sortIds = source.m_sortIds;
    m_hasArrays = source.m_hasArrays;
    ReserveFreshIdsAfter(source);
}

void Btor2IR::AddNode(const Btor2IRNode &node) {
    // Keep the vector order for traversal and an ID index for constant-time lookup.
    ObserveId(node.id);
    Btor2IRNode normalized = node;
    if (normalized.sortId) normalized.sortId = Sort(normalized.sortId).id;
    if (m_sortAliases.count(node.id) ||
        !m_nodeIndex.emplace(node.id, m_nodes.size()).second) {
        throw std::runtime_error("duplicate word-level id " +
                                 std::to_string(node.id));
    }
    m_nodes.push_back(std::move(normalized));
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

void Btor2IR::ValidateSupportedArrays() const {
    for (const auto &[id, sort] : Sorts()) {
        if (sort.tag == BTOR2_TAG_SORT_array) ValidateFlatArraySort(*this, id);
    }
    for (const auto &node : Nodes()) {
        if (IsArray(*this, node.sortId)) {
            switch (node.tag) {
            case BTOR2_TAG_state:
            case BTOR2_TAG_input:
            case BTOR2_TAG_write:
            case BTOR2_TAG_ite:
            case BTOR2_TAG_init:
            case BTOR2_TAG_next: break;
            default: throw UnsupportedArrayUsage(node, "unsupported array-valued operator");
            }
            // Some parser versions do not check an array-valued init RHS sort.
            if (node.tag == BTOR2_TAG_init &&
                IsArray(*this, Node(node.args[1]).sortId) &&
                node.sortId != Node(node.args[1]).sortId)
                throw UnsupportedArrayUsage(node, "array initialization sort mismatch");
        }
        for (uint32_t i = 0; i < node.nargs; ++i) {
            if (!IsArray(*this, Node(node.args[i]).sortId)) continue;
            const bool allowed =
                (node.tag == BTOR2_TAG_read && i == 0) ||
                (node.tag == BTOR2_TAG_write && i == 0) ||
                (node.tag == BTOR2_TAG_ite && (i == 1 || i == 2)) ||
                node.tag == BTOR2_TAG_init || node.tag == BTOR2_TAG_next ||
                node.tag == BTOR2_TAG_eq || node.tag == BTOR2_TAG_neq;
            if (!allowed) throw UnsupportedArrayUsage(node, "unsupported array operand");
        }
    }
}

} // namespace car
