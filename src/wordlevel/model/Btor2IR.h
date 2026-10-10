#pragma once

#include <btor2parser/btor2parser.h>

#include <array>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace car {

// A language feature outside the supported IR subset, distinct from a broken
// internal IR contract or a resource failure during an algorithm's execution.
class Btor2Unsupported : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

// Interned BTOR2 sort descriptor. Component references are canonical sort IDs.
struct Btor2IRSort {
    int64_t id{0};
    Btor2SortTag tag{BTOR2_TAG_SORT_bitvec};
    uint32_t width{0};
    int64_t indexSort{0};
    int64_t elementSort{0};
};

// BTOR2 instruction with stable IDs and source-line information for diagnostics.
struct Btor2IRNode {
    int64_t id{0};
    int64_t line{0};
    Btor2Tag tag{BTOR2_TAG_zero};
    int64_t sortId{0};
    uint32_t nargs{0};
    std::array<int64_t, 3> args{};
    std::string constant;
    std::string symbol;
};

class Btor2IR {
  public:
    Btor2IR() = default;

    // Accepts declared aliases; the returned descriptor always has a canonical ID.
    const Btor2IRSort &Sort(int64_t id) const;
    const Btor2IRNode &Node(int64_t id) const;
    bool HasArrays() const { return m_hasArrays; }

    // Check the flat BV-array subset supported by the current execution passes.
    // This explicit capability check also accepts constructed/transformed IRs;
    // parsing and IR construction themselves can represent unsupported arrays.
    void ValidateSupportedArrays() const;

    const std::vector<Btor2IRNode> &Nodes() const { return m_nodes; }
    // Only canonical declarations are enumerated; aliases are resolved by Sort().
    const std::unordered_map<int64_t, Btor2IRSort> &Sorts() const {
        return m_sorts;
    }

    // Mutation APIs are used by word-level IR-to-IR optimization passes.
    Btor2IRNode &MutableNode(int64_t id);
    std::vector<Btor2IRNode> &MutableNodes() { return m_nodes; }
    // Fresh IDs share the global BTOR2 sort/node namespace.
    int64_t FreshId();
    // Derived IRs call this before copying source declarations incrementally.
    void ReserveFreshIdsAfter(const Btor2IR &source);
    // Intern types structurally. Array component sorts must already exist.
    // Duplicate declarations with different IDs become aliases, not new types.
    int64_t AddSort(const Btor2IRSort &sort);
    // Copy all types/aliases into an empty IR, preserving canonical IDs.
    void CopySortsFrom(const Btor2IR &source);
    // Stores a canonical sortId while preserving the value node's ID and line.
    void AddNode(const Btor2IRNode &node);
    void SetHasArrays(bool value) { m_hasArrays = value; }

  private:
    void ObserveId(int64_t id);

    bool m_hasArrays{false};
    uint64_t m_nextFreshId{1};
    std::vector<Btor2IRNode> m_nodes;
    std::unordered_map<int64_t, Btor2IRSort> m_sorts;
    std::unordered_map<int64_t, int64_t> m_sortAliases;
    std::map<std::tuple<Btor2SortTag, uint32_t, int64_t, int64_t>, int64_t> m_sortIds;
    std::unordered_map<int64_t, size_t> m_nodeIndex;
};

} // namespace car
