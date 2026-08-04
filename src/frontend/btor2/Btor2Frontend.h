#ifndef BTOR2_FRONTEND_H
#define BTOR2_FRONTEND_H

#include <btor2parser/btor2parser.h>

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace car {

// BTOR2 sort descriptor copied from btor2tools into the frontend-owned IR.
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

class Btor2Frontend;

class Btor2IR {
  public:
    Btor2IR() = default;

    const Btor2IRSort &Sort(int64_t id) const;
    const Btor2IRNode &Node(int64_t id) const;
    bool HasArrays() const { return m_hasArrays; }

    const std::vector<Btor2IRNode> &Nodes() const { return m_nodes; }
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
    void AddSort(const Btor2IRSort &sort);
    void AddNode(const Btor2IRNode &node);
    void SetHasArrays(bool value) { m_hasArrays = value; }

  private:
    friend class Btor2Frontend;

    static Btor2IR Parse(const std::string &path);
    void ObserveId(int64_t id);

    bool m_hasArrays{false};
    uint64_t m_nextFreshId{1};
    std::vector<Btor2IRNode> m_nodes;
    std::unordered_map<int64_t, Btor2IRSort> m_sorts;
    std::unordered_map<int64_t, size_t> m_nodeIndex;
};

// Complete BTOR2 frontend entry point: parse the input into Btor2IR and
// validate that it belongs to the word-level subset supported by simpleCAR.
class Btor2Frontend {
  public:
    static Btor2IR LoadIR(const std::string &path);

  private:
    static void Validate(const Btor2IR &ir);
};

} // namespace car

#endif
