#pragma once

#include "Btor2IR.h"
#include "WLTrace.h"
#include <functional>
#include <initializer_list>
#include <map>
#include <tuple>
#include <vector>

namespace car {

// Standalone IR-to-IR pass on a finite, already unrolled formula.
// IR() contains no array eq/neq; it is suitable for an ordinary EMM encoder.
// No Boolector objects, SAT literals, or EMM-private observations cross this API.
class ArrayEqualityEncoder {
  public:
    explicit ArrayEqualityEncoder(const Btor2IR &bounded);
    static bool HasArrayComparisons(const Btor2IR &ir);
    const Btor2IR &IR() const { return m_ir; }

    struct Statistics {
        // queries counts constructed component/address closures;
        // implications counts emitted, unique, nontrivial point constraints.
        size_t nodes{0}, points{0}, comparisons{0}, queries{0}, implications{0};
    };
    const Statistics &Stats() const { return m_stats; }
    // Scalar IR IDs whose values are needed to lift a model of IR().
    std::vector<int64_t> ModelTerms() const;
    using Value = std::function<BitVector(int64_t)>;
    // Complete semantic observations, not private EMM root-read assignments.
    // The result is indexed by array node ID in the bounded input IR.
    std::map<int64_t, ArrayValue> Complete(const Value &value) const;

  private:
    using Ref = size_t;
    struct Node {
        enum class Kind { FreeRoot, Uniform, Store, Choice };
        Kind kind;
        int64_t sortId; // Canonical array sort in m_ir.
        int64_t source;
        Ref left{0}, right{0};
        int64_t condition{0}, address{0}, data{0};
    };
    struct Point { Ref array; int64_t address, data; };
    struct Comparison { Ref left, right; int64_t equal; };

    Ref Array(int64_t id);
    void Encode();
    void PruneGeneratedNodes(const Btor2IR &source);
    int64_t Add(Btor2Tag tag, int64_t sort, std::initializer_list<int64_t> args = {});
    int64_t BitVectorSort(uint32_t width);
    int64_t Input(int64_t sort);
    int64_t Constant(uint32_t width, const std::string &bits);
    int64_t Read(int64_t array, int64_t address);
    int64_t Not(int64_t x) const;
    int64_t And(int64_t x, int64_t y);
    int64_t Or(int64_t x, int64_t y);
    int64_t Eq(int64_t x, int64_t y);
    int64_t Ne(int64_t x, int64_t y) { return Not(Eq(x, y)); }
    int64_t Gate(Btor2Tag tag, int64_t x, int64_t y);
    void Require(int64_t condition);

    Btor2IR m_ir;
    int64_t m_boolSort{0}, m_true{0}, m_false{0};
    std::map<int64_t, int64_t> m_init;
    std::map<int64_t, Ref> m_arrayIds;
    std::map<std::pair<int64_t, int64_t>, int64_t> m_reads;
    std::map<std::tuple<Btor2Tag, int64_t, int64_t>, int64_t> m_gates;
    std::vector<Node> m_nodes;
    std::vector<Point> m_points;
    std::vector<Comparison> m_comparisons;
    Statistics m_stats;
};

} // namespace car
