#pragma once

#include "Btor2IR.h"
#include "WLTrace.h"
#include <map>

namespace car {

// A bounded bad-depth query as a single-frame IR. Scalar transitions remain
// hard constraints; array copies become aliases. Each candidate bad requires
// source constraints only through its own frame, allowing nonextendible traces.
// Uniform arrays use synthetic array states with scalar init and no next.
class WLBoundedUnroller {
  public:
    // A single endpoint, or any endpoint in the inclusive interval [first, last].
    WLBoundedUnroller(const Btor2IR &source, unsigned bound);
    WLBoundedUnroller(const Btor2IR &source, unsigned first, unsigned last);
    const Btor2IR &IR() const { return m_output; }
    // Depth -> (constraints through depth AND bad at depth), in this IR.
    const std::map<unsigned, int64_t> &BadTerms() const { return m_badTerms; }
    // Free ports of the bounded IR, with arrays already completed by the encoder.
    WLTrace DecodeTrace(const WLTraceStep &flat) const;
    WLTrace DecodeTrace(const WLTraceStep &flat, unsigned last) const;

  private:
    struct Interface {
        int64_t original, lowered;
        unsigned time;
        bool array, state;
    };
    int64_t Term(int64_t id, unsigned time);
    int64_t Boolean(Btor2Tag tag, int64_t lhs, int64_t rhs);
    void RequireEqual(int64_t lhs, int64_t rhs);
    void Constraint(int64_t condition);

    const Btor2IR &m_source;
    unsigned m_bound;
    Btor2IR m_output;
    int64_t m_boolSort{0};
    std::map<int64_t, int64_t> m_init, m_next;
    std::map<std::pair<int64_t, unsigned>, int64_t> m_terms;
    std::vector<Interface> m_interface;
    std::map<unsigned, int64_t> m_badTerms;
};

} // namespace car
