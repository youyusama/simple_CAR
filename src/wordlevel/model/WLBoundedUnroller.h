#pragma once

#include "Btor2IR.h"
#include "WLTrace.h"
#include <map>

namespace car {

// F_k as a single-frame IR: scalar transitions become constraints, array copies
// become aliases, and every free choice is time-instantiated. Uniform arrays
// are represented by synthetic array states with scalar init and no next.
class WLBoundedUnroller {
  public:
    WLBoundedUnroller(const Btor2IR &source, unsigned bound);
    const Btor2IR &IR() const { return m_output; }
    // Free ports of the bounded IR, with arrays already completed by the encoder.
    WLTrace DecodeTrace(const WLTraceStep &flat) const;

  private:
    struct Interface {
        int64_t original, lowered;
        unsigned time;
        bool array, state;
    };
    int64_t Term(int64_t id, unsigned time);
    void RequireEqual(int64_t lhs, int64_t rhs);
    void Constraint(int64_t condition);

    const Btor2IR &m_source;
    unsigned m_bound;
    Btor2IR m_output;
    int64_t m_boolSort{0};
    std::map<int64_t, int64_t> m_init, m_next;
    std::map<std::pair<int64_t, unsigned>, int64_t> m_terms;
    std::vector<Interface> m_interface;
};

} // namespace car
