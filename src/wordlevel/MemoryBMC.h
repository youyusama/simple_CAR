#pragma once

#include "WLTrace.h"

#include <optional>
#include <string>

namespace car {

class Log;
class WLModel;

// Exact bounded checker: shared bounded unrolling, optional array equality
// elimination, then EMM on the resulting single-frame equality-free IR.
class MemoryBMC {
  public:
    enum class Status { Counterexample, PrefixSafe, Unknown };

    struct Result {
        Status status{Status::Unknown};
        // Largest depth of the contiguous UNSAT prefix, not just a visited depth.
        std::optional<unsigned> checkedThrough;
        std::optional<unsigned> badDepth;
        std::string reason;
    };

    MemoryBMC(WLModel &model, Log &log);

    // Inclusive: check F_0, ..., F_bound. PrefixSafe is not unbounded Safe.
    Result CheckThrough(unsigned bound);
    const WLTrace &GetTrace() const { return m_trace; }

  private:
    WLModel &m_model;
    Log &m_log;
    WLTrace m_trace;
};

} // namespace car
