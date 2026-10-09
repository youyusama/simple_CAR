#ifndef WL_MEMORY_BMC_H
#define WL_MEMORY_BMC_H

#include "WLTrace.h"

#include <optional>
#include <string>

namespace car {

class Log;
class WLModel;

enum class WLBoundedStatus { Counterexample, PrefixSafe, Unknown };

struct WLBoundedResult {
    WLBoundedStatus status{WLBoundedStatus::Unknown};
    // Largest depth of the contiguous UNSAT prefix, not just a visited depth.
    std::optional<unsigned> checkedThrough;
    std::optional<unsigned> badDepth;
    std::string reason;
};

// Exact bounded checker: shared bounded unrolling, optional array equality
// elimination, then EMM on the resulting single-frame equality-free IR.
class WLMemoryBMC {
  public:
    WLMemoryBMC(WLModel &model, Log &log);

    // Inclusive: check F_0, ..., F_bound. PrefixSafe is not unbounded Safe.
    WLBoundedResult CheckThrough(unsigned bound);
    const WLTrace &GetTrace() const { return m_trace; }

  private:
    WLModel &m_model;
    Log &m_log;
    WLTrace m_trace;
};

} // namespace car

#endif
