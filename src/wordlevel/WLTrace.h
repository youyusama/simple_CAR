#ifndef WL_TRACE_H
#define WL_TRACE_H

#include "WLBitVector.h"

#include <cstdint>
#include <cstddef>
#include <optional>
#include <unordered_map>
#include <vector>

namespace car {

// One explicit cell assignment used for internal concrete replay.
struct WLArrayEntry {
    WLBitVector index;
    WLBitVector value;
};

// With a default this is a total array. Without one, entries must cover the
// entire address domain to be total; otherwise this is a partial observation.
struct WLArrayValue {
    std::optional<WLBitVector> defaultValue;
    std::vector<WLArrayEntry> entries;
};

// Execution choices or concrete candidate values for one IR. Only input/state
// ports belong here; intermediate expression observations belong to simulation.
// Derived states may be omitted: WLSimulator recomputes init/next functions.
// A concrete counterexample candidate is not confirmed until verified.
struct WLTraceStep {
    std::unordered_map<int64_t, WLBitVector> inputValues;
    std::unordered_map<int64_t, WLBitVector> stateValues;
    std::unordered_map<int64_t, WLArrayValue> arrayStateValues;
    std::unordered_map<int64_t, WLArrayValue> arrayInputValues;
};

struct WLTrace {
    std::vector<WLTraceStep> steps;
};

} // namespace car

#endif
