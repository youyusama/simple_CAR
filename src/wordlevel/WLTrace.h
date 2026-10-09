#ifndef WL_TRACE_H
#define WL_TRACE_H

#include "BitVector.h"

#include <cstdint>
#include <cstddef>
#include <optional>
#include <unordered_map>
#include <vector>

namespace car {

// One explicit cell assignment used for internal concrete replay.
struct ArrayEntry {
    BitVector index;
    BitVector value;
};

// With a default this is a total array. Without one, entries must cover the
// entire address domain to be total; otherwise this is a partial observation.
struct ArrayValue {
    std::optional<BitVector> defaultValue;
    std::vector<ArrayEntry> entries;
};

// Execution choices or concrete candidate values for one IR. Only input/state
// ports belong here; intermediate expression observations belong to simulation.
// Derived states may be omitted: WLSimulator recomputes init/next functions.
// A concrete counterexample candidate is not confirmed until verified.
struct WLTraceStep {
    std::unordered_map<int64_t, BitVector> inputValues;
    std::unordered_map<int64_t, BitVector> stateValues;
    std::unordered_map<int64_t, ArrayValue> arrayStateValues;
    std::unordered_map<int64_t, ArrayValue> arrayInputValues;
};

struct WLTrace {
    std::vector<WLTraceStep> steps;
};

} // namespace car

#endif
