#ifndef WL_TYPES_H
#define WL_TYPES_H

#include "WLBitVector.h"

#include <cstdint>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace car {

// Shared data structures for the word-level abstraction pipeline.  The current
// implementation is BTOR2-shaped because BTOR2 is the only word-level frontend.
struct WLMemoryPair {
    // Track memoryState[addressNode] through the requested transition delay.
    int64_t memoryStateId{0};
    int64_t addressNodeId{0};
    unsigned delay{0};
};

enum class WLTraceKind {
    OriginalInput,
    OriginalState,
    AbstractReadInput,
    SelectorState,
    ContentState
};

// Original word-level value and segment represented by a transformed variable.
struct WLValueOrigin {
    WLTraceKind kind;
    int64_t nodeId{0};
    size_t pairIndex{0};
    uint32_t originalBitOffset{0};
    uint32_t originalSegmentWidth{0};
};

using WLIRTraceMap = std::unordered_map<int64_t, WLValueOrigin>;

// One contiguous, least-significant-bit-first AIGER interface range belonging
// to a word-level segment.  Package metadata is stored once per segment.
struct WLTraceSpan {
    WLValueOrigin origin;
    uint32_t firstAigVar{0};
    uint32_t encodedWidth{0};
};

// Mapping from final AIGER interface ranges back to word-level segments.
struct WLTraceMap {
    std::vector<WLTraceSpan> inputSpans;
    std::vector<WLTraceSpan> latchSpans;
    std::vector<WLMemoryPair> memoryPairs;
};

// One decoded checker time step expressed in original word-level values.
// State values contain the initial choices and per-step choices for states
// without next; ordinary successor states are recomputed by WLSimulator.
struct WLReplayStep {
    std::unordered_map<int64_t, WLBitVector> inputValues;
    std::unordered_map<int64_t, WLBitVector> stateValues;
    std::unordered_map<int64_t, WLBitVector> abstractReadValues;
    std::unordered_map<size_t, WLBitVector> selectorValues;
    std::unordered_map<size_t, WLBitVector> contentValues;
};

// Word-level replay seed decoded from a bit-level checker trace.  It may still
// be spurious until WLSimulator validates it against the concrete model.
struct WLReplayTrace {
    std::vector<WLReplayStep> steps;
    std::vector<WLMemoryPair> memoryPairs;
};

// One concrete array assignment retained for BTOR2 witness serialization.
struct WLWitnessArrayEntry {
    WLBitVector index;
    WLBitVector value;
};

// Sparse concrete array value; omitted indices keep their initialized/default value.
struct WLWitnessArrayValue {
    std::vector<WLWitnessArrayEntry> entries;
};

// Concrete values already established by checker/simulator processing.
// Derived successor states may be omitted because BTOR2 witness checkers
// recompute them from the model's next-state functions.
struct WLWitnessStep {
    std::unordered_map<int64_t, WLBitVector> inputValues;
    std::unordered_map<int64_t, WLBitVector> stateValues;
    std::unordered_map<int64_t, WLWitnessArrayValue> arrayStateValues;
};

struct WLWitnessTrace {
    std::vector<WLWitnessStep> steps;
};

} // namespace car

#endif
