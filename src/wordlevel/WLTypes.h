#ifndef WL_TYPES_H
#define WL_TYPES_H

#include "WLBitVector.h"

#include <cstdint>
#include <cstddef>
#include <string>
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

enum class WLTraceBitKind {
    None,
    OriginalInput,
    OriginalState,
    AbstractReadInput,
    SelectorState,
    ContentState,
    GuardState
};

// Provenance of one final AIGER bit used to decode checker traces.
struct WLTraceBit {
    WLTraceBitKind kind{WLTraceBitKind::None};
    int64_t nodeId{0};
    // Bit position in the encoded segment emitted by the transformed model.
    uint32_t bit{0};
    size_t pairIndex{0};
    // Segment metadata used to inject compact package values into original words.
    uint32_t originalBitOffset{0};
    uint32_t originalSegmentWidth{0};
    uint32_t encodedSegmentWidth{0};
    bool resized{false};
};

// Provenance attached to a word-level IR variable before bitblasting.
struct WLIRTraceSource {
    WLTraceBitKind kind{WLTraceBitKind::None};
    int64_t nodeId{0};
    size_t pairIndex{0};
    uint32_t originalBitOffset{0};
    uint32_t originalSegmentWidth{0};
    bool resized{false};
};

using WLIRTraceMap = std::unordered_map<int64_t, WLIRTraceSource>;

// Mapping from final AIGER variables back to original word-level values.
struct WLTraceMap {
    std::unordered_map<uint32_t, WLTraceBit> inputBits;
    std::unordered_map<uint32_t, WLTraceBit> latchBits;
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

// Sparse concrete array value used by the BTOR2 witness serializer.  Entries
// not listed here retain the model's initialized or default value.
struct WLWitnessArrayValue {
    std::unordered_map<std::string, WLBitVector> entries;
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
