#ifndef WL_TYPES_H
#define WL_TYPES_H

#include <cstdint>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace car {

// Shared data structures for the word-level abstraction pipeline.  The current
// implementation is BTOR2-shaped, so compatibility aliases are kept below.
struct WLMemoryPair {
    // Track memoryState[addressNode] through the requested transition delay.
    int64_t memoryStateId{0};
    int64_t addressNodeId{0};
    unsigned delay{0};
};

struct WLArrayRead {
    // Preserve the original read relation for simulator-directed refinement.
    int64_t readNodeId{0};
    int64_t memoryStateId{0};
    int64_t addressNodeId{0};
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

} // namespace car

#endif
