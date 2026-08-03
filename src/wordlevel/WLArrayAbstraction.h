#ifndef WL_ARRAY_ABSTRACTION_H
#define WL_ARRAY_ABSTRACTION_H

#include "Btor2Frontend.h"
#include "WLTypes.h"

#include <vector>

namespace car {

struct WLArrayAbstractionResult {
    Btor2IR ir;
    std::vector<WLMemoryPair> tracePairs;
    std::vector<WLArrayRead> reads;
    WLIRTraceMap traceSources;
};

// IR-to-IR selected-slot abstraction.  The result contains only bit-vector
// nodes and can be consumed by later word-level optimization passes.
class WLArrayAbstraction {
  public:
    static WLArrayAbstractionResult
    Run(const Btor2IR &ir, const std::vector<WLMemoryPair> &pairs);
};

} // namespace car

#endif
