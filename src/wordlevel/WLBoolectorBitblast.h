#ifndef WL_BOOLECTOR_BITBLAST_H
#define WL_BOOLECTOR_BITBLAST_H

#include "Btor2Frontend.h"
#include "WLTypes.h"

extern "C" {
#include "aiger.h"
}

#include <memory>

namespace car {

// Standard lowering of an optimized, array-free word-level IR to AIGER.
std::shared_ptr<aiger> GenerateWLAig(const Btor2IR &ir,
                                     const WLIRTraceMap &traceSources,
                                     WLTraceMap &traceMap);

} // namespace car

#endif
