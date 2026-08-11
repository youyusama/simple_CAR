#ifndef WL_SIMULATOR_H
#define WL_SIMULATOR_H

#include "Btor2Frontend.h"
#include "WLTypes.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace car {

// Simulator-based replay for word-level abstract traces.  It confirms concrete
// counterexamples and reports read mismatches used by memory-abstraction CEGAR.
struct WLReadMismatch {
    // Concrete/abstract read disagreement used to select a refinement pair.
    int64_t readNodeId{0};
    int64_t memoryStateId{0};
    int64_t addressNodeId{0};
    unsigned time{0};
    unsigned delay{0};
};

class WLSimulator {
  public:
    enum class ReplayKind {
        ConcreteCounterexample,
        SpuriousCounterexample
    };

    struct Result {
        ReplayKind kind{ReplayKind::SpuriousCounterexample};
        unsigned badTime{0};
        std::vector<WLReadMismatch> refinementReads;
        WLWitnessTrace witnessTrace;
    };

    explicit WLSimulator(const Btor2IR &ir);
    ~WLSimulator();

    // Replay a decoded word-level seed on the original transition system.
    Result Replay(const WLReplayTrace &trace);

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace car

#endif
