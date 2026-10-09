#pragma once

#include "BaseAlg.h"
#include "Settings.h"
#include "WLTrace.h"
#include "model/ArrayAbstraction.h"

#include <memory>
#include <vector>

namespace car {

class Log;
class Model;
class WLModel;

// CEGAR engine for selected-slot word-level memory abstraction.  It drives an
// ordinary bit-level checker and unified greedy array refinement. WLChecker
// owns this engine when the input contains word-level arrays.
class WLCEGAR {
  public:
    WLCEGAR(const Settings &settings,
            Log &log,
            WLModel &model);
    ~WLCEGAR();

    // Run checker/simulation/refinement; incomplete or no new target is Unknown.
    CheckResult Run();
    const WLTrace &GetTrace() const {
        return m_trace;
    }

  private:
    unsigned MaxDelay() const;
    bool ReloadModel(const std::vector<ArrayAbstraction::TrackingTarget> &targets);

    const Settings &m_settings;
    Log &m_log;
    WLModel &m_model;
    struct AbstractionContext;
    std::unique_ptr<AbstractionContext> BuildAbstractionContext(ArrayAbstraction::Precision precision);
    WLTrace RecoverChoices(
        const std::vector<std::pair<Cube, Cube>> &trace) const;
    ArrayAbstraction m_abstraction;
    std::unique_ptr<AbstractionContext> m_abstractionContext;
    WLTrace m_trace;
};

} // namespace car
