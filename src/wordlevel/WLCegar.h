#ifndef WL_CEGAR_H
#define WL_CEGAR_H

#include "BaseAlg.h"
#include "Settings.h"
#include "WLTypes.h"

#include <memory>
#include <vector>

namespace car {

class Log;
class Model;
class WLModel;

// CEGAR engine for selected-slot word-level memory abstraction.  It drives an
// ordinary bit-level checker, simulator replay, and refinement.  WLChecker
// owns this engine when the input contains word-level arrays.
class WLCegar {
  public:
    WLCegar(const Settings &settings,
            Log &log,
            WLModel &model);
    ~WLCegar();

    // Run checker/replay/refinement iterations until a definitive result.
    CheckResult Run();
    std::vector<std::pair<Cube, Cube>> GetCexTrace();
    const WLWitnessTrace &GetWitnessTrace() const {
        return m_witnessTrace;
    }

  private:
    bool AddPair(const WLMemoryPair &pair);
    bool ReloadModel();
    std::unique_ptr<BaseAlg> CreateBitLevelChecker(Model &model, Log &log);

    const Settings &m_settings;
    Log &m_log;
    WLModel &m_model;
    std::unique_ptr<BaseAlg> m_checker;
    std::vector<WLMemoryPair> m_memoryPairs;
    std::vector<std::pair<Cube, Cube>> m_cexTrace;
    WLWitnessTrace m_witnessTrace;
    bool m_concreteCounterexample{false};
};

} // namespace car

#endif
