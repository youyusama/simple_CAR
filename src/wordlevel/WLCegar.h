#ifndef WL_CEGAR_H
#define WL_CEGAR_H

#include "BaseAlg.h"
#include "Settings.h"
#include "WLTypes.h"

#include <memory>
#include <string>
#include <vector>

namespace car {

class Btor2IR;
class Log;
class Model;
class WLModel;

// CEGAR engine for selected-slot word-level memory abstraction.  It drives an
// ordinary bit-level checker, simulator replay, refinement, and safe-depth
// checks.  WLChecker owns this engine when the input contains word-level arrays.
class WLCegar {
  public:
    WLCegar(const Settings &settings,
            Log &log,
            WLModel &model);
    ~WLCegar();

    // Run checker/replay/refinement iterations until a definitive result.
    CheckResult Run();
    std::vector<std::pair<Cube, Cube>> GetCexTrace();
    int GetSafeDepth() const;
    bool HasConcreteCounterexample() const {
        return m_concreteCounterexample;
    }
    bool WriteCounterexample(const std::string &path);

  private:
    void AddPair(const WLMemoryPair &pair);
    unsigned MaxDelay() const;
    bool ReloadModel();
    std::unique_ptr<BaseAlg> CreateBitLevelChecker(Model &model, Log &log);

    const Settings &m_settings;
    Log &m_log;
    WLModel &m_model;
    std::unique_ptr<BaseAlg> m_checker;
    std::vector<WLMemoryPair> m_memoryPairs;
    std::unique_ptr<Btor2IR> m_ir;
    bool m_concreteCounterexample{false};
};

} // namespace car

#endif
