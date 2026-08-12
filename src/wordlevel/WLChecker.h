#ifndef WL_CHECKER_H
#define WL_CHECKER_H

#include "BaseAlg.h"
#include "WLTypes.h"

#include <memory>
#include <string>

namespace car {

class Log;
class Model;
class WLCegar;
class WLMemoryBMC;
class WLModel;

// Word-level checker wrapper for BTOR2 inputs.  It keeps BTOR2-specific witness
// handling and array CEGAR out of SimpleCAR while still delegating the bit-level
// proof work to the selected ordinary checker.
class WLChecker : public BaseAlg {
  public:
    WLChecker(const Settings &settings,
              WLModel &model,
              Log &log);
    ~WLChecker() override;

    CheckResult Run() override;
    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;
    const WLWitnessTrace &GetWitnessTrace();

  private:
    std::unique_ptr<BaseAlg> CreateBitLevelChecker(Model &model, Log &log);

    const Settings &m_settings;
    Log &m_log;
    WLModel &m_model;
    std::unique_ptr<BaseAlg> m_checker;
    std::unique_ptr<WLCegar> m_cegar;
    std::unique_ptr<WLMemoryBMC> m_memoryBmc;
    WLWitnessTrace m_witnessTrace;
};

} // namespace car

#endif
