#ifndef WL_CHECKER_H
#define WL_CHECKER_H

#include "BaseAlg.h"
#include "WLTypes.h"
#include "model/WLBitblastor.h"

#include <memory>

namespace car {

class Log;
class Model;
class WLCegar;
class WLMemoryBMC;
class WLModel;
class WLPackageResize;

// Word-level checker wrapper for BTOR2 inputs. It keeps BTOR2-specific trace
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
    const WLTrace &GetTrace();

  private:
    void BuildScalarModel();

    const Settings &m_settings;
    Log &m_log;
    WLModel &m_model;
    // Scalar checking owns its encoding; it needs no array abstraction or round.
    std::shared_ptr<aiger> m_aig;
    WLWordLayout m_layout;
    std::unique_ptr<WLPackageResize> m_resize;
    std::unique_ptr<Model> m_bitModel;
    std::unique_ptr<BaseAlg> m_checker;
    std::unique_ptr<WLCegar> m_cegar;
    std::unique_ptr<WLMemoryBMC> m_memoryBmc;
    WLTrace m_trace;
};

} // namespace car

#endif
