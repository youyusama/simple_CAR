#pragma once

#include "BaseAlg.h"
#include "WLTrace.h"
#include "model/Bitblastor.h"

#include <memory>

namespace car {

class Log;
class Model;
class WLCEGAR;
class MemoryBMC;
class WLModel;
class PackageResize;

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
    WordLayout m_layout;
    std::unique_ptr<PackageResize> m_resize;
    std::unique_ptr<Model> m_bitModel;
    std::unique_ptr<BaseAlg> m_checker;
    std::unique_ptr<WLCEGAR> m_cegar;
    std::unique_ptr<MemoryBMC> m_memoryBmc;
    WLTrace m_trace;
};

} // namespace car
