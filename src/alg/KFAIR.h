#ifndef KFAIR_H
#define KFAIR_H

#include "IncrAlg.h"
#include "Log.h"
#include "Model.h"
#include "Settings.h"
#include <memory>
#include <vector>

namespace car {

class KFAIR : public BaseAlg {
  public:
    KFAIR(Settings settings,
          Model &model,
          Log &log);
    CheckResult Run() override;
    void Witness() override;
    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;

  private:
    std::unique_ptr<IncrAlg> MakeSafeChecker();
    bool DetectKLiveCex(IncrAlg &checker);

    Settings m_settings;
    Model &m_model;
    Log &m_log;

    std::vector<FrameList> m_globalWalls;
};

} // namespace car

#endif
