#ifndef RLIVE_H
#define RLIVE_H

#include "IncrAlg.h"
#include "Log.h"
#include "Model.h"
#include "SATSolver.h"
#include "Settings.h"
#include <memory>
#include <vector>

namespace car {

class RLive : public BaseAlg {
  public:
    RLive(Settings settings,
          Model &model,
          Log &log);
    CheckResult Run() override;
    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;

  private:
    std::unique_ptr<IncrAlg> MakeSafeChecker();
    bool CheckReachable(const Cube &s);
    bool PruneDead(const Cube &s);

    Settings m_settings;
    Model &m_model;
    Log &m_log;

    std::vector<FrameList> m_globalShoals;
    std::vector<Cube> m_globalDead;
    std::vector<Cube> m_badStack;

    std::unique_ptr<IncrAlg> m_safeChecker;
    std::shared_ptr<SATSolver> m_pdSolver;

    Cube GetUnsatAssumption(shared_ptr<SATSolver> solver, const Cube &assumptions);
};

} // namespace car

#endif
