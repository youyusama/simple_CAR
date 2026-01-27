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

class rlive : public BaseAlg {
  public:
    rlive(Settings settings,
          Model &model,
          Log &log);
    CheckResult Run() override;
    void Witness() override;
    std::vector<std::pair<cube, cube>> GetCexTrace() override;

  private:
    std::unique_ptr<IncrAlg> MakeSafeChecker();
    bool CheckReachable(const cube &s);
    bool PruneDead(const cube &s);
    static bool Implies(const cube &a, const cube &b);

    Settings m_settings;
    Model &m_model;
    Log &m_log;

    std::vector<FrameList> m_globalShoals;
    std::vector<cube> m_globalDead;
    std::vector<cube> m_badStack;

    std::unique_ptr<IncrAlg> m_safeChecker;
    std::shared_ptr<SATSolver> m_pdSolver;

    cube GetUnsatAssumption(shared_ptr<SATSolver> solver, const cube &assumptions);
};

} // namespace car

#endif
