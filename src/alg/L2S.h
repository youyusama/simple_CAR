#ifndef L2S_H
#define L2S_H

#include "BaseAlg.h"
#include "Log.h"
#include "Model.h"
#include "Settings.h"
#include <memory>
#include <unordered_set>
#include <vector>

namespace car {

class L2S : public BaseAlg {
  public:
    L2S(Settings settings,
        Model &model,
        Log &log);

    CheckResult Run() override;
    void Witness() override;
    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;

  private:
    void Translate();
    std::unique_ptr<BaseAlg> CreateSafetyChecker();

    Settings m_settings;
    Model &m_model;
    Log &m_log;
    std::unique_ptr<BaseAlg> m_checker;
    int m_save;
    std::vector<Var> m_latchCopy;
    std::vector<Var> m_origInputs;
    std::vector<Var> m_origLatches;
    std::unordered_set<Var> m_origInputSet;
    std::unordered_set<Var> m_origLatchSet;
};

} // namespace car

#endif
