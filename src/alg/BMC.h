#pragma once

#include "BaseAlg.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"

namespace car {

class BMC : public BaseAlg {
  public:
    BMC(Settings settings,
        Model &model,
        Log &log);

    CheckResult Run() override;
    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;

  private:
    bool Check();
    bool CheckNonIncremental();
    void CNFGen();
    std::string GetCNFPath(int k) const;
    void WriteDimacs(const std::vector<Clause> &clauses, const std::string &path) const;
    Settings m_settings;
    Log &m_log;
    Model &m_model;
    int m_k;
    int m_maxK;
    int m_step;
    std::shared_ptr<State> m_initialState;
    std::shared_ptr<SATSolver> m_solver;

    // for kissat to store clauses from previous unrolling
    std::vector<Clause> m_clauses;

    CheckResult m_checkResult;
    void Init();
    void GetClausesK(int k, std::vector<Clause> &clauses);
    Lit GetBadK(int k);
    Cube GetConstraintsK(int k);
};

} // namespace car
