#ifndef BMC_H
#define BMC_H

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
    void Witness() override;
    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;

  private:
    bool Check();
    bool CheckNonIncremental();
    Settings m_settings;
    Log &m_log;
    Model &m_model;
    int m_k;
    int m_maxK;
    int m_step;
    shared_ptr<State> m_initialState;
    shared_ptr<SATSolver> m_solver;

    // for kissat to store clauses from previous unrolling
    vector<Clause> m_clauses;

    CheckResult m_checkResult;
    void Init();
    void OutputCounterExample();
    void GetClausesK(int mK, vector<Clause> &clauses);
    Lit GetBadK(int mK);
    Cube GetConstraintsK(int mK);
};

} // namespace car

#endif
