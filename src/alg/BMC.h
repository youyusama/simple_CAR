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

  private:
    bool Check(int badId);
    bool Check_nonincremental(int badId);
    Settings m_settings;
    Log &m_log;
    Model &m_model;
    int m_k;
    int m_maxK;
    int m_step;
    shared_ptr<State> m_initialState;
    int m_badId;
    shared_ptr<SATSolver> m_Solver;

    // for kissat to store clauses from previous unrolling
    vector<clause> m_clauses;

    CheckResult m_checkResult;
    void Init(int badId);
    void OutputCounterExample(int bad);
    void GetClausesK(int m_k, vector<clause> &clauses);
    int GetBadK(int m_k);
    vector<int> GetConstraintsK(int m_k);
};

} // namespace car

#endif
