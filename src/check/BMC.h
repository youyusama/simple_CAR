#ifndef BMC_H
#define BMC_H

#include "BaseChecker.h"
#include "Log.h"
#include "SATSolver.h"

namespace car {

class BMC : public BaseChecker {
  public:
    BMC(Settings settings,
        shared_ptr<Model> model,
        shared_ptr<Log> log);

    CheckResult Run();
    void Witness();

  private:
    bool Check(int badId);
    bool Check_nonincremental(int badId);
    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<Model> m_model;
    int m_k;
    int m_maxK;
    int m_step;
    shared_ptr<State> m_initialState;
    int m_badId;
    shared_ptr<SATSolver> m_Solver;

    // for kissat to store clauses from previous unrolling
    shared_ptr<vector<clause>> m_clauses;

    CheckResult m_checkResult;
    void Init(int badId);
    void OutputCounterExample(int bad);
    void GetClausesK(int m_k, shared_ptr<vector<clause>> clauses);
    int GetBadK(int m_k);
    vector<int> GetConstraintsK(int m_k);
};

} // namespace car

#endif