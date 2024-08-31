#ifndef BMC_H
#define BMC_H

#include "BaseChecker.h"
#ifdef CADICAL
#include "CadicalSolver.h"
#else
#include "MinisatSolver.h"
#endif
#include "Log.h"

namespace car {

class BMC : public BaseChecker {
  public:
    BMC(Settings settings,
        shared_ptr<AigerModel> model,
        shared_ptr<Log> log);

    bool Run();
    bool Check(int badId);

  private:
    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<AigerModel> m_model;
    int m_k;
    int m_maxK;
    shared_ptr<State> m_initialState;
    int m_badId;
    shared_ptr<ISolver> m_Solver;

    void Init(int badId);
    void OutputCounterExample(int bad);
    void GetClausesK(int m_k, shared_ptr<vector<clause>> clauses);
    int GetBadK(int m_k);
    vector<int> GetConstraintsK(int m_k);
};

} // namespace car

#endif