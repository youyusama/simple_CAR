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
    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;

  private:
    bool Check();
    bool CheckNonIncremental();
    void CNFGen();
    string GetCNFPath(int k) const;
    void WriteDimacs(const vector<Clause> &clauses, const string &path) const;
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
    void GetClausesK(int k, vector<Clause> &clauses);
    Lit GetBadK(int k);
    Cube GetConstraintsK(int k);
};

} // namespace car

#endif
