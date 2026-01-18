#ifndef MINICORESOLVER_H
#define MINICORESOLVER_H

#include "ISolver.h"
#include "Model.h"
#include "minicore/src/solver.h"
#include <memory>

namespace car {

class MinicoreSolver : public ISolver, public minicore::Solver {
  public:
    MinicoreSolver(Model &m);
    ~MinicoreSolver();

    void AddClause(const cube &cls) override;
    void AddAssumption(const cube &assumption) override;
    bool Solve() override;
    bool Solve(const cube &assumption) override;
    pair<cube, cube> GetAssignment(bool prime) override;
    unordered_set<int> GetConflict() override;
    inline int GetNewVar() override {
        return ++m_maxId;
    }
    void AddTempClause(const cube &cls) override;
    void ReleaseTempClause() override;
    inline bool GetModel(int id) override {
        if (value(id) == minicore::l_True)
            return true;
        else {
            return false;
        }
    }
    void ClearAssumption() override;
    void PushAssumption(int a) override;
    int PopAssumption() override;

    inline void SetSolveInDomain() override;
    inline void SetDomain(const cube &domain) override;
    inline void SetTempDomain(const cube &domain) override;
    inline void ResetTempDomain() override;

  protected:
    inline int GetLiteralId(const minicore::Lit &l);
    inline minicore::Lit GetLit(int id) {
        minicore::Var var = abs(id);
        while (var >= nVars()) newVar();
        return ((id > 0) ? minicore::mkLit(var) : ~minicore::mkLit(var));
    };

    Model &m_model;
    int m_maxId;
    vector<minicore::Lit> m_assumptions;
    vector<minicore::Lit> m_tempClause;
};

} // namespace car

#endif
