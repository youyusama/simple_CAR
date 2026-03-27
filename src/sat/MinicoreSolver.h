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

    void AddClause(const Cube &cls) override;
    void AddAssumption(const Cube &assumption) override;
    bool Solve() override;
    bool Solve(const Cube &assumption) override;
    pair<Cube, Cube> GetAssignment(bool prime) override;
    unordered_set<Lit, LitHash> GetConflict() override;
    inline Var GetNewVar() override {
        return ++m_maxId;
    }
    void AddTempClause(const Cube &cls) override;
    void ReleaseTempClause() override;
    inline Tbool GetModel(Var id) override {
        if (value(id) == minicore::l_True)
            return T_TRUE;
        else if (value(id) == minicore::l_False)
            return T_FALSE;
        else
            return T_UNDEF;
    }
    void ClearAssumption() override;
    void PushAssumption(Lit a) override;
    Lit PopAssumption() override;

  protected:
    inline minicore::Lit GetLit(int id) {
        Var lit_var = AbsLit(id);
        while (static_cast<int>(lit_var) >= nVars()) newVar();
        return FromSigned(id);
    };
    inline minicore::Lit GetLit(Lit lit) {
        while (static_cast<int>(VarOf(lit)) >= nVars()) newVar();
        return lit;
    }

    Model &m_model;
    Var m_maxId;
    vector<minicore::Lit> m_assumptions;
    vector<minicore::Lit> m_tempClause;
};

} // namespace car

#endif
