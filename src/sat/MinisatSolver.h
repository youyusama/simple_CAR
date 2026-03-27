#ifndef MINISATSOLVER_H
#define MINISATSOLVER_H

#include "ISolver.h"
#include "Model.h"
#include "minisat/core/Solver.h"
#include <memory>

namespace car {

class MinisatSolver : public ISolver, public Minisat::Solver {
  public:
    MinisatSolver(Model &m);
    ~MinisatSolver();

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
        if (model[id] == Minisat::l_True)
            return T_TRUE;
        else if (model[id] == Minisat::l_False) {
            return T_FALSE;
        } else {
            return T_UNDEF;
        }
    }
    void ClearAssumption() override;
    void PushAssumption(Lit a) override;
    Lit PopAssumption() override;

  protected:
    inline Lit GetLiteral(const Minisat::Lit &l);
    inline Minisat::Lit GetLit(int id) {
        Var lit_var = AbsLit(id);
        while (static_cast<int>(lit_var) >= nVars()) newVar();
        Minisat::Var solver_var = static_cast<Minisat::Var>(lit_var);
        return ((id > 0) ? Minisat::mkLit(solver_var) : ~Minisat::mkLit(solver_var));
    };
    inline Minisat::Lit GetLit(Lit lit) {
        while (static_cast<int>(VarOf(lit)) >= nVars()) newVar();
        Minisat::Var solver_var = static_cast<Minisat::Var>(VarOf(lit));
        return Sign(lit) ? ~Minisat::mkLit(solver_var) : Minisat::mkLit(solver_var);
    }

    Model &m_model;
    Var m_maxId;
    Minisat::vec<Minisat::Lit> m_assumptions;
    Var m_tempVar{0};
};

} // namespace car

#endif
