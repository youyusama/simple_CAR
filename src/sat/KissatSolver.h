// created by Jianwen Li
// Kissat API for BMC
#ifndef KISSATSOLVER_H
#define KISSATSOLVER_H

#include "ISolver.h"
#include "Model.h"
extern "C" {
#include "kissat/src/kissat.h"
}
#include <assert.h>
#include <memory>


namespace car {

class KissatSolver : public ISolver {
  public:
    KissatSolver(Model &m);
    ~KissatSolver() { kissat_release(m_solver); }

    void AddClause(const Cube &cls) override;
    void AddAssumption(const Cube &assumption) override {}
    bool Solve() override;
    bool Solve(const Cube &assumption) override { return false; }
    pair<Cube, Cube> GetAssignment(bool prime) override { return pair<Cube, Cube>(Cube(), Cube()); }
    Cube GetUC(bool prime) { return Cube(); }
    unordered_set<Lit, LitHash> GetConflict() override { return unordered_set<Lit, LitHash>(); }
    Var GetNewVar() override { return 0; }
    void AddTempClause(const Cube &cls) override {}
    void ReleaseTempClause() override {}

    inline Tbool GetModel(Var id) override {
        int val = kissat_value(m_solver, id);
        assert(!val);
        if (val < 0)
            return T_FALSE;
        else
            return T_TRUE;
    }
    void ClearAssumption() override {}
    void PushAssumption(Lit a) override {}
    Lit PopAssumption() override { return Lit{}; }

  protected:
    /*
      inline int GetLiteralId(const Lit &l);
      inline Lit GetLit(int id) {
          int var = abs(id) - 1;
          while (var >= nVars()) newVar();
          return ((id > 0) ? MkLit(var) : ~MkLit(var));
      };
      */

    Model &m_model;
    Var m_maxId;
    // vec<Lit> m_assumptions;
    // int m_tempVar;
    kissat *m_solver = NULL;
};

} // namespace car

#endif
