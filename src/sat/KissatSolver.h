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

    void AddClause(const cube &cls) override;
    void AddAssumption(const cube &assumption) override {}
    bool Solve() override;
    bool Solve(const cube &assumption) override { return false; }
    pair<cube, cube> GetAssignment(bool prime) override { return pair<cube, cube>(cube(), cube()); }
    cube GetUC(bool prime) { return cube(); }
    unordered_set<int> GetConflict() override { return unordered_set<int>(); }
    int GetNewVar() override { return 0; }
    void AddTempClause(const cube &cls) override {}
    void ReleaseTempClause() override {}

    inline bool GetModel(int id) override {
        int val = kissat_value(m_solver, id);
        assert(!val);
        if (val < 0)
            return false;
        else
            return true;
    }
    void ClearAssumption() override {}
    void PushAssumption(int a) override {}
    int PopAssumption() override { return 0; }

    // not available
    inline void SetSolveInDomain() override {}
    inline void SetDomain(const cube &domain) override {}
    inline void SetTempDomain(const cube &domain) override {}
    inline void ResetTempDomain() override {}

  protected:
    /*
      inline int GetLiteralId(const Lit &l);
      inline Lit GetLit(int id) {
          int var = abs(id) - 1;
          while (var >= nVars()) newVar();
          return ((id > 0) ? mkLit(var) : ~mkLit(var));
      };
      */

    Model &m_model;
    int m_maxId;
    // vec<Lit> m_assumptions;
    // int m_tempVar;
    kissat *m_solver = NULL;
};

} // namespace car

#endif
