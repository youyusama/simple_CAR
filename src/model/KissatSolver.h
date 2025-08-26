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
    KissatSolver(shared_ptr<Model> m);
    ~KissatSolver() { kissat_release(m_solver); }

    void AddClause(const cube &cls) override;
    void AddAssumption(const shared_ptr<cube> assumption) {}
    bool Solve() override;
    bool Solve(const shared_ptr<cube> assumption) { return false; }
    pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment(bool prime) { return pair<shared_ptr<cube>, shared_ptr<cube>>(NULL, NULL); }
    shared_ptr<cube> GetUC(bool prime) { return NULL; }
    int GetNewVar() { return 0; }
    void AddTempClause(const cube &cls) {}
    void ReleaseTempClause() {}

    inline bool GetModel(int id) {
        int val = kissat_value(m_solver, id);
        assert(!val);
        if (val < 0)
            return false;
        else
            return true;
    }
    void ClearAssumption() {}
    void PushAssumption(int a) {}
    int PopAssumption() { return 0; }

    // not available
    inline void SetSolveInDomain() {}
    inline void SetDomain(const shared_ptr<cube> domain) {}
    inline void SetTempDomain(const shared_ptr<cube> domain) {}
    inline void ResetTempDomain() {}

  protected:
    /*
      inline int GetLiteralId(const Lit &l);
      inline Lit GetLit(int id) {
          int var = abs(id) - 1;
          while (var >= nVars()) newVar();
          return ((id > 0) ? mkLit(var) : ~mkLit(var));
      };
      */

    shared_ptr<Model> m_model;
    int m_maxId;
    // vec<Lit> m_assumptions;
    // int m_tempVar;
    kissat *m_solver = NULL;
};

} // namespace car

#endif