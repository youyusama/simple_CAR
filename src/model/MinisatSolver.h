#ifndef MINISATSOLVER_H
#define MINISATSOLVER_H

#include "ISolver.h"
#include "Model.h"
#include "minisat/core/Solver.h"
#include <memory>

using namespace Minisat;

namespace car {

class MinisatSolver : public ISolver, public Minisat::Solver {
  public:
    MinisatSolver(shared_ptr<Model> m);
    ~MinisatSolver();

    void AddClause(const cube &cls) override;
    void AddAssumption(const shared_ptr<cube> assumption) override;
    bool Solve() override;
    bool Solve(const shared_ptr<cube> assumption) override;
    pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment(bool prime) override;
    shared_ptr<cube> GetUC(bool prime) override;
    inline int GetNewVar() {
        return ++m_maxId;
    }
    void AddTempClause(const cube &cls) override;
    void ReleaseTempClause() override;
    inline bool GetModel(int id) {
        if (model[id - 1] == l_True)
            return true;
        else {
            assert(model[id - 1] == l_False);
            return false;
        }
    }
    void ClearAssumption();
    void PushAssumption(int a);
    int PopAssumption();

    // not available
    inline void SetSolveInDomain() {}
    inline void SetDomain(const shared_ptr<cube> domain) {}
    inline void SetTempDomain(const shared_ptr<cube> domain) {}
    inline void ResetTempDomain() {}

  protected:
    inline int GetLiteralId(const Lit &l);
    inline Lit GetLit(int id) {
        int var = abs(id) - 1;
        while (var >= nVars()) newVar();
        return ((id > 0) ? mkLit(var) : ~mkLit(var));
    };

    shared_ptr<Model> m_model;
    int m_maxId;
    vec<Lit> m_assumptions;
    int m_tempVar;
};

} // namespace car

#endif