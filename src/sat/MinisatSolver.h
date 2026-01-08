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
        if (model[id] == Minisat::l_True)
            return true;
        else {
            assert(model[id] == Minisat::l_False);
            return false;
        }
    }
    void ClearAssumption() override;
    void PushAssumption(int a) override;
    int PopAssumption() override;

    // not available
    inline void SetSolveInDomain() override {}
    inline void SetDomain(const cube &domain) override {}
    inline void SetTempDomain(const cube &domain) override {}
    inline void ResetTempDomain() override {}

  protected:
    inline int GetLiteralId(const Minisat::Lit &l);
    inline Minisat::Lit GetLit(int id) {
        int var = abs(id);
        while (var >= nVars()) newVar();
        return ((id > 0) ? Minisat::mkLit(var) : ~Minisat::mkLit(var));
    };

    Model &m_model;
    int m_maxId;
    Minisat::vec<Minisat::Lit> m_assumptions;
    int m_tempVar;
};

} // namespace car

#endif
