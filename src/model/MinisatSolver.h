#ifndef MINISATSOLVER_H
#define MINISATSOLVER_H

#include "ISolver.h"
#include "Model.h"
#include "minisat/core/Solver.h"
#include <memory>

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
    unordered_set<int> GetConflict() override;
    inline int GetNewVar() {
        return ++m_maxId;
    }
    void AddTempClause(const cube &cls) override;
    void ReleaseTempClause() override;
    inline bool GetModel(int id) {
        if (model[id] == Minisat::l_True)
            return true;
        else {
            assert(model[id] == Minisat::l_False);
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
    inline int GetLiteralId(const Minisat::Lit &l);
    inline Minisat::Lit GetLit(int id) {
        int var = abs(id);
        while (var >= nVars()) newVar();
        return ((id > 0) ? Minisat::mkLit(var) : ~Minisat::mkLit(var));
    };

    shared_ptr<Model> m_model;
    int m_maxId;
    Minisat::vec<Minisat::Lit> m_assumptions;
    int m_tempVar;
};

} // namespace car

#endif