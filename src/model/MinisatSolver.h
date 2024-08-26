#ifndef MINISATSOLVER_H
#define MINISATSOLVER_H

#include "../sat/minisat/core/Solver.h"
#include "AigerModel.h"
#include "ISolver.h"
#include <memory>

using namespace Minisat;

namespace car {

class MinisatSolver : public ISolver, public Minisat::Solver {
  public:
    MinisatSolver();
    ~MinisatSolver();

    void AddClause(const vector<int> &cls) override;
    void AddAssumption(const shared_ptr<vector<int>> assumption) override;
    bool Solve() override;
    bool Solve(const shared_ptr<vector<int>> assumption) override;
    pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> GetAssignment(bool prime) override;
    shared_ptr<vector<int>> GetUC(bool prime) override;
    inline int GetNewVar() {
        return ++m_maxId;
    }
    void AddTempClause(const vector<int> &cls) override;
    void ReleaseTempClause() override;
    inline bool GetModel(int id) {
        if (model[id - 1] == l_True)
            return true;
        else {
            assert(model[id - 1] == l_False);
            return false;
        }
    }

  protected:
    inline int GetLiteralId(const Lit &l);
    inline Lit GetLit(int id) {
        int var = abs(id) - 1;
        while (var >= nVars()) newVar();
        return ((id > 0) ? mkLit(var) : ~mkLit(var));
    };

    shared_ptr<AigerModel> m_model;
    int m_maxId;
    vec<Lit> m_assumptions;
    int m_tempVar;
};

} // namespace car

#endif