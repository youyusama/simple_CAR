#ifndef CARSOLVER_H
#define CARSOLVER_H

#include "../sat/minisat/core/Solver.h"
#include "AigerModel.h"
#include "ISolver.h"
#include <memory>

using namespace Minisat;

namespace car {

class CarSolver : public ISolver, public Minisat::Solver {
  public:
    CarSolver();
    ~CarSolver();

    void AddClause(const clause &cls) override;
    inline void AddAssumption(int id) override { m_assumptions.push(GetLit(id)); }
    void AddAssumption(const shared_ptr<cube> assumption) override;
    bool Solve() override;
    bool Solve(const shared_ptr<cube> assumption) override;
    bool Solve(const shared_ptr<cube> assumption, int frameLevel) override;
    pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment(bool prime) override;
    shared_ptr<cube> GetUC(bool prime) override;
    void AddUC(const cube &uc, int frameLevel) override;
    void AddConstraintOr(const vector<shared_ptr<cube>> frame) override;
    void AddConstraintAnd(const vector<shared_ptr<cube>> frame) override;
    void FlipLastConstrain() override;

    inline int GetNewVar() { return ++m_maxId; }

    inline bool GetModelOfId(int id) {
        if (model[id - 1] == l_True)
            return true;
        else {
            assert(model[id - 1] == l_False);
            return false;
        }
    };

  protected:
    inline int GetLiteralId(const Lit &l);
    inline int GetFrameFlag(int frameLevel);
    inline Lit GetLit(int id) {
        int var = abs(id) - 1;
        while (var >= nVars()) newVar();
        return ((id > 0) ? mkLit(var) : ~mkLit(var));
    };

    int m_maxId;
    shared_ptr<AigerModel> m_model;
    vector<int> m_frameFlags;
    vec<Lit> m_assumptions;
};

} // namespace car

#endif