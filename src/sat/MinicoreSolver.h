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
    unordered_set<int> GetConflict() override;
    inline int GetNewVar() override {
        return ++m_maxId;
    }
    void AddTempClause(const Cube &cls) override;
    void ReleaseTempClause() override;
    inline Tbool GetModel(int id) override {
        if (value(id) == minicore::l_True)
            return T_TRUE;
        else if (value(id) == minicore::l_False)
            return T_FALSE;
        else
            return T_UNDEF;
    }
    void ClearAssumption() override;
    void PushAssumption(int a) override;
    int PopAssumption() override;

  protected:
    inline int GetLiteralId(const minicore::Lit &l);
    inline minicore::Lit GetLit(int id) {
        minicore::Var var = abs(id);
        while (var >= nVars()) newVar();
        return ((id > 0) ? minicore::mkLit(var) : ~minicore::mkLit(var));
    };

    Model &m_model;
    int m_maxId;
    vector<minicore::Lit> m_assumptions;
    vector<minicore::Lit> m_tempClause;
};

} // namespace car

#endif
