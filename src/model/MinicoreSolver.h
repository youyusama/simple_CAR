#ifndef MINICORESOLVER_H
#define MINICORESOLVER_H

#include "ISolver.h"
#include "Model.h"
#include "minicore/src/solver.h"
#include <memory>

namespace car {

class MinicoreSolver : public ISolver, public minicore::Solver {
  public:
    MinicoreSolver(shared_ptr<Model> m);
    ~MinicoreSolver();

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
        if (model[id] == minicore::l_True)
            return true;
        else {
            return false;
        }
    }
    void ClearAssumption();
    void PushAssumption(int a);
    int PopAssumption();

  protected:
    inline int GetLiteralId(const minicore::Lit &l);
    inline minicore::Lit GetLit(int id) {
        minicore::Var var = abs(id);
        while (var >= nVars()) newVar();
        return ((id > 0) ? minicore::mkLit(var) : ~minicore::mkLit(var));
    };

    shared_ptr<Model> m_model;
    int m_maxId;
    vector<minicore::Lit> m_assumptions;
    vector<minicore::Lit> m_tempClause;
};

} // namespace car

#endif