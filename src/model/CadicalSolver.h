#ifndef CADICALSOLVER_H
#define CADICALSOLVER_H

#include "../sat/cadical/src/cadical.hpp"
#include "ISolver.h"
#include "Model.h"
#include <memory>

namespace car {

class CadicalSolver : public ISolver, public CaDiCaL::Solver {
  public:
    CadicalSolver();
    ~CadicalSolver();

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
        if (val(id) > 0)
            return true;
        else {
            assert(val(id) < 0);
            return false;
        }
    }

  protected:
    shared_ptr<Model> m_model;
    int m_maxId;
    shared_ptr<cube> m_assumptions;
    cube m_tempClause;
};

} // namespace car

#endif