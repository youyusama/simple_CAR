#ifndef CADICALSOLVER_H
#define CADICALSOLVER_H

#include "../sat/cadical/src/cadical.hpp"
#include "AigerModel.h"
#include "ISolver.h"
#include <memory>

namespace car {

class CadicalSolver : public ISolver, public CaDiCaL::Solver {
  public:
    CadicalSolver();
    ~CadicalSolver();

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
        if (val(id) > 0)
            return true;
        else {
            assert(val(id) < 0);
            return false;
        }
    }

  protected:
    shared_ptr<AigerModel> m_model;
    int m_maxId;
    shared_ptr<vector<int>> m_assumptions;
    vector<int> m_tempClause;
};

} // namespace car

#endif