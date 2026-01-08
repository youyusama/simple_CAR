#ifndef CADICALSOLVER_H
#define CADICALSOLVER_H

#include "ISolver.h"
#include "Model.h"
#include "cadical/src/cadical.hpp"
#include <memory>

namespace car {

class CadicalSolver : public ISolver, public CaDiCaL::Solver {
  public:
    CadicalSolver(Model &m);
    ~CadicalSolver();

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
        if (val(id) > 0)
            return true;
        else {
            assert(val(id) < 0);
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
    Model &m_model;
    int m_maxId;
    cube m_assumptions;
    cube m_tempClause;
};

} // namespace car

#endif
