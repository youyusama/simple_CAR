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
        if (val(id) > 0)
            return T_TRUE;
        else {
            assert(val(id) < 0);
            return T_FALSE;
        }
    }
    void ClearAssumption() override;
    void PushAssumption(int a) override;
    int PopAssumption() override;

  protected:
    Model &m_model;
    int m_maxId;
    Cube m_assumptions;
    Cube m_tempClause;
};

} // namespace car

#endif
