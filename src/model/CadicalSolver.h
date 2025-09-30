#ifndef CADICALSOLVER_H
#define CADICALSOLVER_H

#include "ISolver.h"
#include "Model.h"
#include "cadical/src/cadical.hpp"
#include <memory>

namespace car {

class CadicalSolver : public ISolver, public CaDiCaL::Solver {
  public:
    CadicalSolver(shared_ptr<Model> m);
    ~CadicalSolver();

    CadicalSolver(const CadicalSolver& src)
        : ISolver(src), CaDiCaL::Solver(),
          m_model(src.m_model),
          m_maxId(src.m_maxId),
          m_assumptions(std::make_shared<cube>()),
          m_tempClause(cube())
    {
        src.CaDiCaL::Solver::copy(static_cast<CaDiCaL::Solver&>(*this));
    }

    std::shared_ptr<ISolver> Clone() const override {
        return std::make_shared<CadicalSolver>(*this);
    }

    int GetOption(const char* name) override {
        return get(name);
    }

    bool SetOption(const char* name, int val) override {
        return set(name, val);
    }

    int Simplify(int rounds = 3) override {
        return simplify(rounds);
    }

    void AddClause(const cube &cls) override;
    void AddAssumption(const shared_ptr<cube> assumption) override;
    bool Solve() override;
    bool Solve(const shared_ptr<cube> assumption) override;
    pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment(bool prime) override;
    shared_ptr<cube> GetUC(bool prime) override;
    unordered_set<int> GetConflict() override;
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
    void ClearAssumption();
    void PushAssumption(int a);
    int PopAssumption();

    // not available
    inline void SetSolveInDomain() {}
    inline void SetDomain(const shared_ptr<cube> domain) {}
    inline void SetTempDomain(const shared_ptr<cube> domain) {}
    inline void ResetTempDomain() {}

  protected:
    shared_ptr<Model> m_model;
    int m_maxId;
    shared_ptr<cube> m_assumptions;
    cube m_tempClause;
};

} // namespace car

#endif