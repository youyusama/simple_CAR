// created by Jianwen Li
// Kissat API for BMC
#ifndef KISSATSOLVER_H
#define KISSATSOLVER_H

#include "ISolver.h"
#include "Model.h"
extern "C" {
#include "kissat/src/kissat.h"
}
#include <assert.h>
#include <memory>
#include <stdexcept>


namespace car {

class KissatSolver : public ISolver {
  public:
    KissatSolver(Model &m);
    ~KissatSolver() {
        if (m_solver != nullptr) {
            kissat_release(m_solver);
        }
    }

    void AddClause(const Cube &cls) override;
    void AddAssumption(const Cube &assumption) override;
    bool Solve() override;
    bool Solve(const Cube &assumption) override;
    pair<Cube, Cube> GetAssignment(bool prime) override;
    unordered_set<Lit, LitHash> GetConflict() override;
    Var GetNewVar() override;
    void AddTempClause(const Cube &cls) override;
    void ReleaseTempClause() override;

    inline Tbool GetModel(Var id) override {
        int val = kissat_value(m_solver, id);
        assert(val != 0);
        if (val < 0) {
            return T_FALSE;
        } else {
            return T_TRUE;
        }
    }
    void ClearAssumption() override;
    void PushAssumption(Lit a) override;
    Lit PopAssumption() override;

  protected:
    Model &m_model;
    Var m_maxId;

    [[noreturn]] static void Unsupported(const char *fn);
    void EnsureReserved(const Cube &cls);

    kissat *m_solver = nullptr;
};

} // namespace car

#endif
