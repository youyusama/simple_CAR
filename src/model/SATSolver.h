#ifndef SATSOLVER_H
#define SATSOLVER_H

#ifdef CADICAL
#include "CadicalSolver.h"
#endif
#include "ISolver.h"
#include "MinisatSolver.h"
#include "Model.h"
#include <memory>

namespace car {

class SATSolver : public ISolver {
  public:
    SATSolver(shared_ptr<Model> model, int slv_kind);
    ~SATSolver() {}

    // general SAT interface
    void AddClause(const cube &cls) {
        m_slv->AddClause(cls);
    }

    void AddAssumption(const shared_ptr<cube> assumption) {
        m_slv->AddAssumption(assumption);
    }

    bool Solve() {
        return m_slv->Solve();
    }

    bool Solve(const shared_ptr<cube> assumption) {
        return m_slv->Solve(assumption);
    }

    pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment(bool prime) {
        return m_slv->GetAssignment(prime);
    }

    shared_ptr<cube> GetUC(bool prime) {
        return m_slv->GetUC(prime);
    }

    int GetNewVar() {
        return m_slv->GetNewVar();
    }

    void AddTempClause(const cube &cls) {
        m_slv->AddTempClause(cls);
    }

    void ReleaseTempClause() {
        m_slv->ReleaseTempClause();
    }

    bool GetModel(int id) {
        return m_slv->GetModel(id);
    }

    void ClearAssumption() {
        m_slv->ClearAssumption();
    }

    void PushAssumption(int a) {
        m_slv->PushAssumption(a);
    }

    int PopAssumption() {
        return m_slv->PopAssumption();
    }

    // SAT interface for IC3/CAR
    void AddTrans();

    void AddConstraints();

    bool SolveFrame(const shared_ptr<cube> assumption, int lvl);

    void AddUC(const cube &uc, int lvl);

    void AddProperty();

    void AddConstraintOr(const vector<shared_ptr<cube>> frame);

    void AddConstraintAnd(const vector<shared_ptr<cube>> frame);

    void FlipLastConstrain();

    pair<shared_ptr<cube>, shared_ptr<cube>> GetStartPair();

    void UpdateStartSolverFlag();

    int GetStartSolverFlag();

  protected:
    shared_ptr<Model> m_model;
    shared_ptr<ISolver> m_slv;

    vector<int> m_frameFlags;
    inline int GetFrameFlag(int lvl) {
        assert(lvl >= 0);
        while (m_frameFlags.size() <= lvl) {
            m_frameFlags.emplace_back(GetNewVar());
        }
        return m_frameFlags[lvl];
    }
};

} // namespace car

#endif