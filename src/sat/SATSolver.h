#ifndef SATSOLVER_H
#define SATSOLVER_H

#ifdef CADICAL
#include "CadicalSolver.h"
#endif
#ifdef KISSAT
#include "KissatSolver.h"
#endif
#include "ISolver.h"
#include "MinicoreSolver.h"
#include "MinisatSolver.h"
#include "Model.h"
#include <memory>

namespace car {

class SATSolver {
  public:
    using frame = std::vector<cube>;
    using FrameList = std::vector<frame>;

    SATSolver(Model &model, MCSATSolver slv_kind);
    ~SATSolver() {}

    // general SAT interface
    void AddClause(const cube &cls) {
        m_slv->AddClause(cls);
    }

    void AddAssumption(const cube &assumption) {
        m_slv->AddAssumption(assumption);
    }

    bool Solve();
    bool Solve(const cube &assumption);

    pair<cube, cube> GetAssignment(bool prime) {
        return m_slv->GetAssignment(prime);
    }

    unordered_set<int> GetConflict() {
        return m_slv->GetConflict();
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

    tbool GetModel(int id) {
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

    // special interface in minicore
    void SetSolveInDomain();

    void SetDomain(const cube &domain);

    void SetTempDomain(const cube &domain);

    void ResetTempDomain();

    void SetDomainCOI(const cube &c);

    void SetTempDomainCOI(const cube &c);

    cube GetDomain();

    // SAT interface for IC3/CAR
    void AddTrans();

    void AddTransK(int k);

    void AddConstraints();

    void AddConstraintsK(int k);

    void AddBad();

    void AddBadk(int k);

    cube GetKUnrolled(const cube &c, int k);

    int AddInvAsLabelK(const FrameList &inv, int k);

    int AddCubeAsLabelK(const cube &c, int k);

    void AddInvAsClauseK(const FrameList &inv, bool neg, int k);

    void AddCubeAsClauseK(const cube &c, bool neg, int k);

    void AddWallConstraints(const std::vector<FrameList> &walls);

    void AddShoalConstraints(const std::vector<FrameList> &shoals,
                             const std::vector<cube> &dead,
                             int shoal_unroll = 1);

    void AddInitialClauses();

    bool SolveFrame(const cube &assumption, int lvl);

    void AddUC(const cube &uc, int lvl);

    void AddUC(const cube &uc);

    void AddProperty();

    void FlipLastConstrain();

    void UpdateStartSolverFlag();

  protected:
    Model &m_model;
    MCSATSolver m_slvKind;
    shared_ptr<ISolver> m_slv;
    bool m_solveInDomain;

    vector<int> m_frameFlags;
    inline int GetFrameFlag(int lvl) {
        assert(lvl >= 0);
        while (m_frameFlags.size() <= lvl) {
            m_frameFlags.emplace_back(GetNewVar());
        }
        return m_frameFlags[lvl];
    }

  private:
    shared_ptr<MinicoreSolver> GetMinicoreSolver() const;
    void AddPermanentVars(shared_ptr<MinicoreSolver> solver, const cube &vars, bool use_coi);
    void AddTemporaryVars(shared_ptr<MinicoreSolver> solver, const cube &vars, bool use_coi);
    void ResetTemporaryVars(shared_ptr<MinicoreSolver> solver);

    int m_true_id;
    size_t m_domain_fixed;
};

} // namespace car

#endif
