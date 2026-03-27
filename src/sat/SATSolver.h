#ifndef SATSOLVER_H
#define SATSOLVER_H

#include "CadicalSolver.h"
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
    using Frame = std::vector<Cube>;
    using FrameList = std::vector<Frame>;

    SATSolver(Model &model, MCSATSolver slvKind);
    ~SATSolver() {}

    // general SAT interface
    void AddClause(const Cube &cls) {
        m_slv->AddClause(cls);
    }

    void AddAssumption(const Cube &assumption) {
        m_slv->AddAssumption(assumption);
    }

    bool Solve();
    bool Solve(const Cube &assumption);

    pair<Cube, Cube> GetAssignment(bool prime) {
        return m_slv->GetAssignment(prime);
    }

    unordered_set<Lit, LitHash> GetConflict() {
        return m_slv->GetConflict();
    }

    Var GetNewVar() {
        return m_slv->GetNewVar();
    }

    void AddTempClause(const Cube &cls) {
        m_slv->AddTempClause(cls);
    }

    void ReleaseTempClause() {
        m_slv->ReleaseTempClause();
    }

    Tbool GetModel(Var id) {
        return m_slv->GetModel(id);
    }

    void ClearAssumption() {
        m_slv->ClearAssumption();
    }

    void PushAssumption(Lit a) {
        m_slv->PushAssumption(a);
    }

    Lit PopAssumption() {
        return m_slv->PopAssumption();
    }

    // special interface in minicore
    void SetSolveInDomain();

    void SetDomain(const Cube &domain);

    void SetTempDomain(const Cube &domain);

    void ResetTempDomain();

    void SetDomainCOI(const Cube &c);

    void SetTempDomainCOI(const Cube &c);

    // SAT interface for IC3/CAR
    void AddTrans();

    void AddTransK(int k);

    void AddConstraints();

    void AddConstraintsK(int k);

    void AddBad();

    void AddBadk(int k);

    Cube GetKUnrolled(const Cube &c, int k);

    Lit AddInvAsLabelK(const FrameList &inv, int k);

    Lit AddCubeAsLabelK(const Cube &c, int k);

    void AddInvAsClauseK(const FrameList &inv, bool neg, int k);

    void AddCubeAsClauseK(const Cube &c, bool neg, int k);

    void AddWallConstraints(const std::vector<FrameList> &walls);

    Cube AddWallConstraintsAsLabels(const std::vector<FrameList> &walls);

    void AddShoalConstraints(const std::vector<FrameList> &shoals,
                             const std::vector<Cube> &dead,
                             int shoalUnroll = 1);

    Cube AddShoalConstraintsAsLabels(const std::vector<FrameList> &shoals,
                                     const std::vector<Cube> &dead,
                                     int shoalUnroll = 1);

    void AddInitialClauses();

    void AddUC(const Cube &uc);

    void AddProperty();

    void FlipLastConstrain();

  protected:
    Model &m_model;
    MCSATSolver m_slvKind;
    shared_ptr<ISolver> m_slv;
    bool m_solveInDomain;

  private:
    shared_ptr<MinicoreSolver> GetMinicoreSolver() const;
    void AddPermanentVars(shared_ptr<MinicoreSolver> solver, const Cube &vars, bool useCoi);
    void AddTemporaryVars(shared_ptr<MinicoreSolver> solver, const Cube &vars, bool useCoi);
    void ResetTemporaryVars(shared_ptr<MinicoreSolver> solver);

    Var m_trueId;
    size_t m_fixedDomainSize;
};

} // namespace car

#endif
