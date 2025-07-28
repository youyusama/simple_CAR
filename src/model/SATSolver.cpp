#include "SATSolver.h"

namespace car {

SATSolver::SATSolver(shared_ptr<Model> model, MCSATSolver slv_kind)
    : m_model(model), m_slvKind(slv_kind) {

    switch (m_slvKind) {
    case MCSATSolver::minisat:
        m_slv = make_shared<MinisatSolver>(m_model);
        break;
#ifdef CADICAL
    case MCSATSolver::cadical:
        m_slv = make_shared<CadicalSolver>(m_model);
        break;
#endif
    case MCSATSolver::minicore:
        m_slv = make_shared<MinicoreSolver>(m_model);
        break;
    default:
        assert(false);
        break;
    }
}


// ================================================================================
// @brief: add transition relation to solver, & T
// @input:
// @output:
// ================================================================================
void SATSolver::AddTrans() {
    vector<clause> &clauses = m_model->GetClauses();
    for (int i = 0; i < clauses.size(); ++i) {
        AddClause(clauses[i]);
    }
}


// ================================================================================
// @brief: add constraints to solver
// @input:
// @output:
// ================================================================================
void SATSolver::AddConstraints() {
    for (auto c : m_model->GetConstraints()) {
        AddClause(clause{c});
        SetDomainCOI(make_shared<cube>(cube{c}));
    }
}


// ================================================================================
// @brief: add transition relation to solver, & T
// @input:
// @output:
// ================================================================================
void SATSolver::AddTransK(int k) {
    vector<clause> &clauses = m_model->GetClauses();
    for (int i = 0; i < clauses.size(); ++i) {
        clause &ori = clauses[i];
        clause cls_k;
        for (int v : ori) {
            cls_k.push_back(m_model->GetPrimeK(v, k));
        }
        AddClause(cls_k);
    }
}


// ================================================================================
// @brief: add constraints to solver
// @input:
// @output:
// ================================================================================
void SATSolver::AddConstraintsK(int k) {
    for (auto c : m_model->GetConstraints()) {
        AddClause(clause{m_model->GetPrimeK(c, k)});
    }
}


void SATSolver::AddBad() {
    int bad = m_model->GetBad();
    AddClause(clause{bad});
}


void SATSolver::AddBadk(int k) {
    int bad = m_model->GetBad();
    int bad_k = m_model->GetPrimeK(bad, k);
    AddClause(clause{bad_k});
}


// ================================================================================
// @brief: assume state and give frame level, check satisfiability, & F_i & ass
// @input:
// @output:
// ================================================================================
bool SATSolver::SolveFrame(const shared_ptr<cube> assumption, int lvl) {
    ClearAssumption();
    PushAssumption(GetFrameFlag(lvl));
    AddAssumption(assumption);
    return Solve();
}


// ================================================================================
// @brief: add cube to frame, F_i = F_i & !uc
// @input:
// @output:
// ================================================================================
void SATSolver::AddUC(const shared_ptr<cube> uc, int lvl) {
    int flag = GetFrameFlag(lvl);
    clause cls;
    cls.reserve(uc->size() + 1);
    cls.emplace_back(-flag);
    for (auto ci : *uc) cls.emplace_back(-ci);

    AddClause(cls);
    SetDomain(make_shared<cube>(cls));
}


// ================================================================================
// @brief: let property hold, & p
// @input:
// @output:
// ================================================================================
void SATSolver::AddProperty() {
    clause cls = clause{m_model->GetProperty()};
    AddClause(cls);
    SetDomainCOI(make_shared<cube>(cls));
}


// ================================================================================
// @brief:
// @input:
// @output:
// ================================================================================
void SATSolver::FlipLastConstrain() {
    int v = PopAssumption();
    AddClause({-v});
}


// ================================================================================
// @brief: update start solver flag
// @input:
// @output:
// ================================================================================
void SATSolver::UpdateStartSolverFlag() {
    if (m_frameFlags.size() > 0) {
        int last_flag = PopAssumption();
        AddClause({-last_flag});
    }
    int flag = GetNewVar();
    m_frameFlags.push_back(flag);
    PushAssumption(flag);
}


} // namespace car
