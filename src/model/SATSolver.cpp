#include "SATSolver.h"

namespace car {

SATSolver::SATSolver(shared_ptr<Model> model, int slv_kind) {
    m_model = model;

    switch (slv_kind) {
    case 0:
        m_slv = make_shared<MinisatSolver>(m_model);
        break;
#ifdef CADICAL
    case 1:
        m_slv = make_shared<CadicalSolver>(m_model);
        break;
#endif
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
    }
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
void SATSolver::AddUC(const cube &uc, int lvl) {
    int flag = GetFrameFlag(lvl);
    clause cls;
    cls.push_back(-flag);
    for (int i = 0; i < uc.size(); ++i) {
        cls.push_back(-uc[i]);
    }
    AddClause(cls);
}


// ================================================================================
// @brief: let property hold, & p
// @input:
// @output:
// ================================================================================
void SATSolver::AddProperty() {
    AddClause(clause{m_model->GetProperty()});
}


// ================================================================================
// @brief: add constraint | O_i
// @input:
// @output:
// ================================================================================
void SATSolver::AddConstraintOr(const vector<shared_ptr<cube>> frame) {
    cube cls;
    for (int i = 0; i < frame.size(); ++i) {
        int flag = GetNewVar();
        cls.push_back(flag);
        for (int j = 0; j < frame[i]->size(); ++j) {
            AddClause(cube{-flag, frame[i]->at(j)});
        }
    }
    AddClause(cls);
}


// ================================================================================
// @brief: add constraint & !O_i
// @input:
// @output:
// ================================================================================
void SATSolver::AddConstraintAnd(const vector<shared_ptr<cube>> frame) {
    int flag = GetNewVar();
    for (int i = 0; i < frame.size(); ++i) {
        cube cls;
        for (int j = 0; j < frame[i]->size(); ++j) {
            cls.push_back(-frame[i]->at(j));
        }
        cls.push_back(-flag);
        AddClause(cls);
    }
    shared_ptr<cube> f = make_shared<cube>();
    f->push_back(flag);
    AddAssumption(f);
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
        PushAssumption(-last_flag);
    }
    int flag = GetNewVar();
    m_frameFlags.push_back(flag);
    PushAssumption(flag);
}


int SATSolver::GetStartSolverFlag() {
    return m_frameFlags.back();
}


} // namespace car
