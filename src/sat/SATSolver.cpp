#include "SATSolver.h"
#include <algorithm>

namespace car {
SATSolver::SATSolver(Model &model, MCSATSolver slv_kind)
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
#ifdef KISSAT
    case MCSATSolver::kissat:
        m_slv = make_shared<KissatSolver>(m_model);
        break;
#endif
    case MCSATSolver::minicore:
        m_slv = make_shared<MinicoreSolver>(m_model);
        break;
    default:
        assert(false);
        break;
    }

    m_solveInDomain = false;
}

bool SATSolver::Solve() {
    return m_slv->Solve();
}

bool SATSolver::Solve(const cube &assumption) {
    return m_slv->Solve(assumption);
}

shared_ptr<MinicoreSolver> SATSolver::GetMinicoreSolver() const {
    if (m_slvKind != MCSATSolver::minicore) return nullptr;
    return static_pointer_cast<MinicoreSolver>(m_slv);
}

void SATSolver::SetSolveInDomain() {
    m_solveInDomain = true;
    auto slv = GetMinicoreSolver();
    if (slv == nullptr) return;
    slv->setSolveInDomain(true);
    m_true_id = m_model.TrueId();
    m_domain_fixed = 1;
}

void SATSolver::AddPermanentVars(shared_ptr<MinicoreSolver> solver, const cube &vars, bool use_coi) {
    if (!solver) return;
    std::vector<char> &domain = solver->domainSet();
    std::vector<minicore::Var> &list = solver->domainList();
    auto &dep_map = m_model.GetDependencyVec();
    ResetTemporaryVars(solver);

    vector<int> work_stack;
    work_stack.emplace_back(abs(m_true_id));
    for (int v : vars) {
        int a = abs(v);
        work_stack.emplace_back(abs(v));
    }

    while (!work_stack.empty()) {
        int cur = work_stack.back();
        work_stack.pop_back();

        bool in_dom = domain[cur];
        if (in_dom) continue;

        if (!in_dom) {
            domain[cur] = 1;
            list.push_back(cur);
            m_domain_fixed++;
        }

        if (use_coi) {
            auto &coi_vec = dep_map[cur];
            for (int d : coi_vec) {
                work_stack.emplace_back(d);
            }
        }
    }
}

void SATSolver::AddTemporaryVars(shared_ptr<MinicoreSolver> solver, const cube &vars, bool use_coi) {
    if (!solver) return;
    std::vector<char> &domain = solver->domainSet();
    std::vector<minicore::Var> &list = solver->domainList();
    auto &dep_map = m_model.GetDependencyVec();
    ResetTemporaryVars(solver);

    vector<int> work_stack;
    work_stack.emplace_back(abs(m_true_id));
    for (int v : vars) {
        int a = abs(v);
        work_stack.emplace_back(abs(v));
    }

    while (!work_stack.empty()) {
        int cur = work_stack.back();
        work_stack.pop_back();

        bool in_dom = domain[cur];
        if (in_dom) continue;

        domain[cur] = 1;
        list.push_back(cur);

        if (use_coi) {
            auto &coi_vec = dep_map[cur];
            for (int d : coi_vec) {
                work_stack.emplace_back(d);
            }
        }
    }
}

void SATSolver::ResetTemporaryVars(shared_ptr<MinicoreSolver> solver) {
    if (!solver) return;
    std::vector<char> &domain = solver->domainSet();
    std::vector<minicore::Var> &list = solver->domainList();

    for (size_t i = m_domain_fixed; i < list.size(); ++i) {
        int v = list[i];
        domain[v] = 0;
    }
    list.resize(m_domain_fixed);
}

void SATSolver::SetDomain(const cube &domain) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    AddPermanentVars(slv, domain, false);
}

void SATSolver::SetTempDomain(const cube &domain) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    AddTemporaryVars(slv, domain, false);
}

void SATSolver::ResetTempDomain() {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    ResetTemporaryVars(slv);
}

void SATSolver::SetDomainCOI(const cube &c) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    AddPermanentVars(slv, c, true);
}

void SATSolver::SetTempDomainCOI(const cube &c) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    AddTemporaryVars(slv, c, true);
}

cube SATSolver::GetDomain() {
    if (!m_solveInDomain) return {};
    auto slv = GetMinicoreSolver();
    if (!slv) return {};
    cube res;
    std::vector<minicore::Var> &list = slv->domainList();
    for (size_t i = 0; i < list.size(); ++i) {
        if (i == m_domain_fixed) res.emplace_back(-1);
        res.emplace_back(list[i]);
    }
    return res;
}


// ================================================================================
// @brief: add transition relation to solver, & T
// @input:
// @output:
// ================================================================================
void SATSolver::AddTrans() {
    if (m_solveInDomain) {
        vector<clause> &clauses = m_model.GetClauses();
        for (int i = 0; i < clauses.size(); ++i) {
            AddClause(clauses[i]);
        }
    } else {
        vector<clause> &clauses = m_model.GetSimpClauses();
        for (int i = 0; i < clauses.size(); ++i) {
            AddClause(clauses[i]);
        }
    }
}


// ================================================================================
// @brief: add constraints to solver
// @input:
// @output:
// ================================================================================
void SATSolver::AddConstraints() {
    for (auto c : m_model.GetConstraints()) {
        AddClause(clause{c});
        SetDomainCOI(cube{c});
    }
}


// ================================================================================
// @brief: add transition relation to solver, & T
// @input:
// @output:
// ================================================================================
void SATSolver::AddTransK(int k) {
    vector<clause> &clauses = m_model.GetSimpClauses();
    for (int i = 0; i < clauses.size(); ++i) {
        clause &ori = clauses[i];
        clause cls_k;
        for (int v : ori) {
            cls_k.push_back(m_model.GetPrimeK(v, k));
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
    for (auto c : m_model.GetConstraints()) {
        AddClause(clause{m_model.GetPrimeK(c, k)});
    }
}


void SATSolver::AddBad() {
    int bad = m_model.GetBad();
    AddClause(clause{bad});
}


void SATSolver::AddBadk(int k) {
    int bad = m_model.GetBad();
    int bad_k = m_model.GetPrimeK(bad, k);
    AddClause(clause{bad_k});
}

cube SATSolver::GetKUnrolled(const cube &c, int k) {
    if (k == 0) return c;
    cube out;
    out.reserve(c.size());
    for (int lit : c) {
        out.emplace_back(m_model.GetPrimeK(lit, k));
    }
    return out;
}

int SATSolver::AddInvAsLabelK(const FrameList &inv, int k) {
    int sl = GetNewVar();

    vector<int> o_labels;
    o_labels.reserve(inv.size());
    for (const auto &f : inv) {
        int ol = GetNewVar();
        for (const auto &fc : f) {
            cube fc_k = GetKUnrolled(fc, k);
            clause cls;
            cls.reserve(fc_k.size() + 1);
            for (int lit : fc_k) cls.emplace_back(-lit);
            cls.emplace_back(-ol);
            AddClause(cls);
        }
        o_labels.emplace_back(ol);
    }
    clause sl_clause;
    sl_clause.reserve(o_labels.size() + 1);
    for (int ol : o_labels) sl_clause.emplace_back(ol);
    sl_clause.emplace_back(-sl);
    AddClause(sl_clause);

    for (const auto &f : inv) {
        clause tmp;
        tmp.reserve(f.size() + 1);
        for (const auto &fc : f) {
            int cl = GetNewVar();
            tmp.emplace_back(cl);
            cube fc_k = GetKUnrolled(fc, k);
            for (int lit : fc_k) {
                AddClause(clause{-cl, lit});
            }
        }
        tmp.emplace_back(sl);
        AddClause(tmp);
    }

    return sl;
}

int SATSolver::AddCubeAsLabelK(const cube &c, int k) {
    cube c_k = GetKUnrolled(c, k);
    if (c_k.size() == 1) return c_k[0];

    int sl = GetNewVar();
    clause c_to_sl;
    c_to_sl.reserve(c_k.size() + 1);
    for (int lit : c_k) c_to_sl.emplace_back(-lit);
    c_to_sl.emplace_back(sl);
    AddClause(c_to_sl);
    for (int lit : c_k) {
        AddClause(clause{-sl, lit});
    }

    return sl;
}

void SATSolver::AddInvAsClauseK(const FrameList &inv, bool neg, int k) {
    int l_inv = AddInvAsLabelK(inv, k);
    if (neg)
        AddClause(clause{-l_inv});
    else
        AddClause(clause{l_inv});
}

void SATSolver::AddCubeAsClauseK(const cube &c, bool neg, int k) {
    int l_cube = AddCubeAsLabelK(c, k);
    if (neg)
        AddClause(clause{-l_cube});
    else
        AddClause(clause{l_cube});
}

void SATSolver::AddWallConstraints(const std::vector<FrameList> &walls) {
    if (walls.empty()) return;
    for (const auto &inv : walls) {
        int p = AddInvAsLabelK(inv, 0);
        int pp = AddInvAsLabelK(inv, 1);
        AddClause(clause{-p, pp});
        AddClause(clause{-pp, p});
    }
}

cube SATSolver::AddWallConstraintsAsLabels(const std::vector<FrameList> &walls) {
    cube labels;
    for (const auto &inv : walls) {
        int p = AddInvAsLabelK(inv, 0);
        int pp = AddInvAsLabelK(inv, 1);
        int l = GetNewVar();
        AddClause(clause{-l, -p, pp});
        AddClause(clause{-l, p, -pp});
        AddClause(clause{l, p, pp});
        AddClause(clause{l, -p, -pp});
        labels.emplace_back(l);
    }
    return labels;
}

void SATSolver::AddShoalConstraints(const std::vector<FrameList> &shoals,
                                    const std::vector<cube> &dead,
                                    int shoal_unroll) {
    int unroll = shoal_unroll >= 1 ? shoal_unroll : 1;
    for (int u = 0; u <= unroll; ++u) {
        for (const auto &inv : shoals) {
            AddInvAsClauseK(inv, true, u);
        }
        for (const auto &d : dead) {
            AddCubeAsClauseK(d, true, u);
        }
    }
}

cube SATSolver::AddShoalConstraintsAsLabels(const std::vector<FrameList> &shoals,
                                            const std::vector<cube> &dead,
                                            int shoal_unroll) {
    cube labels;
    int unroll = shoal_unroll >= 1 ? shoal_unroll : 1;
    for (int u = 0; u <= unroll; ++u) {
        for (const auto &inv : shoals) {
            labels.emplace_back(-AddInvAsLabelK(inv, u));
        }
        for (const auto &d : dead) {
            labels.emplace_back(-AddCubeAsLabelK(d, u));
        }
    }
    return labels;
}


// ================================================================================
// @brief: add initial constraints on gate intialized latches
// @input:
// @output:
// ================================================================================
void SATSolver::AddInitialClauses() {
    vector<clause> &clauses = m_model.GetInitialClauses();
    for (auto &c : clauses) {
        AddClause(c);
        SetDomainCOI(c);
    }
}


// ================================================================================
// @brief: assume state and give frame level, check satisfiability, & F_i & ass
// @input:
// @output:
// ================================================================================
bool SATSolver::SolveFrame(const cube &assumption, int lvl) {
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
    cls.reserve(uc.size() + 1);
    cls.emplace_back(-flag);
    for (auto ci : uc) cls.emplace_back(-ci);

    AddClause(cls);
    SetDomain(cls);
}

void SATSolver::AddUC(const cube &uc) {
    clause cls;
    cls.reserve(uc.size());
    for (auto ci : uc) cls.emplace_back(-ci);

    AddClause(cls);
    SetDomain(cls);
}


// ================================================================================
// @brief: let property hold, & p
// @input:
// @output:
// ================================================================================
void SATSolver::AddProperty() {
    clause cls = clause{m_model.GetProperty()};
    AddClause(cls);
    SetDomainCOI(cls);
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
