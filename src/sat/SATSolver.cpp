#include "SATSolver.h"
#include <algorithm>

namespace car {
SATSolver::SATSolver(Model &model, MCSATSolver slvKind)
    : m_model(model), m_slvKind(slvKind) {

    switch (m_slvKind) {
    case MCSATSolver::minisat:
        m_slv = make_shared<MinisatSolver>(m_model);
        break;
    case MCSATSolver::cadical:
        m_slv = make_shared<CadicalSolver>(m_model);
        break;
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
    m_trueId = m_model.TrueId();
    m_fixedDomainSize = 0;
}

bool SATSolver::Solve() {
    return m_slv->Solve();
}

bool SATSolver::Solve(const Cube &assumption) {
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
    m_trueId = m_model.TrueId();
    m_fixedDomainSize = 1;
}

void SATSolver::AddPermanentVars(shared_ptr<MinicoreSolver> solver, const Cube &vars, bool useCoi) {
    if (!solver) return;
    std::vector<char> &domain = solver->domainSet();
    std::vector<minicore::Var> &list = solver->domainList();
    auto &dep_map = m_model.GetDependencyVec();
    ResetTemporaryVars(solver);

    vector<Var> work_stack;
    work_stack.emplace_back(m_trueId);
    for (Lit v : vars) {
        work_stack.emplace_back(VarOf(v));
    }

    while (!work_stack.empty()) {
        Var cur = work_stack.back();
        work_stack.pop_back();

        bool in_dom = domain[cur];
        if (in_dom) continue;

        if (!in_dom) {
            domain[cur] = 1;
            list.push_back(cur);
            m_fixedDomainSize++;
        }

        if (useCoi) {
            auto &coi_vec = dep_map[cur];
            for (Var d : coi_vec) {
                work_stack.emplace_back(d);
            }
        }
    }
}

void SATSolver::AddTemporaryVars(shared_ptr<MinicoreSolver> solver, const Cube &vars, bool useCoi) {
    if (!solver) return;
    std::vector<char> &domain = solver->domainSet();
    std::vector<minicore::Var> &list = solver->domainList();
    auto &dep_map = m_model.GetDependencyVec();
    ResetTemporaryVars(solver);

    vector<Var> work_stack;
    work_stack.emplace_back(m_trueId);
    for (Lit v : vars) {
        work_stack.emplace_back(VarOf(v));
    }

    while (!work_stack.empty()) {
        Var cur = work_stack.back();
        work_stack.pop_back();

        bool in_dom = domain[cur];
        if (in_dom) continue;

        domain[cur] = 1;
        list.push_back(cur);

        if (useCoi) {
            auto &coi_vec = dep_map[cur];
            for (Var d : coi_vec) {
                work_stack.emplace_back(d);
            }
        }
    }
}

void SATSolver::ResetTemporaryVars(shared_ptr<MinicoreSolver> solver) {
    if (!solver) return;
    std::vector<char> &domain = solver->domainSet();
    std::vector<minicore::Var> &list = solver->domainList();

    for (size_t i = m_fixedDomainSize; i < list.size(); ++i) {
        Var v = static_cast<Var>(list[i]);
        domain[v] = 0;
    }
    list.resize(m_fixedDomainSize);
}

void SATSolver::SetDomain(const Cube &domain) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    AddPermanentVars(slv, domain, false);
}

void SATSolver::SetTempDomain(const Cube &domain) {
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

void SATSolver::SetDomainCOI(const Cube &c) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    AddPermanentVars(slv, c, true);
}

void SATSolver::SetTempDomainCOI(const Cube &c) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    AddTemporaryVars(slv, c, true);
}


// ================================================================================
// @brief: add transition relation to solver, & T
// @input:
// @output:
// ================================================================================
void SATSolver::AddTrans() {
    if (m_solveInDomain) {
        vector<Clause> &clauses = m_model.GetClauses();
        for (int i = 0; i < clauses.size(); ++i) {
            AddClause(clauses[i]);
        }
    } else {
        vector<Clause> &clauses = m_model.GetSimpClauses();
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
        AddClause(Clause{c});
        SetDomainCOI(Cube{c});
    }
}


// ================================================================================
// @brief: add transition relation to solver, & T
// @input:
// @output:
// ================================================================================
void SATSolver::AddTransK(int k) {
    vector<Clause> &clauses = m_model.GetSimpClauses();
    for (int i = 0; i < clauses.size(); ++i) {
        Clause &ori = clauses[i];
        Clause cls_k;
        for (Lit v : ori) {
            cls_k.push_back(m_model.EnsurePrimeK(v, k));
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
    for (Lit c : m_model.GetConstraints()) {
        AddClause(Clause{m_model.EnsurePrimeK(c, k)});
    }
}


void SATSolver::AddBad() {
    AddClause(Clause{m_model.GetBad()});
}


void SATSolver::AddBadk(int k) {
    AddClause(Clause{m_model.EnsurePrimeK(m_model.GetBad(), k)});
}

Cube SATSolver::GetKUnrolled(const Cube &c, int k) {
    if (k == 0) return c;
    Cube out;
    out.reserve(c.size());
    for (Lit lit : c) {
        out.emplace_back(m_model.EnsurePrimeK(lit, k));
    }
    return out;
}

Lit SATSolver::AddInvAsLabelK(const FrameList &inv, int k) {
    Var sl = GetNewVar();

    vector<Var> o_labels;
    o_labels.reserve(inv.size());
    for (const auto &f : inv) {
        Var ol = GetNewVar();
        for (const auto &fc : f) {
            Cube fc_k = GetKUnrolled(fc, k);
            Clause cls;
            cls.reserve(fc_k.size() + 1);
            for (Lit lit : fc_k) cls.emplace_back(~lit);
            cls.emplace_back(~MkLit(ol));
            AddClause(cls);
        }
        o_labels.emplace_back(ol);
    }
    Clause sl_clause;
    sl_clause.reserve(o_labels.size() + 1);
    for (Var ol : o_labels) sl_clause.emplace_back(MkLit(ol));
    sl_clause.emplace_back(~MkLit(sl));
    AddClause(sl_clause);

    for (const auto &f : inv) {
        Clause tmp;
        tmp.reserve(f.size() + 1);
        for (const auto &fc : f) {
            Var cl = GetNewVar();
            tmp.emplace_back(MkLit(cl));
            Cube fc_k = GetKUnrolled(fc, k);
            for (Lit lit : fc_k) {
                AddClause(Clause{~MkLit(cl), lit});
            }
        }
        tmp.emplace_back(MkLit(sl));
        AddClause(tmp);
    }

    return MkLit(sl);
}

Lit SATSolver::AddCubeAsLabelK(const Cube &c, int k) {
    Cube c_k = GetKUnrolled(c, k);
    if (c_k.size() == 1) return c_k[0];

    Var sl = GetNewVar();
    Clause c_to_sl;
    c_to_sl.reserve(c_k.size() + 1);
    for (Lit lit : c_k) c_to_sl.emplace_back(~lit);
    c_to_sl.emplace_back(MkLit(sl));
    AddClause(c_to_sl);
    for (Lit lit : c_k) {
        AddClause(Clause{~MkLit(sl), lit});
    }

    return MkLit(sl);
}

void SATSolver::AddInvAsClauseK(const FrameList &inv, bool neg, int k) {
    Lit l_inv = AddInvAsLabelK(inv, k);
    if (neg)
        AddClause(Clause{~l_inv});
    else
        AddClause(Clause{l_inv});
}

void SATSolver::AddCubeAsClauseK(const Cube &c, bool neg, int k) {
    Lit l_cube = AddCubeAsLabelK(c, k);
    if (neg)
        AddClause(Clause{~l_cube});
    else
        AddClause(Clause{l_cube});
}

void SATSolver::AddWallConstraints(const std::vector<FrameList> &walls) {
    if (walls.empty()) return;
    for (const auto &inv : walls) {
        Lit p = AddInvAsLabelK(inv, 0);
        Lit pp = AddInvAsLabelK(inv, 1);
        AddClause(Clause{~p, pp});
        AddClause(Clause{~pp, p});
    }
}

Cube SATSolver::AddWallConstraintsAsLabels(const std::vector<FrameList> &walls) {
    Cube labels;
    for (const auto &inv : walls) {
        Lit p = AddInvAsLabelK(inv, 0);
        Lit pp = AddInvAsLabelK(inv, 1);
        Lit l = MkLit(GetNewVar());
        AddClause(Clause{~l, ~p, pp});
        AddClause(Clause{~l, p, ~pp});
        AddClause(Clause{l, p, pp});
        AddClause(Clause{l, ~p, ~pp});
        labels.emplace_back(l);
    }
    return labels;
}

void SATSolver::AddShoalConstraints(const std::vector<FrameList> &shoals,
                                    const std::vector<Cube> &dead,
                                    int shoalUnroll) {
    int unroll = shoalUnroll >= 1 ? shoalUnroll : 1;
    for (int u = 1; u <= unroll; ++u) {
        for (const auto &inv : shoals) {
            AddInvAsClauseK(inv, true, u);
        }
        for (const auto &d : dead) {
            AddCubeAsClauseK(d, true, u);
        }
    }
}

Cube SATSolver::AddShoalConstraintsAsLabels(const std::vector<FrameList> &shoals,
                                            const std::vector<Cube> &dead,
                                            int shoalUnroll) {
    Cube labels;
    int unroll = shoalUnroll >= 1 ? shoalUnroll : 1;
    for (int u = 1; u <= unroll; ++u) {
        for (const auto &inv : shoals) {
            labels.emplace_back(~AddInvAsLabelK(inv, u));
        }
        for (const auto &d : dead) {
            labels.emplace_back(~AddCubeAsLabelK(d, u));
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
    vector<Clause> &clauses = m_model.GetInitialClauses();
    for (auto &c : clauses) {
        AddClause(c);
        SetDomainCOI(c);
    }
}


void SATSolver::AddUC(const Cube &uc) {
    Clause cls;
    cls.reserve(uc.size());
    for (auto ci : uc) cls.emplace_back(~ci);

    AddClause(cls);
    SetDomain(cls);
}


// ================================================================================
// @brief: let property hold, & p
// @input:
// @output:
// ================================================================================
void SATSolver::AddProperty() {
    Clause cls = Clause{m_model.GetProperty()};
    AddClause(cls);
    SetDomainCOI(cls);
}


// ================================================================================
// @brief:
// @input:
// @output:
// ================================================================================
void SATSolver::FlipLastConstrain() {
    Lit v = PopAssumption();
    AddClause({~v});
}


} // namespace car
