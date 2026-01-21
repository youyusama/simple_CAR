#include "SATSolver.h"
#include <algorithm>
#include <chrono>

namespace car {
namespace {
inline uint64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
} // namespace

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
    uint64_t prop_before = 0;
    uint64_t domain_len = 0;
    uint64_t pre_ns = 0;
    uint64_t search_ns = 0;
    uint64_t post_ns = 0;

    auto slv = GetMinicoreSolver();
    if (slv) {
        prop_before = slv->propagations;
        if (m_solveInDomain) {
            domain_len = slv->domainList().size();
        }
    }

    uint64_t start_ns = nowNs();
    bool res = m_slv->Solve();
    uint64_t elapsed_ns = nowNs() - start_ns;

    uint64_t prop_delta = 0;
    if (slv) {
        prop_delta = slv->propagations - prop_before;
        const auto &prof = slv->profileStats();
        pre_ns = prof.solve_pre_ns;
        search_ns = prof.solve_search_ns;
        post_ns = prof.solve_post_ns;
    }

    m_lastSolve.domain = domain_len;
    m_lastSolve.props = prop_delta;
    m_lastSolve.time_ns = elapsed_ns;
    m_lastSolve.domain_ns = m_pending_domain_ns;
    m_lastSolve.pre_ns = pre_ns;
    m_lastSolve.search_ns = search_ns;
    m_lastSolve.post_ns = post_ns;

    m_stats.calls++;
    m_stats.sum_domain += domain_len;
    m_stats.sum_props += prop_delta;
    m_stats.sum_time_ns += elapsed_ns;
    m_stats.sum_domain_ns += m_pending_domain_ns;
    m_stats.sum_pre_ns += pre_ns;
    m_stats.sum_search_ns += search_ns;
    m_stats.sum_post_ns += post_ns;
    m_pending_domain_ns = 0;

    return res;
}

bool SATSolver::Solve(const cube &assumption) {
    uint64_t prop_before = 0;
    uint64_t domain_len = 0;
    uint64_t pre_ns = 0;
    uint64_t search_ns = 0;
    uint64_t post_ns = 0;

    auto slv = GetMinicoreSolver();
    if (slv) {
        prop_before = slv->propagations;
        if (m_solveInDomain) {
            domain_len = slv->domainList().size();
        }
    }

    uint64_t start_ns = nowNs();
    bool res = m_slv->Solve(assumption);
    uint64_t elapsed_ns = nowNs() - start_ns;

    uint64_t prop_delta = 0;
    if (slv) {
        prop_delta = slv->propagations - prop_before;
        const auto &prof = slv->profileStats();
        pre_ns = prof.solve_pre_ns;
        search_ns = prof.solve_search_ns;
        post_ns = prof.solve_post_ns;
    }

    m_lastSolve.domain = domain_len;
    m_lastSolve.props = prop_delta;
    m_lastSolve.time_ns = elapsed_ns;
    m_lastSolve.domain_ns = m_pending_domain_ns;
    m_lastSolve.pre_ns = pre_ns;
    m_lastSolve.search_ns = search_ns;
    m_lastSolve.post_ns = post_ns;

    m_stats.calls++;
    m_stats.sum_domain += domain_len;
    m_stats.sum_props += prop_delta;
    m_stats.sum_time_ns += elapsed_ns;
    m_stats.sum_domain_ns += m_pending_domain_ns;
    m_stats.sum_pre_ns += pre_ns;
    m_stats.sum_search_ns += search_ns;
    m_stats.sum_post_ns += post_ns;
    m_pending_domain_ns = 0;

    return res;
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
    uint64_t start_ns = nowNs();
    AddPermanentVars(slv, domain, false);
    m_pending_domain_ns += nowNs() - start_ns;
}

void SATSolver::SetTempDomain(const cube &domain) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    uint64_t start_ns = nowNs();
    AddTemporaryVars(slv, domain, false);
    m_pending_domain_ns += nowNs() - start_ns;
}

void SATSolver::ResetTempDomain() {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    uint64_t start_ns = nowNs();
    ResetTemporaryVars(slv);
    m_pending_domain_ns += nowNs() - start_ns;
}

void SATSolver::SetDomainCOI(const cube &c) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    uint64_t start_ns = nowNs();
    AddPermanentVars(slv, c, true);
    m_pending_domain_ns += nowNs() - start_ns;
}

void SATSolver::SetTempDomainCOI(const cube &c) {
    if (!m_solveInDomain) return;
    auto slv = GetMinicoreSolver();
    if (!slv) return;
    uint64_t start_ns = nowNs();
    AddTemporaryVars(slv, c, true);
    m_pending_domain_ns += nowNs() - start_ns;
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
