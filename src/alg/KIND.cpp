#include "KIND.h"

#include <csignal>
#include <fstream>

namespace car {

KIND::KIND(Settings settings,
           Model &model,
           Log &log) : m_settings(settings),
                       m_log(log),
                       m_model(model),
                       m_k(0),
                       m_maxK(settings.bmcK),
                       m_baseAddedTransitions(0),
                       m_indAddedTransitions(0),
                       m_checkResult(CheckResult::Unknown),
                       m_useUnrollingCache(false) {
    State::num_inputs = model.GetNumInputs();
    State::num_latches = model.GetNumLatches();
    global_log = &m_log;
}

CheckResult KIND::Run() {
    signal(SIGINT, SignalHandler);
    Init();

    if (m_settings.solver == MCSATSolver::kissat) {
        m_checkResult = CheckNonIncremental();
    } else {
        m_checkResult = Check();
    }

    m_log.PrintCustomStatistics();
    return m_checkResult;
}

std::vector<std::pair<Cube, Cube>> KIND::GetCexTrace() {
    assert(m_checkResult == CheckResult::Unsafe);
    assert(m_cexSolver != nullptr);

    std::vector<std::pair<Cube, Cube>> trace;
    trace.reserve(m_k + 1);
    for (int k = 0; k <= m_k; ++k) {
        Cube inputs;
        for (auto i : m_model.GetModelInputs()) {
            Lit ip = m_model.EnsurePrimeK(MkLit(i), k);
            inputs.emplace_back(m_cexSolver->GetModel(VarOf(ip)) == T_TRUE ? MkLit(i) : ~MkLit(i));
        }
        Cube latches;
        for (auto l : m_model.GetModelLatches()) {
            Lit lp = m_model.EnsurePrimeK(MkLit(l), k);
            latches.emplace_back(m_cexSolver->GetModel(VarOf(lp)) == T_TRUE ? MkLit(l) : ~MkLit(l));
        }
        trace.emplace_back(inputs, latches);
    }

    return trace;
}

void KIND::Init() {
    m_k = 0;
    m_checkResult = CheckResult::Unknown;
    m_baseAddedTransitions = 0;
    m_indAddedTransitions = 0;
    m_useUnrollingCache = false;
    m_cexSolver.reset();
    m_diffVars.clear();
    m_nonIncIndBlockedPairs.clear();
}

void KIND::InitBaseSolver() {
    m_baseSolver = std::make_shared<SATSolver>(m_model, m_settings.solver);
    AddInitial(m_baseSolver);
}

void KIND::InitIndSolver() {
    m_indSolver = std::make_shared<SATSolver>(m_model, m_settings.solver);
}

CheckResult KIND::Check() {
    [[maybe_unused]] auto check_scope = m_log.Section("KIND_Check");
    m_useUnrollingCache = false;
    InitBaseSolver();
    InitIndSolver();

    while (true) {
        LOG_L(m_log, 1, "KIND Bound: ", m_k);

        AdvanceBaseToNextK();
        AdvanceIndSolverToNextK();

        if (CheckBaseCase()) {
            return CheckResult::Unsafe;
        }

        bool ind_safe = CheckInductiveStep();
        if (ind_safe) {
            return CheckResult::Safe;
        }

        if (m_maxK != -1 && m_k >= m_maxK) {
            return CheckResult::Unknown;
        }

        ++m_k;
    }
}

CheckResult KIND::CheckNonIncremental() {
    [[maybe_unused]] auto check_scope = m_log.Section("KIND_CheckNonInc");
    m_useUnrollingCache = true;

    while (true) {
        LOG_L(m_log, 1, "KIND Bound: ", m_k);

        if (CheckBaseCaseNonIncremental(m_k)) {
            return CheckResult::Unsafe;
        }

        bool ind_safe = CheckInductiveStepNonIncremental(m_k);
        if (ind_safe) {
            return CheckResult::Safe;
        }

        if (m_maxK != -1 && m_k >= m_maxK) {
            return CheckResult::Unknown;
        }

        ++m_k;
    }
}

bool KIND::CheckBaseCase() {
    [[maybe_unused]] auto base_scope = m_log.Section("KIND_Base");
    AddConstraintsK(m_baseSolver, m_k);

    Lit k_bad = GetBadK(m_k);
    LOG_L(m_log, 3, "Assumption: ", k_bad);
    bool sat = m_baseSolver->Solve(Cube{k_bad});
    if (sat) {
        m_cexSolver = m_baseSolver;
        return true;
    }

    m_baseSolver->AddClause({~k_bad});
    LOG_L(m_log, 3, "Add Clause: ", ~k_bad);
    return false;
}

bool KIND::CheckInductiveStep() {
    [[maybe_unused]] auto ind_scope = m_log.Section("KIND_Ind");
    AddConstraintsK(m_indSolver, m_k);

    Lit k_bad = GetBadK(m_k);
    LOG_L(m_log, 3, "Assumption: ", k_bad);
    bool sat = m_indSolver->Solve(Cube{k_bad});
    if (!sat) {
        return true;
    }

    std::vector<StatePairKey> equal_pairs = FindEqualStatePairs(m_indSolver, m_k);
    LOG_L(m_log, 2, "Inductive step SAT, equal state pairs found: ", equal_pairs.size());
    if (!equal_pairs.empty()) {
        AddStateDisequalities(m_indSolver, equal_pairs);
        LOG_L(m_log, 2, "Added lazy simple-path constraints: ", equal_pairs.size());
    }

    m_indSolver->AddClause({~k_bad});
    LOG_L(m_log, 3, "Add Clause: ", ~k_bad);
    return false;
}

bool KIND::CheckBaseCaseNonIncremental(int k) {
    [[maybe_unused]] auto base_scope = m_log.Section("KIND_BaseNonInc");
    auto solver = std::make_shared<SATSolver>(m_model, m_settings.solver);
    AddInitial(solver);
    for (int i = 0; i <= k; ++i) {
        AddClausesK(solver, i);
    }
    for (int i = 0; i <= k; ++i) {
        AddConstraintsK(solver, i);
    }
    solver->AddClause({GetBadK(k)});

    bool sat = solver->Solve();
    if (sat) {
        m_cexSolver = solver;
    }
    return sat;
}

bool KIND::CheckInductiveStepNonIncremental(int k) {
    [[maybe_unused]] auto ind_scope = m_log.Section("KIND_IndNonInc");
    auto solver = std::make_shared<SATSolver>(m_model, m_settings.solver);
    for (int i = 0; i <= k; ++i) {
        AddClausesK(solver, i);
    }
    for (int i = 0; i <= k; ++i) {
        AddConstraintsK(solver, i);
    }
    for (int i = 0; i < k; ++i) {
        solver->AddClause({~GetBadK(i)});
    }
    solver->AddClause({GetBadK(k)});
    ReplayBlockedPairsNonIncremental(solver, k);

    bool sat = solver->Solve();
    if (!sat) {
        return true;
    }

    std::vector<StatePairKey> equal_pairs = FindEqualStatePairs(solver, k);
    std::vector<StatePairKey> new_pairs;
    new_pairs.reserve(equal_pairs.size());
    for (const StatePairKey &pair : equal_pairs) {
        if (m_nonIncIndBlockedPairs.insert(pair).second) {
            new_pairs.push_back(pair);
        }
    }

    LOG_L(m_log, 2, "Non-incremental inductive step SAT, equal state pairs found: ", equal_pairs.size());
    if (!new_pairs.empty()) {
        LOG_L(m_log, 2, "Stored lazy simple-path constraints for future rebuilds: ", new_pairs.size());
    }
    return false;
}

void KIND::AdvanceBaseToNextK() {
    AddClausesK(m_baseSolver, m_baseAddedTransitions);
    ++m_baseAddedTransitions;
}

void KIND::AdvanceIndSolverToNextK() {
    AddClausesK(m_indSolver, m_indAddedTransitions);
    ++m_indAddedTransitions;
}

void KIND::AddClausesK(std::shared_ptr<SATSolver> solver, int k) {
    if (!m_useUnrollingCache) {
        std::vector<Clause> clauses;
        GetClausesK(k, clauses);
        for (const Clause &clause : clauses) {
            solver->AddClause(clause);
            LOG_L(m_log, 3, "Add Clause: ", CubeToStr(clause));
        }
        return;
    }

    const std::vector<Clause> &clauses = GetClausesKCached(k);
    for (const Clause &clause : clauses) {
        solver->AddClause(clause);
        LOG_L(m_log, 3, "Add Clause: ", CubeToStr(clause));
    }
}

void KIND::AddConstraintsK(std::shared_ptr<SATSolver> solver, int k) {
    if (!m_useUnrollingCache) {
        Cube constraints = GetConstraintsK(k);
        for (Lit c : constraints) {
            solver->AddClause({c});
            LOG_L(m_log, 3, "Add Clause: ", c);
        }
        return;
    }

    for (Lit c : GetConstraintsKCached(k)) {
        solver->AddClause({c});
        LOG_L(m_log, 3, "Add Clause: ", c);
    }
}

void KIND::AddInitial(std::shared_ptr<SATSolver> solver) {
    for (Lit lit : m_model.GetInitialState()) {
        solver->AddClause({lit});
    }
    solver->AddInitialClauses();
}

void KIND::AddStateDisequality(std::shared_ptr<SATSolver> solver, int i, int j) {
    auto &latches = m_model.GetModelLatches();
    if (latches.empty()) {
        solver->AddClause({});
        return;
    }

    Clause diff_clause;
    diff_clause.reserve(latches.size());

    for (Var latch : latches) {
        Lit li = m_model.EnsurePrimeK(MkLit(latch), i);
        Lit lj = m_model.EnsurePrimeK(MkLit(latch), j);
        Lit diff = MkLit(GetDiffVar(i, j, latch));

        solver->AddClause({li, lj, ~diff});
        solver->AddClause({~li, ~lj, ~diff});
        solver->AddClause({li, ~lj, diff});
        solver->AddClause({~li, lj, diff});

        diff_clause.push_back(diff);
    }

    solver->AddClause(diff_clause);
}

void KIND::AddStateDisequalities(std::shared_ptr<SATSolver> solver,
                                 const std::vector<StatePairKey> &pairs) {
    for (const StatePairKey &pair : pairs) {
        AddStateDisequality(solver, pair.i, pair.j);
    }
}

void KIND::ReplayBlockedPairsNonIncremental(std::shared_ptr<SATSolver> solver, int k) {
    size_t replayed = 0;
    for (const StatePairKey &pair : m_nonIncIndBlockedPairs) {
        if (pair.j <= k) {
            AddStateDisequality(solver, pair.i, pair.j);
            ++replayed;
        }
    }
    LOG_L(m_log, 2, "Replayed blocked non-incremental state pairs: ", replayed);
}

std::vector<KIND::StatePairKey> KIND::FindEqualStatePairs(const std::shared_ptr<SATSolver> &solver, int k) const {
    std::vector<StatePairKey> pairs;
    auto &latches = m_model.GetModelLatches();

    for (int i = 0; i < k; ++i) {
        for (int j = i + 1; j <= k; ++j) {
            bool equal = true;
            for (Var latch : latches) {
                Lit li = m_model.EnsurePrimeK(MkLit(latch), i);
                Lit lj = m_model.EnsurePrimeK(MkLit(latch), j);
                if (solver->GetModel(VarOf(li)) != solver->GetModel(VarOf(lj))) {
                    equal = false;
                    break;
                }
            }
            if (equal) {
                pairs.push_back(StatePairKey{i, j});
            }
        }
    }

    return pairs;
}

Var KIND::GetDiffVar(int i, int j, Var latch) {
    if (i > j) {
        std::swap(i, j);
    }

    DiffKey key{i, j, latch};
    auto it = m_diffVars.find(key);
    if (it != m_diffVars.end()) {
        return it->second;
    }

    Var diff_var = m_model.GetNewVar();
    m_diffVars.emplace(key, diff_var);
    return diff_var;
}

const std::vector<Clause> &KIND::GetClausesKCached(int k) {
    if (k >= static_cast<int>(m_transitionClausesCache.size())) {
        m_transitionClausesCache.resize(k + 1);
        m_transitionClausesReady.resize(k + 1, 0);
    }

    if (!m_transitionClausesReady[k]) {
        GetClausesK(k, m_transitionClausesCache[k]);
        m_transitionClausesReady[k] = 1;
    }

    return m_transitionClausesCache[k];
}

const Cube &KIND::GetConstraintsKCached(int k) {
    if (k >= static_cast<int>(m_constraintsCache.size())) {
        m_constraintsCache.resize(k + 1);
        m_constraintsReady.resize(k + 1, 0);
    }

    if (!m_constraintsReady[k]) {
        m_constraintsCache[k] = GetConstraintsK(k);
        m_constraintsReady[k] = 1;
    }

    return m_constraintsCache[k];
}

void KIND::GetClausesK(int k, std::vector<Clause> &clauses) {
    auto &original_clauses = m_model.GetSimpClauses();
    for (Clause &ori : original_clauses) {
        Clause cls_k;
        for (Lit v : ori) {
            cls_k.push_back(m_model.EnsurePrimeK(v, k));
        }
        clauses.push_back(cls_k);
    }
}

Lit KIND::GetBadK(int k) {
    return m_model.EnsurePrimeK(m_model.GetBad(), k);
}

Cube KIND::GetConstraintsK(int k) {
    Cube res;
    for (Lit c : m_model.GetConstraints()) {
        res.push_back(m_model.EnsurePrimeK(c, k));
    }
    return res;
}

} // namespace car
