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
                       m_auxAddedTransitions(0),
                       m_auxAddedUniqueLevel(-1),
                       m_checkResult(CheckResult::Unknown),
                       m_auxMode(m_settings.fUniq ? AuxMode::Forward : AuxMode::Induction) {
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

void KIND::Witness() {
    if (m_checkResult == CheckResult::Unsafe) {
        OutputCounterExample();
    }
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
    m_auxAddedTransitions = 0;
    m_auxAddedUniqueLevel = -1;
    m_cexSolver.reset();
    m_diffVars.clear();
}

void KIND::InitBaseSolver() {
    m_baseSolver = std::make_shared<SATSolver>(m_model, m_settings.solver);
    AddInitial(m_baseSolver);
}

void KIND::InitAuxSolver() {
    m_auxSolver = std::make_shared<SATSolver>(m_model, m_settings.solver);
    if (m_auxMode == AuxMode::Forward) {
        AddInitial(m_auxSolver);
    }
}

CheckResult KIND::Check() {
    [[maybe_unused]] auto check_scope = m_log.Section("KIND_Check");
    InitBaseSolver();
    InitAuxSolver();

    while (true) {
        LOG_L(m_log, 1, "KIND Bound: ", m_k);

        AdvanceBaseToNextK();
        if (m_auxMode == AuxMode::Forward) {
            AdvanceForwardToNextK();
        } else {
            AdvanceInductionToNextK();
        }

        if (CheckBaseCase()) {
            return CheckResult::Unsafe;
        }

        bool aux_safe = (m_auxMode == AuxMode::Forward)
                            ? CheckForwardCondition()
                            : CheckInductiveStep();
        if (aux_safe) {
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

    while (true) {
        LOG_L(m_log, 1, "KIND Bound: ", m_k);

        if (CheckBaseCaseNonIncremental(m_k)) {
            return CheckResult::Unsafe;
        }

        bool aux_safe = (m_auxMode == AuxMode::Forward)
                            ? CheckForwardConditionNonIncremental(m_k)
                            : CheckInductiveStepNonIncremental(m_k);
        if (aux_safe) {
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
    AddConstraintsK(m_auxSolver, m_k);

    Lit k_bad = GetBadK(m_k);
    LOG_L(m_log, 3, "Assumption: ", k_bad);
    bool sat = m_auxSolver->Solve(Cube{k_bad});
    if (!sat) {
        return true;
    }

    m_auxSolver->AddClause({~k_bad});
    LOG_L(m_log, 3, "Add Clause: ", ~k_bad);
    return false;
}

bool KIND::CheckForwardCondition() {
    [[maybe_unused]] auto fwd_scope = m_log.Section("KIND_Fwd");
    AddConstraintsK(m_auxSolver, m_k);
    AddUniqueConstraintsK(m_auxSolver, m_k);
    return !m_auxSolver->Solve();
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

    return !solver->Solve();
}

bool KIND::CheckForwardConditionNonIncremental(int k) {
    [[maybe_unused]] auto fwd_scope = m_log.Section("KIND_FwdNonInc");
    auto solver = std::make_shared<SATSolver>(m_model, m_settings.solver);
    AddInitial(solver);
    for (int i = 0; i <= k; ++i) {
        AddClausesK(solver, i);
    }
    for (int i = 0; i <= k; ++i) {
        AddConstraintsK(solver, i);
    }
    AddUniqueConstraintsK(solver, k);

    return !solver->Solve();
}

void KIND::AdvanceBaseToNextK() {
    AddClausesK(m_baseSolver, m_baseAddedTransitions);
    ++m_baseAddedTransitions;
}

void KIND::AdvanceInductionToNextK() {
    AddClausesK(m_auxSolver, m_auxAddedTransitions);
    ++m_auxAddedTransitions;
}

void KIND::AdvanceForwardToNextK() {
    AddClausesK(m_auxSolver, m_auxAddedTransitions);
    ++m_auxAddedTransitions;
}

void KIND::AddClausesK(std::shared_ptr<SATSolver> solver, int k) {
    std::vector<Clause> clauses;
    GetClausesK(k, clauses);
    for (const Clause &clause : clauses) {
        solver->AddClause(clause);
        LOG_L(m_log, 3, "Add Clause: ", CubeToStr(clause));
    }
}

void KIND::AddConstraintsK(std::shared_ptr<SATSolver> solver, int k) {
    for (Lit c : GetConstraintsK(k)) {
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

void KIND::AddUniqueConstraintsK(std::shared_ptr<SATSolver> solver, int k) {
    if (m_auxMode == AuxMode::Forward && solver == m_auxSolver) {
        for (int level = m_auxAddedUniqueLevel + 1; level <= k; ++level) {
            for (int i = 0; i < level; ++i) {
                AddStateDisequality(solver, i, level);
            }
            m_auxAddedUniqueLevel = level;
        }
        return;
    }

    for (int j = 0; j <= k; ++j) {
        for (int i = 0; i < j; ++i) {
            AddStateDisequality(solver, i, j);
        }
    }
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

void KIND::OutputCounterExample() {
    assert(m_cexSolver != nullptr);

    auto start_index = m_settings.aigFilePath.find_last_of("/\\");
    if (start_index == string::npos) {
        start_index = 0;
    } else {
        ++start_index;
    }
    auto end_index = m_settings.aigFilePath.find_last_of(".");
    assert(end_index != string::npos);
    string aig_name = m_settings.aigFilePath.substr(start_index, end_index - start_index);
    string cex_path = m_settings.witnessOutputDir + aig_name + ".cex";

    std::ofstream cex_file(cex_path);
    cex_file << "1" << std::endl
             << "b0" << std::endl;

    for (int i = 0; i < m_model.GetNumLatches(); ++i) {
        int latch_id = m_model.GetNumInputs() + i + 1;
        cex_file << ((m_cexSolver->GetModel(latch_id) == T_TRUE) ? "1" : "0");
    }
    cex_file << std::endl;
    for (int j = 0; j <= m_k; ++j) {
        for (int i = 0; i < m_model.GetNumInputs(); ++i) {
            Lit input_id = m_model.EnsurePrimeK(MkLit(static_cast<Var>(i + 1)), j);
            cex_file << ((m_cexSolver->GetModel(VarOf(input_id)) == T_TRUE) ? "1" : "0");
        }
        cex_file << std::endl;
    }

    cex_file << "." << std::endl;
}

} // namespace car
