#include "BMC.h"

#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace car {

namespace fs = std::filesystem;

BMC::BMC(Settings settings,
         Model &model,
         Log &log) : m_settings(settings),
                     m_model(model),
                     m_log(log) {
    State::num_inputs = model.GetNumInputs();
    State::num_latches = model.GetNumLatches();
    global_log = &m_log;
    m_k = 0;
    m_maxK = m_settings.bmcK;
    m_checkResult = CheckResult::Unknown;
    m_step = m_settings.bmcStep;
    m_clauses.clear();
}


CheckResult BMC::Run() {
    signal(SIGINT, SignalHandler);
    if (m_settings.bmcCnf) {
        CNFGen();
        m_log.PrintCustomStatistics();
        std::exit(EXIT_SUCCESS);
    }

    if (m_settings.solver == MCSATSolver::kissat) {
        if (CheckNonIncremental()) {
            m_checkResult = CheckResult::Unsafe;
        }
    } else {
        if (Check())
            m_checkResult = CheckResult::Unsafe;
    }

    m_log.PrintCustomStatistics();

    return m_checkResult;
}

std::vector<std::pair<Cube, Cube>> BMC::GetCexTrace() {
    assert(m_checkResult == CheckResult::Unsafe);
    std::vector<std::pair<Cube, Cube>> trace;

    trace.reserve(m_k + 1);
    for (int k = 0; k <= m_k; k++) {
        Cube inputs;
        for (auto i : m_model.GetModelInputs()) {
            Lit ip = m_model.EnsurePrimeK(MkLit(i), k);
            inputs.emplace_back(m_solver->GetModel(VarOf(ip)) == T_TRUE ? MkLit(i) : ~MkLit(i));
        }
        Cube latches;
        for (auto l : m_model.GetModelLatches()) {
            Lit lp = m_model.EnsurePrimeK(MkLit(l), k);
            latches.emplace_back(m_solver->GetModel(VarOf(lp)) == T_TRUE ? MkLit(l) : ~MkLit(l));
        }
        trace.emplace_back(pair<Cube, Cube>(inputs, latches));
    }

    return trace;
}


void BMC::CNFGen() {
    [[maybe_unused]] auto cnf_scope = m_log.Section("BMC_CNFGen");
    const int target_k = m_settings.bmcCnfK;
    LOG_L(m_log, 1, "Generate BMC CNF at bound: ", target_k);

    vector<Clause> clauses;

    for (Lit lit : m_model.GetInitialState()) {
        clauses.push_back(Clause{lit});
    }
    for (const Clause &clause : m_model.GetInitialClauses()) {
        clauses.push_back(clause);
    }

    for (int i = 0; i <= target_k; ++i) {
        GetClausesK(i, clauses);
    }

    for (int i = 0; i <= target_k; ++i) {
        for (Lit constraint : GetConstraintsK(i)) {
            clauses.push_back(Clause{constraint});
        }
    }

    clauses.push_back(Clause{GetBadK(target_k)});

    const string cnf_path = GetCNFPath(target_k);
    fs::path out_dir = fs::path(cnf_path).parent_path();
    if (!out_dir.empty()) {
        fs::create_directories(out_dir);
    }

    WriteDimacs(clauses, cnf_path);
    LOG_L(m_log, 0, "BMC CNF written to ", cnf_path);
}


bool BMC::Check() {
    [[maybe_unused]] auto check_scope = m_log.Section("BMC_Check");
    Init();

    while (true) {
        LOG_L(m_log, 1, "BMC Bound: ", m_k);

        vector<Clause> clauses;
        GetClausesK(m_k, clauses);

        // & T^k
        {
            [[maybe_unused]] auto clause_scope = m_log.Section("Add_Trans_Cls");
            for (int i = 0; i < clauses.size(); ++i) {
                m_solver->AddClause(clauses[i]);
                LOG_L(m_log, 3, "Add Clause: ", CubeToStr(clauses[i]));
            }
        }

        // assume( bad^k & cons^k )
        Lit k_bad = GetBadK(m_k);
        Cube assumptions;
        assumptions.push_back(k_bad);
        for (auto c : GetConstraintsK(m_k)) {
            assumptions.push_back(c);
        }
        LOG_L(m_log, 3, "Assumption: ", CubeToStr(assumptions));
        {
            [[maybe_unused]] auto sat_scope = m_log.Section("SAT_BMC_Inc");
            bool sat = m_solver->Solve(assumptions);
            if (sat) return true;
        }

        // & cons^k
        {
            [[maybe_unused]] auto clause_scope = m_log.Section("Add_Cons_Cls");
            for (auto c : GetConstraintsK(m_k)) {
                m_solver->AddClause({c});
                LOG_L(m_log, 3, "Add Clause: ", c);
            }
        }
        // & !bad^k
        {
            [[maybe_unused]] auto clause_scope = m_log.Section("Add_Prop_Cls");
            m_solver->AddClause({~k_bad});
            LOG_L(m_log, 3, "Add Clause: ", ~k_bad);
        }
        m_k++;
        if (m_maxK != -1 && m_k > m_maxK) return false;
    }
}

bool BMC::CheckNonIncremental() {
    [[maybe_unused]] auto check_scope = m_log.Section("BMC_CheckNonInc");
    // before Clause^k ConstraintsK(k) bad^k

    // before Clause^k Clause^(k+1) Clause^(k+2) ConstraintsK(k) ConstraintsK(k+1) ConstraintsK(k+2) (bad^(k)|bad^(k+1)|bad^(k+2))
    Clause bad_clause;
    bad_clause.reserve(m_step);
    // Pre-allocate m_step memory
    while (true) {
        Init();
        // add clauses before K unrollings to the Kissat solver
        {
            [[maybe_unused]] auto clause_scope = m_log.Section("Add_Init_Cls");
            for (int i = 0; i < m_clauses.size(); ++i) {
                m_solver->AddClause(m_clauses[i]);
                LOG_L(m_log, 
                    3, "Add Clause: ", CubeToStr(m_clauses[i]));
            }
        }
        bad_clause.clear();
        for (int s = 0; s < m_step; s++) {
            LOG_L(m_log, 1, "BMC Bound: ", m_k);

            vector<Clause> clauses;
            GetClausesK(m_k, clauses);

            // & T^k
            {
                [[maybe_unused]] auto clause_scope = m_log.Section("Add_Trans_Cls");
                for (int i = 0; i < clauses.size(); ++i) {
                    m_solver->AddClause(clauses[i]);
                    m_clauses.emplace_back(clauses[i]); // store for further use
                    LOG_L(m_log, 3, "Add Clause: ", CubeToStr(clauses[i]));
                }
            }

            Lit k_bad = GetBadK(m_k);

            bad_clause.push_back(k_bad);
            // m_Solver->AddClause({k_bad});
            LOG_L(m_log, 3, "Add Clause: ", k_bad);

            {
                [[maybe_unused]] auto clause_scope = m_log.Section("Add_Cons_Cls");
                for (auto c : GetConstraintsK(m_k)) {
                    m_solver->AddClause({c});
                    m_clauses.push_back({c}); // store for further use
                    LOG_L(m_log, 3, "Add Clause: ", c);
                }
            }

            Clause cl{~k_bad}; // store bad^k for
            m_clauses.emplace_back(cl);

            m_k++;
            if (m_maxK != -1 && m_k > m_maxK) {
                [[maybe_unused]] auto final_sat_scope = m_log.Section("Add_Bad_Cls");
                m_solver->AddClause(bad_clause);
                [[maybe_unused]] auto sat_scope = m_log.Section("SAT_BMC_NonInc");
                bool sat = m_solver->Solve();
                if (sat)
                    return true;
                else
                    return false;
            }
        }
        [[maybe_unused]] auto final_sat_scope = m_log.Section("Add_Bad_Cls");
        m_solver->AddClause(bad_clause);
        [[maybe_unused]] auto sat_scope = m_log.Section("SAT_BMC_NonInc");
        bool sat = m_solver->Solve();
        if (sat)
            return true;
    }
}


string BMC::GetCNFPath(int k) const {
    const fs::path aig_path(m_settings.aigFilePath);
    const string file_name = aig_path.stem().string() + ".bmc_k" + std::to_string(k) + ".cnf";
    return (fs::path(m_settings.bmcCnfDir) / file_name).string();
}


void BMC::WriteDimacs(const vector<Clause> &clauses, const string &path) const {
    vector<SignedVec> dimacs_clauses;
    dimacs_clauses.reserve(clauses.size());

    Var max_var = 0;
    for (const Clause &clause : clauses) {
        bool clause_is_true = false;
        SignedVec dimacs_clause;
        dimacs_clause.reserve(clause.size());

        for (Lit lit : clause) {
            if (IsConstTrue(lit)) {
                clause_is_true = true;
                break;
            }
            if (IsConstFalse(lit)) {
                continue;
            }

            dimacs_clause.push_back(ToSigned(lit));
            max_var = std::max(max_var, VarOf(lit));
        }

        if (!clause_is_true) {
            dimacs_clauses.push_back(std::move(dimacs_clause));
        }
    }

    std::ofstream cnf_file(path);
    if (!cnf_file.is_open()) {
        throw std::runtime_error("failed to open CNF output file: " + path);
    }

    cnf_file << "c simpleCAR BMC CNF" << std::endl;
    cnf_file << "c source " << m_settings.aigFilePath << std::endl;
    cnf_file << "c k " << m_settings.bmcCnfK << std::endl;
    cnf_file << "p cnf " << max_var << " " << dimacs_clauses.size() << std::endl;

    for (const SignedVec &clause : dimacs_clauses) {
        for (int lit : clause) {
            cnf_file << lit << " ";
        }
        cnf_file << "0" << std::endl;
    }
}


void BMC::Init() {
    [[maybe_unused]] auto init_scope = m_log.Section("BMC_Init");
    m_solver = make_shared<SATSolver>(m_model, m_settings.solver);

    // send initial state
    for (auto l : m_model.GetInitialState()) {
        m_solver->AddClause({l});
    }
    m_solver->AddInitialClauses();
}


void BMC::GetClausesK(int k, vector<Clause> &clauses) {
    auto &original_clauses = m_model.GetSimpClauses();
    for (int i = 0; i < original_clauses.size(); ++i) {
        Clause &ori = original_clauses[i];
        Clause cls_k;
        for (Lit v : ori) {
            cls_k.push_back(m_model.EnsurePrimeK(v, k));
        }
        clauses.push_back(cls_k);
    }
}


Lit BMC::GetBadK(int k) {
    return m_model.EnsurePrimeK(m_model.GetBad(), k);
}


Cube BMC::GetConstraintsK(int k) {
    Cube res;
    for (Lit c : m_model.GetConstraints()) {
        res.push_back(m_model.EnsurePrimeK(c, k));
    }
    return res;
}


} // namespace car
