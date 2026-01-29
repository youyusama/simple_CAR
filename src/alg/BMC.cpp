#include "BMC.h"

namespace car {

BMC::BMC(Settings settings,
         Model &model,
         Log &log) : m_settings(settings),
                     m_model(model),
                     m_log(log) {
    State::numInputs = model.GetNumInputs();
    State::numLatches = model.GetNumLatches();
    GLOBAL_LOG = &m_log;
    m_k = 0;
    m_maxK = m_settings.bmcK;
    m_checkResult = CheckResult::Unknown;
    m_step = m_settings.bmc_step;
    m_clauses.clear();
}


CheckResult BMC::Run() {
    signal(SIGINT, signalHandler);
    if (m_settings.solver == MCSATSolver::kissat) {
        if (Check_nonincremental()) {
            m_checkResult = CheckResult::Unsafe;
        }
    } else {
        if (Check())
            m_checkResult = CheckResult::Unsafe;
    }

    m_log.PrintCustomStatistics();

    return m_checkResult;
}

void BMC::Witness() {
    if (m_checkResult == CheckResult::Unsafe) {
        OutputCounterExample();
    }
}


std::vector<std::pair<cube, cube>> BMC::GetCexTrace() {
    assert(m_checkResult == CheckResult::Unsafe);
    std::vector<std::pair<cube, cube>> trace;

    trace.reserve(m_k + 1);
    for (int k = 0; k <= m_k; k++) {
        cube inputs;
        for (auto i : m_model.GetModelInputs()) {
            auto ip = m_model.GetPrimeK(i, k);
            inputs.emplace_back(m_Solver->GetModel(ip) == t_True ? i : -i);
        }
        cube latches;
        for (auto l : m_model.GetModelLatches()) {
            auto lp = m_model.GetPrimeK(l, k);
            latches.emplace_back(m_Solver->GetModel(lp) == t_True ? l : -l);
        }
        trace.emplace_back(pair<cube, cube>(inputs, latches));
    }

    return trace;
}


bool BMC::Check() {
    [[maybe_unused]] auto checkScope = m_log.Section("BMC_Check");
    Init();

    while (true) {
        LOG_L(m_log, 1, "BMC Bound: ", m_k);

        vector<clause> clauses;
        GetClausesK(m_k, clauses);

        // & T^k
        {
            [[maybe_unused]] auto clauseScope = m_log.Section("Add_Trans_Cls");
            for (int i = 0; i < clauses.size(); ++i) {
                m_Solver->AddClause(clauses[i]);
                LOG_L(m_log, 3, "Add Clause: ", CubeToStr(clauses[i]));
            }
        }

        // assume( bad^k & cons^k )
        int k_bad = GetBadK(m_k);
        cube assumptions;
        assumptions.push_back(k_bad);
        for (auto c : GetConstraintsK(m_k)) {
            assumptions.push_back(c);
        }
        LOG_L(m_log, 3, "Assumption: ", CubeToStr(assumptions));
        {
            [[maybe_unused]] auto satScope = m_log.Section("SAT_BMC_Inc");
            bool sat = m_Solver->Solve(assumptions);
            if (sat) return true;
        }

        // & cons^k
        {
            [[maybe_unused]] auto clauseScope = m_log.Section("Add_Cons_Cls");
            for (auto c : GetConstraintsK(m_k)) {
                m_Solver->AddClause({c});
                LOG_L(m_log, 3, "Add Clause: ", c);
            }
        }
        // & !bad^k
        {
            [[maybe_unused]] auto clauseScope = m_log.Section("Add_Prop_Cls");
            m_Solver->AddClause({-k_bad});
            LOG_L(m_log, 3, "Add Clause: ", -k_bad);
        }
        m_k++;
        if (m_maxK != -1 && m_k > m_maxK) return false;
    }
}

bool BMC::Check_nonincremental() {
    [[maybe_unused]] auto checkScope = m_log.Section("BMC_CheckNonInc");
    // before clause^k ConstraintsK(k) bad^k

    // before clause^k clause^(k+1) clause^(k+2) ConstraintsK(k) ConstraintsK(k+1) ConstraintsK(k+2) (bad^(k)|bad^(k+1)|bad^(k+2))
    clause badClause;
    badClause.reserve(m_step);
    // Pre-allocate m_step memory
    while (true) {
        Init();
        // add clauses before K unrollings to the Kissat solver
        {
            [[maybe_unused]] auto clauseScope = m_log.Section("Add_Init_Cls");
            for (int i = 0; i < m_clauses.size(); ++i) {
                m_Solver->AddClause(m_clauses[i]);
                LOG_L(m_log, 
                    3, "Add Clause: ", CubeToStr(m_clauses[i]));
            }
        }
        badClause.clear();
        for (int s = 0; s < m_step; s++) {
            LOG_L(m_log, 1, "BMC Bound: ", m_k);

            vector<clause> clauses;
            GetClausesK(m_k, clauses);

            // & T^k
            {
                [[maybe_unused]] auto clauseScope = m_log.Section("Add_Trans_Cls");
                for (int i = 0; i < clauses.size(); ++i) {
                    m_Solver->AddClause(clauses[i]);
                    m_clauses.emplace_back(clauses[i]); // store for further use
                    LOG_L(m_log, 3, "Add Clause: ", CubeToStr(clauses[i]));
                }
            }

            int k_bad = GetBadK(m_k);

            badClause.push_back({k_bad});
            // m_Solver->AddClause({k_bad});
            LOG_L(m_log, 3, "Add Clause: ", k_bad);

            {
                [[maybe_unused]] auto clauseScope = m_log.Section("Add_Cons_Cls");
                for (auto c : GetConstraintsK(m_k)) {
                    m_Solver->AddClause({c});
                    m_clauses.push_back({c}); // store for further use
                    LOG_L(m_log, 3, "Add Clause: ", c);
                }
            }

            clause cl({-k_bad}); // store bad^k for
            m_clauses.emplace_back(cl);

            m_k++;
            if (m_maxK != -1 && m_k > m_maxK) {
                [[maybe_unused]] auto finalSatScope = m_log.Section("Add_Bad_Cls");
                m_Solver->AddClause(badClause);
                [[maybe_unused]] auto satScope = m_log.Section("SAT_BMC_NonInc");
                bool sat = m_Solver->Solve();
                if (sat)
                    return true;
                else
                    return false;
            }
        }
        [[maybe_unused]] auto finalSatScope = m_log.Section("Add_Bad_Cls");
        m_Solver->AddClause(badClause);
        [[maybe_unused]] auto satScope = m_log.Section("SAT_BMC_NonInc");
        bool sat = m_Solver->Solve();
        if (sat)
            return true;
    }
}


void BMC::Init() {
    [[maybe_unused]] auto initScope = m_log.Section("BMC_Init");
    m_Solver = make_shared<SATSolver>(m_model, m_settings.solver);

    // send initial state
    for (auto l : m_model.GetInitialState()) {
        m_Solver->AddClause({l});
    }
    m_Solver->AddInitialClauses();
}


void BMC::GetClausesK(int m_k, vector<clause> &clauses) {
    auto &originalClauses = m_model.GetSimpClauses();
    for (int i = 0; i < originalClauses.size(); ++i) {
        clause &ori = originalClauses[i];
        clause cls_k;
        for (int v : ori) {
            cls_k.push_back(m_model.GetPrimeK(v, m_k));
        }
        clauses.push_back(cls_k);
    }
}


int BMC::GetBadK(int m_k) {
    return m_model.GetPrimeK(m_model.GetBad(), m_k);
}


vector<int> BMC::GetConstraintsK(int m_k) {
    vector<int> res;
    for (auto c : m_model.GetConstraints()) {
        res.push_back(m_model.GetPrimeK(c, m_k));
    }
    return res;
}


void BMC::OutputCounterExample() {
    // get outputfile
    auto startIndex = m_settings.aigFilePath.find_last_of("/\\");
    if (startIndex == string::npos) {
        startIndex = 0;
    } else {
        startIndex++;
    }
    auto endIndex = m_settings.aigFilePath.find_last_of(".");
    assert(endIndex != string::npos);
    string aigName = m_settings.aigFilePath.substr(startIndex, endIndex - startIndex);
    string cexPath = m_settings.witnessOutputDir + aigName + ".cex";
    std::ofstream cexFile;
    cexFile.open(cexPath);

    cexFile << "1" << endl
            << "b0" << endl;

    for (int i = 0; i < m_model.GetNumLatches(); i++) {
        int latch_id = m_model.GetNumInputs() + i + 1;
        cexFile << ((m_Solver->GetModel(latch_id) == t_True) ? "1" : "0");
    }
    cexFile << endl;
    for (int j = 0; j <= m_k; j++) {
        for (int i = 0; i < m_model.GetNumInputs(); i++) {
            int input_id = m_model.GetPrimeK(i + 1, j);
            cexFile << ((m_Solver->GetModel(input_id) == t_True) ? "1" : "0");
        }
        cexFile << endl;
    }

    cexFile << "." << endl;
    cexFile.close();
    return;
}

} // namespace car
