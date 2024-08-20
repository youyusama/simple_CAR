#include "BMC.h"

namespace car {

BMC::BMC(Settings settings,
         shared_ptr<AigerModel> model,
         shared_ptr<Log> log) : m_settings(settings),
                                m_model(model),
                                m_log(log) {
    State::numInputs = model->GetNumInputs();
    State::numLatches = model->GetNumLatches();
    GLOBAL_LOG = m_log;
    m_k = 0;
    m_maxK = m_settings.bmc_k;
}


bool BMC::Run() {
    signal(SIGINT, signalHandler);

    bool result = Check(m_model->GetBad());

    m_log->PrintStatistics();
    if (result) {
        m_log->L(0, "Unsafe");
        if (m_settings.witness)
            OutputCounterExample(m_model->GetBad());
    } else {
        m_log->L(0, "Unknown");
    }

    return true;
}


bool BMC::Check(int badId) {
    Init(badId);

    while (true) {
        m_log->L(1, "BMC Bound: ", m_k);

        shared_ptr<vector<clause>> clauses = make_shared<vector<clause>>();
        GetClausesK(m_k, clauses);

        // & T^k
        for (int i = 0; i < clauses->size(); ++i) {
            m_Solver->AddClause(clauses->at(i));
            m_log->L(3, "Add Clause: ", CubeToStr(make_shared<cube>(clauses->at(i))));
        }

        // assume( bad^k & cons^k )
        int k_bad = GetBadK(m_k);
        shared_ptr<cube> assumptions(new cube());
        assumptions->push_back(k_bad);
        for (auto c : GetConstraintsK(k_bad)) {
            assumptions->push_back(c);
        }
        m_log->L(3, "Assumption: ", CubeToStr(assumptions));
        m_log->Tick();
        bool sat = m_Solver->Solve(assumptions);
        m_log->StatMainSolver();
        if (sat) return true;

        // & cons^k
        for (auto c : GetConstraintsK(k_bad)) {
            m_Solver->AddClause({c});
            m_log->L(3, "Add Clause: ", c);
        }
        // & !bad^k
        m_Solver->AddClause({-k_bad});
        m_log->L(3, "Add Clause: ", -k_bad);
        m_k++;
        if (m_maxK != -1 && m_k > m_maxK) return false;
    }
}


void BMC::Init(int badId) {
    m_badId = badId;
    m_Solver = make_shared<CarSolver>();

    // send initial state
    for (auto l : m_model->GetInitialState()) {
        m_Solver->AddClause({l});
    }
}


void BMC::GetClausesK(int m_k, shared_ptr<vector<clause>> clauses) {
    auto &originalClauses = m_model->GetClauses();
    for (int i = 0; i < originalClauses.size(); ++i) {
        clause &ori = originalClauses[i];
        clause cls_k;
        for (int v : ori) {
            cls_k.push_back(m_model->GetPrimeK(v, m_k));
        }
        clauses->push_back(cls_k);
    }
}


int BMC::GetBadK(int m_k) {
    return m_model->GetPrimeK(m_badId, m_k);
}


vector<int> BMC::GetConstraintsK(int m_k) {
    vector<int> res;
    for (auto c : m_model->GetConstraints()) {
        res.push_back(m_model->GetPrimeK(c, m_k));
    }
    return res;
}


void BMC::OutputCounterExample(int bad) {
    // get outputfile
    auto startIndex = m_settings.aigFilePath.find_last_of("/");
    if (startIndex == string::npos) {
        startIndex = 0;
    } else {
        startIndex++;
    }
    auto endIndex = m_settings.aigFilePath.find_last_of(".");
    assert(endIndex != string::npos);
    string aigName = m_settings.aigFilePath.substr(startIndex, endIndex - startIndex);
    string cexPath = m_settings.outputDir + aigName + ".cex";
    std::ofstream cexFile;
    cexFile.open(cexPath);

    cexFile << "1" << endl
            << "b0" << endl;

    for (int i = 0; i < m_model->GetNumLatches(); i++) {
        int latch_id = m_model->GetNumInputs() + i + 1;
        cexFile << m_Solver->GetModelOfId(latch_id) ? "1" : "0";
    }
    cexFile << endl;
    for (int j = 0; j <= m_k; j++) {
        for (int i = 0; i < m_model->GetNumInputs(); i++) {
            int input_id = m_model->GetPrimeK(i + 1, j);
            cexFile << m_Solver->GetModelOfId(input_id) ? "1" : "0";
        }
        cexFile << endl;
    }

    cexFile << "." << endl;
    cexFile.close();
    return;
}

} // namespace car