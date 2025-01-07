#include "AigerModel.h"


namespace car {

AigerModel::AigerModel(Settings settings) {
    m_settings = settings;
    string aigFilePath = settings.aigFilePath;

    m_aig = aiger_init();
    aiger_open_and_read_from_file(m_aig, aigFilePath.c_str());
    if (aiger_error(m_aig)) {
        cout << "aiger error" << endl;
        exit(0);
    }
    if (GetNumBad() > 1) {
        cout << "aiger has more than one property to check" << endl;
        exit(0);
    }
    if (!aiger_is_reencoded(m_aig)) {
        aiger_reencode(m_aig);
    }
    Init();
}


void AigerModel::Init() {
    m_maxId = m_aig->maxvar + 2;
    m_trueId = m_maxId - 1;
    m_falseId = m_maxId;
    CollectConstants();
    CollectConstraints();
    CollectBad();
    CollectInitialState();
    CollectNextValueMapping();
    unordered_map<int, int> MapOfPrime = m_nextValueOfLatch;
    m_MapsOfLatchPrimeK.push_back(MapOfPrime);
    CollectClauses();
    CreateSimpSolver();
}


void AigerModel::CollectConstants() {
    for (int i = 0; i < m_aig->num_ands; ++i) {
        aiger_and &aa = m_aig->ands[i];
        if (IsTrue(aa.rhs0) && IsTrue(aa.rhs1)) {
            m_trues.insert(aa.lhs);
        } else if (IsFalse(aa.rhs0) || IsFalse(aa.rhs1)) {
            m_trues.insert(aa.lhs + 1);
        }
    }
}


void AigerModel::CollectConstraints() {
    for (int i = 0; i < m_aig->num_constraints; ++i) {
        if (!IsTrue(m_aig->constraints[i].lit))
            m_constraints.push_back(GetCarId(m_aig->constraints[i].lit));
    }
}


void AigerModel::CollectBad() {
    for (int i = 0; i < m_aig->num_outputs; ++i) {
        m_bad = GetCarId(m_aig->outputs[i].lit);
    }
    for (int i = 0; i < m_aig->num_bad; ++i) {
        m_bad = GetCarId(m_aig->bad[i].lit);
    }
}


void AigerModel::CollectInitialState() {
    for (int i = 0; i < m_aig->num_latches; ++i) {
        if (m_aig->latches[i].reset == 0)
            m_initialState.push_back(-(m_aig->latches[i].lit >> 1));
        else if (m_aig->latches[i].reset == 1)
            m_initialState.push_back(m_aig->latches[i].lit >> 1);
        else {
            // placeholder
        }
    }
}


void AigerModel::CollectNextValueMapping() {
    for (int i = 0; i < m_aig->num_latches; i++) {
        int var = m_aig->latches[i].lit >> 1;
        assert(var == m_aig->num_inputs + i + 1);

        // pay attention to the special case when nextVal = 0 or 1
        if (IsFalse(m_aig->latches[i].next)) {
            m_nextValueOfLatch.insert(pair<int, int>(var, m_falseId));
            InsertIntoPreValueMapping(m_falseId, var);
        } else if (IsTrue(m_aig->latches[i].next)) {
            m_nextValueOfLatch.insert(pair<int, int>(var, m_trueId));
            InsertIntoPreValueMapping(m_trueId, var);
        } else {
            int nextVal = GetCarId(m_aig->latches[i].next);
            m_nextValueOfLatch.insert(pair<int, int>(var, nextVal));
            InsertIntoPreValueMapping(abs(nextVal), (nextVal > 0) ? var : -var);
        }
    }
}

void AigerModel::CollectClauses() {
    // contraints, outputs and latches gates are stored in order,
    // as the need for start solver construction
    unordered_set<unsigned> exist_gates;
    vector<unsigned> gates;

    CollectNecessaryAndGatesFromConstraints(exist_gates, gates);
    for (auto it = gates.begin(); it != gates.end(); it++) {
        aiger_and *aa = aiger_is_and(m_aig, *it);
        AddAndGateToClause(aa);
    }

    // if l1 and l2 have same prime l', then l1 and l2 shoud have same value, except the initial states
    if (m_settings.forward) {
        int init = ++m_maxId;
        int cons = ++m_maxId;
        m_clauses.emplace_back(clause{init, cons});
        for (auto it = m_preValueOfLatch.begin(); it != m_preValueOfLatch.end(); it++) {
            if (it->second.size() > 1) {
                m_maxId++;
                for (int p : it->second) {
                    m_clauses.emplace_back(clause{-cons, -p, m_maxId});
                    m_clauses.emplace_back(clause{-cons, p, -m_maxId});
                }
            }
        }
        for (int i : m_initialState) {
            m_clauses.emplace_back(clause{-init, i});
        }
    }

    m_outputsStart = m_clauses.size();

    // create clauses for outputs
    gates.clear();
    CollectNecessaryAndGates(m_aig->outputs, m_aig->num_outputs, exist_gates, gates, false);
    for (auto it = gates.begin(); it != gates.end(); it++) {
        if (*it == 0) continue;
        aiger_and *aa = aiger_is_and(m_aig, *it);
        AddAndGateToClause(aa);
    }
    // create clauses for bad
    gates.clear();
    CollectNecessaryAndGates(m_aig->bad, m_aig->num_bad, exist_gates, gates, false);
    for (auto it = gates.begin(); it != gates.end(); it++) {
        if (*it == 0) continue;
        aiger_and *aa = aiger_is_and(m_aig, *it);
        AddAndGateToClause(aa);
    }

    m_latchesStart = m_clauses.size();

    // create clauses for latches
    vector<unsigned>().swap(gates);
    CollectNecessaryAndGates(m_aig->latches, m_aig->num_latches, exist_gates, gates, true);
    for (auto it = gates.begin(); it != gates.end(); it++) {
        if (*it == 0) continue;
        aiger_and *aa = aiger_is_and(m_aig, *it);
        AddAndGateToClause(aa);
    }

    // create clauses for true and false
    m_clauses.emplace_back(clause{m_trueId});
    m_clauses.emplace_back(clause{-m_falseId});
}


void AigerModel::CollectNecessaryAndGates(const aiger_symbol *as, const int as_size,
                                          unordered_set<unsigned> &exist_gates, vector<unsigned> &gates, bool next) {
    for (int i = 0; i < as_size; ++i) {
        aiger_and *aa;
        if (next)
            aa = IsAndGate(as[i].next);
        else {
            aa = IsAndGate(as[i].lit);
            if (aa == NULL) {
                if (IsTrue(as[i].lit)) {
                    m_bad = m_trueId;
                } else if (IsFalse(as[i].lit))
                    m_bad = m_falseId;
            }
        }
        FindAndGates(aa, exist_gates, gates);
    }
}


void AigerModel::CollectNecessaryAndGatesFromConstraints(unordered_set<unsigned> &exist_gates, vector<unsigned> &gates) {
    for (int i = 0; i < m_aig->num_constraints; ++i) {
        aiger_and *aa = IsAndGate(m_aig->constraints[i].lit);
        if (aa == NULL) {
            if (IsFalse(m_aig->constraints[i].lit)) {
                m_bad = m_falseId;
            }
        }
        FindAndGates(aa, exist_gates, gates);
    }
}


void AigerModel::FindAndGates(const aiger_and *aa, unordered_set<unsigned> &exist_gates, vector<unsigned> &gates) {
    if (aa == NULL || aa == nullptr) {
        return;
    }
    if (exist_gates.find(aa->lhs) != exist_gates.end()) {
        return;
    }
    gates.emplace_back(aa->lhs);
    exist_gates.emplace(aa->lhs);
    aiger_and *aa0 = IsAndGate(aa->rhs0);
    FindAndGates(aa0, exist_gates, gates);

    aiger_and *aa1 = IsAndGate(aa->rhs1);
    FindAndGates(aa1, exist_gates, gates);
}


void AigerModel::AddAndGateToClause(const aiger_and *aa) {
    if (IsTrue(aa->rhs0)) {
        m_clauses.emplace_back(clause{GetCarId(aa->lhs), -GetCarId(aa->rhs1)});
        m_clauses.emplace_back(clause{-GetCarId(aa->lhs), GetCarId(aa->rhs1)});
    } else if (IsTrue(aa->rhs1)) {
        m_clauses.emplace_back(clause{GetCarId(aa->lhs), -GetCarId(aa->rhs0)});
        m_clauses.emplace_back(clause{-GetCarId(aa->lhs), GetCarId(aa->rhs0)});
    } else {
        m_clauses.emplace_back(clause{GetCarId(aa->lhs), -GetCarId(aa->rhs0), -GetCarId(aa->rhs1)});
        m_clauses.emplace_back(clause{-GetCarId(aa->lhs), GetCarId(aa->rhs0)});
        m_clauses.emplace_back(clause{-GetCarId(aa->lhs), GetCarId(aa->rhs1)});
    }
}


inline void AigerModel::InsertIntoPreValueMapping(const int key, const int value) {
    auto it = m_preValueOfLatch.find(key);
    if (it == m_preValueOfLatch.end()) {
        m_preValueOfLatch.insert(pair<int, vector<int>>(key, vector<int>{value}));
    } else {
        it->second.emplace_back(value);
    }
}


inline aiger_and *AigerModel::IsAndGate(const unsigned lit) {
    if (!IsTrue(lit) && !IsFalse(lit))
        return aiger_is_and(m_aig, aiger_strip(lit));
    return nullptr;
}


Lit getLit(shared_ptr<SimpSolver> sslv, int id) {
    int var = abs(id) - 1;
    while (var >= sslv->nVars()) sslv->newVar();
    return ((id > 0) ? mkLit(var) : ~mkLit(var));
};


void addClause(shared_ptr<SimpSolver> sslv, const clause &cls) {
    vec<Lit> literals;
    for (int i = 0; i < cls.size(); ++i) {
        literals.push(getLit(sslv, cls[i]));
    }
    bool result = sslv->addClause(literals);
    assert(result != false);
}


void AigerModel::CreateSimpSolver() {
    m_simpSolver = make_shared<SimpSolver>();
    for (int i = 0; i < m_aig->num_inputs + m_aig->num_latches; i++) {
        Var nv = m_simpSolver->newVar();
        m_simpSolver->setFrozen(nv, true);
    }
    for (int i = m_aig->num_inputs + 1; i < m_aig->num_inputs + m_aig->num_latches + 1; i++) {
        Var p = abs(GetPrime(i)) - 1;
        while (p >= m_simpSolver->nVars()) {
            m_simpSolver->newVar();
        }
        m_simpSolver->setFrozen(p, true);
    }
    for (int i = 0; i < m_constraints.size(); i++) {
        Var cons_var = abs(m_constraints[i]) - 1;
        while (cons_var >= m_simpSolver->nVars()) m_simpSolver->newVar();
        m_simpSolver->setFrozen(cons_var, true);
    }
    Var bad_var = abs(m_bad) - 1;
    while (bad_var >= m_simpSolver->nVars()) m_simpSolver->newVar();
    m_simpSolver->setFrozen(bad_var, true);

    for (int i = 0; i < m_clauses.size(); i++) {
        addClause(m_simpSolver, m_clauses[i]);
    }
    m_simpSolver->eliminate(true);
}


shared_ptr<SimpSolver> AigerModel::GetSimpSolver() {
    return m_simpSolver;
}


void AigerModel::GetPreValueOfLatchMap(unordered_map<int, vector<int>> &map) {
    map = m_preValueOfLatch;
}


int AigerModel::GetPrimeK(const int id, int k) {
    if (k == 0) return id;
    if (k - 1 >= m_MapsOfLatchPrimeK.size())
        m_MapsOfLatchPrimeK.push_back(unordered_map<int, int>());
    if (IsLatch(id)) return GetPrimeK(GetPrime(id), k - 1);

    unordered_map<int, int> &k_map = m_MapsOfLatchPrimeK[k - 1];
    unordered_map<int, int>::iterator it = k_map.find(abs(id));
    if (it != k_map.end())
        return id > 0 ? it->second : -(it->second);
    else {
        auto res = k_map.insert(pair<int, int>(abs(id), ++m_maxId));
        return id > 0 ? res.first->second : -(res.first->second);
    }
}


void AigerModel::GenerateBMCLEC() {
    int bound_k = m_settings.bmc_k;

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
    string outputPath = m_settings.outputDir + aigName + "." + to_string(bound_k) + ".aag";

    // circuit
    aiger *lec_aig = aiger_init();
    lec_aig->maxvar = m_aig->maxvar + 1;
    // free latches as inputs
    for (unsigned i = 0; i < m_aig->num_latches; i++) {
        aiger_symbol &latch = m_aig->latches[i];
        if (latch.reset != 0 && latch.reset != 1) {
            aiger_add_input(lec_aig, latch.lit, "");
        }
    }
    for (int k = 0; k <= bound_k; k++) {
        // inputs
        for (unsigned i = 0; i < m_aig->num_inputs; i++) {
            aiger_symbol &input = m_aig->inputs[i];
            unsigned input_lit_k = GetLitPrimeK(lec_aig, input.lit, k);
            // cout << "add input " << input.lit << " " << input_lit_k << endl;
            aiger_add_input(lec_aig, input_lit_k, "");
        }
    }
    for (int k = 0; k <= bound_k; k++) {
        // and gates
        for (unsigned i = 0; i < m_aig->num_ands; i++) {
            aiger_and &gate = m_aig->ands[i];
            unsigned rhs0_lit_k = GetLitPrimeK(lec_aig, gate.rhs0, k);
            unsigned rhs1_lit_k = GetLitPrimeK(lec_aig, gate.rhs1, k);
            unsigned lhs_lit_k = GetLitPrimeK(lec_aig, gate.lhs, k);
            // cout << "-" << endl;
            // cout << "add and " << gate.lhs << " " << gate.rhs0 << " " << gate.rhs1 << endl;
            // cout << "add and " << lhs_lit_k << " " << rhs0_lit_k << " " << rhs1_lit_k << endl;
            aiger_add_and(lec_aig, lhs_lit_k, rhs0_lit_k, rhs1_lit_k);
        }
    }
    aiger_symbol &output = m_aig->outputs[0];
    unsigned output_lit_k = GetLitPrimeK(lec_aig, output.lit, bound_k);
    aiger_add_output(lec_aig, output_lit_k, "");

    aiger_reencode(lec_aig);
    aiger_open_and_write_to_file(lec_aig, outputPath.c_str());
}


unsigned AigerModel::GetLitPrimeK(aiger *aig, const unsigned lit, const int k) {
    if (lit == 0 || lit == 1) return lit;
    aiger_symbol *latch = aiger_is_latch(m_aig, lit);
    if (k == 0) {
        if (latch != nullptr) {
            return latch->reset;
        } else {
            return lit;
        }
    }
    if (k - 1 >= m_MapsOfLitPrimeK.size())
        m_MapsOfLitPrimeK.push_back(unordered_map<unsigned, unsigned>());
    if (latch != nullptr) {
        return GetLitPrimeK(aig, aiger_sign(lit) == 0 ? latch->next : aiger_not(latch->next), k - 1);
    }

    unordered_map<unsigned, unsigned> &k_map = m_MapsOfLitPrimeK[k - 1];
    unordered_map<unsigned, unsigned>::iterator it = k_map.find(aiger_strip(lit));
    if (it != k_map.end())
        return aiger_sign(lit) == 0 ? it->second : aiger_not(it->second);
    else {
        unsigned new_lit = (++m_aig->maxvar) * 2;
        auto res = k_map.insert(pair<unsigned, unsigned>(aiger_strip(lit), new_lit));
        return aiger_sign(lit) == 0 ? res.first->second : aiger_not(res.first->second);
    }
}


} // namespace car