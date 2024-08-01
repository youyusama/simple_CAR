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

    // create clauses for constraints
    for (int cons : m_constraints) {
        m_clauses.emplace_back(clause{cons});
    }

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

} // namespace car