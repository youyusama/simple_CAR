#include "Model.h"


namespace car {

Model::Model(Settings settings, shared_ptr<Log> log) : m_settings(settings),
                                                       m_log(log) {
    string aigFilePath = settings.aigFilePath;

    m_aig = aiger_init();
    aiger_open_and_read_from_file(m_aig, aigFilePath.c_str());
    if (aiger_error(m_aig)) {
        cout << "aiger parse error" << endl;
        exit(0);
    }
    int num_bad = GetNumBad();
    if (num_bad > 1) {
        cout << "aiger has more than one property to check" << endl;
        exit(0);
    } else if (num_bad == 0) {
        cout << "aiger has no safety property to check" << endl;
        exit(0);
    }
    if (!aiger_is_reencoded(m_aig)) {
        aiger_reencode(m_aig);
    }
    Init();
}


void Model::Init() {
    m_maxId = m_aig->maxvar + 2;
    m_trueId = m_maxId - 1;
    m_falseId = m_maxId;
    m_andGateStartId = m_aig->num_inputs + m_aig->num_latches + 1;
    CollectConstants();
    CollectConstraints();
    CollectBad();
    CollectInitialState();
    CollectNextValueMapping();
    CollectClauses();
    CollectCOIInputs();
    m_innards = make_shared<unordered_set<int>>();
    m_innardsVec = make_shared<vector<int>>();
    if (m_settings.internalSignals) {
        CollectInnards();
        CollectInnardsClauses();
    }
    SimplifyClauses();
    m_log->L(1, "Model initialized: ",
             m_aig->num_inputs, " inputs, ", m_aig->num_latches, " latches, ", m_aig->num_constraints, " constraints, ");
    m_log->L(1, "                   ", m_clauses.size(), " clauses , ", m_simpClauses.size(), " simplified clauses.");
}


void Model::CollectConstants() {
    for (int i = 0; i < m_aig->num_ands; ++i) {
        aiger_and &aa = m_aig->ands[i];
        if (IsTrue(aa.rhs0) && IsTrue(aa.rhs1)) {
            m_trues.insert(aa.lhs);
        } else if (IsFalse(aa.rhs0) || IsFalse(aa.rhs1)) {
            m_trues.insert(aa.lhs + 1);
        }
    }
    m_trues.insert(GetAigerLit(m_trueId));
    m_trues.insert(GetAigerLit(m_falseId) + 1);
}


void Model::CollectConstraints() {
    for (int i = 0; i < m_aig->num_constraints; ++i) {
        m_constraints.push_back(GetCarId(m_aig->constraints[i].lit));
    }
}


void Model::CollectBad() {
    for (int i = 0; i < m_aig->num_outputs; ++i) {
        m_bad = GetCarId(m_aig->outputs[i].lit);
    }
    for (int i = 0; i < m_aig->num_bad; ++i) {
        m_bad = GetCarId(m_aig->bad[i].lit);
    }
}


void Model::CollectInitialState() {
    for (int i = 0; i < m_aig->num_latches; ++i) {
        unsigned &lit = m_aig->latches[i].lit;
        unsigned &reset = m_aig->latches[i].reset;

        if (reset == 0)
            m_initialState.push_back(-(lit >> 1));
        else if (reset == 1)
            m_initialState.push_back(lit >> 1);
        else if (reset != lit && aiger_is_and(m_aig, reset)) {
            m_initialClauses.emplace_back(clause{GetCarId(lit), -GetCarId(reset)});
            m_initialClauses.emplace_back(clause{-GetCarId(lit), GetCarId(reset)});
        }
    }
}


void Model::CollectNextValueMapping() {
    m_primeMaps.push_back(unordered_map<int, int>());
    unordered_map<int, int> &prime_map = m_primeMaps[0];

    for (int i = 0; i < m_aig->num_latches; i++) {
        int id = m_aig->latches[i].lit >> 1;

        int next_id = GetCarId(m_aig->latches[i].next);
        prime_map.insert(pair<int, int>(id, next_id));
        m_preValueOfLatchMap[abs(next_id)].emplace_back((next_id > 0) ? id : -id);
    }
}


void Model::CollectClauses() {
    // get coi gates for transition relations, constraints, and property
    unordered_set<unsigned> coi_lits;
    for (int i = 0; i < m_aig->num_latches; i++)
        coi_lits.emplace(aiger_strip(m_aig->latches[i].next));
    for (int i = 0; i < m_aig->num_constraints; i++)
        coi_lits.emplace(aiger_strip(m_aig->constraints[i].lit));
    coi_lits.emplace(abs(m_bad) * 2);

    // tranverse and gates reversely
    for (int i = m_aig->num_ands - 1; i >= 0; i--) {
        aiger_and &a = m_aig->ands[i];
        if (coi_lits.find(a.lhs) != coi_lits.end()) {
            if (TryConvertXOR(a.lhs, coi_lits))
                continue;
            if (TryConvertITE(a.lhs, coi_lits))
                continue;
            ConvertAND(a.lhs, coi_lits);
        }
    }

    // create clauses for true and false
    m_clauses.emplace_back(clause{m_trueId});
    m_clauses.emplace_back(clause{-m_falseId});
}


bool Model::TryConvertXOR(const unsigned a, unordered_set<unsigned> &coi_lits) {
    aiger_and *aa = aiger_is_and(const_cast<aiger *>(m_aig), a);
    assert(aa != nullptr);

    unsigned a0 = aa->rhs0;
    unsigned a1 = aa->rhs1;
    if (!aiger_sign(a0) || !aiger_sign(a1)) return false;

    aiger_and *aa0 = aiger_is_and(const_cast<aiger *>(m_aig), a0);
    aiger_and *aa1 = aiger_is_and(const_cast<aiger *>(m_aig), a1);
    if (aa0 == nullptr || aa1 == nullptr) return false;

    unsigned a00 = aa0->rhs0;
    unsigned a01 = aa0->rhs1;
    unsigned a10 = aa1->rhs0;
    unsigned a11 = aa1->rhs1;

    if (a00 == aiger_not(a10) && a01 == aiger_not(a11)) {
        if (a00 == a01) return false;
        // is XOR
        m_clauses.emplace_back(clause{-GetCarId(a00), -GetCarId(a01), -GetCarId(a)});
        m_clauses.emplace_back(clause{GetCarId(a00), GetCarId(a01), -GetCarId(a)});
        m_clauses.emplace_back(clause{-GetCarId(a00), GetCarId(a01), GetCarId(a)});
        m_clauses.emplace_back(clause{GetCarId(a00), -GetCarId(a01), GetCarId(a)});

        coi_lits.emplace(aiger_strip(a00));
        coi_lits.emplace(aiger_strip(a01));

        m_dependencyMap[GetCarId(a)].emplace_back(abs(GetCarId(a00)));
        m_dependencyMap[GetCarId(a)].emplace_back(abs(GetCarId(a01)));

        return true;
    }
    return false;
}


bool Model::TryConvertITE(const unsigned a, unordered_set<unsigned> &coi_lits) {
    aiger_and *aa = aiger_is_and(const_cast<aiger *>(m_aig), a);
    assert(aa != nullptr);

    unsigned a0 = aa->rhs0;
    unsigned a1 = aa->rhs1;
    if (!aiger_sign(a0) || !aiger_sign(a1)) return false;

    aiger_and *aa0 = aiger_is_and(const_cast<aiger *>(m_aig), a0);
    aiger_and *aa1 = aiger_is_and(const_cast<aiger *>(m_aig), a1);
    if (aa0 == nullptr || aa1 == nullptr) return false;

    unsigned a00 = aa0->rhs0;
    unsigned a01 = aa0->rhs1;
    unsigned a10 = aa1->rhs0;
    unsigned a11 = aa1->rhs1;

    std::vector<unsigned> ite;
    if (a00 == aiger_not(a10)) {
        ite = {a00, aiger_not(a01), aiger_not(a11)};
    } else if (a00 == aiger_not(a11)) {
        ite = {a00, aiger_not(a01), aiger_not(a10)};
    } else if (a01 == aiger_not(a10)) {
        ite = {a01, aiger_not(a00), aiger_not(a11)};
    } else if (a01 == aiger_not(a11)) {
        ite = {a01, aiger_not(a00), aiger_not(a10)};
    } else {
        return false;
    }

    // is ITE
    m_clauses.emplace_back(clause{-GetCarId(ite[0]), -GetCarId(ite[1]), GetCarId(a)});
    m_clauses.emplace_back(clause{GetCarId(ite[0]), -GetCarId(ite[2]), GetCarId(a)});
    m_clauses.emplace_back(clause{-GetCarId(ite[0]), GetCarId(ite[1]), -GetCarId(a)});
    m_clauses.emplace_back(clause{GetCarId(ite[0]), GetCarId(ite[2]), -GetCarId(a)});

    coi_lits.emplace(aiger_strip(ite[0]));
    coi_lits.emplace(aiger_strip(ite[1]));
    coi_lits.emplace(aiger_strip(ite[2]));

    m_dependencyMap[GetCarId(a)].emplace_back(abs(GetCarId(ite[0])));
    m_dependencyMap[GetCarId(a)].emplace_back(abs(GetCarId(ite[1])));
    m_dependencyMap[GetCarId(a)].emplace_back(abs(GetCarId(ite[2])));

    return true;
}


void Model::ConvertAND(unsigned a, unordered_set<unsigned> &coi_lits) {
    aiger_and *aa = aiger_is_and(const_cast<aiger *>(m_aig), a);
    assert(aa != nullptr);

    m_clauses.emplace_back(clause{GetCarId(aa->lhs), -GetCarId(aa->rhs0), -GetCarId(aa->rhs1)});
    m_clauses.emplace_back(clause{-GetCarId(aa->lhs), GetCarId(aa->rhs0)});
    m_clauses.emplace_back(clause{-GetCarId(aa->lhs), GetCarId(aa->rhs1)});

    coi_lits.emplace(aiger_strip(aa->rhs0));
    coi_lits.emplace(aiger_strip(aa->rhs1));

    m_dependencyMap[GetCarId(aa->lhs)].emplace_back(abs(GetCarId(aa->rhs0)));
    m_dependencyMap[GetCarId(aa->lhs)].emplace_back(abs(GetCarId(aa->rhs1)));
}


void Model::CollectCOIInputs() {
    unordered_set<unsigned> coi_lits;
    for (int i = 0; i < m_aig->num_constraints; i++) {
        coi_lits.emplace(aiger_strip(m_aig->constraints[i].lit));
    }
    coi_lits.emplace(abs(m_bad) * 2);

    for (int i = m_aig->num_ands - 1; i >= 0; i--) {
        aiger_and &a = m_aig->ands[i];
        if (coi_lits.find(a.lhs) != coi_lits.end()) {
            coi_lits.emplace(aiger_strip(a.rhs0));
            coi_lits.emplace(aiger_strip(a.rhs1));
        }
    }

    m_COIInputs = make_shared<vector<int>>();
    unsigned inputs_end = m_aig->num_inputs * 2 + 1;
    for (unsigned lit = 2; lit < inputs_end; lit += 2) {
        if (coi_lits.find(lit) != coi_lits.end())
            m_COIInputs->emplace_back(GetCarId(lit));
    }
}


shared_ptr<cube> Model::GetCOIDomain(const shared_ptr<cube> c) {
    unordered_set<int> coi_vars;
    queue<int> todo_vars;
    for (int v : *c) {
        todo_vars.emplace(abs(v));
        coi_vars.emplace(abs(v));
    }
    while (!todo_vars.empty()) {
        int v = todo_vars.front();
        if (m_dependencyMap.find(v) != m_dependencyMap.end()) {
            for (int d : m_dependencyMap[v]) {
                if (coi_vars.find(d) == coi_vars.end()) {
                    todo_vars.emplace(d);
                    coi_vars.emplace(d);
                }
            }
        }
        todo_vars.pop();
    }

    shared_ptr<cube> domain = make_shared<cube>(coi_vars.begin(), coi_vars.end());
    domain->emplace_back(m_trueId);
    domain->emplace_back(m_falseId);
    return domain;
}


void Model::GetPreValueOfLatchMap(unordered_map<int, vector<int>> &map) {
    map = m_preValueOfLatchMap;
}


int Model::GetPrimeK(const int id, int k) {
    if (k == 0) return id;
    if (k >= m_primeMaps.size())
        m_primeMaps.push_back(unordered_map<int, int>());
    if (IsLatch(id)) return GetPrimeK(GetPrime(id), k - 1);

    unordered_map<int, int> &k_map = m_primeMaps[k - 1];
    unordered_map<int, int>::iterator it = k_map.find(abs(id));
    if (it != k_map.end())
        return id > 0 ? it->second : -(it->second);
    else {
        auto res = k_map.insert(pair<int, int>(abs(id), ++m_maxId));
        return id > 0 ? res.first->second : -(res.first->second);
    }
}


int Model::InnardsLogiclvlDFS(unsigned aig_id) {
    auto it = m_innards_lvl.find(aig_id / 2);
    if (it != m_innards_lvl.end())
        return it->second;
    aiger_and *aa = aiger_is_and(const_cast<aiger *>(m_aig), aiger_strip(aig_id));
    int lvl;
    if (aa) {
        int l_lvl = InnardsLogiclvlDFS(aa->rhs0);
        int r_lvl = InnardsLogiclvlDFS(aa->rhs1);
        if (l_lvl > r_lvl)
            lvl = l_lvl + 1;
        else
            lvl = r_lvl + 1;
    } else {
        lvl = 0;
    }
    m_innards_lvl.insert(pair<int, int>(aig_id / 2, lvl));
    return lvl;
}


void Model::CollectInnards() {
    for (int i = 0; i < m_aig->num_ands; ++i) {
        // input-free innard check
        aiger_and &aa = m_aig->ands[i];
        int l = GetCarId(aa.lhs);
        int r0 = GetCarId(aa.rhs0);
        int r1 = GetCarId(aa.rhs1);
        bool b0 = IsConstant(r0) || IsLatch(r0) || m_innards->find(abs(r0)) != m_innards->end();
        bool b1 = IsConstant(r1) || IsLatch(r1) || m_innards->find(abs(r1)) != m_innards->end();
        if (b0 && b1) {
            m_innards->emplace(l);
            InnardsLogiclvlDFS(aa.lhs);
        }
    }
    m_innardsVec->assign(m_innards->begin(), m_innards->end());
    sort(m_innardsVec->begin(), m_innardsVec->end());
}


void Model::CollectInnardsClauses() {
    for (int i = 0; i < m_aig->num_ands; ++i) {
        aiger_and &aa = m_aig->ands[i];
        if (m_innards->find(GetCarId(aa.lhs)) == m_innards->end())
            continue;

        // add clauses
        int pl, pr0, pr1;
        // primed left
        if (GetPrime(GetCarId(aa.lhs)) == 0) {
            m_maxId++;
            m_primeMaps[0].insert(pair<int, int>(GetCarId(aa.lhs), m_maxId));
            m_preValueOfLatchMap[m_maxId].emplace_back(GetCarId(aa.lhs));
        }
        pl = GetPrime(GetCarId(aa.lhs));
        // primed right 0
        if (aiger_is_latch(const_cast<aiger *>(m_aig), aiger_strip(aa.rhs0))) {
            pr0 = GetPrime(GetCarId(aa.rhs0));
        } else if (IsConstant(GetCarId(aa.rhs0))) { // constant
            if (IsTrue(aa.rhs0))
                pr0 = m_trueId;
            else
                pr0 = m_falseId;
        } else if (aiger_is_and(const_cast<aiger *>(m_aig), aiger_strip(aa.rhs0))) {
            assert(GetPrime(GetCarId(aa.rhs0)) != 0);
            pr0 = GetPrime(GetCarId(aa.rhs0));
        }
        // primed right 1
        if (aiger_is_latch(const_cast<aiger *>(m_aig), aiger_strip(aa.rhs1))) {
            pr1 = GetPrime(GetCarId(aa.rhs1));
        } else if (IsConstant(GetCarId(aa.rhs1))) { // constant
            if (IsTrue(aa.rhs1))
                pr1 = m_trueId;
            else
                pr1 = m_falseId;
        } else if (aiger_is_and(const_cast<aiger *>(m_aig), aiger_strip(aa.rhs1))) {
            assert(GetPrime(GetCarId(aa.rhs1)) != 0);
            pr1 = GetPrime(GetCarId(aa.rhs1));
        }

        m_clauses.emplace_back(vector<int>{pl, -pr0, -pr1});
        m_clauses.emplace_back(vector<int>{-pl, pr0});
        m_clauses.emplace_back(vector<int>{-pl, pr1});
    }
}


void Model::SimplifyClauses() {
    std::shared_ptr<CaDiCaL::Solver> solver = std::make_shared<CaDiCaL::Solver>();
    for (auto &c : m_clauses) {
        solver->clause(c);
    }
    // freeze variables
    for (int i = 1; i < m_andGateStartId; i++) solver->freeze(i);
    // freeze primed variables
    for (int i = m_aig->num_inputs + 1; i < m_andGateStartId; i++) solver->freeze(GetPrime(i));
    // freeze constraints
    for (int i : m_constraints) solver->freeze(i);
    if (m_settings.internalSignals) {
        for (int i : *m_innardsVec) {
            solver->freeze(i);
            solver->freeze(GetPrime(i));
        }
    }
    solver->freeze(m_trueId);
    solver->freeze(m_falseId);
    solver->freeze(m_bad);

    class carClauseIterator : public CaDiCaL::ClauseIterator {
      public:
        ~carClauseIterator() {}
        bool clause(const std::vector<int> &cls) {
            simp_clauses.emplace_back(cls);
            return true;
        }
        vector<std::vector<int>> &getClauses() {
            return simp_clauses;
        }

      private:
        vector<std::vector<int>> simp_clauses;
    };

    carClauseIterator it;
    solver->simplify();
    solver->traverse_clauses(it);
    // cout << "clauses: " << m_clauses.size() << endl;
    // cout << "simplified clauses: " << it.getClauses().size() << endl;
    m_simpClauses = it.getClauses();
}


} // namespace car