#include "Model.h"


namespace car {

Model::Model(Settings settings) {
    m_settings = settings;
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
    CollectConstants();
    CollectConstraints();
    CollectBad();
    CollectInitialState();
    CollectNextValueMapping();
    m_innards = make_shared<set<int>>();
    if (m_settings.internalSignals) {
        CollectInnards();
    }
    CollectClauses();
    CollectCOIInputs();
    // CreateSimpSolver();
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
        if (m_aig->latches[i].reset == 0)
            m_initialState.push_back(-(m_aig->latches[i].lit >> 1));
        else if (m_aig->latches[i].reset == 1)
            m_initialState.push_back(m_aig->latches[i].lit >> 1);
    }
}


void Model::CollectNextValueMapping() {
    m_primeMaps.push_back(unordered_map<int, int>());
    unordered_map<int, int> &prime_map = m_primeMaps[0];

    for (int i = 0; i < m_aig->num_latches; i++) {
        int id = m_aig->latches[i].lit >> 1;

        int next_id = GetCarId(m_aig->latches[i].next);
        prime_map.insert(pair<int, int>(id, next_id));
        InsertIntoPreValueMapping(abs(next_id), (next_id > 0) ? id : -id);
    }
}

void Model::CollectClauses() {
    // get coi gates for transition relations, constraints, and property
    set<unsigned> coi_lits;

    for (int i = 0; i < m_aig->num_latches; i++) {
        coi_lits.insert(aiger_strip(m_aig->latches[i].next));
    }
    for (int i = 0; i < m_aig->num_constraints; i++) {
        coi_lits.insert(aiger_strip(m_aig->constraints[i].lit));
    }
    coi_lits.insert(abs(m_bad) * 2);

    vector<unsigned> coi_gates;
    for (int i = m_aig->num_ands - 1; i >= 0; i--) {
        aiger_and &a = m_aig->ands[i];
        if (coi_lits.find(a.lhs) != coi_lits.end()) {
            coi_gates.push_back(a.lhs);
            coi_lits.insert(aiger_strip(a.rhs0));
            coi_lits.insert(aiger_strip(a.rhs1));
        }
    }

    for (unsigned and_gate : coi_gates) {
        aiger_and *a = aiger_is_and(m_aig, and_gate);
        assert(a != nullptr);
        AddAndGateToClause(a);
    }

    // todo: add this constraint inside forward checker
    // if l1 and l2 have same prime l', then l1 and l2 shoud have same value, except the initial states
    // if (m_settings.forward) {
    //     int init = ++m_maxId;
    //     int cons = ++m_maxId;
    //     m_clauses.emplace_back(clause{init, cons});
    //     for (auto it = m_preValueOfLatchMap.begin(); it != m_preValueOfLatchMap.end(); it++) {
    //         if (it->second.size() > 1) {
    //             m_maxId++;
    //             for (int p : it->second) {
    //                 m_clauses.emplace_back(clause{-cons, -p, m_maxId});
    //                 m_clauses.emplace_back(clause{-cons, p, -m_maxId});
    //             }
    //         }
    //     }
    //     for (int i : m_initialState) {
    //         m_clauses.emplace_back(clause{-init, i});
    //     }
    // }

    // create clauses for true and false
    m_clauses.emplace_back(clause{m_trueId});
    m_clauses.emplace_back(clause{-m_falseId});
}


void Model::CollectCOIInputs() {
    set<unsigned> coi_lits;
    for (int i = 0; i < m_aig->num_constraints; i++) {
        coi_lits.insert(aiger_strip(m_aig->constraints[i].lit));
    }
    coi_lits.insert(abs(m_bad) * 2);

    vector<unsigned> coi_gates;
    for (int i = m_aig->num_ands - 1; i >= 0; i--) {
        aiger_and &a = m_aig->ands[i];
        if (coi_lits.find(a.lhs) != coi_lits.end()) {
            coi_gates.push_back(a.lhs);
            coi_lits.insert(aiger_strip(a.rhs0));
            coi_lits.insert(aiger_strip(a.rhs1));
        }
    }

    m_COIInputs = make_shared<vector<int>>();
    for (unsigned lit : coi_lits) {
        unsigned inputs_max = m_aig->num_inputs * 2;
        if (lit > 0 && lit <= inputs_max) {
            m_COIInputs->push_back(GetCarId(lit));
        } else if (lit > 0)
            break;
    }
}


void Model::AddAndGateToClause(const aiger_and *aa) {
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


inline void Model::InsertIntoPreValueMapping(const int key, const int value) {
    auto it = m_preValueOfLatchMap.find(key);
    if (it == m_preValueOfLatchMap.end()) {
        m_preValueOfLatchMap.insert(pair<int, vector<int>>(key, vector<int>{value}));
    } else {
        it->second.emplace_back(value);
    }
}


// Minisat::Lit getLit(shared_ptr<Minisat::SimpSolver> sslv, int id) {
//     int var = abs(id) - 1;
//     while (var >= sslv->nVars()) sslv->newVar();
//     return ((id > 0) ? Minisat::mkLit(var) : ~Minisat::mkLit(var));
// };


// void addClause(shared_ptr<Minisat::SimpSolver> sslv, const clause &cls) {
//     Minisat::vec<Minisat::Lit> literals;
//     for (int i = 0; i < cls.size(); ++i) {
//         literals.push(getLit(sslv, cls[i]));
//     }
//     bool result = sslv->addClause(literals);
//     assert(result != false);
// }


// void Model::CreateSimpSolver() {
//     m_simpSolver = make_shared<Minisat::SimpSolver>();
//     for (int i = 0; i < m_aig->num_inputs + m_aig->num_latches; i++) {
//         Minisat::Var nv = m_simpSolver->newVar();
//         m_simpSolver->setFrozen(nv, true);
//     }
//     for (int i = m_aig->num_inputs + 1; i < m_aig->num_inputs + m_aig->num_latches + 1; i++) {
//         Minisat::Var p = abs(GetPrime(i)) - 1;
//         while (p >= m_simpSolver->nVars()) {
//             m_simpSolver->newVar();
//         }
//         m_simpSolver->setFrozen(p, true);
//     }
//     for (int i = 0; i < m_constraints.size(); i++) {
//         Minisat::Var cons_var = abs(m_constraints[i]) - 1;
//         while (cons_var >= m_simpSolver->nVars()) m_simpSolver->newVar();
//         m_simpSolver->setFrozen(cons_var, true);
//     }
//     Minisat::Var bad_var = abs(m_bad) - 1;
//     while (bad_var >= m_simpSolver->nVars()) m_simpSolver->newVar();
//     m_simpSolver->setFrozen(bad_var, true);

//     if (m_settings.internalSignals) {
//         for (int i : *m_innards) {
//             Minisat::Var inn_var = abs(GetPrime(i)) - 1;
//             while (inn_var >= m_simpSolver->nVars()) m_simpSolver->newVar();
//             m_simpSolver->setFrozen(inn_var, true);
//         }
//     }

//     for (int i = 0; i < m_clauses.size(); i++) {
//         addClause(m_simpSolver, m_clauses[i]);
//     }
//     m_simpSolver->eliminate(true);
// }


// shared_ptr<Minisat::SimpSolver> Model::GetSimpSolver() {
//     return m_simpSolver;
// }


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
            InsertIntoPreValueMapping(m_maxId, GetCarId(aa.lhs));
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


} // namespace car