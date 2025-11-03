#include "Model.h"
#include <bitset>


namespace car {

int EquivalenceManager::Find(int a) {
    int sign = (a > 0) ? 1 : -1;
    int key = abs(a);

    auto root_info = FindRootRecursive(key);
    return root_info.first * root_info.second * sign;
}


void EquivalenceManager::AddEquivalence(int a, int b) {
    int root_a = Find(a);
    int root_b = Find(b);
    // alreadly equivalent
    if (root_a == root_b) return;

    int key_a = abs(root_a);
    int key_b = abs(root_b);

    // merge two groups
    if (key_a < key_b) {
        if (b > 0)
            m_equivalenceMap[key_b] = root_a;
        else
            m_equivalenceMap[key_b] = -root_a;
    } else {
        if (a > 0)
            m_equivalenceMap[key_a] = root_b;
        else
            m_equivalenceMap[key_a] = -root_b;
    }
}


pair<int, int> EquivalenceManager::FindRootRecursive(int key) {
    auto it = m_equivalenceMap.find(key);
    if (it == m_equivalenceMap.end()) {
        return {key, 1};
    }

    int next_id = it->second;
    int next_key = abs(next_id);
    assert(key != next_key);
    int sign = (next_id > 0) ? 1 : -1;

    auto root_info = FindRootRecursive(next_key);

    m_equivalenceMap[key] = root_info.first * root_info.second * sign;

    return {root_info.first, root_info.second * sign};
}


Model::Model(Settings settings, shared_ptr<Log> log) : m_settings(settings),
                                                       m_log(log) {
    // load aiger
    string aigFilePath = settings.aigFilePath;
    m_aiger = shared_ptr<aiger>(aiger_init(), aigerDeleter);
    aiger_open_and_read_from_file(m_aiger.get(), aigFilePath.c_str());
    if (aiger_error(m_aiger.get())) {
        cout << "aiger parse error" << endl;
        exit(0);
    }
    if (!aiger_is_reencoded(m_aiger.get())) {
        aiger_reencode(m_aiger.get());
    }

    // create circuit graph
    m_circuitGraph = make_shared<CircuitGraph>(m_aiger);

    // multiple bad to check
    int num_bad = m_circuitGraph->bad.size();
    if (num_bad > 1) {
        m_log->L(0, "aiger has more than one safety property to check.");
        exit(0);
    } else if (num_bad == 0) {
        m_log->L(0, "aiger has no safety property to check.");
        exit(0);
    }

    m_log->L(1, "Model initialized: ",
             m_circuitGraph->numInputs, " inputs, ", m_circuitGraph->numLatches, " latches, ",
             m_circuitGraph->numAnds, " gates, ", m_circuitGraph->numConstraints, " constraints.");
    m_log->L(1, "COI Refined Model: ",
             m_circuitGraph->modelInputs.size(), " inputs, ", m_circuitGraph->modelLatches.size(), " latches, ", m_circuitGraph->modelGates.size(), " gates.");
    m_maxId = m_circuitGraph->trueId;

    // try to find equivalences
    m_equivalenceManager = make_shared<EquivalenceManager>();
    if (m_settings.eq == 1) {
        SimplifyModelByTernarySimulation();
        ApplyEquivalence();
        UpdateDependencyMap();
        SimplifyModelByRandomSimulation();
    } else if (m_settings.eq == 2) {
        SimplifyModelByTernarySimulation();
    } else if (m_settings.eq == 3) {
        SimplifyModelByRandomSimulation();
    }

    // apply the equivalences to the circuit graph
    ApplyEquivalence();

    // update dependency map
    UpdateDependencyMap();

    // initial state
    CollectInitialState();

    // bad property
    m_bad = m_circuitGraph->bad[0];

    // prime variable mapping
    CollectNextValueMapping();

    // internal signals
    if (m_settings.internalSignals) {
        CollectInnards();
    }

    // transform to CNF
    CollectClauses();
    SimplifyClauses();

    // cout << "model latches:" << endl;
    // for (auto l : m_circuitGraph->modelLatches)
    //     cout << l << " ";

    // cout << "model gates:" << endl;
    // for (auto g : m_circuitGraph->modelGates)
    //     cout << g << " ";

    // cout << "model inputs:" << endl;
    // for (auto i : m_circuitGraph->modelInputs)
    //     cout << i << " ";
    // cout << "property coi inputs:" << endl;
    // for (auto i : m_circuitGraph->propertyCOIInputs)
    //     cout << i << " ";

    // cout << "clauses" << endl;
    // for (auto c : m_clauses) {
    //     for (auto l : c) {
    //         cout << l << " ";
    //     }
    //     cout << endl;
    // }

    m_log->L(1, "Model reduced: ",
             m_circuitGraph->modelInputs.size(), " inputs, ", m_circuitGraph->modelLatches.size(), " latches, ", m_circuitGraph->modelGates.size(), " gates.");
    m_log->L(1, "Transformed model: ", m_clauses.size(), " clauses, ", m_simpClauses.size(), " simplified clauses.");
}


void Model::ApplyEquivalence() {
    if (m_equivalenceManager->Size() == 0) return;

    // refine the model by equivalence
    for (auto it = m_circuitGraph->modelLatches.begin(); it != m_circuitGraph->modelLatches.end();) {
        m_circuitGraph->latchResetMap[*it] = m_equivalenceManager->Find(m_circuitGraph->latchResetMap[*it]);
        m_circuitGraph->latchNextMap[*it] = m_equivalenceManager->Find(m_circuitGraph->latchNextMap[*it]);

        if (m_equivalenceManager->HasEquivalence(*it)) {
            it = m_circuitGraph->modelLatches.erase(it);
        } else
            it++;
    }
    for (auto it = m_circuitGraph->modelGates.begin(); it != m_circuitGraph->modelGates.end();) {
        if (m_equivalenceManager->HasEquivalence(*it)) {
            it = m_circuitGraph->modelGates.erase(it);
        } else {
            auto &gate = m_circuitGraph->gatesMap[*it];
            for (int i = 0; i < gate.fanins.size(); i++) {
                gate.fanins[i] = m_equivalenceManager->Find(gate.fanins[i]);
            }
            it++;
        }
    }
    for (int i = 0; i < m_circuitGraph->bad.size(); i++) {
        m_circuitGraph->bad[i] = m_equivalenceManager->Find(m_circuitGraph->bad[i]);
    }

    for (int i = 0; i < m_circuitGraph->constraints.size(); i++) {
        m_circuitGraph->constraints[i] = m_equivalenceManager->Find(m_circuitGraph->constraints[i]);
    }

    m_circuitGraph->COIRefine();
}


void Model::UpdateDependencyMap() {
    m_dependencyMap.clear();
    for (int i = m_circuitGraph->modelGates.size() - 1; i >= 0; i--) {
        int g = m_circuitGraph->modelGates[i];
        for (int fanin : m_circuitGraph->gatesMap[g].fanins) {
            // dependency
            assert(abs(fanin) <= g);
            m_dependencyMap[g].emplace(abs(fanin));
        }
    }
}


void Model::CollectInitialState() {
    for (auto l : m_circuitGraph->modelLatches) {
        int reset = m_circuitGraph->latchResetMap[l];

        if (reset == m_equivalenceManager->Find(TrueId())) {
            m_initialState.push_back(l);
        } else if (reset == -m_equivalenceManager->Find(TrueId())) {
            m_initialState.push_back(-l);
        } else if (reset != l && IsAnd(reset)) {
            m_initialClauses.emplace_back(clause{l, -reset});
            m_initialClauses.emplace_back(clause{-l, reset});
        }
    }
}


void Model::CollectNextValueMapping() {
    // reset
    m_maxId = m_circuitGraph->trueId;
    m_primeMaps.clear();
    m_primeMaps.push_back(unordered_map<int, int>());

    unordered_map<int, int> &prime_map = m_primeMaps[0];

    for (auto l : m_circuitGraph->latches) {
        int next = m_circuitGraph->latchNextMap[l];
        prime_map[l] = next;
    }
}


void Model::CollectClauses() {
    m_clauses.clear();
    for (int g_id : m_circuitGraph->modelGates) {
        auto g = m_circuitGraph->gatesMap[g_id];
        int fanout = g.fanout;
        int fanin0 = g.fanins[0];
        int fanin1 = g.fanins[1];
        if (g.gateType == CircuitGate::GateType::AND) {
            m_clauses.emplace_back(clause{fanout, -fanin0, -fanin1});
            m_clauses.emplace_back(clause{-fanout, fanin0});
            m_clauses.emplace_back(clause{-fanout, fanin1});
        } else if (g.gateType == CircuitGate::GateType::XOR) {
            m_clauses.emplace_back(clause{fanout, -fanin0, fanin1});
            m_clauses.emplace_back(clause{fanout, fanin0, -fanin1});
            m_clauses.emplace_back(clause{-fanout, fanin0, fanin1});
            m_clauses.emplace_back(clause{-fanout, -fanin0, -fanin1});
        } else if (g.gateType == CircuitGate::GateType::ITE) {
            int fanin2 = g.fanins[2];
            m_clauses.emplace_back(clause{fanout, -fanin0, -fanin1});
            m_clauses.emplace_back(clause{fanout, fanin0, -fanin2});
            m_clauses.emplace_back(clause{-fanout, -fanin0, fanin1});
            m_clauses.emplace_back(clause{-fanout, fanin0, fanin2});
        }
    }
    int trueId = m_equivalenceManager->Find(TrueId());
    m_clauses.emplace_back(clause{trueId});
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
                    todo_vars.emplace(abs(d));
                    coi_vars.emplace(abs(d));
                }
            }
        }
        todo_vars.pop();
    }

    shared_ptr<cube> domain = make_shared<cube>(coi_vars.begin(), coi_vars.end());
    domain->emplace_back(abs(m_equivalenceManager->Find(TrueId())));
    return domain;
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
        auto res = k_map.insert(pair<int, int>(abs(id), GetNewId()));
        return id > 0 ? res.first->second : -(res.first->second);
    }
}


int Model::InnardsLogiclvlDFS(int id) {
    auto it = m_innards_lvl.find(id);
    if (it != m_innards_lvl.end())
        return it->second;
    int lvl = 0;
    if (IsAnd(id)) {
        auto gate = m_circuitGraph->gatesMap[id];
        for (auto fanin : gate.fanins) {
            int fanin_lvl = InnardsLogiclvlDFS(abs(fanin));
            if (fanin_lvl + 1 > lvl)
                lvl = fanin_lvl + 1;
        }
    } else {
        lvl = 0;
    }
    m_innards_lvl.insert(pair<int, int>(id, lvl));
    return lvl;
}


void Model::CollectInnards() {
    for (int i = 0; i < m_circuitGraph->modelGates.size(); i++) {
        int g = m_circuitGraph->modelGates[i];

        // decide whether the gate is an innard
        bool is_innard = true;
        for (int fanin : m_circuitGraph->gatesMap[g].fanins) {
            bool b = IsConstant(fanin) ||
                     IsLatch(fanin) ||
                     m_innards.find(abs(fanin)) != m_innards.end();
            is_innard &= b;
        }

        // the gate is an innard
        if (is_innard) {
            m_innards.emplace(g);
            InnardsLogiclvlDFS(g);

            // build a new gate
            CircuitGate gate(m_circuitGraph->gatesMap[g]);
            if (GetPrime(g) == 0) {
                m_primeMaps[0].insert(pair<int, int>(g, GetNewId()));
            }
            int p_fanout = GetPrime(g);
            gate.fanout = p_fanout;
            assert(p_fanout > 0);

            for (int i = 0; i < gate.fanins.size(); i++) {
                int fanin = gate.fanins[i];
                int p_fanin;
                if (IsLatch(fanin)) {
                    p_fanin = GetPrime(fanin);
                } else if (IsConstant(fanin)) {
                    if (IsTrue(fanin))
                        p_fanin = TrueId();
                    else
                        p_fanin = -TrueId();
                } else if (IsAnd(fanin)) {
                    assert(GetPrime(fanin) != 0);
                    p_fanin = GetPrime(fanin);
                }
                gate.fanins[i] = p_fanin;
            }
            m_circuitGraph->gatesMap[p_fanout] = gate;
        }
    }
    for (int g_id : m_innards) m_circuitGraph->modelGates.emplace_back(GetPrime(g_id));
    m_innardsVec.assign(m_innards.begin(), m_innards.end());
    sort(m_innardsVec.begin(), m_innardsVec.end());
}


void Model::SimplifyClauses() {
    std::shared_ptr<CaDiCaL::Solver> solver = std::make_shared<CaDiCaL::Solver>();
    for (auto &c : m_clauses) {
        solver->clause(c);
    }
    // freeze variables
    for (auto v : m_circuitGraph->modelInputs) solver->freeze(v);
    for (auto v : m_circuitGraph->modelLatches) {
        solver->freeze(v);
        solver->freeze(GetPrime(v));
    }
    // freeze constraints
    for (int i : m_circuitGraph->constraints) solver->freeze(i);
    if (m_settings.internalSignals) {
        for (int i : m_innardsVec) {
            solver->freeze(i);
            solver->freeze(GetPrime(i));
        }
    }
    solver->freeze(TrueId());
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


bool Model::SimplifyModelByTernarySimulation() {
    m_log->L(1, "Simplify model by ternary simulation.");

    m_log->Tick();
    TernarySimulator simulator(m_circuitGraph, m_log);
    simulator.simulate(250);
    if (!simulator.isCycleReached()) return false;
    m_log->L(1, "Simulation takes ", m_log->Tock(), " seconds.");

    // find equivalent latches
    unordered_map<string, vector<int>> signaturesVariablesMap;
    EncodeStatesToSignatuers(simulator.getStates(), signaturesVariablesMap);
    int eq_counter = 0;

    // signatures to equivalent latches
    for (auto &s : signaturesVariablesMap) {
        if (s.second.size() > 1) {
            // the neg version of variables is processed
            if (m_equivalenceManager->HasEquivalence(s.second[0]) ||
                m_equivalenceManager->HasEquivalence(s.second[1])) continue;

            // get the var0 with the smallest id
            vector<int> equal_vars(s.second);
            sort(equal_vars.begin(), equal_vars.end(), cmp);

            // equivalent var
            int var0 = equal_vars[0];

            // let other vars equal to var0
            for (int i = 1; i < equal_vars.size(); i++) {
                int vari = equal_vars[i];
                eq_counter++;
                m_equivalenceManager->AddEquivalence(var0, vari);
            }
        }
    }
    m_log->L(1, "Found ", eq_counter, " equivalent latches.");

    // find equivalent gates
    unordered_map<string, vector<int>> signaturesGatesMap;
    EncodeStatesToSignatuers(simulator.getGateStates(), signaturesGatesMap);
    eq_counter = 0;

    // signatures to equivalent latches
    for (auto &s : signaturesGatesMap) {
        if (s.second.size() > 1) {
            // the neg version of variables is processed
            if (m_equivalenceManager->HasEquivalence(s.second[0]) ||
                m_equivalenceManager->HasEquivalence(s.second[1])) continue;

            // get the var0 with the smallest id
            vector<int> equal_vars(s.second);
            sort(equal_vars.begin(), equal_vars.end(), cmp);

            // equivalent var
            int var0 = equal_vars[0];

            // let other vars equal to var0
            for (int i = 1; i < equal_vars.size(); i++) {
                int vari = equal_vars[i];
                eq_counter++;
                m_equivalenceManager->AddEquivalence(var0, vari);
            }
        }
    }
    m_log->L(1, "Found ", eq_counter, " equivalent gates.");

    return true;
}


void Model::SimplifyModelByRandomSimulation() {
    m_log->L(1, "Simplify model by random simulation.");
    if (m_equivalenceSolver != nullptr) m_equivalenceSolver = nullptr;

    m_log->Tick();
    TernarySimulator simulator(m_circuitGraph, m_log);
    vector<shared_ptr<unordered_map<int, tbool>>> simulation_values;
    for (int i = 0; i < NUM_CHUNKS; i++) {
        simulator.simulateRandom(64);
        for (auto &values : simulator.getValues()) {
            simulation_values.emplace_back(values);
        }
    }
    m_log->L(1, "Simulation takes ", m_log->Tock(), " seconds.");

    // find may equivalent latches
    VarMapN64 signaturesVariablesMap;
    vector<int> eqcheck_latches = m_circuitGraph->modelLatches;
    eqcheck_latches.emplace_back(TrueId());
    EncodeStatesToN64Signatuers(simulation_values, eqcheck_latches, signaturesVariablesMap);
    int mayeq_counter = 0;
    int eq_counter = 0;

    auto start_time = chrono::steady_clock::now();
    // signatures to equivalent variables
    for (auto &s : signaturesVariablesMap) {
        if (chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start_time).count() > m_settings.eqTimeout) {
            m_log->L(1, "Equivalent latch checking timeout after ", m_settings.eqTimeout, " seconds.");
            break;
        }

        if (s.second.size() < 2) continue;

        vector<int> may_equal_vars(s.second);
        sort(may_equal_vars.begin(), may_equal_vars.end(), cmp);

        for (int i = 0; i < may_equal_vars.size() - 1; i++) {
            if (m_equivalenceManager->HasEquivalence(may_equal_vars[i])) continue;

            for (int j = i + 1; j < may_equal_vars.size(); j++) {
                if (m_equivalenceManager->HasEquivalence(may_equal_vars[j])) continue;

                mayeq_counter++;
                if (CheckLatchEquivalenceBySAT(may_equal_vars[i], may_equal_vars[j])) {
                    eq_counter++;
                    m_equivalenceManager->AddEquivalence(may_equal_vars[i], may_equal_vars[j]);
                }
            }
        }
    }
    m_log->L(1, "Found ", eq_counter, "/", mayeq_counter, " equivalent latches.");
    if (mayeq_counter > 0)
        m_log->L(1, "Guessing Correct Ratio: ", eq_counter * 100 / (double)mayeq_counter, "%.");

    if (m_equivalenceSolver != nullptr) m_equivalenceSolver = nullptr;
    // find may equivalent variables
    signaturesVariablesMap.clear();
    vector<int> eqcheck_gates = m_circuitGraph->modelGates;
    eqcheck_gates.emplace_back(TrueId());
    EncodeStatesToN64Signatuers(simulation_values, eqcheck_gates, signaturesVariablesMap);
    mayeq_counter = 0;
    eq_counter = 0;

    // signatures to equivalent variables
    for (auto &s : signaturesVariablesMap) {
        if (chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start_time).count() > m_settings.eqTimeout) {
            m_log->L(1, "Equivalent latch checking timeout after ", m_settings.eqTimeout, " seconds.");
            break;
        }
        if (s.second.size() < 2) continue;

        vector<int> may_equal_vars(s.second);
        sort(may_equal_vars.begin(), may_equal_vars.end(), cmp);

        for (int i = 1; i < may_equal_vars.size(); i++) {
            mayeq_counter++;
            if (!m_equivalenceManager->IsEquivalent(may_equal_vars[0], may_equal_vars[i]) &&
                CheckGateEquivalenceBySAT(may_equal_vars[0], may_equal_vars[i])) {
                eq_counter++;
                m_equivalenceManager->AddEquivalence(may_equal_vars[0], may_equal_vars[i]);
            }
        }
    }
    m_log->L(1, "Found ", eq_counter, "/", mayeq_counter, " equivalent gates.");
    if (mayeq_counter > 0)
        m_log->L(1, "Guessing Correct Ratio: ", eq_counter * 100 / (double)mayeq_counter, "%.");
    if (m_equivalenceSolver != nullptr) m_equivalenceSolver = nullptr;
}


void Model::EncodeStatesToSignatuers(const vector<shared_ptr<vector<int>>> &states, unordered_map<string, vector<int>> &signatures) {
    // encode locations
    unordered_map<int, vector<int>> signal_locations;
    for (int i = 0; i < states.size(); i++) {
        const auto &state = *states[i];
        for (auto v : state) {
            signal_locations[v].emplace_back(i + 1);
            signal_locations[-v].emplace_back(-i - 1);
        }
    }
    // remove incomplete locations
    for (auto it = signal_locations.begin(); it != signal_locations.end();) {
        if (it->second.size() < states.size()) {
            it = signal_locations.erase(it);
        } else {
            ++it;
        }
    }
    // locations to signatures
    for (auto &l : signal_locations) {
        stringstream ss;
        for (int i : l.second) {
            ss << i;
        }
        signatures[ss.str()].emplace_back(l.first);
    }
}


void Model::EncodeStatesToN64Signatuers(const vector<shared_ptr<unordered_map<int, tbool>>> &values, const vector<int> &vars, VarMapN64 &signatures) {
    assert(values.size() == 64 * NUM_CHUNKS);

    for (auto l : vars) {
        SignatureN64 signature;
        for (int i = 0; i < values.size(); i++) {
            const auto &vmapi = *values[i];
            int j = i / 64;
            signature.chunks[j] = signature.chunks[j] << 1;
            if (vmapi.at(l) == t_True) {
                signature.chunks[j] |= 1;
            }
        }

        SignatureN64 neg_signature = ~signature;
        if (signatures.find(signature) != signatures.end()) {
            signatures[signature].emplace_back(l);
        } else if (signatures.find(neg_signature) != signatures.end()) {
            signatures[neg_signature].emplace_back(-l);
        } else {
            signatures[signature].emplace_back(l);
        }
    }
}


bool Model::CheckLatchEquivalenceBySAT(int a, int b) {
    // initial step
    if (m_circuitGraph->latchResetMap.find(abs(a)) == m_circuitGraph->latchResetMap.end())
        return false;
    int init_a = (a > 0) ? m_circuitGraph->latchResetMap[a] : -m_circuitGraph->latchResetMap[-a];
    if (b == TrueId()) {
        if (init_a != TrueId()) return false;
    } else if (b == -TrueId()) {
        if (init_a != -TrueId()) return false;
    } else {
        if (m_circuitGraph->latchResetMap.find(abs(b)) == m_circuitGraph->latchResetMap.end())
            return false;
        int init_b = (b > 0) ? m_circuitGraph->latchResetMap[b] : -m_circuitGraph->latchResetMap[-b];
        if (init_a != init_b) return false;
    }

    // inductive step
    if (m_equivalenceSolver == nullptr ||
        m_eqSolverUnsats > 1000) {
        m_eqSolverUnsats = 0;
        ApplyEquivalence();
        UpdateDependencyMap();
        CollectNextValueMapping();
        CollectClauses();

        m_equivalenceSolver = make_unique<minicore::Solver>();
        for (auto &c : m_clauses) {
            m_equivalenceSolver->addClause(m_equivalenceSolver->intVec2LitVec(c));
        }
        m_equivalenceSolver->solve_in_domain = true;
        // m_equivalenceSolver->verbosity = 1;
    }

    // (a <-> b) -> (a' <-> b')
    // (a <-> b) & !(a' <-> b') is unsat
    // (a | !b) & (!a | b) & (a' | b') & (!a' | !b')
    int a_prime = GetPrime(a);
    int b_prime = GetPrime(b);

    m_equivalenceSolver->resetTempDomain();
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({a, -b}));
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({-a, b}));
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({a_prime, b_prime}));
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({-a_prime, -b_prime}));

    auto d = GetCOIDomain(make_shared<cube>(cube{abs(a), abs(b), abs(a_prime), abs(b_prime)}));
    m_equivalenceSolver->setTempDomain(*d);

    minicore::lbool res = m_equivalenceSolver->solve();
    bool unsat = (res == minicore::l_False);

    if (unsat) {
        m_equivalenceSolver->addClause(m_equivalenceSolver->intVec2LitVec({a, -b}));
        m_equivalenceSolver->addClause(m_equivalenceSolver->intVec2LitVec({-a, b}));
        m_eqSolverUnsats++;
    }
    return unsat;
}


bool Model::CheckGateEquivalenceBySAT(int a, int b) {
    if (m_equivalenceSolver == nullptr ||
        m_eqSolverUnsats > 1000) {
        m_eqSolverUnsats = 0;
        ApplyEquivalence();
        UpdateDependencyMap();
        CollectNextValueMapping();
        CollectClauses();

        m_equivalenceSolver = make_unique<minicore::Solver>();
        m_equivalenceSolver->setRestartLimit(1);
        for (auto &c : m_clauses) {
            m_equivalenceSolver->addClause(m_equivalenceSolver->intVec2LitVec(c));
        }
        m_equivalenceSolver->solve_in_domain = true;
    }

    // (a <-> b)
    // !(a <-> b) is unsat
    // ((a & !b) | (b & !a))
    // (a | b) & (!a | !b)
    m_equivalenceSolver->resetTempDomain();
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({a, b}));
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({-a, -b}));

    auto d = GetCOIDomain(make_shared<cube>(cube{abs(a), abs(b)}));
    m_equivalenceSolver->setTempDomain(*d);

    minicore::lbool res = m_equivalenceSolver->solve();
    bool unsat = (res == minicore::l_False);

    if (unsat) {
        m_equivalenceSolver->addClause(m_equivalenceSolver->intVec2LitVec({a, -b}));
        m_equivalenceSolver->addClause(m_equivalenceSolver->intVec2LitVec({-a, b}));
        m_eqSolverUnsats++;
    }
    return unsat;
}

} // namespace car