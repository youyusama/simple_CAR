#include "Model.h"
#include "DAGCNFSimplifier.h"
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


Model::Model(Settings settings, Log &log) : m_settings(settings),
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
    int num_justice = m_circuitGraph->justice.size();
    if (num_bad > 0 && num_justice > 0) {
        m_log.L(0, "aiger has both safety and justice properties.");
        exit(0);
    }
    if (num_bad == 0 && num_justice == 0) {
        m_log.L(0, "aiger has no property to check.");
        exit(0);
    }
    if (num_bad > 1) {
        m_log.L(0, "aiger has more than one safety property to check.");
        exit(0);
    }
    if (num_justice > 1) {
        m_log.L(0, "aiger has more than one justice property to check.");
        exit(0);
    }

    m_log.L(1, "Model initialized: ",
            m_circuitGraph->numInputs, " inputs, ", m_circuitGraph->numLatches, " latches, ",
            m_circuitGraph->numAnds, " gates, ", m_circuitGraph->numConstraints, " constraints.");
    m_log.L(1, "COI Refined Model: ",
            m_circuitGraph->modelInputs.size(), " inputs, ", m_circuitGraph->modelLatches.size(), " latches, ", m_circuitGraph->modelGates.size(), " gates.");
    m_maxId = m_circuitGraph->numVar + 1;

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

    // initial state
    CollectInitialState();

    // property to check
    if (num_bad == 1) {
        m_bad = m_circuitGraph->bad[0];
        m_propKind = PropKind::Safety;
    } else if (num_justice == 1) {
        // liveness extraction
        m_bad = BuildLiveness();
        m_propKind = PropKind::Liveness;
    }

    // prime variable mapping
    CollectNextValueMapping();

    // internal signals
    if (m_settings.internalSignals) {
        CollectInnards();
    }

    // transform to CNF
    CollectClauses();

    // DAG CNF simplification
    SimplifyDAGClauses();

    // update dependency by DAG CNF
    UpdateDependencyVecDAGCNF();

    // further clause simplification
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

    m_log.L(1, "Model reduced: ",
            m_circuitGraph->modelInputs.size(), " inputs, ", m_circuitGraph->modelLatches.size(), " latches, ", m_circuitGraph->modelGates.size(), " gates.");
    m_log.L(1, "Transformed model: ", m_clauses.size(), " clauses, ", m_simpClauses.size(), " simplified clauses.");
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
    for (size_t i = 0; i < m_circuitGraph->fairness.size(); i++) {
        m_circuitGraph->fairness[i] = m_equivalenceManager->Find(m_circuitGraph->fairness[i]);
    }
    for (size_t i = 0; i < m_circuitGraph->justice.size(); i++) {
        for (size_t j = 0; j < m_circuitGraph->justice[i].size(); j++) {
            m_circuitGraph->justice[i][j] = m_equivalenceManager->Find(m_circuitGraph->justice[i][j]);
        }
    }

    m_circuitGraph->trueId = m_equivalenceManager->Find(m_circuitGraph->trueId);

    m_circuitGraph->COIRefine();
}


void Model::UpdateDependencyMap() {
    m_dependencyVec.assign(m_maxId + 1, vector<int>());
    for (int i = m_circuitGraph->modelGates.size() - 1; i >= 0; i--) {
        int g = m_circuitGraph->modelGates[i];
        for (int fanin : m_circuitGraph->gatesMap[g].fanins) {
            // dependency
            m_dependencyVec[g].emplace_back(abs(fanin));
        }
    }

    m_coiCache.clear();
    m_coiCacheReady.clear();
    m_coiVisited.clear();
    m_coiCacheVisited.clear();
    m_coiDomain.clear();
    m_coiCacheTodo.clear();

    m_coiCache.resize(m_maxId + 1);
    m_coiCacheReady.assign(m_maxId + 1, 0);
    m_coiVisited.assign(m_maxId + 1, 0);
    m_coiCacheVisited.assign(m_maxId + 1, 0);
}


void Model::UpdateDependencyVecDAGCNF() {
    m_dependencyVec.assign(m_maxId + 1, vector<int>());
    for (auto &c : m_clauses) {
        for (size_t i = 0; i + 1 < c.size(); ++i) {
            m_dependencyVec[abs(c.back())].emplace_back(abs(c[i]));
        }
    }
    for (auto &deps : m_dependencyVec) {
        sort(deps.begin(), deps.end());
        deps.erase(unique(deps.begin(), deps.end()), deps.end());
    }

    m_coiCache.clear();
    m_coiCacheReady.clear();
    m_coiVisited.clear();
    m_coiCacheVisited.clear();
    m_coiDomain.clear();
    m_coiCacheTodo.clear();

    m_coiCache.resize(m_maxId + 1);
    m_coiCacheReady.assign(m_maxId + 1, 0);
    m_coiVisited.assign(m_maxId + 1, 0);
    m_coiCacheVisited.assign(m_maxId + 1, 0);
}


void Model::CollectInitialState() {
    for (auto l : m_circuitGraph->modelLatches) {
        int reset = m_circuitGraph->latchResetMap[l];

        if (reset == TrueId()) {
            m_initialState.push_back(l);
        } else if (reset == -TrueId()) {
            m_initialState.push_back(-l);
        } else if (reset != l && IsAnd(reset)) {
            m_initialClauses.emplace_back(clause{l, -reset});
            m_initialClauses.emplace_back(clause{-l, reset});
        }
    }
}


void Model::CollectNextValueMapping() {
    // reset
    m_maxId = m_circuitGraph->numVar + 1;
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
    m_clauses.reserve(m_circuitGraph->modelGates.size() * 3 + 1);

    // true id first for the correctness of dag cnf simplifier,
    // a more rubust way is needed in the future
    m_clauses.emplace_back(clause{TrueId()});

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
}

int Model::GetLatchReset(int latch) const {
    int key = abs(latch);
    auto it = m_circuitGraph->latchResetMap.find(key);
    assert(it != m_circuitGraph->latchResetMap.end());
    int val = it->second;
    return latch > 0 ? val : -val;
}

int Model::GetLatchNext(int latch) const {
    int key = abs(latch);
    auto it = m_circuitGraph->latchNextMap.find(key);
    assert(it != m_circuitGraph->latchNextMap.end());
    int val = it->second;
    return latch > 0 ? val : -val;
}

void Model::SetBad(int bad) {
    m_bad = bad;
    m_circuitGraph->bad.clear();
    m_circuitGraph->bad.emplace_back(bad);
}

void Model::Rebuild() {
    m_initialState.clear();
    m_initialClauses.clear();
    CollectInitialState();
    CollectNextValueMapping();
    CollectClauses();
    SimplifyDAGClauses();
    UpdateDependencyVecDAGCNF();
    SimplifyClauses();
    m_equivalenceSolver.reset();
    m_eqSolverUnsats = 0;
}

int Model::BuildSingleFairness(const vector<int> &conds) {
    if (conds.size() == 1) return conds[0];

    vector<int> monitors;
    monitors.reserve(conds.size());
    for (size_t i = 0; i < conds.size(); i++) {
        monitors.emplace_back(m_circuitGraph->AddLatchVar());
    }

    vector<int> triggers;
    triggers.reserve(conds.size());
    int accept = TrueId();
    for (size_t i = 0; i < conds.size(); i++) {
        int trigger = m_circuitGraph->MakeOr(conds[i], monitors[i]);
        triggers.emplace_back(trigger);
        accept = m_circuitGraph->MakeAnd(accept, trigger);
    }

    int inp = m_circuitGraph->AddInputVar();
    int reset = m_circuitGraph->MakeOr(inp, accept);

    for (size_t i = 0; i < conds.size(); i++) {
        int next = m_circuitGraph->MakeAnd(-reset, triggers[i]);
        m_circuitGraph->SetLatchResetNext(monitors[i], -TrueId(), next);
    }

    return accept;
}

int Model::BuildLiveness() {
    assert(m_circuitGraph->justice.size() == 1);

    vector<int> conds = m_circuitGraph->fairness;
    const vector<int> &just = m_circuitGraph->justice[0];
    conds.insert(conds.end(), just.begin(), just.end());

    int accept = BuildSingleFairness(conds);
    return accept;
}

cube Model::GetCOIDomain(const cube &c) {
    m_coiDomain.clear();
    for (int v : c) {
        int a = abs(v);
        EnsureCOICache(a);
        for (int d : m_coiCache[a]) {
            if (!m_coiVisited[d]) {
                m_coiVisited[d] = 1;
                m_coiDomain.emplace_back(d);
            }
        }
    }

    for (int v : m_coiDomain) m_coiVisited[v] = 0;

    cube domain = m_coiDomain;
    domain.emplace_back(abs(TrueId()));
    return domain;
}

void Model::EnsureCOICache(int v) {
    if (m_coiCacheReady[v]) return;

    m_coiCacheReady[v] = 1;
    m_coiCache[v].clear();
    m_coiCacheTodo.clear();

    m_coiCacheTodo.emplace_back(v);
    m_coiCacheVisited[v] = 1;

    for (size_t i = 0; i < m_coiCacheTodo.size(); ++i) {
        int cur = m_coiCacheTodo[i];
        m_coiCache[v].emplace_back(cur);
        for (int d : m_dependencyVec[cur]) {
            if (!m_coiCacheVisited[d]) {
                m_coiCacheVisited[d] = 1;
                m_coiCacheTodo.emplace_back(d);
            }
        }
    }

    for (int t : m_coiCache[v]) m_coiCacheVisited[t] = 0;
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

void Model::SimplifyDAGClauses() {
    DAGCNFSimplifier simplifier;
    for (auto v : m_circuitGraph->modelInputs) simplifier.FreezeVar(v);
    for (auto v : m_circuitGraph->modelLatches) {
        simplifier.FreezeVar(v);
        simplifier.FreezeVar(GetPrime(v));
    }
    for (int i : m_circuitGraph->constraints) simplifier.FreezeVar(i);
    if (m_settings.internalSignals) {
        for (int i : m_innardsVec) {
            simplifier.FreezeVar(i);
            simplifier.FreezeVar(GetPrime(i));
        }
    }
    simplifier.FreezeVar(TrueId());
    simplifier.FreezeVar(m_bad);

    m_clauses = simplifier.Simplify(m_clauses, TrueId());
}


bool Model::SimplifyModelByTernarySimulation() {
    m_log.L(1, "Simplify model by ternary simulation.");

    m_log.Tick();
    TernarySimulator simulator(m_circuitGraph, m_log);
    simulator.simulate(250);
    if (!simulator.isCycleReached()) return false;
    m_log.L(1, "Simulation takes ", m_log.Tock(), " seconds.");

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
    m_log.L(1, "Found ", eq_counter, " equivalent latches.");

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
    m_log.L(1, "Found ", eq_counter, " equivalent gates.");

    return true;
}


void Model::SimplifyModelByRandomSimulation() {
    m_log.L(1, "Simplify model by random simulation.");
    if (m_equivalenceSolver != nullptr) m_equivalenceSolver = nullptr;

    m_log.Tick();
    TernarySimulator simulator(m_circuitGraph, m_log);
    vector<vector<tbool>> simulation_values;
    for (int i = 0; i < NUM_CHUNKS; i++) {
        simulator.simulateRandom(64);
        for (auto &values : simulator.getValues()) {
            simulation_values.emplace_back(values);
        }
    }
    m_log.L(1, "Simulation takes ", m_log.Tock(), " seconds.");

    VarMapN64 signaturesVariablesMap;
    int mayeq_counter = 0;
    int eq_counter = 0;
    auto start_time = chrono::steady_clock::now();

    // find may equivalent latches
    vector<int> eqcheck_latches = m_circuitGraph->modelLatches;
    eqcheck_latches.emplace_back(TrueId());
    EncodeStatesToN64Signatuers(simulation_values, eqcheck_latches, signaturesVariablesMap);

    // signatures to equivalent variables
    for (auto &s : signaturesVariablesMap) {
        if (chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start_time).count() > m_settings.eqTimeout) {
            m_log.L(1, "Equivalent latch checking timeout after ", m_settings.eqTimeout, " seconds.");
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
    m_log.L(1, "Found ", eq_counter, "/", mayeq_counter, " equivalent latches.");
    if (mayeq_counter > 0)
        m_log.L(1, "Guessing Correct Ratio: ", eq_counter * 100 / (double)mayeq_counter, "%.");

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
            m_log.L(1, "Equivalent gate checking timeout after ", m_settings.eqTimeout, " seconds.");
            break;
        }
        if (s.second.size() < 2) continue;

        vector<int> may_equal_vars(s.second);
        sort(may_equal_vars.begin(), may_equal_vars.end(), cmp);

        if (may_equal_vars.size() <= 3) {
            for (int i = 0; i + 1 < may_equal_vars.size(); i++) {
                for (int j = i + 1; j < may_equal_vars.size(); j++) {
                    int a = may_equal_vars[i];
                    int b = may_equal_vars[j];
                    if (m_equivalenceManager->IsEquivalent(a, b)) {
                        continue;
                    }
                    mayeq_counter++;
                    if (CheckGateEquivalenceBySAT(a, b)) {
                        eq_counter++;
                        m_equivalenceManager->AddEquivalence(a, b);
                    }
                }
            }
        } else {
            int k_rep = std::min<int>(3, may_equal_vars.size());
            vector<int> reps(may_equal_vars.begin(), may_equal_vars.begin() + k_rep);
            for (int i = k_rep; i < may_equal_vars.size(); i++) {
                int v = may_equal_vars[i];
                bool already_equiv = false;
                for (int r : reps) {
                    if (m_equivalenceManager->IsEquivalent(r, v)) {
                        already_equiv = true;
                        break;
                    }
                }
                if (already_equiv) continue;

                for (int r : reps) {
                    mayeq_counter++;
                    if (CheckGateEquivalenceBySAT(r, v)) {
                        eq_counter++;
                        m_equivalenceManager->AddEquivalence(r, v);
                        break;
                    }
                }
            }
        }
    }
    m_log.L(1, "Found ", eq_counter, "/", mayeq_counter, " equivalent gates.");
    if (mayeq_counter > 0)
        m_log.L(1, "Guessing Correct Ratio: ", eq_counter * 100 / (double)mayeq_counter, "%.");
    if (m_equivalenceSolver != nullptr) m_equivalenceSolver = nullptr;
}


void Model::EncodeStatesToSignatuers(const vector<vector<int>> &states, unordered_map<string, vector<int>> &signatures) {
    // encode locations
    unordered_map<int, vector<int>> signal_locations;
    for (int i = 0; i < states.size(); i++) {
        const auto &state = states[i];
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


void Model::EncodeStatesToN64Signatuers(const vector<vector<tbool>> &values, const vector<int> &vars, VarMapN64 &signatures) {
    assert(values.size() == 64 * NUM_CHUNKS);

    for (auto l : vars) {
        SignatureN64 signature;
        for (int i = 0; i < values.size(); i++) {
            const auto &vmapi = values[i];
            int j = i / 64;
            signature.chunks[j] = signature.chunks[j] << 1;
            if (vmapi[l] == t_True) {
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
    if (b == TrueId() && init_a != TrueId()) {
        return false;
    } else if (b == -TrueId() && init_a != -TrueId()) {
        return false;
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
        // UpdateDependencyMap();
        CollectNextValueMapping();
        CollectClauses();
        SimplifyDAGClauses();
        UpdateDependencyVecDAGCNF();

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
    {
        // only keep temp act var
        // need to be more robust in the future
        std::vector<char> &dom = m_equivalenceSolver->domainSet();
        std::fill(dom.begin() + 1, dom.end(), 0);
        m_equivalenceSolver->domainList().resize(1);
    }
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({a, -b}));
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({-a, b}));
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({a_prime, b_prime}));
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({-a_prime, -b_prime}));

    cube d = GetCOIDomain(cube{abs(a), abs(b), abs(a_prime), abs(b_prime)});
    {
        std::vector<char> &dom = m_equivalenceSolver->domainSet();
        std::vector<minicore::Var> &list = m_equivalenceSolver->domainList();
        for (auto v : d) {
            while (v >= m_equivalenceSolver->nVars()) m_equivalenceSolver->newVar();
            if (!dom[v]) {
                dom[v] = 1;
                list.push_back(v);
            }
        }
    }

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
        // UpdateDependencyMap();
        CollectNextValueMapping();
        CollectClauses();
        SimplifyDAGClauses();
        UpdateDependencyVecDAGCNF();

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
    {
        // only keep temp act var
        std::vector<char> &dom = m_equivalenceSolver->domainSet();
        std::fill(dom.begin() + 1, dom.end(), 0);
        m_equivalenceSolver->domainList().resize(1);
    }
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({a, b}));
    m_equivalenceSolver->addTempClause(m_equivalenceSolver->intVec2LitVec({-a, -b}));

    cube d = GetCOIDomain(cube{abs(a), abs(b)});
    {
        std::vector<char> &dom = m_equivalenceSolver->domainSet();
        std::vector<minicore::Var> &list = m_equivalenceSolver->domainList();
        for (auto v : d) {
            while (v >= m_equivalenceSolver->nVars()) m_equivalenceSolver->newVar();
            if (!dom[v]) {
                dom[v] = 1;
                list.push_back(v);
            }
        }
    }

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
