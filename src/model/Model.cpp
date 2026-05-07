#include "Model.h"
#include "DAGCNFSimplifier.h"
#include "SATSim.h"
#include "WitnessBuilder.h"
#include <bitset>

#include "cadical/src/cadical.hpp"


namespace car {
namespace {

struct EquivalenceCheckStats {
    int equivalent = 0;
    int candidates = 0;
    bool timeout = false;
};

template <typename CheckFn, typename AddFn, typename IsEquivalentFn>
EquivalenceCheckStats CheckSignatureEquivalenceGroups(DynamicSignatureMap &signatures,
                                                      chrono::steady_clock::time_point start_time,
                                                      int timeout_seconds,
                                                      int representative_limit,
                                                      CheckFn check_equivalence,
                                                      AddFn add_equivalence,
                                                      IsEquivalentFn is_equivalent) {
    EquivalenceCheckStats stats;
    constexpr size_t SMALL_GROUP_LIMIT = 3;
    representative_limit = std::max(1, representative_limit);

    auto timed_out = [&]() {
        return chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start_time).count() > timeout_seconds;
    };

    auto try_pair = [&](Lit a, Lit b) {
        if (is_equivalent(a, b)) return false;

        stats.candidates++;
        if (check_equivalence(a, b)) {
            stats.equivalent++;
            add_equivalence(a, b);
            return true;
        }
        return false;
    };

    for (auto &s : signatures) {
        if (timed_out()) {
            stats.timeout = true;
            break;
        }
        if (s.second.size() < 2) continue;

        vector<Lit> may_equal_vars(s.second);
        sort(may_equal_vars.begin(), may_equal_vars.end());

        if (may_equal_vars.size() <= SMALL_GROUP_LIMIT) {
            for (size_t i = 0; i + 1 < may_equal_vars.size(); i++) {
                for (size_t j = i + 1; j < may_equal_vars.size(); j++) {
                    try_pair(may_equal_vars[i], may_equal_vars[j]);
                    if (timed_out()) {
                        stats.timeout = true;
                        return stats;
                    }
                }
            }
            continue;
        }

        int k_rep = std::min<int>(representative_limit, may_equal_vars.size());
        vector<Lit> reps(may_equal_vars.begin(), may_equal_vars.begin() + k_rep);
        for (size_t i = k_rep; i < may_equal_vars.size(); i++) {
            Lit v = may_equal_vars[i];
            bool already_equiv = false;
            for (Lit r : reps) {
                if (is_equivalent(r, v)) {
                    already_equiv = true;
                    break;
                }
            }
            if (already_equiv) continue;

            for (Lit r : reps) {
                bool merged = try_pair(r, v);
                if (timed_out()) {
                    stats.timeout = true;
                    return stats;
                }
                if (merged) break;
            }
        }
    }

    return stats;
}

} // namespace

Lit EquivalenceManager::FindLit(Lit a) {
    Lit root = FindRootRecursive(VarOf(a));
    return Sign(a) ? ~root : root;
}


void EquivalenceManager::AddEquivalence(Lit a, Lit b) {
    Lit root_a = FindLit(a);
    Lit root_b = FindLit(b);
    // alreadly equivalent
    if (root_a == root_b) return;

    Var key_a = VarOf(root_a);
    Var key_b = VarOf(root_b);

    // merge two groups
    if (key_a < key_b) {
        m_equivalenceMap[key_b] = Sign(b) ? ~root_a : root_a;
    } else {
        m_equivalenceMap[key_a] = Sign(a) ? ~root_b : root_b;
    }
}

Lit EquivalenceManager::FindRootRecursive(Var key) {
    auto it = m_equivalenceMap.find(key);
    if (it == m_equivalenceMap.end()) {
        return MkLit(key);
    }

    Lit next = it->second;
    Var next_key = VarOf(next);
    assert(key != next_key);

    Lit root = FindRootRecursive(next_key);
    Lit result = Sign(next) ? ~root : root;

    m_equivalenceMap[key] = result;

    return result;
}


Model::Model(Settings settings, Log &log) : m_settings(settings),
                                            m_log(log) {
    // load aiger
    string aig_file_path = settings.aigFilePath;
    m_aiger = shared_ptr<aiger>(aiger_init(), AigerDeleter);
    aiger_open_and_read_from_file(m_aiger.get(), aig_file_path.c_str());
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
        LOG_L(m_log, 0, "aiger has both safety and justice properties.");
        exit(0);
    }
    if (num_bad == 0 && num_justice == 0) {
        LOG_L(m_log, 0, "aiger has no property to check.");
        exit(0);
    }
    if (num_bad > 1) {
        LOG_L(m_log, 0, "aiger has more than one safety property to check.");
        exit(0);
    }
    if (num_justice > 1) {
        LOG_L(m_log, 0, "aiger has more than one justice property to check.");
        exit(0);
    }

    EliminateGateResets();

    // property to check
    if (num_bad == 1) {
        m_bad = m_circuitGraph->bad[0];
        m_propKind = PropKind::Safety;
    } else if (num_justice == 1) {
        // liveness extraction
        m_bad = BuildLiveness();
        m_propKind = PropKind::Liveness;
    }

    LOG_L(m_log, 1, "Model initialized: ",
          m_circuitGraph->numInputs, " inputs, ", m_circuitGraph->numLatches, " latches, ",
          m_circuitGraph->numAnds, " gates, ", m_circuitGraph->numConstraints, " constraints.");
    LOG_L(m_log, 1, "COI Refined Model: ",
          m_circuitGraph->modelInputs.size(), " inputs, ", m_circuitGraph->modelLatches.size(), " latches, ", m_circuitGraph->modelGates.size(), " gates.");

    m_cnfTrueVar = m_circuitGraph->numVar + 1;
    m_maxId = m_cnfTrueVar;

    // try to find equivalences
    m_equivalenceManager = make_shared<EquivalenceManager>();
    if (m_settings.eq == 1) {
        SimplifyModelByTernarySimulation();
        ApplyEquivalence();
        SimplifyModelBySATSimulation();
    } else if (m_settings.eq == 2) {
        SimplifyModelByTernarySimulation();
    } else if (m_settings.eq == 3) {
        SimplifyModelByRandomSimulation();
    } else if (m_settings.eq == 4) {
        SimplifyModelBySATSimulation();
    }

    // apply the equivalences to the circuit graph
    ApplyEquivalence();

    // initial state
    CollectInitialState();

    // constraints
    CollectConstraints();

    // prime variable mapping
    CollectNextValueMapping();

    // internal signals
    if (m_settings.internalSignals) {
        CollectInnards();
    }

    // transform to CNF
    CollectClauses();

    // DAG clause simplification on raw clauses
    SimplifyDAGClauses();

    // lower raw clauses to CNF clauses
    CollectCNFClauses();

    // update dependency by DAG CNF
    UpdateDependencyVecDAGCNF();

    // further Clause simplification
    SimplifyClauses();

    LOG_L(m_log, 1, "Model reduced: ",
          m_circuitGraph->modelInputs.size(), " inputs, ", m_circuitGraph->modelLatches.size(), " latches, ", m_circuitGraph->modelGates.size(), " gates.");
    LOG_L(m_log, 1, "Transformed model: ", m_cnfClauses.size(), " clauses, ", m_simpClauses.size(), " simplified clauses.");
}


void Model::SetTsimReachedStateCubes(const std::vector<Cube> &cubes) {
    m_equivalenceWitness = EquivalenceWitness();
    m_equivalenceWitness.has_reached_state_region = true;
    m_equivalenceWitness.reached_state_cubes = cubes;
    m_equivalenceWitnessReady = false;
}


const EquivalenceWitness &Model::GetEquivalenceWitness() {
    if (!m_equivalenceWitnessReady) {
        BuildEquivalenceWitness();
    }
    return m_equivalenceWitness;
}


void Model::RefineWitnessPropertyLit(WitnessBuilder &builder) {
    const EquivalenceWitness &witness = GetEquivalenceWitness();
    builder.RegisterEquivalenceWitness(witness);

    // Cons_pre := (& eq_clauses) & (| reached_state_cubes)
    std::vector<unsigned> preprocess_terms;
    preprocess_terms.reserve(witness.equivalence_clauses.size() + 1);
    for (const Clause &clause : witness.equivalence_clauses) {
        preprocess_terms.push_back(builder.BuildClause(clause));
    }
    if (witness.has_reached_state_region) {
        std::vector<unsigned> state_terms;
        state_terms.reserve(witness.reached_state_cubes.size());
        for (const Cube &cube : witness.reached_state_cubes) {
            state_terms.push_back(builder.BuildCube(cube));
        }
        preprocess_terms.push_back(builder.BuildOr(state_terms));
    }

    // P := P & Cons_pre
    unsigned preprocess_lit = builder.BuildAnd(preprocess_terms);
    LOG_L(m_log, 1, "Preprocess lit: ", preprocess_lit);
    builder.SetPropertyLit(builder.BuildAnd({builder.GetPropertyLit(), preprocess_lit}));
}


void Model::ApplyEquivalence() {
    if (m_equivalenceManager->Size() == 0) return;

    // refine the model by equivalence
    for (auto it = m_circuitGraph->modelLatches.begin(); it != m_circuitGraph->modelLatches.end();) {
        m_circuitGraph->latchResetMap[*it] = m_equivalenceManager->FindLit(m_circuitGraph->latchResetMap[*it]);
        m_circuitGraph->latchNextMap[*it] = m_equivalenceManager->FindLit(m_circuitGraph->latchNextMap[*it]);

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
            for (size_t i = 0; i < gate.fanins.size(); i++) {
                gate.fanins[i] = m_equivalenceManager->FindLit(gate.fanins[i]);
            }
            it++;
        }
    }
    for (size_t i = 0; i < m_circuitGraph->bad.size(); i++) {
        m_circuitGraph->bad[i] = m_equivalenceManager->FindLit(m_circuitGraph->bad[i]);
    }

    for (size_t i = 0; i < m_circuitGraph->constraints.size(); i++) {
        m_circuitGraph->constraints[i] = m_equivalenceManager->FindLit(m_circuitGraph->constraints[i]);
    }
    for (size_t i = 0; i < m_circuitGraph->fairness.size(); i++) {
        m_circuitGraph->fairness[i] = m_equivalenceManager->FindLit(m_circuitGraph->fairness[i]);
    }
    for (size_t i = 0; i < m_circuitGraph->justice.size(); i++) {
        for (size_t j = 0; j < m_circuitGraph->justice[i].size(); j++) {
            m_circuitGraph->justice[i][j] = m_equivalenceManager->FindLit(m_circuitGraph->justice[i][j]);
        }
    }

    m_bad = m_equivalenceManager->FindLit(m_bad);

    m_circuitGraph->COIRefine();
    m_circuitGraph->CollectPropertyCOIInputs();
}


void Model::BuildEquivalenceClauses(std::vector<Clause> &out) {
    out.clear();
    const auto &eq_map = m_equivalenceManager->GetEquivalenceMap();
    out.reserve(eq_map.size() * 2);
    for (const auto &entry : eq_map) {
        Lit lhs = MkLit(entry.first);
        Lit rhs = m_equivalenceManager->FindLit(lhs);
        if (rhs == lhs) continue;
        out.push_back(Clause{lhs, ~rhs});
        out.push_back(Clause{~lhs, rhs});
    }
}

void Model::NormalizeReachedStateRegion(EquivalenceWitness &witness) {
    if (!witness.has_reached_state_region) return;

    for (Cube &cube : witness.reached_state_cubes) {
        LitSet cube_set;
        cube_set.NewSet(cube);

        Cube reduced;
        reduced.reserve(cube.size());
        for (Lit lit : cube) {
            Lit representative = m_equivalenceManager->FindLit(lit);
            if (representative != lit && cube_set.Has(representative)) {
                continue;
            }
            reduced.push_back(lit);
        }
        cube.swap(reduced);
    }
}


void Model::BuildEquivalenceWitness() {
    BuildEquivalenceClauses(m_equivalenceWitness.equivalence_clauses);
    NormalizeReachedStateRegion(m_equivalenceWitness);
    m_equivalenceWitnessReady = true;
}


void Model::EliminateGateResets() {
    Var init_latch = VAR_UNDEF;
    vector<Var> latches = m_circuitGraph->modelLatches;

    for (Var latch : latches) {
        Lit reset = m_circuitGraph->latchResetMap[latch];
        if (IsConst(reset) || reset == MkLit(latch)) continue;

        if (init_latch == VAR_UNDEF) {
            init_latch = NewLatchVar();
            SetLatchReset(init_latch, LIT_TRUE);
            SetLatchNext(init_latch, LIT_FALSE);
        }

        Lit init_eq = MakeXNOR(MkLit(latch), reset);
        Lit init_constraint = MakeOR(~MkLit(init_latch), init_eq);
        m_circuitGraph->constraints.emplace_back(init_constraint);
        m_circuitGraph->numConstraints++;
        SetLatchReset(latch, MkLit(latch));
    }

    if (init_latch != VAR_UNDEF) {
        m_hasResetGateInit = true;
        m_circuitGraph->COIRefine();
        m_circuitGraph->CollectPropertyCOIInputs();
    }
}


void Model::UpdateDependencyVecDAGCNF() {
    m_dependencyVec.assign(m_maxId + 1, vector<Var>());
    for (auto &c : m_cnfClauses) {
        for (size_t i = 0; i + 1 < c.size(); ++i) {
            m_dependencyVec[VarOf(c.back())].emplace_back(VarOf(c[i]));
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
    m_initialState.clear();

    for (Var l : m_circuitGraph->modelLatches) {
        Lit reset = m_circuitGraph->latchResetMap[l];

        if (reset == LIT_TRUE) {
            m_initialState.push_back(ToCNFLit(MkLit(l)));
        } else if (reset == LIT_FALSE) {
            m_initialState.push_back(ToCNFLit(~MkLit(l)));
        }
    }
}


void Model::CollectConstraints() {
    m_constraints.clear();
    m_constraints.reserve(m_circuitGraph->constraints.size());
    for (Lit lit : m_circuitGraph->constraints) {
        m_constraints.emplace_back(ToCNFLit(lit));
    }
}


void Model::CollectNextValueMapping() {
    // reset
    m_maxId = m_circuitGraph->numVar + 1;
    m_primeMaps.clear();
    m_lookupPrime.clear();
    m_primeMaps.push_back(unordered_map<Var, Lit, std::hash<Var>>());

    for (Var l : m_circuitGraph->latches) {
        SetPrimeMap0(l, ToCNFLit(m_circuitGraph->latchNextMap[l]));
    }
}


void Model::CollectClauses() {
    m_rawClauses.clear();
    m_rawClauses.reserve(m_circuitGraph->modelGates.size() * 4);

    for (Var g_id : m_circuitGraph->modelGates) {
        auto g = m_circuitGraph->gatesMap[g_id];
        Lit fanout = MkLit(g.fanout);
        Lit fanin0 = g.fanins[0];
        Lit fanin1 = g.fanins[1];
        if (g.gateType == CircuitGate::GateType::AND) {
            m_rawClauses.emplace_back(Clause{fanout, ~fanin0, ~fanin1});
            m_rawClauses.emplace_back(Clause{~fanout, fanin0});
            m_rawClauses.emplace_back(Clause{~fanout, fanin1});
        } else if (g.gateType == CircuitGate::GateType::XOR) {
            m_rawClauses.emplace_back(Clause{fanout, ~fanin0, fanin1});
            m_rawClauses.emplace_back(Clause{fanout, fanin0, ~fanin1});
            m_rawClauses.emplace_back(Clause{~fanout, fanin0, fanin1});
            m_rawClauses.emplace_back(Clause{~fanout, ~fanin0, ~fanin1});
        } else if (g.gateType == CircuitGate::GateType::ITE) {
            Lit fanin2 = g.fanins[2];
            m_rawClauses.emplace_back(Clause{fanout, ~fanin0, ~fanin1});
            m_rawClauses.emplace_back(Clause{fanout, fanin0, ~fanin2});
            m_rawClauses.emplace_back(Clause{~fanout, ~fanin0, fanin1});
            m_rawClauses.emplace_back(Clause{~fanout, fanin0, fanin2});
        }
    }
}


void Model::CollectCNFClauses() {
    m_cnfClauses.clear();
    m_cnfClauses.reserve(m_rawClauses.size() + 1);
    m_cnfClauses.emplace_back(Clause{MkLit(TrueId())});
    for (const Clause &cls : m_rawClauses) {
        m_cnfClauses.emplace_back(ToCNFClause(cls));
    }
}


Lit Model::GetLatchResetLit(Var latch) const {
    auto it = m_circuitGraph->latchResetMap.find(latch);
    assert(it != m_circuitGraph->latchResetMap.end());
    return it->second;
}

Lit Model::GetLatchNextLit(Var latch) const {
    auto it = m_circuitGraph->latchNextMap.find(latch);
    assert(it != m_circuitGraph->latchNextMap.end());
    return it->second;
}

void Model::SetLatchReset(Var latch, Lit reset) {
    m_circuitGraph->latchResetMap[latch] = reset;
}

void Model::SetLatchNext(Var latch, Lit next) {
    m_circuitGraph->latchNextMap[latch] = next;
}

void Model::SetBad(Lit bad) {
    m_bad = bad;
}


Var Model::NewInputVar() {
    return m_circuitGraph->NewInputVar();
}

Var Model::NewLatchVar() {
    return m_circuitGraph->NewLatchVar();
}


Lit Model::MakeAND(Lit a, Lit b) {
    Lit lit_true = LIT_TRUE;
    Lit lit_false = LIT_FALSE;
    if (a == lit_true) return b;
    if (b == lit_true) return a;
    if (a == lit_false || b == lit_false) return lit_false;
    if (a == b) return a;
    if (a == ~b) return LIT_FALSE;
    return MkLit(m_circuitGraph->NewAndGate(a, b));
}


Lit Model::MakeOR(Lit a, Lit b) {
    return ~MakeAND(~a, ~b);
}


Lit Model::MakeXOR(Lit a, Lit b) {
    Lit t1 = MakeAND(a, ~b);
    Lit t2 = MakeAND(~a, b);
    return MakeOR(t1, t2);
}


Lit Model::MakeXNOR(Lit a, Lit b) {
    return ~MakeXOR(a, b);
}


Lit Model::MakeITE(Lit i, Lit t, Lit e) {
    Lit t1 = MakeAND(i, t);
    Lit t2 = MakeAND(~i, e);
    return MakeOR(t1, t2);
}


void Model::Rebuild() {
    CollectInitialState();
    CollectConstraints();
    CollectNextValueMapping();
    CollectClauses();
    SimplifyDAGClauses();
    CollectCNFClauses();
    UpdateDependencyVecDAGCNF();
    SimplifyClauses();

    LOG_L(m_log, 1, "Model rebuilt: ",
          m_circuitGraph->modelInputs.size(), " inputs, ", m_circuitGraph->modelLatches.size(), " latches, ", m_circuitGraph->modelGates.size(), " gates.");
    LOG_L(m_log, 1, "Transformed model: ", m_cnfClauses.size(), " clauses, ", m_simpClauses.size(), " simplified clauses.");
}


Lit Model::BuildSingleFairness(const Cube &conds) {
    if (conds.size() == 1) return conds[0];

    vector<Var> monitors;
    monitors.reserve(conds.size());
    for (size_t i = 0; i < conds.size(); i++) {
        monitors.emplace_back(NewLatchVar());
    }

    // trigger_i = cond_i || monitor_i
    // accept = trigger_0 && trigger_1 && ... && trigger_n
    Cube triggers;
    triggers.reserve(conds.size());
    Lit accept = LIT_TRUE;
    for (size_t i = 0; i < conds.size(); i++) {
        Lit trigger = MakeOR(conds[i], MkLit(monitors[i]));
        triggers.emplace_back(trigger);
        accept = MakeAND(accept, trigger);
    }

    Var inp = NewInputVar();
    Lit reset = MakeOR(MkLit(inp), accept);

    // Init(monitor_i) = false
    // Next(monitor_i) = if (reset) then false else trigger_i
    for (size_t i = 0; i < conds.size(); i++) {
        Lit next = MakeAND(~reset, triggers[i]);
        SetLatchReset(monitors[i], LIT_FALSE);
        SetLatchNext(monitors[i], next);
    }

    return accept;
}


Lit Model::BuildLiveness() {
    assert(m_circuitGraph->justice.size() == 1);

    Cube conds = m_circuitGraph->fairness;
    const Cube &just = m_circuitGraph->justice[0];
    conds.insert(conds.end(), just.begin(), just.end());

    return BuildSingleFairness(conds);
}


Cube Model::GetCOIDomain(const Cube &c) {
    m_coiDomain.clear();
    for (Lit lit : c) {
        Var a = VarOf(lit);
        EnsureCOICache(a);
        for (Var d : m_coiCache[a]) {
            if (!m_coiVisited[d]) {
                m_coiVisited[d] = 1;
                m_coiDomain.emplace_back(d);
            }
        }
    }

    for (Var v : m_coiDomain) m_coiVisited[v] = 0;

    Cube domain;
    domain.reserve(m_coiDomain.size() + 1);
    for (Var v : m_coiDomain) domain.emplace_back(MkLit(v));
    domain.emplace_back(MkLit(TrueId()));
    return domain;
}


Clause Model::ToCNFClause(const Clause &cls) const {
    Clause out;
    out.reserve(cls.size());
    for (Lit lit : cls) {
        out.emplace_back(ToCNFLit(lit));
    }
    return out;
}


void Model::EnsureCOICache(Var v) {
    if (m_coiCacheReady[v]) return;

    m_coiCacheReady[v] = 1;
    m_coiCache[v].clear();
    m_coiCacheTodo.clear();

    m_coiCacheTodo.emplace_back(v);
    m_coiCacheVisited[v] = 1;

    for (size_t i = 0; i < m_coiCacheTodo.size(); ++i) {
        Var cur = m_coiCacheTodo[i];
        m_coiCache[v].emplace_back(cur);
        for (Var d : m_dependencyVec[cur]) {
            if (!m_coiCacheVisited[d]) {
                m_coiCacheVisited[d] = 1;
                m_coiCacheTodo.emplace_back(d);
            }
        }
    }

    for (Var t : m_coiCache[v]) m_coiCacheVisited[t] = 0;
}


Lit Model::EnsurePrimeK(Lit id, int k) {
    if (k == 0) return id;
    if (IsConstant(id)) return id;
    if (k >= m_primeMaps.size())
        m_primeMaps.push_back(unordered_map<Var, Lit, std::hash<Var>>());
    if (IsLatch(id)) return EnsurePrimeK(LookupPrime(id), k - 1);

    auto &k_map = m_primeMaps[k - 1];
    auto it = k_map.find(VarOf(id));
    Var prime_var = 0;
    if (it != k_map.end()) {
        prime_var = VarOf(it->second);
    } else {
        Lit prime_lit = MkLit(GetNewVar());
        auto res = k_map.insert(pair<Var, Lit>(VarOf(id), prime_lit));
        prime_var = VarOf(res.first->second);
    }
    return MkLit(prime_var, Sign(id));
}


int Model::InnardsLogiclvlDFS(Var id) {
    auto it = m_innardsLvl.find(id);
    if (it != m_innardsLvl.end())
        return it->second;
    int lvl = 0;
    if (m_circuitGraph->andsSet.find(id) != m_circuitGraph->andsSet.end()) {
        auto gate = m_circuitGraph->gatesMap[id];
        for (auto fanin : gate.fanins) {
            int fanin_lvl = InnardsLogiclvlDFS(VarOf(fanin));
            if (fanin_lvl + 1 > lvl)
                lvl = fanin_lvl + 1;
        }
    } else {
        lvl = 0;
    }
    m_innardsLvl.insert(pair<int, int>(id, lvl));
    return lvl;
}


void Model::CollectInnards() {
    for (size_t i = 0; i < m_circuitGraph->modelGates.size(); i++) {
        Var g = m_circuitGraph->modelGates[i];

        // decide whether the gate is an innard
        bool is_innard = true;
        for (Lit fanin : m_circuitGraph->gatesMap[g].fanins) {
            bool b = IsConstant(fanin) ||
                     IsLatch(fanin) ||
                     m_innards.find(VarOf(fanin)) != m_innards.end();
            is_innard &= b;
        }

        // the gate is an innard
        if (is_innard) {
            m_innards.emplace(g);
            InnardsLogiclvlDFS(g);

            // build a new gate
            CircuitGate gate(m_circuitGraph->gatesMap[g]);
            if (!HasPrimeMap0(g)) {
                SetPrimeMap0(g, MkLit(GetNewVar()));
            }
            Var p_fanout = VarOf(LookupPrime(MkLit(g)));
            gate.fanout = p_fanout;
            assert(p_fanout > 0);

            for (size_t i = 0; i < gate.fanins.size(); i++) {
                Lit fanin = gate.fanins[i];
                Lit p_fanin;
                if (IsLatch(fanin)) {
                    p_fanin = LookupPrime(fanin);
                } else if (IsConstant(fanin)) {
                    if (IsTrue(fanin))
                        p_fanin = LIT_TRUE;
                    else
                        p_fanin = LIT_FALSE;
                } else if (IsAnd(fanin)) {
                    p_fanin = LookupPrime(fanin);
                }
                gate.fanins[i] = p_fanin;
            }
            m_circuitGraph->gatesMap[p_fanout] = gate;
        }
    }
    for (Var g_id : m_innards) m_circuitGraph->modelGates.emplace_back(VarOf(LookupPrime(MkLit(g_id))));
    m_innardsVec.assign(m_innards.begin(), m_innards.end());
    sort(m_innardsVec.begin(), m_innardsVec.end());
}


void Model::SimplifyClauses() {
    std::shared_ptr<CaDiCaL::Solver> solver = std::make_shared<CaDiCaL::Solver>();
    for (auto &c : m_cnfClauses) {
        solver->clause(ToSignedVec(c));
    }
    // freeze variables
    for (auto v : m_circuitGraph->modelInputs) solver->freeze(v);
    for (auto v : m_circuitGraph->modelLatches) {
        solver->freeze(v);
        solver->freeze(static_cast<int>(VarOf(LookupPrime(MkLit(v)))));
    }
    // freeze constraints
    for (Lit i : m_circuitGraph->constraints) solver->freeze(static_cast<int>(VarOf(i)));
    if (m_settings.internalSignals) {
        for (Var i : m_innardsVec) {
            solver->freeze(i);
            solver->freeze(static_cast<int>(VarOf(LookupPrime(MkLit(i)))));
        }
    }
    solver->freeze(TrueId());
    solver->freeze(static_cast<int>(VarOf(m_bad)));

    class CarClauseIterator : public CaDiCaL::ClauseIterator {
      public:
        ~CarClauseIterator() {}
        bool clause(const std::vector<int> &cls) {
            m_simpClauses.emplace_back(FromSignedVec(cls));
            return true;
        }
        vector<Clause> &GetClauses() {
            return m_simpClauses;
        }

      private:
        vector<Clause> m_simpClauses;
    };

    CarClauseIterator it;
    solver->simplify();
    solver->traverse_clauses(it);
    // cout << "clauses: " << m_cnfClauses.size() << endl;
    // cout << "simplified clauses: " << it.GetClauses().size() << endl;
    m_simpClauses = it.GetClauses();
}

void Model::SimplifyDAGClauses() {
    DAGCNFSimplifier simplifier;
    for (auto v : m_circuitGraph->modelInputs) simplifier.FreezeVar(v);
    for (auto v : m_circuitGraph->modelLatches) {
        simplifier.FreezeVar(v);
        simplifier.FreezeVar(VarOf(LookupPrime(MkLit(v))));
    }
    for (Lit i : m_circuitGraph->constraints) simplifier.FreezeVar(VarOf(i));
    if (m_settings.internalSignals) {
        for (Var i : m_innardsVec) {
            simplifier.FreezeVar(i);
            simplifier.FreezeVar(VarOf(LookupPrime(MkLit(i))));
        }
    }
    simplifier.FreezeVar(TrueId());
    simplifier.FreezeVar(VarOf(m_bad));

    m_rawClauses = simplifier.Simplify(m_rawClauses);
}


bool Model::SimplifyModelByTernarySimulation() {
    LOG_L(m_log, 1, "Simplify model by ternary simulation.");

    m_log.Tick();
    TernarySimulator simulator(m_circuitGraph, m_log);
    simulator.Simulate(250);
    if (!simulator.IsCycleReached()) return false;
    LOG_L(m_log, 1, "Simulation takes ", m_log.Tock(), " seconds.");
    SetTsimReachedStateCubes(simulator.GetStates());

    // find equivalent latches
    vector<Cube> latch_states = simulator.GetStates();
    DynamicSignatureMap signatures_variables_map;
    EncodeStatesToSignatures(latch_states, signatures_variables_map);
    int eq_counter = 0;

    // signatures to equivalent latches
    for (auto &s : signatures_variables_map) {
        if (s.second.size() > 1) {
            // the neg version of variables is processed
            if (m_equivalenceManager->HasEquivalence(s.second[0]) ||
                m_equivalenceManager->HasEquivalence(s.second[1])) continue;

            // get the var0 with the smallest id
            vector<Lit> equal_vars(s.second);
            sort(equal_vars.begin(), equal_vars.end());

            // equivalent var
            Lit var0 = equal_vars[0];

            // let other vars equal to var0
            for (size_t i = 1; i < equal_vars.size(); i++) {
                Lit vari = equal_vars[i];
                eq_counter++;
                m_equivalenceManager->AddEquivalence(var0, vari);
            }
        }
    }
    LOG_L(m_log, 1, "Found ", eq_counter, " equivalent latches.");

    // find equivalent gates
    vector<Cube> gate_states = simulator.GetGateStates();
    DynamicSignatureMap signatures_gates_map;
    EncodeStatesToSignatures(gate_states, signatures_gates_map);
    eq_counter = 0;

    // signatures to equivalent latches
    for (auto &s : signatures_gates_map) {
        if (s.second.size() > 1) {
            // the neg version of variables is processed
            if (m_equivalenceManager->HasEquivalence(s.second[0]) ||
                m_equivalenceManager->HasEquivalence(s.second[1])) continue;

            // get the var0 with the smallest id
            vector<Lit> equal_vars(s.second);
            sort(equal_vars.begin(), equal_vars.end());

            // equivalent var
            Lit var0 = equal_vars[0];

            // let other vars equal to var0
            for (size_t i = 1; i < equal_vars.size(); i++) {
                Lit vari = equal_vars[i];
                eq_counter++;
                m_equivalenceManager->AddEquivalence(var0, vari);
            }
        }
    }
    LOG_L(m_log, 1, "Found ", eq_counter, " equivalent gates.");

    return true;
}


void Model::SimplifyModelByRandomSimulation() {
    LOG_L(m_log, 1, "Simplify model by random simulation.");
    if (m_gateEqSolver != nullptr) m_gateEqSolver = nullptr;

    m_log.Tick();
    TernarySimulator simulator(m_circuitGraph, m_log);
    vector<vector<Tbool>> simulation_values;
    constexpr int RANDOM_SIM_ROUNDS = 128;
    for (int i = 0; i < RANDOM_SIM_ROUNDS; i++) {
        simulator.SimulateRandom(64);
        for (auto &values : simulator.GetValues()) {
            simulation_values.emplace_back(values);
        }
    }
    LOG_L(m_log, 1, "Simulation takes ", m_log.Tock(), " seconds.");

    DynamicSignatureMap signatures_variables_map;
    int mayeq_counter = 0;
    int eq_counter = 0;
    auto start_time = chrono::steady_clock::now();
    auto latch_check_start = chrono::steady_clock::now();

    // find may equivalent latches
    Cube eqcheck_latches;
    eqcheck_latches.reserve(m_circuitGraph->modelLatches.size());
    for (Var v : m_circuitGraph->modelLatches) eqcheck_latches.emplace_back(MkLit(v));
    EncodeTernaryValuesToBitSignatures(simulation_values, eqcheck_latches, signatures_variables_map);

    ResetLatchEquivalenceSolvers();

    EquivalenceCheckStats latch_stats = CheckSignatureEquivalenceGroups(
        signatures_variables_map, start_time, m_settings.eqTimeout, 8,
        [&](Lit a, Lit b) { return CheckLatchEquivalenceBySAT(a, b); },
        [&](Lit a, Lit b) { m_equivalenceManager->AddEquivalence(a, b); },
        [&](Lit a, Lit b) { return m_equivalenceManager->IsEquivalent(a, b); });
    if (latch_stats.timeout)
        LOG_L(m_log, 1, "Equivalent latch checking timeout after ", m_settings.eqTimeout, " seconds.");
    mayeq_counter = latch_stats.candidates;
    eq_counter = latch_stats.equivalent;

    LOG_L(m_log, 1, "Found ", eq_counter, "/", mayeq_counter, " equivalent latches.");
    if (mayeq_counter > 0)
        LOG_L(m_log, 1, "Guessing Correct Ratio: ", eq_counter * 100 / (double)mayeq_counter, "%.");
    LOG_L(m_log, 1, "Random-simulated latch equivalence checking takes ",
          chrono::duration<double>(chrono::steady_clock::now() - latch_check_start).count(), " seconds.");

    if (m_gateEqSolver != nullptr) m_gateEqSolver = nullptr;
    auto gate_check_start = chrono::steady_clock::now();
    // find may equivalent variables
    signatures_variables_map.clear();
    Cube eqcheck_gates;
    eqcheck_gates.reserve(m_circuitGraph->modelGates.size());
    for (Var v : m_circuitGraph->modelGates) eqcheck_gates.emplace_back(MkLit(v));
    EncodeTernaryValuesToBitSignatures(simulation_values, eqcheck_gates, signatures_variables_map);
    mayeq_counter = 0;
    eq_counter = 0;

    EquivalenceCheckStats gate_stats = CheckSignatureEquivalenceGroups(
        signatures_variables_map, start_time, m_settings.eqTimeout, 3,
        [&](Lit a, Lit b) { return CheckGateEquivalenceBySAT(a, b); },
        [&](Lit a, Lit b) { m_equivalenceManager->AddEquivalence(a, b); },
        [&](Lit a, Lit b) { return m_equivalenceManager->IsEquivalent(a, b); });
    if (gate_stats.timeout)
        LOG_L(m_log, 1, "Equivalent gate checking timeout after ", m_settings.eqTimeout, " seconds.");
    mayeq_counter = gate_stats.candidates;
    eq_counter = gate_stats.equivalent;

    LOG_L(m_log, 1, "Found ", eq_counter, "/", mayeq_counter, " equivalent gates.");
    if (mayeq_counter > 0)
        LOG_L(m_log, 1, "Guessing Correct Ratio: ", eq_counter * 100 / (double)mayeq_counter, "%.");
    LOG_L(m_log, 1, "Random-simulated gate equivalence checking takes ",
          chrono::duration<double>(chrono::steady_clock::now() - gate_check_start).count(), " seconds.");
    if (m_gateEqSolver != nullptr) m_gateEqSolver = nullptr;
}


void Model::SimplifyModelBySATSimulation() {
    LOG_L(m_log, 1, "Simplify model by SAT-based latch simulation.");
    if (m_gateEqSolver != nullptr) m_gateEqSolver = nullptr;
    ResetLatchEquivalenceSolvers();

    CollectInitialState();
    CollectConstraints();
    CollectNextValueMapping();
    CollectClauses();
    SimplifyDAGClauses();
    CollectCNFClauses();
    UpdateDependencyVecDAGCNF();

    auto start_time = chrono::steady_clock::now();
    m_log.Tick();
    SATSimulator simulator(m_circuitGraph, m_cnfClauses, m_constraints, m_initialState, TrueId());
    vector<vector<Tbool>> samples = simulator.InitSimulation(64);
    vector<vector<Tbool>> transition_samples = simulator.TransitionSimulation(samples, 640);
    samples.insert(samples.end(), transition_samples.begin(), transition_samples.end());
    const vector<vector<Tbool>> &gate_samples = simulator.GetGateSamples();
    LOG_L(m_log, 1, "SAT simulation generated ", samples.size(), " latch samples and ", gate_samples.size(),
          " gate samples in ", m_log.Tock(), " seconds.");
    if (samples.empty()) return;

    Cube eqcheck_latches;
    eqcheck_latches.reserve(m_circuitGraph->modelLatches.size());
    for (Var v : m_circuitGraph->modelLatches) eqcheck_latches.emplace_back(MkLit(v));

    DynamicSignatureMap signatures_variables_map;
    auto latch_check_start = chrono::steady_clock::now();
    EncodeTernaryValuesToBitSignatures(samples, eqcheck_latches, signatures_variables_map);

    int mayeq_counter = 0;
    int eq_counter = 0;

    ResetLatchEquivalenceSolvers();

    EquivalenceCheckStats latch_stats = CheckSignatureEquivalenceGroups(
        signatures_variables_map, start_time, m_settings.eqTimeout, 8,
        [&](Lit a, Lit b) { return CheckLatchEquivalenceBySAT(a, b); },
        [&](Lit a, Lit b) { m_equivalenceManager->AddEquivalence(a, b); },
        [&](Lit a, Lit b) { return m_equivalenceManager->IsEquivalent(a, b); });
    if (latch_stats.timeout)
        LOG_L(m_log, 1, "SAT-based equivalent latch checking timeout after ", m_settings.eqTimeout, " seconds.");
    mayeq_counter = latch_stats.candidates;
    eq_counter = latch_stats.equivalent;

    LOG_L(m_log, 1, "Found ", eq_counter, "/", mayeq_counter, " SAT-simulated equivalent latches.");
    if (mayeq_counter > 0)
        LOG_L(m_log, 1, "Guessing Correct Ratio: ", eq_counter * 100 / (double)mayeq_counter, "%.");
    LOG_L(m_log, 1, "SAT-simulated latch equivalence checking takes ",
          chrono::duration<double>(chrono::steady_clock::now() - latch_check_start).count(), " seconds.");
    ResetLatchEquivalenceSolvers();

    if (m_gateEqSolver != nullptr) m_gateEqSolver = nullptr;
    auto gate_check_start = chrono::steady_clock::now();
    signatures_variables_map.clear();
    Cube eqcheck_gates;
    eqcheck_gates.reserve(m_circuitGraph->modelGates.size());
    for (Var v : m_circuitGraph->modelGates) eqcheck_gates.emplace_back(MkLit(v));
    EncodeTernaryValuesToBitSignatures(gate_samples, eqcheck_gates, signatures_variables_map);
    mayeq_counter = 0;
    eq_counter = 0;

    EquivalenceCheckStats gate_stats = CheckSignatureEquivalenceGroups(
        signatures_variables_map, start_time, m_settings.eqTimeout, 3,
        [&](Lit a, Lit b) { return CheckGateEquivalenceBySAT(a, b); },
        [&](Lit a, Lit b) { m_equivalenceManager->AddEquivalence(a, b); },
        [&](Lit a, Lit b) { return m_equivalenceManager->IsEquivalent(a, b); });
    if (gate_stats.timeout)
        LOG_L(m_log, 1, "SAT-based equivalent gate checking timeout after ", m_settings.eqTimeout, " seconds.");
    mayeq_counter = gate_stats.candidates;
    eq_counter = gate_stats.equivalent;

    LOG_L(m_log, 1, "Found ", eq_counter, "/", mayeq_counter, " SAT-simulated equivalent gates.");
    if (mayeq_counter > 0)
        LOG_L(m_log, 1, "Guessing Correct Ratio: ", eq_counter * 100 / (double)mayeq_counter, "%.");
    LOG_L(m_log, 1, "SAT-simulated gate equivalence checking takes ",
          chrono::duration<double>(chrono::steady_clock::now() - gate_check_start).count(), " seconds.");
    if (m_gateEqSolver != nullptr) m_gateEqSolver = nullptr;
}


void Model::EncodeStatesToSignatures(const vector<Cube> &states, DynamicSignatureMap &signatures) {
    const size_t num_bits = states.size();
    if (num_bits == 0) return;
    const size_t num_chunks = (num_bits + 63) / 64;
    const size_t last_bits = num_bits % 64;
    const uint64_t last_mask = (last_bits == 0) ? UINT64_MAX : ((uint64_t{1} << last_bits) - 1);

    auto insert_signature = [&](Lit lit, const DynamicSignature &signature) {
        DynamicSignature neg_signature(signature);
        for (uint64_t &chunk : neg_signature) chunk = ~chunk;
        neg_signature.back() &= last_mask;

        if (signatures.find(signature) != signatures.end()) {
            signatures[signature].emplace_back(lit);
        } else if (signatures.find(neg_signature) != signatures.end()) {
            signatures[neg_signature].emplace_back(~lit);
        } else {
            signatures[signature].emplace_back(lit);
        }
    };

    vector<Var> vars;
    unordered_map<Var, DynamicSignature> var_signatures;
    unordered_map<Var, size_t> known_counts;

    for (size_t i = 0; i < states.size(); ++i) {
        for (Lit lit : states[i]) {
            Var var = VarOf(lit);

            auto it = var_signatures.find(var);
            if (it == var_signatures.end()) {
                auto inserted = var_signatures.emplace(var, DynamicSignature(num_chunks, 0));
                it = inserted.first;
                known_counts[var] = 0;
                vars.emplace_back(var);
            }

            known_counts[var]++;
            if (!Sign(lit)) it->second[i / 64] |= (uint64_t{1} << (i % 64));
        }
    }

    for (Var var : vars) {
        if (known_counts[var] == num_bits) insert_signature(MkLit(var), var_signatures[var]);
    }
    insert_signature(LIT_FALSE, DynamicSignature(num_chunks, 0));
}


void Model::EncodeTernaryValuesToBitSignatures(const vector<vector<Tbool>> &values, const Cube &vars, DynamicSignatureMap &signatures) {
    const size_t num_bits = values.size();
    if (num_bits == 0) return;
    const size_t num_chunks = (num_bits + 63) / 64;
    const size_t last_bits = num_bits % 64;
    const uint64_t last_mask = (last_bits == 0) ? UINT64_MAX : ((uint64_t{1} << last_bits) - 1);

    auto insert_signature = [&](Lit lit, const DynamicSignature &signature) {
        DynamicSignature neg_signature(signature);
        for (uint64_t &chunk : neg_signature) chunk = ~chunk;
        neg_signature.back() &= last_mask;

        if (signatures.find(signature) != signatures.end()) {
            signatures[signature].emplace_back(lit);
        } else if (signatures.find(neg_signature) != signatures.end()) {
            signatures[neg_signature].emplace_back(~lit);
        } else {
            signatures[signature].emplace_back(lit);
        }
    };

    for (Lit lit : vars) {
        if (IsConst(lit)) continue;

        DynamicSignature signature(num_chunks, 0);
        bool known = true;
        for (size_t i = 0; i < values.size(); ++i) {
            const auto &vmap = values[i];
            if (VarOf(lit) >= vmap.size()) {
                known = false;
                break;
            }
            Tbool value = vmap[VarOf(lit)];
            if (Sign(lit)) value = !value;
            if (value == T_TRUE)
                signature[i / 64] |= (uint64_t{1} << (i % 64));
            else if (value == T_FALSE)
                continue;
            else {
                known = false;
                break;
            }
        }
        if (!known) continue;

        insert_signature(lit, signature);
    }
    insert_signature(LIT_FALSE, DynamicSignature(num_chunks, 0));
}


bool Model::TryGetConstInit(Lit lit, Lit &out) const {
    if (IsConst(lit)) {
        out = lit;
        return true;
    }

    auto it = m_circuitGraph->latchResetMap.find(VarOf(lit));
    if (it == m_circuitGraph->latchResetMap.end()) return false;

    Lit reset = it->second;
    if (reset != LIT_TRUE && reset != LIT_FALSE) return false;

    out = Sign(lit) ? ~reset : reset;
    return true;
}


void Model::ResetLatchEquivalenceSolvers() {
    m_latchEqBaseSolver = nullptr;
    m_latchEqIndSolver = nullptr;
}


void Model::EnsureLatchEqBaseSolver() {
    if (m_latchEqBaseSolver != nullptr) return;

    if (m_cnfClauses.empty()) {
        CollectInitialState();
        CollectConstraints();
        CollectNextValueMapping();
        CollectClauses();
        SimplifyDAGClauses();
        CollectCNFClauses();
        UpdateDependencyVecDAGCNF();
    }

    m_latchEqBaseSolver = make_unique<minicore::Solver>();
    m_latchEqBaseSolver->setRestartLimit(1);
    m_latchEqBaseSolver->newVarUntil(static_cast<minicore::Var>(m_maxId));
    for (const Clause &c : m_cnfClauses) m_latchEqBaseSolver->addClause(c);
    for (Lit c : m_constraints) m_latchEqBaseSolver->addClause(Clause{c});
    for (Lit lit : m_initialState) m_latchEqBaseSolver->addClause(Clause{lit});
}


void Model::EnsureLatchEqIndSolver() {
    if (m_latchEqIndSolver != nullptr) return;

    if (m_cnfClauses.empty()) {
        ApplyEquivalence();
        CollectConstraints();
        CollectNextValueMapping();
        CollectClauses();
        SimplifyDAGClauses();
        CollectCNFClauses();
        UpdateDependencyVecDAGCNF();
    }

    m_latchEqIndSolver = make_unique<minicore::Solver>();
    m_latchEqIndSolver->setRestartLimit(1);
    m_latchEqIndSolver->newVarUntil(static_cast<minicore::Var>(m_maxId));
    for (const Clause &c : m_cnfClauses) m_latchEqIndSolver->addClause(c);
    for (Lit c : m_constraints) m_latchEqIndSolver->addClause(Clause{c});
    m_latchEqIndSolver->setSolveInDomain(true);

    Cube constraints_domain = GetCOIDomain(m_constraints);
    std::vector<char> &dom = m_latchEqIndSolver->domainSet();
    std::vector<minicore::Var> &list = m_latchEqIndSolver->domainList();
    for (Lit v : constraints_domain) {
        Var vv = VarOf(v);
        if (!dom[vv]) {
            dom[vv] = 1;
            list.push_back(vv);
        }
    }
}


bool Model::CheckLatchEquivalenceBase(Lit aLit, Lit bLit) {
    Lit init_a, init_b;
    bool has_init_a = TryGetConstInit(aLit, init_a);
    bool has_init_b = TryGetConstInit(bLit, init_b);

    if (has_init_a && has_init_b) return init_a == init_b;
    if (!m_hasResetGateInit) return false;

    EnsureLatchEqBaseSolver();
    Clause c1 = ToCNFClause(Clause{aLit, bLit});
    Clause c2 = ToCNFClause(Clause{~aLit, ~bLit});
    m_latchEqBaseSolver->addTempClause(c1);
    m_latchEqBaseSolver->addTempClause(c2);

    minicore::lbool res = m_latchEqBaseSolver->solve();
    m_latchEqBaseSolver->releaseTempClause();
    return res == minicore::l_False;
}


bool Model::CheckLatchEquivalenceInd(Lit aLit, Lit bLit) {
    EnsureLatchEqIndSolver();
    std::vector<char> &dom = m_latchEqIndSolver->domainSet();
    std::vector<minicore::Var> &list = m_latchEqIndSolver->domainList();
    size_t base_domain_size = list.size();

    auto restore_domain = [&]() {
        for (size_t i = base_domain_size; i < list.size(); ++i) {
            dom[list[i]] = 0;
        }
        list.resize(base_domain_size);
    };

    Lit a_prime = IsConst(aLit) ? aLit : LookupPrime(aLit);
    Lit b_prime = IsConst(bLit) ? bLit : LookupPrime(bLit);
    {
        Clause c1 = ToCNFClause(Clause{aLit, ~bLit});
        Clause c2 = ToCNFClause(Clause{~aLit, bLit});
        Clause c3 = ToCNFClause(Clause{a_prime, b_prime});
        Clause c4 = ToCNFClause(Clause{~a_prime, ~b_prime});
        m_latchEqIndSolver->addTempClause(c1);
        m_latchEqIndSolver->addTempClause(c2);
        m_latchEqIndSolver->addTempClause(c3);
        m_latchEqIndSolver->addTempClause(c4);
    }

    Cube d = GetCOIDomain(Cube{aLit, bLit, a_prime, b_prime});
    {
        for (Lit v : d) {
            Var vv = VarOf(v);
            if (!dom[vv]) {
                dom[vv] = 1;
                list.push_back(vv);
            }
        }
    }

    minicore::lbool res = m_latchEqIndSolver->solve();
    bool unsat = (res == minicore::l_False);
    restore_domain();
    m_latchEqIndSolver->releaseTempClause();

    if (unsat) {
        Clause c1 = ToCNFClause(Clause{aLit, ~bLit});
        Clause c2 = ToCNFClause(Clause{~aLit, bLit});
        m_latchEqIndSolver->addClause(c1);
        m_latchEqIndSolver->addClause(c2);
    }
    return unsat;
}


bool Model::CheckLatchEquivalenceBySAT(Lit aLit, Lit bLit) {
    if (!CheckLatchEquivalenceBase(aLit, bLit)) return false;
    return CheckLatchEquivalenceInd(aLit, bLit);
}


bool Model::CheckGateEquivalenceBySAT(Lit aLit, Lit bLit) {
    if (m_gateEqSolver == nullptr) {
        if (m_cnfClauses.empty()) {
            ApplyEquivalence();
            CollectConstraints();
            CollectNextValueMapping();
            CollectClauses();
            SimplifyDAGClauses();
            CollectCNFClauses();
            UpdateDependencyVecDAGCNF();
        }

        m_gateEqSolver = make_unique<minicore::Solver>();
        m_gateEqSolver->setRestartLimit(1);
        m_gateEqSolver->newVarUntil(static_cast<minicore::Var>(m_maxId));
        for (const Clause &c : m_cnfClauses) m_gateEqSolver->addClause(c);
        m_gateEqSolver->setSolveInDomain(true);
    }

    // (a <-> b)
    // !(a <-> b) is unsat
    // ((a & !b) | (b & !a))
    // (a | b) & (!a | !b)
    std::vector<char> &dom = m_gateEqSolver->domainSet();
    std::vector<minicore::Var> &list = m_gateEqSolver->domainList();
    size_t base_domain_size = list.size();

    auto restore_domain = [&]() {
        for (size_t i = base_domain_size; i < list.size(); ++i) {
            dom[list[i]] = 0;
        }
        list.resize(base_domain_size);
    };

    {
        Clause c1 = ToCNFClause(Clause{aLit, bLit});
        Clause c2 = ToCNFClause(Clause{~aLit, ~bLit});
        m_gateEqSolver->addTempClause(c1);
        m_gateEqSolver->addTempClause(c2);
    }

    Cube d = GetCOIDomain(Cube{aLit, bLit});
    {
        for (Lit v : d) {
            Var vv = VarOf(v);
            if (!dom[vv]) {
                dom[vv] = 1;
                list.push_back(vv);
            }
        }
    }

    minicore::lbool res = m_gateEqSolver->solve();
    bool unsat = (res == minicore::l_False);
    restore_domain();
    m_gateEqSolver->releaseTempClause();

    if (unsat) {
        Clause c1 = ToCNFClause(Clause{aLit, ~bLit});
        Clause c2 = ToCNFClause(Clause{~aLit, bLit});
        m_gateEqSolver->addClause(c1);
        m_gateEqSolver->addClause(c2);
    }
    return unsat;
}


int Model::KLivenessIncrement() {
    Var latch = NewLatchVar();
    Lit q = m_bad;
    // Init(k) = false
    // Next(k) = q ? true : k
    Lit init = LIT_FALSE;
    Lit next = MakeITE(q, LIT_TRUE, MkLit(latch));
    SetLatchReset(latch, init);
    SetLatchNext(latch, next);
    // q_k = q & k
    m_bad = MakeAND(q, MkLit(latch));

    m_kliveStep++;

    // store signals
    m_kliveSignals.resize(m_kliveStep + 1);
    m_kliveSignals[m_kliveStep] = MkLit(latch);

    // get clauses
    m_kliveTransClauses.resize(m_kliveStep + 1);
    vector<Clause> k_clauses;
    // and gate
    k_clauses.emplace_back(Clause{m_bad, ~q, ~MkLit(latch)});
    k_clauses.emplace_back(Clause{~m_bad, q});
    k_clauses.emplace_back(Clause{~m_bad, MkLit(latch)});
    // Ite gate
    k_clauses.emplace_back(Clause{next, ~q, LIT_FALSE});
    k_clauses.emplace_back(Clause{next, q, ~MkLit(latch)});
    k_clauses.emplace_back(Clause{~next, ~q, LIT_TRUE});
    k_clauses.emplace_back(Clause{~next, q, MkLit(latch)});
    vector<Clause> k_cnf_clauses;
    k_cnf_clauses.reserve(k_clauses.size());
    for (const Clause &cls : k_clauses) {
        k_cnf_clauses.emplace_back(ToCNFClause(cls));
    }
    m_kliveTransClauses[m_kliveStep] = k_cnf_clauses;

    // rebuild manually
    m_initialState.emplace_back(~MkLit(latch));
    SetPrimeMap0(latch, ToCNFLit(next));
    m_rawClauses.insert(m_rawClauses.end(), k_clauses.begin(), k_clauses.end());
    m_cnfClauses.insert(m_cnfClauses.end(), k_cnf_clauses.begin(), k_cnf_clauses.end());
    m_simpClauses.insert(m_simpClauses.end(), k_cnf_clauses.begin(), k_cnf_clauses.end());
    return m_kliveStep;
}

} // namespace car
