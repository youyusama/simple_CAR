#ifndef MODEL_H
#define MODEL_H

extern "C" {
#include "aiger.h"
}

#include "CarTypes.h"
#include "CircuitGraph.h"
#include "Log.h"
#include "Settings.h"
#include "TernarySim.h"
#include "minicore/src/solver.h"
#include <algorithm>
#include <assert.h>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <math.h>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

namespace car {

class WitnessBuilder;

struct EquivalenceWitness {
    std::vector<Clause> equivalence_clauses;
    std::vector<Cube> reached_state_cubes;
    bool has_reached_state_region{false};
};

class EquivalenceManager {
  public:
    EquivalenceManager() {}
    ~EquivalenceManager() {}

    Lit FindLit(Lit a); // update and get the equivalence of a

    void AddEquivalence(Lit a, Lit b);

    inline bool IsEquivalent(Lit a, Lit b) { return FindLit(a) == FindLit(b); }

    inline bool HasEquivalence(Var a) { return m_equivalenceMap.count(a) > 0; }
    inline bool HasEquivalence(Lit a) { return HasEquivalence(VarOf(a)); }

    inline int Size() { return m_equivalenceMap.size(); }

    const unordered_map<Var, Lit, std::hash<Var>> &GetEquivalenceMap() const { return m_equivalenceMap; }

    void PrintEquivalenceMap() {
        for (const auto &it : m_equivalenceMap) {
            cout << it.first << " -> " << it.second << endl;
        }
    }

  private:
    unordered_map<Var, Lit, std::hash<Var>> m_equivalenceMap;

    Lit FindRootRecursive(Var key);
};


template <int N>
struct SimulationSignature {
    std::array<uint64_t, N> chunks;

    SimulationSignature() {
        chunks.fill(0);
    }

    bool operator==(const SimulationSignature<N> &other) const {
        return chunks == other.chunks;
    }

    SimulationSignature<N> operator~() const {
        SimulationSignature<N> result;
        for (int i = 0; i < N; i++) {
            result.chunks[i] = ~chunks[i];
        }
        return result;
    }
};


template <int N>
struct SimulationSignatureHash {
    std::size_t operator()(const SimulationSignature<N> &s) const {
        std::size_t h = 0;
        std::hash<uint64_t> hasher;
        for (const auto &chunk : s.chunks) {
            h ^= hasher(chunk) + 0x9e3779b9 + (h << 6) + (h >> 2);
        }
        return h;
    }
};

constexpr size_t NUM_CHUNKS = 128;
using SignatureN64 = SimulationSignature<NUM_CHUNKS>;
using VarMapN64 = std::unordered_map<SignatureN64, std::vector<Lit>, SimulationSignatureHash<NUM_CHUNKS>>;

class Model;

struct KLivenessCounter {
    unsigned int k = 0;
    int cur = 0;
    vector<Var> latches;
};

class Model {
  public:
    enum class PropKind {
        Safety,
        Liveness
    };

    Model(Settings settings, Log &log);

    inline Var TrueId() {
        return m_cnfTrueVar;
    }

    inline unsigned NumVar() {
        return m_circuitGraph->numVar;
    }

    inline bool IsTrue(Lit lit) {
        return m_equivalenceManager->FindLit(lit) == LIT_TRUE;
    }

    inline bool IsFalse(Lit lit) {
        return m_equivalenceManager->FindLit(lit) == LIT_FALSE;
    }

    inline bool IsConstant(Lit lit) {
        return IsTrue(lit) || IsFalse(lit);
    }

    inline bool IsLatch(Lit lit) {
        return m_circuitGraph->latchesSet.find(VarOf(lit)) != m_circuitGraph->latchesSet.end();
    }

    inline bool IsInput(Lit lit) {
        return m_circuitGraph->inputsSet.find(VarOf(lit)) != m_circuitGraph->inputsSet.end();
    }

    inline bool IsAnd(Lit lit) {
        return m_circuitGraph->andsSet.find(VarOf(lit)) != m_circuitGraph->andsSet.end();
    }

    inline shared_ptr<aiger> GetAiger() { return m_aiger; }
    inline shared_ptr<const aiger> GetAiger() const { return m_aiger; }

    inline CircuitGraph *GetCircuitGraph() { return m_circuitGraph.get(); }
    inline const CircuitGraph *GetCircuitGraph() const { return m_circuitGraph.get(); }

    inline int GetNumInputs() const { return m_circuitGraph->numInputs; }
    inline int GetNumLatches() const { return m_circuitGraph->numLatches; }
    inline Cube &GetInitialState() { return m_initialState; }

    inline vector<Var> &GetModelInputs() { return m_circuitGraph->modelInputs; }
    inline vector<Var> &GetModelLatches() { return m_circuitGraph->modelLatches; }
    inline vector<Var> &GetModelGates() { return m_circuitGraph->modelGates; }

    inline Lit GetBad() { return ToCNFLit(m_bad); }
    inline Lit GetProperty() { return ~ToCNFLit(m_bad); }

    int GetKLiveStep() { return m_kliveStep; }
    int KLivenessIncrement();
    Lit GetKLiveSignal(int k) { return m_kliveSignals[k]; }
    vector<Clause> GetKLiveClauses(int k) { return m_kliveTransClauses[k]; }

    inline PropKind GetPropKind() const { return m_propKind; }

    inline Lit LookupPrime(Lit lit) {
        assert(m_primeMaps[0].find(VarOf(lit)) != m_primeMaps[0].end());
        Lit prime_lit = m_primeMaps[0][VarOf(lit)];
        return Sign(lit) ? ~prime_lit : prime_lit;
    }

    Lit EnsurePrimeK(Lit id, int k);

    vector<Clause> &GetClauses() { return m_cnfClauses; }

    vector<Clause> &GetSimpClauses() { return m_simpClauses; }

    vector<Clause> &GetInitialClauses() { return m_initialClauses; }

    const Cube &GetConstraints() { return m_constraints; };

    inline bool IsInnard(int id) {
        if (m_settings.internalSignals &&
            m_innards.find(AbsLit(id)) != m_innards.end()) {
            return true;
        } else {
            return false;
        }
    }

    vector<Var> &GetInnards() { return m_innardsVec; };

    int GetInnardslvl(Var id) {
        unordered_map<Var, int>::iterator it = m_innardsLvl.find(id);
        if (it == m_innardsLvl.end()) return 0;
        return it->second;
    }

    int GetInnardslvl(Lit lit) {
        return GetInnardslvl(VarOf(lit));
    }

    vector<Var> &GetPropertyCOIInputs() { return m_circuitGraph->propertyCOIInputs; };

    Cube GetCOIDomain(const Cube &c);

    const vector<vector<Var>> &GetDependencyVec() const { return m_dependencyVec; }

    const unordered_map<Var, Lit, std::hash<Var>> &GetEquivalenceMap() const {
        return m_equivalenceManager->GetEquivalenceMap();
    }

    void RefineWitnessPropertyLit(WitnessBuilder &builder);

    Lit GetLatchResetLit(Var latch) const;
    Lit GetLatchNextLit(Var latch) const;
    void SetLatchReset(Var latch, Lit reset);
    void SetLatchNext(Var latch, Lit next);
    void SetBad(Lit bad);
    void Rebuild();
    Var NewInputVar();
    Var NewLatchVar();
    Var GetNewVar() { return ++m_maxId; }
    Lit MakeAND(Lit a, Lit b);
    Lit MakeOR(Lit a, Lit b);
    Lit MakeXOR(Lit a, Lit b);
    Lit MakeXNOR(Lit a, Lit b);
    Lit MakeITE(Lit i, Lit t, Lit e);

  private:
    void SetTsimReachedStateCubes(const std::vector<Cube> &cubes);

    void ApplyEquivalence();

    void UpdateDependencyMap();

    void UpdateDependencyVecDAGCNF();

    void CollectConstraints();

    void CollectInitialState();

    void CollectNextValueMapping();

    void CollectCNFClauses();

    void CollectClauses();

    int InnardsLogiclvlDFS(Var id);

    void CollectInnards();

    void SimplifyClauses();

    void SimplifyDAGClauses();

    Lit BuildLiveness();

    Lit BuildSingleFairness(const Cube &conds);

    bool SimplifyModelByTernarySimulation();

    void SimplifyModelByRandomSimulation();

    void EncodeStatesToSignatuers(const vector<Cube> &states, unordered_map<string, vector<Lit>> &signatures);

    void EncodeStatesToN64Signatuers(const vector<vector<Tbool>> &values, const Cube &vars, VarMapN64 &signatures);

    bool CheckLatchEquivalenceBySAT(Lit a, Lit b);

    bool CheckGateEquivalenceBySAT(Lit a, Lit b);

    void EnsureCOICache(Var v);

    void BuildEquivalenceWitness();

    void BuildEquivalenceClauses(std::vector<Clause> &out);

    void NormalizeReachedStateRegion(EquivalenceWitness &witness);

    const EquivalenceWitness &GetEquivalenceWitness();

    inline Lit ToCNFLit(Lit lit) const {
        if (!IsConst(lit)) return lit;
        return IsConstTrue(lit) ? MkLit(m_cnfTrueVar) : ~MkLit(m_cnfTrueVar);
    }

    Clause ToCNFClause(const Clause &cls) const;

    Settings m_settings;
    Log &m_log;
    shared_ptr<aiger> m_aiger;
    shared_ptr<CircuitGraph> m_circuitGraph;

    Var m_cnfTrueVar{0};
    Var m_maxId;
    Cube m_initialState;
    Cube m_constraints;
    Lit m_bad;
    KLivenessCounter m_kliveCounter;
    PropKind m_propKind{PropKind::Safety};
    vector<Clause> m_rawClauses;
    vector<Clause> m_cnfClauses; // CNF, e.g. (a|b|c) * (-a|c)
    vector<Clause> m_simpClauses;
    vector<Clause> m_initialClauses;

    vector<unordered_map<Var, Lit, std::hash<Var>>> m_primeMaps;
    unordered_map<int, vector<int>> m_preValueOfLatchMap;

    vector<vector<Var>> m_dependencyVec;

    vector<vector<Var>> m_coiCache;
    vector<uint8_t> m_coiCacheReady;
    vector<uint8_t> m_coiVisited;
    vector<uint8_t> m_coiCacheVisited;
    vector<Var> m_coiDomain;
    vector<Var> m_coiCacheTodo;

    shared_ptr<EquivalenceManager> m_equivalenceManager;

    unique_ptr<minicore::Solver> m_equivalenceSolver;
    int m_eqSolverUnsats{0};

    EquivalenceWitness m_equivalenceWitness;
    bool m_equivalenceWitnessReady{false};

    unordered_set<Var> m_innards;
    vector<Var> m_innardsVec;
    unordered_map<Var, int> m_innardsLvl;

    int m_kliveStep{0};
    Cube m_kliveSignals;
    vector<vector<Clause>> m_kliveTransClauses;
};
} // namespace car

#endif
