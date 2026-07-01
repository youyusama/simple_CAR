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
#include "WitnessBuilder.h"
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


using DynamicSignature = std::vector<uint64_t>;

struct DynamicSignatureHash {
    std::size_t operator()(const DynamicSignature &s) const {
        std::size_t h = 0;
        std::hash<uint64_t> hasher;
        for (uint64_t chunk : s) {
            h ^= hasher(chunk) + 0x9e3779b9 + (h << 6) + (h >> 2);
        }
        return h;
    }
};

using DynamicSignatureMap = std::unordered_map<DynamicSignature, std::vector<Lit>, DynamicSignatureHash>;

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
    Model(Settings settings, Log &log, std::shared_ptr<aiger> aig);

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
        size_t idx = PackedIndex(lit);
        assert(idx < m_lookupPrime.size());
        assert(m_lookupPrime[idx] != Lit{});
        return m_lookupPrime[idx];
    }

    Lit EnsurePrimeK(Lit id, int k);

    vector<Clause> &GetClauses() { return m_cnfClauses; }

    vector<Clause> &GetSimpClauses() { return m_simpClauses; }

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
    void InitializeFromAiger();

    void SetTsimReachedStateCubes(const std::vector<Cube> &cubes);

    void ApplyEquivalence();

    void EliminateGateResets();

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

    void SimplifyModelBySATSimulation();

    void EncodeStatesToSignatures(const vector<Cube> &states, DynamicSignatureMap &signatures);

    void EncodeTernaryValuesToBitSignatures(const vector<vector<Tbool>> &values, const Cube &vars, DynamicSignatureMap &signatures);

    bool CheckLatchEquivalenceBySAT(Lit a, Lit b);

    void ResetLatchEquivalenceSolvers();

    void EnsureLatchEqBaseSolver();

    void EnsureLatchEqIndSolver();

    bool CheckLatchEquivalenceBase(Lit a, Lit b);

    bool CheckLatchEquivalenceInd(Lit a, Lit b);

    bool TryGetConstInit(Lit lit, Lit &out) const;

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

    inline bool HasPrimeMap0(Var v) const {
        return m_primeMaps[0].find(v) != m_primeMaps[0].end();
    }

    inline void SetPrimeMap0(Var v, Lit prime_lit) {
        m_primeMaps[0][v] = prime_lit;

        size_t pos = PackedIndex(MkLit(v));
        size_t neg = PackedIndex(~MkLit(v));
        if (neg >= m_lookupPrime.size()) {
            m_lookupPrime.resize(neg + 1, Lit{});
        }
        m_lookupPrime[pos] = prime_lit;
        m_lookupPrime[neg] = ~prime_lit;
    }

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

    vector<unordered_map<Var, Lit, std::hash<Var>>> m_primeMaps;
    vector<Lit> m_lookupPrime;
    unordered_map<int, vector<int>> m_preValueOfLatchMap;

    vector<vector<Var>> m_dependencyVec;

    vector<vector<Var>> m_coiCache;
    vector<uint8_t> m_coiCacheReady;
    vector<uint8_t> m_coiVisited;
    vector<uint8_t> m_coiCacheVisited;
    vector<Var> m_coiDomain;
    vector<Var> m_coiCacheTodo;

    shared_ptr<EquivalenceManager> m_equivalenceManager;

    unique_ptr<minicore::Solver> m_gateEqSolver;
    bool m_hasResetGateInit{false};
    unique_ptr<minicore::Solver> m_latchEqBaseSolver;
    unique_ptr<minicore::Solver> m_latchEqIndSolver;

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
