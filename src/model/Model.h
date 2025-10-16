#ifndef MODEL_H
#define MODEL_H

extern "C" {
#include "aiger.h"
}

#include "CircuitGraph.h"
#include "Log.h"
#include "Settings.h"
#include "TernarySim.h"
#include "cadical/src/cadical.hpp"
#include <algorithm>
#include <assert.h>
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

typedef vector<int> cube;
typedef vector<int> clause;

namespace car {

class EquivalenceManager {
  public:
    EquivalenceManager() {}
    ~EquivalenceManager() {}

    int Find(int a); // update and get the equivalence of a

    void AddEquivalence(int a, int b);

    inline bool IsEquivalent(int a, int b) { return Find(a) == Find(b); }

    inline bool HasEquivalence(int a) { return m_equivalenceMap.count(abs(a)) > 0; }

    inline int Size() { return m_equivalenceMap.size(); }

    const unordered_map<int, int> &GetEquivalenceMap() const { return m_equivalenceMap; }

  private:
    unordered_map<int, int> m_equivalenceMap;

    pair<int, int> FindRootRecursive(int key);
};


template <int N>
struct SimulationSignature {
    std::array<uint64_t, N> chunks;

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

constexpr size_t NUM_CHUNKS = 16;
using SignatureN64 = SimulationSignature<NUM_CHUNKS>;
using VarMapN64 = std::unordered_map<SignatureN64, std::vector<int>, SimulationSignatureHash<NUM_CHUNKS>>;


class Model {
  public:
    Model(Settings settings, shared_ptr<Log> log);

    inline int TrueId() {
        return m_circuitGraph->trueId;
    }

    inline bool IsTrue(const int id) {
        return m_equivalenceManager->Find(id) == m_equivalenceManager->Find(TrueId());
    }

    inline bool IsFalse(const int id) {
        return m_equivalenceManager->Find(id) == -m_equivalenceManager->Find(TrueId());
    }

    inline bool IsConstant(const int id) {
        if (IsTrue(id) || IsFalse(id))
            return true;
        else
            return false;
    }

    inline bool IsLatch(int id) {
        if (m_circuitGraph->latchesSet.find(abs(id)) != m_circuitGraph->latchesSet.end())
            return true;
        else
            return false;
    }

    inline bool IsInput(int id) {
        if (m_circuitGraph->inputsSet.find(abs(id)) != m_circuitGraph->inputsSet.end())
            return true;
        else
            return false;
    }


    inline bool IsAnd(int id) {
        if (m_circuitGraph->andsSet.find(abs(id)) != m_circuitGraph->andsSet.end())
            return true;
        else
            return false;
    }


    inline int GetCarId(const unsigned lit) {
        if (lit == 0)
            return -TrueId();
        else if (lit == 1)
            return TrueId();
        return (aiger_sign(lit) == 0) ? lit >> 1 : -(lit >> 1);
    }

    inline unsigned GetAigerLit(const int car_id) {
        if (car_id > 0)
            return car_id << 1;
        else
            return (-car_id << 1) + 1;
    }

    inline shared_ptr<aiger> GetAiger() { return m_aiger; }

    inline int GetNumInputs() { return m_circuitGraph->numInputs; }
    inline int GetNumLatches() { return m_circuitGraph->numLatches; }
    inline vector<int> &GetInitialState() { return m_initialState; }

    inline vector<int> &GetModelInputs() { return m_circuitGraph->modelInputs; }
    inline vector<int> &GetModelLatches() { return m_circuitGraph->modelLatches; }
    inline vector<int> &GetModelGates() { return m_circuitGraph->modelGates; }

    inline int GetBad() { return m_bad; }
    inline int GetProperty() { return -m_bad; }

    inline int GetPrime(const int id) {
        unordered_map<int, int>::iterator it = m_primeMaps[0].find(abs(id));
        if (it == m_primeMaps[0].end()) return 0;
        return id > 0 ? it->second : -(it->second);
    }

    int GetPrimeK(const int id, int k);

    vector<clause> &GetClauses() { return m_clauses; }

    vector<clause> &GetSimpClauses() { return m_simpClauses; }

    vector<clause> &GetInitialClauses() { return m_initialClauses; }

    vector<int> GetConstraints() { return m_circuitGraph->constraints; };

    inline bool IsInnard(int id) {
        if (m_settings.internalSignals &&
            m_innards.find(abs(id)) != m_innards.end()) {
            return true;
        } else {
            return false;
        }
    }

    vector<int> &GetInnards() { return m_innardsVec; };

    int GetInnardslvl(int id) {
        unordered_map<int, int>::iterator it = m_innards_lvl.find(abs(id));
        if (it == m_innards_lvl.end()) return 0;
        return it->second;
    }

    vector<int> &GetPropertyCOIInputs() { return m_circuitGraph->propertyCOIInputs; };

    shared_ptr<cube> GetCOIDomain(const shared_ptr<cube> c);

    const unordered_map<int, int> &GetEquivalenceMap() const {
        return m_equivalenceManager->GetEquivalenceMap();
    }

  private:
    void ApplyEquivalence();

    void UpdateDependencyMap();

    void CollectInitialState();

    void CollectNextValueMapping();

    void CollectClauses();

    int InnardsLogiclvlDFS(int id);

    void CollectInnards();

    void SimplifyClauses();

    bool SimplifyModelByTernarySimulation();

    void SimplifyModelByRandomSimulation();

    void EncodeStatesToSignatuers(const vector<shared_ptr<vector<int>>> &states, unordered_map<string, vector<int>> &signatures);

    void EncodeStatesToN64Signatuers(const vector<shared_ptr<unordered_map<int, tbool>>> &values, const vector<int> &vars, VarMapN64 &signatures);

    bool CheckLatchEquivalenceBySAT(int a, int b);

    bool CheckGateEquivalenceBySAT(int a, int b);

    inline int GetNewId() { return ++m_maxId; };

    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<aiger> m_aiger;
    shared_ptr<CircuitGraph> m_circuitGraph;

    int m_maxId;
    cube m_initialState;
    int m_bad;
    vector<clause> m_clauses; // CNF, e.g. (a|b|c) * (-a|c)
    vector<clause> m_simpClauses;
    vector<clause> m_initialClauses;

    vector<unordered_map<int, int>> m_primeMaps;
    unordered_map<int, vector<int>> m_preValueOfLatchMap;

    unordered_map<int, unordered_set<int>> m_dependencyMap;

    shared_ptr<EquivalenceManager> m_equivalenceManager;

    shared_ptr<CaDiCaL::Solver> m_equivalenceSolver;
    int m_eqSolverUnsats;

    unordered_set<int> m_innards;
    vector<int> m_innardsVec;
    unordered_map<int, int> m_innards_lvl;
};
} // namespace car

#endif