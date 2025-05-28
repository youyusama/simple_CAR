#ifndef AIGERBASEMODEL_H
#define AIGERBASEMODEL_H

extern "C" {
#include "aiger.h"
}
#ifndef CADICAL
#include "../sat/minisat/core/Solver.h"
#include "../sat/minisat/simp/SimpSolver.h"
using namespace Minisat;
#endif

#include "Settings.h"
#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace car;
using namespace std;

typedef vector<int> cube;
typedef vector<int> clause;

namespace car {

inline bool cmp(int a, int b) {
    return abs(a) < abs(b);
}

class Model {
  public:
    Model(Settings settings);

    inline bool IsTrue(const unsigned lit) {
        return (lit == 1) || (m_trues.find(lit) != m_trues.end());
    }

    inline bool IsFalse(const unsigned lit) {
        return (lit == 0) || (m_trues.find(aiger_not(lit)) != m_trues.end());
    }

    inline bool IsConstant(const int id) {
        unsigned lit = id > 0 ? id * 2 : -id * 2 + 1;
        if (IsTrue(lit) || IsFalse(lit))
            return true;
        else
            return false;
    }

    inline bool IsLatch(int id) {
        if (abs(id) > m_aig->num_inputs && abs(id) <= m_aig->num_inputs + m_aig->num_latches)
            return true;
        else
            return false;
    }

    inline bool IsInput(int id) {
        if (abs(id) > 0 && abs(id) <= m_aig->num_inputs)
            return true;
        else
            return false;
    }

    inline bool IsInnard(int id) {
        if (m_innards->find(abs(id)) != m_innards->end()) {
            return true;
        } else {
            return false;
        }
    }

    inline int GetCarId(const unsigned lit) {
        if (lit == 0)
            return m_falseId;
        else if (lit == 1)
            return m_trueId;
        return (aiger_sign(lit) == 0) ? lit >> 1 : -(lit >> 1);
    }

    inline aiger *GetAig() { return m_aig; }

    inline int GetNumInputs() { return m_aig->num_inputs; }
    inline int GetNumLatches() { return m_aig->num_latches; }
    inline int GetNumBad() { return m_aig->num_outputs + m_aig->num_bad; }
    inline int GetMaxId() { return m_maxId; }
    inline int GetOutputsStart() { return m_outputsStart; }
    inline int GetLatchesStart() { return m_latchesStart; }
    inline int GetTrueId() { return m_trueId; }
    inline int GetFalseId() { return m_falseId; }
    inline cube &GetInitialState() { return m_initialState; }
    inline int &GetBad() { return m_bad; }

    inline vector<int> GetPrevious(int id) {
        if (m_preValueOfLatchMap.count(abs(id)) > 0) {
            return m_preValueOfLatchMap[abs(id)];
        } else {
            return vector<int>();
        }
    }

    inline int GetPrime(const int id) {
        unordered_map<int, int>::iterator it = m_primeMaps[0].find(abs(id));
        assert(it != m_primeMaps[0].end());
        return id > 0 ? it->second : -(it->second);
    }

    int GetPrimeK(const int id, int k);

    vector<clause> &GetClauses() { return m_clauses; }

    inline int GetProperty() { return -m_bad; }
#ifndef CADICAL
    shared_ptr<SimpSolver> GetSimpSolver();
#endif
    void GetPreValueOfLatchMap(unordered_map<int, vector<int>> &map);

    vector<int> GetConstraints() { return m_constraints; };

    shared_ptr<cube> GetInnardsImplied(shared_ptr<cube> uc);

    int GetClauseOfInnards(shared_ptr<cube> innards, vector<cube> &clss);

    shared_ptr<set<int>> GetInnards() { return m_innards; };

    int GetInnardslvl(int id) {
        unordered_map<int, int>::iterator it = m_innards_lvl.find(abs(id));
        if (it == m_innards_lvl.end()) return 0;
        return it->second;
    }

  private:
    void Init();

    void CollectConstants();

    void CollectConstraints();

    void CollectBad();

    void CollectInitialState();

    void CollectNextValueMapping();

    void CollectClauses();

    void CollectNecessaryAndGates(const aiger_symbol *as, const int as_size,
                                  std::unordered_set<unsigned> &exist_gates, std::vector<unsigned> &gates, bool next);

    void CollectNecessaryAndGatesFromConstraints(unordered_set<unsigned> &exist_gates, vector<unsigned> &gates);

    void FindAndGates(const aiger_and *aa, unordered_set<unsigned> &exist_gates, vector<unsigned> &gates);

    void AddAndGateToClause(const aiger_and *aa);

    inline void InsertIntoPreValueMapping(const int key, const int value);

    inline aiger_and *IsAndGate(const unsigned id);

    int InnardsLogiclvlDFS(unsigned aig_id);

    void CollectInnards();

    void CollectInnardsClauses();

    Settings m_settings;
    aiger *m_aig;
    int m_maxId;
    int m_trueId;
    int m_falseId;
    int m_outputsStart; // the index of cls_ to point the start position of outputs
    int m_latchesStart; // the index of cls_ to point the start position of latches

    cube m_initialState;
    int m_bad;
    vector<int> m_constraints;
    vector<clause> m_clauses;   // CNF, e.g. (a|b|c) * (-a|c)
    unordered_set<int> m_trues; // variables that are always true

    vector<unordered_map<int, int>> m_primeMaps;
    unordered_map<int, vector<int>> m_preValueOfLatchMap;

    shared_ptr<set<int>> m_innards;
    unordered_map<int, int> m_innards_lvl;

#ifndef CADICAL
    void CreateSimpSolver();
    shared_ptr<SimpSolver> m_simpSolver;
#endif
};
} // namespace car

#endif