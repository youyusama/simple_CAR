#ifndef AIGERBASEMODEL_H
#define AIGERBASEMODEL_H

extern "C" {
#include "aiger.h"
}

#include "Log.h"
#include "Settings.h"
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

inline bool cmp(int a, int b) {
    if (abs(a) != abs(b))
        return abs(a) < abs(b);
    else
        return a < b;
}

class Model {
  public:
    Model(Settings settings, shared_ptr<Log> log);

    inline bool IsTrue(const unsigned lit) {
        return (lit == 1) || (m_trues.find(lit) != m_trues.end());
    }

    inline bool IsFalse(const unsigned lit) {
        return (lit == 0) || (m_trues.find(aiger_not(lit)) != m_trues.end());
    }

    inline bool IsConstant(const int id) {
        if (IsTrue(GetAigerLit(id)) || IsFalse(GetAigerLit(id)))
            return true;
        else
            return false;
    }

    inline bool IsLatch(int id) {
        if (abs(id) > m_aig->num_inputs && abs(id) < m_andGateStartId)
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

    inline bool IsAnd(int id) {
        if (abs(id) >= m_andGateStartId && abs(id) < m_trueId)
            return true;
        else
            return false;
    }

    inline bool IsInnard(int id) {
        if (m_settings.internalSignals &&
            m_innards->find(abs(id)) != m_innards->end()) {
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

    inline unsigned GetAigerLit(const int car_id) {
        if (car_id > 0)
            return car_id << 1;
        else
            return (-car_id << 1) + 1;
    }

    inline aiger *GetAig() { return m_aig; }

    inline int GetNumInputs() { return m_aig->num_inputs; }
    inline int GetNumLatches() { return m_aig->num_latches; }
    inline int GetNumBad() { return m_aig->num_outputs + m_aig->num_bad; }
    inline int GetMaxId() { return m_maxId; }
    inline int GetTrueId() { return m_trueId; }
    inline int GetFalseId() { return m_falseId; }
    inline cube &GetInitialState() { return m_initialState; }
    inline int &GetBad() { return m_bad; }
    inline int GetProperty() { return -m_bad; }

    inline vector<int> GetPrevious(int id) {
        if (m_preValueOfLatchMap.count(abs(id)) > 0) {
            return m_preValueOfLatchMap[abs(id)];
        } else {
            return vector<int>();
        }
    }

    inline int GetPrime(const int id) {
        unordered_map<int, int>::iterator it = m_primeMaps[0].find(abs(id));
        if (it == m_primeMaps[0].end()) return 0;
        return id > 0 ? it->second : -(it->second);
    }

    int GetPrimeK(const int id, int k);

    vector<clause> &GetClauses() { return m_clauses; }

    vector<clause> &GetSimpClauses() { return m_simpClauses; }

    vector<clause> &GetInitialClauses() { return m_initialClauses; }

    void GetPreValueOfLatchMap(unordered_map<int, vector<int>> &map);

    vector<int> GetConstraints() { return m_constraints; };

    shared_ptr<vector<int>> GetInnards() { return m_innardsVec; };

    int GetInnardslvl(int id) {
        unordered_map<int, int>::iterator it = m_innards_lvl.find(abs(id));
        if (it == m_innards_lvl.end()) return 0;
        return it->second;
    }

    shared_ptr<vector<int>> GetCOIInputs() { return m_COIInputs; };

    shared_ptr<cube> GetCOIDomain(const shared_ptr<cube> c);

  private:
    void Init();

    void CollectConstants();

    void CollectConstraints();

    void CollectBad();

    void CollectInitialState();

    void CollectNextValueMapping();

    void CollectClauses();

    bool TryConvertXOR(const unsigned a, unordered_set<unsigned> &coi_lits);

    bool TryConvertITE(const unsigned a, unordered_set<unsigned> &coi_lits);

    void ConvertAND(const unsigned a, unordered_set<unsigned> &coi_lits);

    int InnardsLogiclvlDFS(unsigned aig_id);

    void CollectInnards();

    void CollectInnardsClauses();

    void CollectCOIInputs();

    void SimplifyClauses();

    Settings m_settings;
    shared_ptr<Log> m_log;
    aiger *m_aig;
    int m_maxId;
    int m_trueId;
    int m_falseId;
    int m_andGateStartId;

    cube m_initialState;
    int m_bad;
    vector<int> m_constraints;
    vector<clause> m_clauses; // CNF, e.g. (a|b|c) * (-a|c)
    vector<clause> m_simpClauses;
    vector<clause> m_initialClauses;
    unordered_set<int> m_trues; // variables that are always true

    vector<unordered_map<int, int>> m_primeMaps;
    unordered_map<int, vector<int>> m_preValueOfLatchMap;

    unordered_map<int, vector<int>> m_dependencyMap;

    shared_ptr<unordered_set<int>> m_innards;
    shared_ptr<vector<int>> m_innardsVec;
    unordered_map<int, int> m_innards_lvl;

    shared_ptr<vector<int>> m_COIInputs;
};
} // namespace car

#endif