#ifndef AIGERBASEMODEL_H
#define AIGERBASEMODEL_H

extern "C" {
#include "aiger.h"
}
#include "../sat/minisat/core/Solver.h"
#include "../sat/minisat/simp/SimpSolver.h"
#include "Settings.h"
#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <tuple>

typedef std::string string;
typedef std::vector<int> cube;
template <typename T>
using sptr = std::shared_ptr<T>;
using namespace Minisat;

namespace car {

class AigerModel {
public:
  AigerModel(Settings settings);

  bool IsTrue(const unsigned id) {
    return (id == 1) || (m_trues.find(id) != m_trues.end());
  }

  bool IsFalse(const unsigned id) {
    return (id == 0) || (m_trues.find((id % 2 == 0) ? (id + 1) : (id - 1)) != m_trues.end());
  }

  bool IsLatch(int id) {
    if (abs(id) > m_numInputs && abs(id) <= m_numInputs + m_numLatches) {
      return true;
    } else {
      return false;
    }
  }

  bool IsInput(int id) {
    if (abs(id) > 0 && abs(id) <= m_numInputs) {
      return true;
    } else {
      return false;
    }
  }

  int GetCarId(const unsigned id) {
    if (id == 0) {
      // placeholder
    }
    return ((id % 2 == 0) ? (id / 2) : -(id / 2));
  }

  std::shared_ptr<std::vector<int>> Get_next_latches_for_pine(std::vector<int> &current_latches);

#pragma region get &set
  int GetNumInputs() { return m_numInputs; }
  int GetNumLatches() { return m_numLatches; }
  int GetNumOutputs() { return m_numOutputs; }
  int GetMaxId() { return m_maxId; }
  int GetOutputsStart() { return m_outputsStart; }
  int GetLatchesStart() { return m_latchesStart; }
  int GetTrueId() { return m_trueId; }
  int GetFalseId() { return m_falseId; }
  std::vector<int> &GetInitialState() { return m_initialState; }
  std::vector<int> &GetOutputs() { return m_outputs; }
  std::vector<int> GetPrevious(int id) {
    if (m_preValueOfLatch.count(abs(id)) > 0) {
      return m_preValueOfLatch[abs(id)];
    } else {
      return std::vector<int>();
    }
  }
  int GetPrime(const int id) {
    std::unordered_map<int, int>::iterator it = m_nextValueOfLatch.find(abs(id));
    if (it == m_nextValueOfLatch.end()) {
      return 0;
    }
    return id > 0 ? it->second : -(it->second);
  }


  std::vector<std::vector<int>> &GetClause() { return m_clauses; }

  std::vector<int> GetNegBad();

  sptr<std::vector<float>> get_aiger_order();

  sptr<SimpSolver> get_sslv();
#pragma endregion

#pragma region private methods
private:
  void Init(aiger *aig);

  void CollectTrues(const aiger *aig);

  void CollectTrues_RECUR(const aiger *aig);

  void CollectConstraints(const aiger *aig);

  void CollectOutputs(const aiger *aig);

  void CollectInitialState(const aiger *aig);

  void CollectNextValueMapping(const aiger *aig);

  void CollectClauses(const aiger *aig);

  void CollectNecessaryAndGates(const aiger *aig, const aiger_symbol *as, const int as_size,
                                std::unordered_set<unsigned> &exist_gates, std::vector<unsigned> &gates, bool next);

  void CollectNecessaryAndGatesFromConstrain(const aiger *aig, const aiger_symbol *as, const int as_size,
                                             std::unordered_set<unsigned> &exist_gates, std::vector<unsigned> &gates);

  void FindAndGates(const aiger_and *aa, const aiger *aig, std::unordered_set<unsigned> &exist_gates, std::vector<unsigned> &gates);

  void AddAndGateToClause(const aiger_and *aa);

  inline void InsertIntoPreValueMapping(const int key, const int value);

  inline aiger_and *IsAndGate(const unsigned id, const aiger *aig);

  int ComputeAndGate_RECUR(aiger_and &aa, const aiger *aig);

  void collect_and_gates_for_pine_r(const aiger_and *aa, const aiger *aig);

  void Collect_and_gates_for_pine(const aiger *aig);

  int compute_latch_r(int id, std::shared_ptr<std::unordered_map<int, int>> current_values);

  static bool cmp(int a, int b) {
    return abs(a) < abs(b);
  }

  void print_aiger_gml(const aiger *aig);

  void build_aiger_order(const aiger *aig);

  void create_sslv();

  string GetFileName(string filePath) {
    auto startIndex = filePath.find_last_of("/");
    if (startIndex == string::npos) {
      startIndex = 0;
    } else {
      startIndex++;
    }
    auto endIndex = filePath.find_last_of(".");
    assert(endIndex != string::npos);
    return filePath.substr(startIndex, endIndex - startIndex);
  }

#pragma endregion

#pragma region private member variables
  Settings m_settings;
  int m_maxId;
  int m_numInputs;
  int m_numLatches;
  int m_numAnds;
  int m_numConstraints;
  int m_numOutputs;
  int m_trueId;
  int m_falseId;
  int m_outputsStart; // the index of cls_ to point the start position of outputs
  int m_latchesStart; // the index of cls_ to point the start position of latches

  std::unordered_map<int, std::pair<int, int>> m_ands_gates_for_pine;

  std::vector<int> m_initialState;
  std::vector<int> m_outputs;
  std::vector<int> m_constraints;
  std::vector<std::vector<int>> m_clauses; // CNF, e.g. (a|b|c) * (-a|c)
  std::unordered_set<int> m_trues;         // variables that are always true
  std::unordered_map<int, int> m_nextValueOfLatch;
  std::unordered_map<int, int> m_nextValueOfGate;              // next value of and gate
  std::unordered_map<int, std::vector<int>> m_preValueOfLatch; // e.g. 6 16, 8 16. 16 -> 6,8

  sptr<std::vector<float>> m_aiger_order;
  sptr<SimpSolver> m_sslv;

#pragma endregion
};


} // namespace car


#endif