#include "AigerModel.h"


namespace car {

AigerModel::AigerModel(Settings settings) {
  m_settings = settings;
  string aigFilePath = settings.aigFilePath;

  aiger *aig = aiger_init();
  aiger_open_and_read_from_file(aig, aigFilePath.c_str());
  if (aiger_error(aig)) {
    // placeholder
  }
  if (!aiger_is_reencoded(aig)) {
    aiger_reencode(aig);
  }

  Init(aig);
}


void AigerModel::Init(aiger *aig) {
  m_numInputs = aig->num_inputs;
  m_numLatches = aig->num_latches;
  m_numAnds = aig->num_ands;
  m_numConstraints = aig->num_constraints;
  m_numOutputs = aig->num_outputs;
  m_maxId = aig->maxvar + 2;
  m_trueId = m_maxId - 1;
  m_falseId = m_maxId;

  CollectTrues(aig);
  CollectConstraints(aig);
  CollectOutputs(aig);
  CollectInitialState(aig);
  CollectNextValueMapping(aig);
  CollectClauses(aig);
  //   Collect_and_gates_for_pine(aig);
}

void AigerModel::CollectTrues(const aiger *aig) {
  for (int i = 0; i < aig->num_ands; ++i) {
    aiger_and &aa = aig->ands[i];
    // and gate is always an even number in aiger
    if (aa.lhs % 2 != 0) {
      // placeholder
    }
    if (IsTrue(aa.rhs0) && IsTrue(aa.rhs1)) {
      m_trues.insert(aa.lhs);
    } else if (IsFalse(aa.rhs0) || IsFalse(aa.rhs1)) {
      m_trues.insert(aa.lhs + 1);
    }
  }
}

void AigerModel::CollectTrues_RECUR(const aiger *aig) {
  for (int i = 0; i < aig->num_ands; ++i) {
    aiger_and &aa = aig->ands[i];
    // and gate is always an even number in aiger
    if (aa.lhs % 2 != 0) {
      // placeholder
    }
    switch (ComputeAndGate_RECUR(aa, aig)) {
    case 0: // flase
      m_trues.insert(aa.lhs + 1);
      break;
    case 1: // true
      m_trues.insert(aa.lhs);
      break;
    case 2: // unkown value
      // nothing to do
      break;
    }
  }
}


int AigerModel::ComputeAndGate_RECUR(aiger_and &aa, const aiger *aig) {
  if (IsFalse(aa.rhs0) || IsFalse(aa.rhs1)) {
    m_trues.insert(aa.lhs + 1);
    return 0;
  } else {
    if (!IsTrue(aa.rhs0)) {
      aiger_and *aa0 = IsAndGate(aa.rhs0, aig);
      if (aa0 != nullptr) {
        ComputeAndGate_RECUR(*aa0, aig);
      }
    }
    if (!IsTrue(aa.rhs1)) {
      aiger_and *aa1 = IsAndGate(aa.rhs1, aig);
      if (aa1 != nullptr) {
        ComputeAndGate_RECUR(*aa1, aig);
      }
    }
    if (IsFalse(aa.rhs0) || IsFalse(aa.rhs1)) {
      m_trues.insert(aa.lhs + 1);
      return 0;
    } else if (IsTrue(aa.rhs0) && IsTrue(aa.rhs1)) {
      m_trues.insert(aa.lhs);
      return 1;
    } else
      return 2;
  }
}


void AigerModel::CollectConstraints(const aiger *aig) {
  for (int i = 0; i < aig->num_constraints; ++i) {
    int id = static_cast<int>(aig->constraints[i].lit);
    m_constraints.push_back((id % 2 == 0) ? (id / 2) : -(id / 2));
  }
}


int AigerModel::compute_latch_r(int id, std::shared_ptr<std::unordered_map<int, int>> current_values) {
  if (IsInput(id)) return 2;
  if (current_values->count(abs(id)) > 0) {
    if (current_values->at(abs(id)) == 2) return 2;
    if (id > 0)
      return current_values->at(abs(id));
    else
      return 1 - current_values->at(abs(id));
  }
  if (id == GetTrueId() || id == -GetFalseId()) {
    current_values->insert(std::pair<int, int>(abs(id), 1));
    return 1;
  }
  if (id == -GetTrueId() || id == GetFalseId()) {
    current_values->insert(std::pair<int, int>(abs(id), 0));
    return 0;
  }
  if (m_ands_gates_for_pine.count(abs(id)) > 0) {
    int r0 = compute_latch_r(m_ands_gates_for_pine[abs(id)].first, current_values);
    int r1 = compute_latch_r(m_ands_gates_for_pine[abs(id)].second, current_values);
    if (r0 == 1 && r1 == 1) {
      current_values->insert(std::pair<int, int>(abs(id), 1));
      if (id > 0)
        return 1;
      else
        return 0;
    }
    if (r0 == 0 || r1 == 0) {
      current_values->insert(std::pair<int, int>(abs(id), 0));
      if (id > 0)
        return 0;
      else
        return 1;
    }
  }
  current_values->insert(std::pair<int, int>(abs(id), 2));
  return 2;
}


std::shared_ptr<std::vector<int>> AigerModel::Get_next_latches_for_pine(std::vector<int> &current_latches) {
  std::shared_ptr<std::vector<int>> next_latches(new std::vector<int>());
  std::shared_ptr<std::unordered_map<int, int>> current_values(new std::unordered_map<int, int>());
  for (auto l : current_latches)
    current_values->insert(std::pair<int, int>(abs(l), l > 0 ? 1 : 0));
  for (int i = GetNumInputs() + 1, end = GetNumInputs() + GetNumLatches() + 1; i < end; i++) {
    int v = compute_latch_r(GetPrime(i), current_values);
    if (v == 1) {
      next_latches->emplace_back(i);
    }
    if (v == 0) {
      next_latches->emplace_back(-i);
    }
    // if (v == 2){// just for debug
    //     std::cout<<"next latch: "<<i<<" is:"<<GetPrime(i)<<" unknown!!"<<std::endl;
    // }
  }
  // std::cout<<"current values"<<std::endl;
  // for (auto v: *current_values){
  //     if (v.second==1) std::cout<<v.first<<" ";
  //     else if (v.second==0) std::cout<<-v.first<<" ";
  // }
  // std::cout<<std::endl;
  // std::sort(next_latches->begin(), next_latches->end(), cmp);

  return next_latches;
}


void AigerModel::CollectOutputs(const aiger *aig) {
  for (int i = 0; i < aig->num_outputs; ++i) {
    int id = static_cast<int>(aig->outputs[i].lit);
    m_outputs.push_back((id % 2 == 0) ? (id / 2) : -(id / 2));
  }
}

void AigerModel::CollectInitialState(const aiger *aig) {
  for (int i = 0; i < aig->num_latches; ++i) {
    if (aig->latches[i].reset == 0)
      m_initialState.push_back(-(m_numInputs + 1 + i));
    else if (aig->latches[i].reset == 1)
      m_initialState.push_back(m_numInputs + 1 + i);
    else {
      // placeholder
    }
  }
}

void AigerModel::CollectNextValueMapping(const aiger *aig) {
  for (int i = 0; i < aig->num_latches; i++) {
    int val = (int)aig->latches[i].lit;
    // a latch should not be a negative number
    if (val % 2 != 0) {
      // placeholder
    }
    val = val / 2;
    // make sure our assumption about latches is correct
    if (val != (m_numInputs + 1 + i)) {
      // placeholder
    }

    // pay attention to the special case when nextVal = 0 or 1
    if (IsFalse(aig->latches[i].next)) {
      m_nextValueOfLatch.insert(std::pair<int, int>(val, m_falseId));
      InsertIntoPreValueMapping(m_falseId, val);
    } else if (IsTrue(aig->latches[i].next)) {
      m_nextValueOfLatch.insert(std::pair<int, int>(val, m_trueId));
      InsertIntoPreValueMapping(m_trueId, val);
    } else {
      int nextVal = static_cast<int>(aig->latches[i].next);
      nextVal = (nextVal % 2 == 0) ? (nextVal / 2) : -(nextVal / 2);
      m_nextValueOfLatch.insert(std::pair<int, int>(val, nextVal));
      InsertIntoPreValueMapping(abs(nextVal), (nextVal > 0) ? val : -val);
    }
  }
}

void AigerModel::CollectClauses(const aiger *aig) {
  // contraints, outputs and latches gates are stored in order,
  // as the need for start solver construction
  std::unordered_set<unsigned> exist_gates;
  std::vector<unsigned> gates;
  // gates.resize(max_id_ + 1, 0);
  // create clauses for constraints
  CollectNecessaryAndGatesFromConstrain(aig, aig->constraints, aig->num_constraints, exist_gates, gates);

  for (std::vector<unsigned>::iterator it = gates.begin(); it != gates.end(); it++) {
    aiger_and *aa = aiger_is_and(const_cast<aiger *>(aig), *it);
    if (aa == NULL) {
      // placeholder
    }
    AddAndGateToClause(aa);
  }

  // if l1 and l2 have same prime l', then l1 and l2 shoud have same value
  if (m_settings.forward) {
    int init = ++m_maxId;
    int cons = ++m_maxId;
    m_clauses.emplace_back(std::vector<int>{init, cons});
    for (std::unordered_map<int, std::vector<int>>::iterator it = m_preValueOfLatch.begin(); it != m_preValueOfLatch.end(); it++) {
      if (it->second.size() > 1) {
        m_maxId++;
        for (int p : it->second) {
          m_clauses.emplace_back(std::vector<int>{-cons, -p, m_maxId});
          m_clauses.emplace_back(std::vector<int>{-cons, p, -m_maxId});
        }
      }
    }
    for (int i : m_initialState) {
      m_clauses.emplace_back(std::vector<int>{-init, i});
    }
  }

  m_outputsStart = m_clauses.size();
  // create clauses for outputs
  std::vector<unsigned>().swap(gates);
  CollectNecessaryAndGates(aig, aig->outputs, aig->num_outputs, exist_gates, gates, false);

  for (std::vector<unsigned>::iterator it = gates.begin(); it != gates.end(); it++) {
    if (*it == 0) continue;
    aiger_and *aa = aiger_is_and(const_cast<aiger *>(aig), *it);
    if (aa == NULL) {
      // placeholder
    }
    AddAndGateToClause(aa);
  }
  m_latchesStart = m_clauses.size();

  // create clauses for latches
  std::vector<unsigned>().swap(gates);
  CollectNecessaryAndGates(aig, aig->latches, aig->num_latches, exist_gates, gates, true);
  for (std::vector<unsigned>::iterator it = gates.begin(); it != gates.end(); it++) {
    if (*it == 0) continue;
    aiger_and *aa = aiger_is_and(const_cast<aiger *>(aig), *it);
    if (aa == NULL) {
      // placeholder
    }
    AddAndGateToClause(aa);
  }

  // create clauses for true and false
  m_clauses.emplace_back(std::vector<int>{m_trueId});
  m_clauses.emplace_back(std::vector<int>{-m_falseId});
}

void AigerModel::CollectNecessaryAndGates(const aiger *aig, const aiger_symbol *as, const int as_size,
                                          std::unordered_set<unsigned> &exist_gates, std::vector<unsigned> &gates, bool next) {
  for (int i = 0; i < as_size; ++i) {
    aiger_and *aa;
    if (next)
      aa = IsAndGate(as[i].next, aig);
    else {
      aa = IsAndGate(as[i].lit, aig);
      if (aa == NULL) {
        if (IsTrue(as[i].lit)) {
          m_outputs[i] = m_trueId;
        } else if (IsFalse(as[i].lit))
          m_outputs[i] = m_falseId;
      }
    }
    FindAndGates(aa, aig, exist_gates, gates);
  }
}

void AigerModel::CollectNecessaryAndGatesFromConstrain(const aiger *aig, const aiger_symbol *as, const int as_size,
                                                       std::unordered_set<unsigned> &exist_gates, std::vector<unsigned> &gates) {
  for (int i = 0; i < as_size; ++i) {
    aiger_and *aa;
    aa = IsAndGate(as[i].lit, aig);
    if (aa == NULL) {
      if (IsFalse(as[i].lit)) {
        m_outputs[i] = m_falseId;
      }
    }
    FindAndGates(aa, aig, exist_gates, gates);
  }
}

void AigerModel::FindAndGates(const aiger_and *aa, const aiger *aig, std::unordered_set<unsigned> &exist_gates, std::vector<unsigned> &gates) {
  if (aa == NULL || aa == nullptr) {
    return;
  }
  if (exist_gates.find(aa->lhs) != exist_gates.end()) {
    return;
  }
  gates.emplace_back(aa->lhs);
  exist_gates.emplace(aa->lhs);
  aiger_and *aa0 = IsAndGate(aa->rhs0, aig);
  FindAndGates(aa0, aig, exist_gates, gates);

  aiger_and *aa1 = IsAndGate(aa->rhs1, aig);
  FindAndGates(aa1, aig, exist_gates, gates);
}

void AigerModel::AddAndGateToClause(const aiger_and *aa) {
  if (aa == nullptr || IsTrue(aa->lhs) || IsFalse(aa->lhs)) {
    // placeholder
  }

  if (IsTrue(aa->rhs0)) {
    m_clauses.emplace_back(std::vector<int>{GetCarId(aa->lhs), -GetCarId(aa->rhs1)});
    m_clauses.emplace_back(std::vector<int>{-GetCarId(aa->lhs), GetCarId(aa->rhs1)});
  } else if (IsTrue(aa->rhs1)) {
    m_clauses.emplace_back(std::vector<int>{GetCarId(aa->lhs), -GetCarId(aa->rhs0)});
    m_clauses.emplace_back(std::vector<int>{-GetCarId(aa->lhs), GetCarId(aa->rhs0)});
  } else {
    m_clauses.emplace_back(std::vector<int>{GetCarId(aa->lhs), -GetCarId(aa->rhs0), -GetCarId(aa->rhs1)});
    m_clauses.emplace_back(std::vector<int>{-GetCarId(aa->lhs), GetCarId(aa->rhs0)});
    m_clauses.emplace_back(std::vector<int>{-GetCarId(aa->lhs), GetCarId(aa->rhs1)});
  }
}

inline void AigerModel::InsertIntoPreValueMapping(const int key, const int value) {
  std::unordered_map<int, std::vector<int>>::iterator it = m_preValueOfLatch.find(key);
  if (it == m_preValueOfLatch.end()) {
    m_preValueOfLatch.insert(std::pair<int, std::vector<int>>(key, std::vector<int>{value}));
  } else {
    it->second.emplace_back(value);
  }
}

inline aiger_and *AigerModel::IsAndGate(const unsigned id, const aiger *aig) {
  if (!IsTrue(id) && !IsFalse(id)) {
    return aiger_is_and(const_cast<aiger *>(aig), (id % 2 == 0) ? id : (id - 1));
  }
  return nullptr;
}


void AigerModel::collect_and_gates_for_pine_r(const aiger_and *aa, const aiger *aig) {
  if (aa != nullptr) {
    std::pair<int, int> andp(GetCarId(aa->rhs0), GetCarId(aa->rhs1));
    auto r = m_ands_gates_for_pine.insert(std::pair<int, std::pair<int, int>>(GetCarId(aa->lhs), andp));
    if (r.second == false) return;
    aiger_and *aa1 = IsAndGate(aa->rhs0, aig);
    collect_and_gates_for_pine_r(aa1, aig);
    aiger_and *aa2 = IsAndGate(aa->rhs1, aig);
    collect_and_gates_for_pine_r(aa2, aig);
  }
}

// collect necessary and gates for pine optimization
void AigerModel::Collect_and_gates_for_pine(const aiger *aig) {
  for (int i = 0; i < GetNumLatches(); i++) {
    aiger_and *aa = IsAndGate(aig->latches[i].next, aig);
    collect_and_gates_for_pine_r(aa, aig);
  }

  // for (auto iter = m_ands_gates_for_pine.begin(); iter!=m_ands_gates_for_pine.end(); iter++){
  //     std::cout<<iter->first<<" "<<iter->second.first<<" "<<iter->second.second<<std::endl;
  // }
}


} // namespace car