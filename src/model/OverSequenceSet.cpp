#include "OverSequenceSet.h"

namespace car {
void OverSequenceSet::Insert(std::shared_ptr<cube> uc, int index) {
  auto res = Ucs.insert(*uc);
  if (!res.second) rep_counter++;
  if (index >= m_sequence.size()) {
    m_sequence.emplace_back(frame());
    m_block_counter.emplace_back(0);
  }
  m_sequence[index].emplace_back(&*res.first);
  m_blockSolver->AddUnsatisfiableCore(*uc, index);
}

void OverSequenceSet::GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int>>> &out) {
  if (frameLevel >= m_sequence.size()) return;
  frame frame_i = m_sequence[frameLevel];
  std::vector<std::shared_ptr<std::vector<int>>> res;
  res.reserve(frame_i.size());
  for (auto i : frame_i) {
    std::shared_ptr<std::vector<int>> temp(new std::vector<int>());
    temp->resize(i->size());
    std::copy(i->begin(), i->end(), temp->begin());
    res.emplace_back(temp);
  }
  out = res;
  return;
}

bool OverSequenceSet::IsBlockedByFrame(std::vector<int> &latches, int frameLevel) {
  // by for checking
  int latch_index, num_inputs;
  num_inputs = m_model->GetNumInputs();
  for (auto uc : m_sequence[frameLevel]) { // for each uc
    bool blocked = true;
    for (int j = 0; j < uc->size(); j++) { // for each literal
      latch_index = abs(uc->at(j)) - num_inputs - 1;
      if (latches[latch_index] != uc->at(j)) {
        blocked = false;
        break;
      }
    }
    if (blocked) {
      return true;
    }
  }
  return false;
}


bool OverSequenceSet::IsBlockedByFrame_lazy(std::vector<int> &latches, int frameLevel) {
  int &counter = m_block_counter[frameLevel];
  if (counter == -1) { // by sat
    std::vector<int> assumption;
    assumption.reserve(latches.size());
    for (int l : latches) {
      assumption.emplace_back(l);
    }

    bool result = m_blockSolver->SolveWithAssumption(assumption, frameLevel);
    if (!result) {
      return true;
    } else {
      return false;
    }
  }
  if (m_sequence[frameLevel].size() > 3000) {
    counter++;
  }
  // whether it's need to change the way of checking
  clock_t start_time, sat_time, for_time;
  if (counter > 1000) {
    start_time = clock();
    std::vector<int> assumption;
    assumption.reserve(latches.size());
    for (int l : latches) {
      assumption.emplace_back(l);
    }
    m_blockSolver->SolveWithAssumption(assumption, frameLevel);
    sat_time = clock();
  }
  // by for checking
  int latch_index, num_inputs;
  num_inputs = m_model->GetNumInputs();
  for (auto uc : m_sequence[frameLevel]) { // for each uc
    bool blocked = true;
    for (int j = 0; j < uc->size(); j++) { // for each literal
      latch_index = abs(uc->at(j)) - num_inputs - 1;
      if (latches[latch_index] != uc->at(j)) {
        blocked = false;
        break;
      }
    }
    if (blocked) {
      return true;
    }
  }
  if (counter > 1000) {
    for_time = clock();
    if (sat_time - start_time > for_time - sat_time)
      counter = 0;
    else
      counter = -1;
  }
  return false;
}

int OverSequenceSet::GetLength() {
  return m_sequence.size();
}

void OverSequenceSet::propagate(int level) {
  return;
}

} // namespace car