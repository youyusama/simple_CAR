#include "OverSequenceNI.h"

namespace car {
void OverSequenceNI::Insert(std::shared_ptr<std::vector<int>> uc, int index) {
  if (index >= m_sequence.size()) {
    m_sequence.emplace_back(frame());
    m_blockSolvers.emplace_back(new BlockSolver(m_model));
    m_block_counter.emplace_back(0);
  }
  m_sequence[index].insert(*uc);
  m_blockSolvers[index]->AddUnsatisfiableCore(*uc, index);
}

void OverSequenceNI::GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int>>> &out) {
  if (frameLevel >= m_sequence.size()) return;
  frame frame_i = m_sequence[frameLevel];
  std::vector<std::shared_ptr<std::vector<int>>> res;
  res.reserve(frame_i.size());
  for (frame::iterator j = frame_i.begin(); j != frame_i.end(); j++) {
    std::shared_ptr<std::vector<int>> temp(new std::vector<int>());
    temp->resize(j->size());
    std::copy(j->begin(), j->end(), temp->begin());
    res.emplace_back(temp);
  }
  out = res;
  return;
}

bool OverSequenceNI::IsBlockedByFrame(std::vector<int> &latches, int frameLevel) {
  blockQuerryTimes++;

  int latch_index, num_inputs;
  num_inputs = m_model->GetNumInputs();
  for (frame::iterator uc = m_sequence[frameLevel].begin(); uc != m_sequence[frameLevel].end(); uc++) { // for each uc
    bool blocked = true;
    for (int j = 0; j < uc->size(); j++) { // for each literal
      latch_index = abs(uc->at(j)) - num_inputs - 1;
      if (latches[latch_index] != uc->at(j)) {
        blocked = false;
        break;
      }
    }
    if (blocked) {
      blockedTimes++;
      return true;
    }
  }
  return false;
}


bool OverSequenceNI::IsBlockedByFrame_lazy(std::vector<int> &latches, int frameLevel) {
  blockQuerryTimes++;
  int &counter = m_block_counter[frameLevel];
  if (counter == -1) { // by sat
    std::vector<int> assumption;
    assumption.reserve(latches.size());
    for (int l : latches) {
      assumption.emplace_back(l);
    }

    bool result = m_blockSolvers[frameLevel]->SolveWithAssumption(assumption, frameLevel);
    if (!result) {
      blockedTimes++;
      return true;
    } else {
      return false;
    }
  }
  counter++;
  // whether it's need to change the way of checking
  clock_t start_time, sat_time, for_time;
  if (counter > 1000) {
    start_time = clock();
    std::vector<int> assumption;
    assumption.reserve(latches.size());
    for (int l : latches) {
      assumption.emplace_back(l);
    }
    m_blockSolvers[frameLevel]->SolveWithAssumption(assumption, frameLevel);
    sat_time = clock();
  }
  // by for checking
  int latch_index, num_inputs;
  num_inputs = m_model->GetNumInputs();
  for (frame::iterator uc = m_sequence[frameLevel].begin(); uc != m_sequence[frameLevel].end(); uc++) { // for each uc
    bool blocked = true;
    for (int j = 0; j < uc->size(); j++) { // for each literal
      latch_index = abs(uc->at(j)) - num_inputs - 1;
      if (latches[latch_index] != uc->at(j)) {
        blocked = false;
        break;
      }
    }
    if (blocked) {
      blockedTimes++;
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


bool OverSequenceNI::IsBlockedByFrame_sat(std::vector<int> &latches, int frameLevel) {
  blockQuerryTimes++;

  std::vector<int> assumption;
  assumption.reserve(latches.size());
  for (int l : latches) {
    assumption.emplace_back(l);
  }

  bool result = m_blockSolvers[frameLevel]->SolveWithAssumption(assumption, frameLevel);
  if (!result) {
    blockedTimes++;
    return true;
  } else {
    return false;
  }
}

int OverSequenceNI::GetLength() {
  return m_sequence.size();
}

void OverSequenceNI::propagate(int level) {
  return;
}

std::vector<int> OverSequenceNI::GetStat() {
  std::vector<int> temp;
  temp.push_back(blockQuerryTimes);
  temp.push_back(blockedTimes);
  temp.push_back(implyQuerryTimes);
  temp.push_back(implyTimes);
  // int allCubeNum = 0, repeatCubeNum = 0;
  // for (int i = 0; i < m_sequence.size(); i++)
  // 	allCubeNum += m_sequence[i].size();
  // for(int i = 0; i < m_sequence.size()-1; i++){
  // 	for (int j = i+1; j< m_sequence.size(); j++){
  // 		for (frame::iterator c = m_sequence[i].begin(); c != m_sequence[i].end(); c++) {
  // 			frame::iterator it = m_sequence[j].find(*c);
  // 			if (it != m_sequence[j].end()){
  // 				repeatCubeNum ++;
  // 				m_sequence[j].erase(c);
  // 			}
  // 		}
  // 	}
  // }
  return temp;
}

} // namespace car