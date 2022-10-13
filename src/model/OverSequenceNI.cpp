#include "OverSequenceNI.h"

namespace car {

static bool _cmp(int a, int b) {
  if (abs(a) != abs(b))
    return abs(a) < abs(b);
  else
    return a < b;
}

// ================================================================================
// @brief: if a->b
// @input:
// @output:
// ================================================================================
bool OverSequenceNI::is_imply(cube a, cube b) {
  if (a.size() >= b.size())
    return false;
  if (std::includes(b.begin(), b.end(), a.begin(), a.end(), _cmp))
    return true;
  else
    return false;
}

bool OverSequenceNI::Insert(std::shared_ptr<cube> uc, int index) {
  m_blockSolver->AddUnsatisfiableCore(*uc, index);
  if (index >= m_sequence.size()) {
    m_sequence.emplace_back(frame());
    m_block_counter.emplace_back(0);
  }
  auto res = Ucs.insert(*uc);
  if (!res.second) rep_counter++;
  frame tmp;
  for (auto f_uc : m_sequence[index]) {
    if (!is_imply(*res.first, *f_uc->c))
      tmp.emplace(f_uc);
  }
  l_cube *tmp_lc = new l_cube();
  ;
  tmp_lc->c = &*res.first;
  tmp.emplace(tmp_lc);
  m_sequence[index].swap(tmp);
  if (index != 1 && !IsBlockedByFrame_lazy(*uc, index - 1)) {
    tmp_lc->l = false;
    return false;
  }
  return true;
}


// ================================================================================
// @brief: init frame 0
// @input: init state
// @output:
// ================================================================================
void OverSequenceNI::Init_Frame_0(sptr<cube> latches) {
  m_sequence.emplace_back(frame());
  m_block_counter.emplace_back(0);
  for (auto l : *latches) {
    sptr<cube> puc(new std::vector<int>{-l});
    auto res = Ucs.insert(*puc);
    l_cube *tmp_lc = new l_cube();
    tmp_lc->c = &*res.first;
    m_sequence[0].emplace(tmp_lc);
    m_blockSolver->AddUnsatisfiableCore(*puc, 0);
  }
}


void OverSequenceNI::GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int>>> &out) {
  if (frameLevel >= m_sequence.size()) return;
  frame frame_i = m_sequence[frameLevel];
  std::vector<std::shared_ptr<std::vector<int>>> res;
  res.reserve(frame_i.size());
  for (auto i : frame_i) {
    std::shared_ptr<std::vector<int>> temp(new std::vector<int>());
    temp->resize(i->c->size());
    std::copy(i->c->begin(), i->c->end(), temp->begin());
    res.emplace_back(temp);
  }
  out = res;
  return;
}


bool OverSequenceNI::IsBlockedByFrame_lazy(std::vector<int> &latches, int frameLevel) {
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
  // by imply checking
  for (auto uc : m_sequence[frameLevel]) { // for each uc
    if (std::includes(latches.begin(), latches.end(), uc->c->begin(), uc->c->end(), _cmp)) return true;
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

int OverSequenceNI::GetLength() {
  return m_sequence.size();
}

void OverSequenceNI::set_solver(std::shared_ptr<ISolver> slv) {
  m_mainSolver = slv;
}

const std::vector<int> *OverSequenceNI::GetBlocker(std::shared_ptr<std::vector<int>> latches, int framelevel) {
  if (framelevel >= m_sequence.size()) return new std::vector<int>();
  // by imply checking
  for (auto uc : m_sequence[framelevel]) { // for each uc
    if (std::includes(latches->begin(), latches->end(), uc->c->begin(), uc->c->end(), _cmp)) return uc->c;
  }
  return new std::vector<int>();
}

void OverSequenceNI::propagate(int level) {
  // std::cout << "propagate " << level << std::endl;
  frame &fi = m_sequence[level];
  frame &fi_plus_1 = m_sequence[level + 1];
  std::set<l_cube *>::iterator iter;
  for (auto *uc : fi) {
    iter = fi_plus_1.find(uc);
    if (iter != fi_plus_1.end()) continue; // propagated

    if (!uc->l) continue;

    std::vector<int> ass;
    ass.reserve(uc->c->size());
    for (auto i : *uc->c) {
      ass.emplace_back(m_model->GetPrime(i));
    }
    if (!m_mainSolver->SolveWithAssumption(ass, level)) {
      frame tmp;
      for (auto f_uc : fi_plus_1) {
        if (!is_imply(*uc->c, *f_uc->c))
          tmp.emplace(f_uc);
      }
      tmp.emplace(uc);
      fi_plus_1.swap(tmp);
      // std::cout << "propagate uc: ";
      // for (auto i : *uc) {
      //   std::cout << i << " ";
      // }
      // std::cout << "to level " << level + 1 << std::endl;
      // fi_plus_1.insert(uc);
      m_blockSolver->AddUnsatisfiableCore(*uc->c, level + 1);
      m_mainSolver->AddUnsatisfiableCore(*uc->c, level + 1);
    }
  }
  return;
}


// ================================================================================
// @brief: propagate the uc from lvl +1
// @input: uc already in lvl
// @output: uc cannot propagate to lvl
// ================================================================================
int OverSequenceNI::propagate_uc_from_lvl(sptr<cube> uc, int lvl) {
  while (lvl + 1 < m_sequence.size()) {
    frame &fi = m_sequence[lvl];
    frame &fi_plus_1 = m_sequence[lvl + 1];
    std::vector<int> ass;
    ass.reserve(uc->size());
    for (auto i : *uc) {
      ass.emplace_back(m_model->GetPrime(i));
    }
    if (!m_mainSolver->SolveWithAssumption(ass, lvl)) {
      frame tmp;
      auto res = Ucs.insert(*uc);
      for (auto f_uc : fi_plus_1) {
        if (!is_imply(*res.first, *f_uc->c))
          tmp.emplace(f_uc);
      }
      l_cube *tmp_lc = new l_cube();
      ;
      tmp_lc->c = &*res.first;
      tmp.emplace(tmp_lc);
      fi_plus_1.swap(tmp);
      m_blockSolver->AddUnsatisfiableCore(*uc, lvl + 1);
      m_mainSolver->AddUnsatisfiableCore(*uc, lvl + 1);
    } else {
      break;
    }
    lvl++;
  }
  return lvl + 1;
}

} // namespace car