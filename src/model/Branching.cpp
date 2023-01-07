#include "Branching.h"

namespace car {
Branching::Branching(int type) {
  branching_type = type;
  conflict_index = 1;
  mini = 1 << 20;
  counts.clear();
}


Branching::~Branching() {}


void Branching::update(const cube *uc) {
  if (uc->size() == 0) return;
  conflict_index++;
  switch (branching_type) {
  case 1: {
    decay();
    break;
  }
  case 2: {
    if (conflict_index == 256) {
      for (int i = mini; i < counts.size(); i++)
        counts[i] *= 0.5;
      conflict_index = 0;
    }
    break;
  }
  }
  // assumes cube is ordered
  int sz = abs(uc->back());
  if (sz >= counts.size()) counts.resize(sz + 1);
  if (mini > abs(uc->at(0))) mini = abs(uc->at(0));
  for (auto l : *uc) {
    switch (branching_type) {
    case 1:
    case 2: {
      counts[abs(l)]++;
      break;
    }
    case 3:
      counts[abs(l)] = (counts[abs(l)] + conflict_index) / 2.0;
      break;
    }
  }
}


void Branching::decay() {
  for (int i = mini; i < counts.size(); i++)
    counts[i] *= 0.99;
}


void Branching::decay(const cube *uc, int gap = 1) {
  if (uc->size() == 0) return;
  conflict_index++;
  // assumes cube is ordered
  int sz = abs(uc->back());
  if (sz >= counts.size()) counts.resize(sz + 1);
  if (mini > abs(uc->at(0))) mini = abs(uc->at(0));
  for (auto l : *uc) {
    counts[abs(l)] *= 1 - 0.01 * (gap - 1);
  }
}


float Branching::prior_of(int lit) {
  if (abs(lit) >= counts.size()) return 0;
  return counts[abs(lit)];
}


int Branching::counts_len() {
  return counts.size();
}


std::vector<float> *Branching::get_counts() {
  return &counts;
}
} // namespace car