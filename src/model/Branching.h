#ifndef BRANCHING_H
#define BRANCHING_H

#include "AigerModel.h"
#include "Settings.h"

namespace car {

class Branching {
public:
  Branching(int type = 0);
  ~Branching();
  void update(const cube *uc);
  void decay();
  void decay(const cube *uc, int gap);
  float prior_of(int lit);
  int counts_len();
  std::vector<float> *get_counts();

private:
  int branching_type; // 0: sum 1: vsids 2: acids 3: MAB -1: static
  int conflict_index;
  int mini;
  std::vector<float> counts;
};

} // namespace car

#endif