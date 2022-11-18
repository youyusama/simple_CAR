#ifndef BRANCHING_H
#define BRANCHING_H

#include "AigerModel.h"

namespace car {

class Branching {
public:
  Branching();
  ~Branching();
  void update(const cube *uc);
  void decay();
  void decay(const cube *uc, int gap);
  float prior_of(int lit);
  int counts_len();
  std::vector<float> *get_counts();

private:
  int conflict_index;
  int mini;
  std::vector<float> counts;
};

} // namespace car

#endif