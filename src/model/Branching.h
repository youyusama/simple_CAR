#ifndef BRANCHING_H
#define BRANCHING_H

#include "AigerModel.h"
#include "Settings.h"

namespace car {

class Branching {
  public:
    Branching(int type = 1);
    ~Branching();
    void update(const cube *uc);
    void decay();
    void decay(const cube *uc, int gap);
    int counts_len();
    std::vector<float> *get_counts();

    inline float prior_of(int lit) {
        if (abs(lit) >= counts.size()) return 0;
        return counts[abs(lit)];
    }

  private:
    int branching_type; // 1: sum 2: vsids 3: acids 4: MAB (to do) 0: static
    int conflict_index;
    int mini;
    std::vector<float> counts;
};

} // namespace car

#endif