#ifndef BRANCHING_H
#define BRANCHING_H

#include "AigerModel.h"
#include "Settings.h"

namespace car {

class Branching {
  public:
    Branching(int type = 1);
    ~Branching();
    void Update(const shared_ptr<cube> uc);
    void Decay();
    void Decay(const shared_ptr<cube> uc, int gap);

    inline float PriorityOf(int lit) {
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