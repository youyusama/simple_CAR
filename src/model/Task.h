#ifndef TASK_H
#define TASK_H

#include "State.h"
#include <memory>
namespace car {

struct Task {
  public:
    Task(shared_ptr<State> inState, int inFrameLevel, bool inIsLocated) : state(inState),
                                                                          frameLevel(inFrameLevel),
                                                                          isLocated(inIsLocated) {};
    bool isLocated;
    int frameLevel;
    shared_ptr<State> state;
};

} // namespace car

#endif