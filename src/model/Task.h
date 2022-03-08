#ifndef TASK_H
#define TASK_H

#include "State.h"
#include<memory>
namespace car
{
///
//state SAT(state, T * Over_{frameLevel})
///
struct Task
{
public:
    Task(std::shared_ptr<State> inState, int inFrameLevel, bool inIsLocated) : state(inState), frameLevel(inFrameLevel), isLocated(inIsLocated){};
    bool isLocated;
    int frameLevel;
    std::shared_ptr<State> state;
};

}//namespace car

#endif