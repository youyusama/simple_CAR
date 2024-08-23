#ifndef STATE_H
#define STATE_H

#include <memory>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace std;
typedef vector<int> cube;
typedef vector<int> clause;

namespace car {

class State {
  public:
    State(shared_ptr<State> inPreState,
          shared_ptr<cube> inInputs,
          shared_ptr<cube> inLatches,
          int inDepth) : preState(inPreState),
                         inputs(inInputs),
                         latches(inLatches),
                         depth(inDepth) {}
    static int numInputs;
    static int numLatches;

    string GetLatchesString();
    string GetInputsString();

    int depth;
    shared_ptr<State> preState = nullptr;
    shared_ptr<cube> inputs;
    shared_ptr<cube> latches;
};


} // namespace car

#endif
