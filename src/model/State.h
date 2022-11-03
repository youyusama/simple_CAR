#ifndef STATE_H
#define STATE_H

#include <memory>
#include <stdlib.h>
#include <string>
#include <vector>
namespace car {

class State {
public:
  State(std::shared_ptr<State> inPreState, std::shared_ptr<std::vector<int>> inInputs, std::shared_ptr<std::vector<int>> inLatches, int inDepth) : preState(inPreState), inputs(inInputs), latches(inLatches), depth(inDepth) {
    pine_state_type = 0;
  }

  std::string GetValueOfLatches();
  std::string GetValueOfInputs();
  static int numInputs;
  static int numLatches;
  int depth;
  std::shared_ptr<State> preState = nullptr;
  std::shared_ptr<std::vector<int>> inputs;
  std::shared_ptr<std::vector<int>> latches;

  int pine_state_type; // 0: uninitialized 1: solved 2: not suitable
  std::shared_ptr<std::vector<int>> pine_assumptions;
  std::shared_ptr<std::vector<int>> pine_l_index;
  int pine_l_list_type; // 0: end with zero 1: end with loop
};


} // namespace car

#endif
