#ifndef STATE_H
#define STATE_H

#include <vector>
#include <string>
#include <stdlib.h>
#include <memory>
namespace car
{

class State
{
public:
	State(std::shared_ptr<State> inPreState, std::shared_ptr<std::vector<int> > inInputs, std::shared_ptr<std::vector<int> > inLatches, int inDepth):
		preState(inPreState), inputs(inInputs), latches(inLatches), depth(inDepth)
	{
		
	}

	std::string GetValueOfLatches();

	std::string GetValueOfInputs();
	static int numInputs;
	static int numLatches;
	int depth;
	std::shared_ptr<State> preState = nullptr;
	std::shared_ptr<std::vector<int> > inputs;
	std::shared_ptr<std::vector<int> > latches;
	
};


}//namespace car

#endif
