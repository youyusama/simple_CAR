#include "State.h"

namespace car
{
	int State::numInputs = -1;
    int State::numLatches = -1;

    std::string State::GetValueOfLatches()
	{
		std::string result = "";
		result.reserve(numLatches);
		int j = 0;
		for (int i = 0; i < State::numLatches; ++i)
		{
			if (State::numInputs + i + 1 < abs((*latches)[j]))
			{
				result += "x";
			}
			else
			{
				result += (latches->at(j) > 0) ? "1" : "0";
				++j;
			}
		}
		return result;
	}

	std::string State::GetValueOfInputs()
	{
		std::string result = "";
		result.reserve(numInputs);
		for (int i = 0; i < numInputs; ++i)
		{
			result += (inputs->at(i) > 0) ? "1" : "0";
		}
		return result;
		
	}
}//namespace car