#ifndef CleanCARCHECKERF_H
#define CleanCARCHECKERF_H

#include "BaseChecker.h"
#include "State.h"
#include "IOverSequence.h"
#include "OverSequence.h"
#include "OverSequenceForProp.h"
#include "UnderSequence.h"
#include "ISolver.h"
#include "MainSolver.h"
#include "InvSolver.h"
#include "StartSolver.h"
#include "Task.h"
#include "Log.h"

#include <memory>

namespace car
{

class CleanCARCheckerF : public BaseChecker
{
public:
	CleanCARCheckerF(Settings settings, std::shared_ptr<AigerModel> model);

	bool Run();

	bool Check(int badId);

private:
	void Init(int badId);

	void AddUnsatisfiableCore(std::shared_ptr<std::vector<int> > uc, int frameLevel);

	bool ImmediateSatisfiable(int badId);

	bool isInvExisted();

	bool IsInvariant(int frameLevel);

	int GetNewLevel(std::shared_ptr<State> state, int start = 0);

	
	void GetAssumption(std::shared_ptr<State> state, int frameLevel, std::vector<int>& ass)
	{
		ass.reserve(state->latches->size());
		ass.insert(ass.begin(), state->latches->begin(), state->latches->end());
		for(auto& x:ass)
			x = m_model->GetPrime(x);
	}

	
	std::shared_ptr<State> EnumerateStartState(){
		if (m_startSovler->SolveWithAssumption()){
				return m_startSovler->GetStartState();
		}	else{
				return nullptr;
		}
	}

	int m_minUpdateLevel;
	std::shared_ptr<IOverSequence> m_overSequence;
	//IOverSequence* m_overSequence;
	UnderSequence m_underSequence;
	Settings m_settings;
	std::shared_ptr<Log> m_log;
	std::shared_ptr<AigerModel> m_model;
	std::shared_ptr<State> m_initialState;
	std::shared_ptr<ISolver> m_mainSolver;
	std::shared_ptr<ISolver> m_invSolver;
  std::shared_ptr<StartSolver> m_startSovler;
	std::vector<std::shared_ptr<std::vector<int> > > m_rotation;
};


}//namespace car

#endif