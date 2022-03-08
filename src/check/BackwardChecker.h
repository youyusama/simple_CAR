#ifndef BACKWARDCHECKER_H
#define BACKWARDCHECKER_H

#include "BaseChecker.h"
#include "State.h"
#include "IOverSequence.h"
#include "OverSequence.h"
#include "OverSequenceForProp.h"
#include "UnderSequence.h"
#include "ISolver.h"
#include "MainSolver.h"
#include "InvSolver.h"
#include "Task.h"
#include "Log.h"
#include <assert.h>
#include <memory>
#include "restart.h"


namespace car
{



class BackwardChecker : public BaseChecker
{
public:
	BackwardChecker(Settings settings, std::shared_ptr<AigerModel> model);
	bool Run();
	bool Check(int badId);
private:
	void Init();

	void AddUnsatisfiableCore(std::shared_ptr<std::vector<int> > uc, int frameLevel);

	bool ImmediateSatisfiable(int badId);

	bool isInvExisted();

	bool IsInvariant(int frameLevel);

	int GetNewLevel(std::shared_ptr<State> state, int start = 0);

	string GetFileName(string filePath)
	{
		auto startIndex = filePath.find_last_of("/");
		if (startIndex == string::npos)
		{
			startIndex = 0;
		}
		else
		{
			startIndex++;
		}
		auto endIndex = filePath.find_last_of(".");
		assert (endIndex != string::npos);
		return filePath.substr(startIndex, endIndex-startIndex);	
	}

	void GetPriority (std::shared_ptr<std::vector<int> > latches, const int frameLevel, std::vector<int>& res) 
	{    
		if (frameLevel+1 >= m_overSequence->GetLength())
		{
			return;
		}
	    std::vector<std::shared_ptr<std::vector<int> > > frame;
		m_overSequence->GetFrame(frameLevel+1, frame);
	    if (frame.size () == 0)  
		{
	    	return;
		}
	    	
	    std::shared_ptr<std::vector<int> > uc = frame[frame.size()-1];
	    res.reserve (uc->size());
	    for (int i = 0; i < uc->size() ; ++ i) 
		{
	    	if ((*latches)[abs((*uc)[i])-m_model->GetNumInputs()-1] == (*uc)[i]) 
			{
	    		res.push_back ((*uc)[i]);
	    	}
	    }
	}

	
	void GetAssumption(std::shared_ptr<State> state, int frameLevel, std::vector<int>& ass)
	{
		if (m_settings.inter)
		{
			GetPriority(state->latches, frameLevel, ass);
		}

		ass.reserve(ass.size() + state->latches->size());

		if (m_settings.rotate)
		{
			std::vector<int> tmp;
			tmp.reserve (state->latches->size());
			int aa = m_rotation.size();
			if (frameLevel + 2 > m_rotation.size()) 
			{
				ass.insert(ass.end(), state->latches->begin(), state->latches->end());
				return;
			}
			else if (m_rotation[frameLevel+1] == nullptr)
			{
				ass.insert(ass.end(), state->latches->begin(), state->latches->end());
				return;
			}
			std::vector<int>& cube = *m_rotation[frameLevel+1];
			for (int i = 0; i < cube.size (); ++ i) {
				if ((*state->latches)[abs(cube[i])-m_model->GetNumInputs()-1] == cube[i]) 
					ass.push_back (cube[i]);
				else
					tmp.push_back (-cube[i]);
			}
			ass.insert(ass.end(), tmp.begin(), tmp.end());
		}
		else
		{
			ass.insert(ass.end(), state->latches->begin(), state->latches->end());
		}
	}

	void PushToRotation(std::shared_ptr<State> state, int frameLevel)
	{
		while (frameLevel + 2 > m_rotation.size())
		{
			m_rotation.push_back(nullptr);
		}
		m_rotation[frameLevel+1] = state->latches;
	}

	void Propagation()
	{
		OverSequenceForProp* sequence = dynamic_cast<OverSequenceForProp*>(m_overSequence.get());
		for (int frameLevel = 0; frameLevel < sequence->GetLength()-1; ++frameLevel)
		{
			std::vector<std::shared_ptr<std::vector<int> > > unpropFrame = sequence->GetUnProp(frameLevel);
			std::vector<std::shared_ptr<std::vector<int> > > propFrame = sequence->GetProp(frameLevel);
			std::vector<std::shared_ptr<std::vector<int> > > tmp;
			for (int j = 0; j < unpropFrame.size(); ++j)
			{	
				if (sequence->IsBlockedByFrame(*unpropFrame[j], frameLevel+1))
				{
					propFrame.push_back(unpropFrame[j]);
					continue;
				}

				bool result = m_mainSolver->SolveWithAssumption(*unpropFrame[j], frameLevel);
				if (!result)
				{
					AddUnsatisfiableCore(unpropFrame[j], frameLevel+1);
					sequence->InsertIntoProped(unpropFrame[j], frameLevel);
				}
				else
				{
					tmp.push_back(unpropFrame[j]);
				}
			}
			tmp.swap(unpropFrame);
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
	std::vector<std::shared_ptr<std::vector<int> > > m_rotation;
	std::shared_ptr<Restart> m_restart;
};

}//namespace car

#endif