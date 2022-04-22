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
#include "Vis.h"
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


	void print_vector(std::vector<int>& v, std::string s = ""){
		std::cout<<std::endl<<s<<std::endl;
		for (auto l : v){
			std::cout<<l<<" ";
		}
	}


	bool static id_comp(int a, int b){
		if (abs(a) != abs(b)) return abs(a)<abs(b);
		else return a < b;
	}


	void build_ass_by_l_list(std::shared_ptr<std::vector<int> > l_0, std::vector<int>& ass){
		ass.resize(l_0->size());
		std::vector<int>::iterator ass_iter = ass.begin();
		std::vector<int> l_im1(*l_0); // l_(i-1)
		bool end_flag = true;
		do{
			std::shared_ptr<std::vector<int> > next_l_im1 = m_model->Get_next_latches_for_pine(l_im1);
			// print_vector(l_im1, "l_i-1 ========");
			// print_vector(*next_l_im1, "get next(l_i-1) ============");
			std::vector<int> l_i(l_0->size());
			auto iter = std::set_intersection(l_im1.begin(), l_im1.end(), next_l_im1->begin(), next_l_im1->end(), l_i.begin(), id_comp);
			l_i.resize(iter-l_i.begin());
			// print_vector(l_i, "get l_i ==============");
			ass_iter = std::set_difference(l_im1.begin(), l_im1.end(), l_i.begin(), l_i.end(), ass_iter, id_comp);
			if (l_i.empty()) end_flag = false;
			if (l_i.size() == l_im1.size()){
				ass.insert(ass_iter, l_i.begin(), l_i.end());
				ass.resize(l_0->size());
				end_flag = false;
			}
			l_i.swap(l_im1);
		}while (end_flag);
		std::reverse(ass.begin(), ass.end());
		// print_vector(ass, "ordered ass: =========");
	}

	
	void GetAssumption(std::shared_ptr<State> state, int frameLevel, std::vector<int>& ass)
	{
		if (m_settings.pine){
			std::shared_ptr<std::vector<int> > nextl = m_model->Get_next_latches_for_pine(*state->latches);
			if (nextl->size()/(float)m_model->GetNumLatches()>0.2){
				build_ass_by_l_list(state->latches, ass);
				return;
			}
		}

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
	std::shared_ptr<Vis> m_vis;
	Settings m_settings;
	std::shared_ptr<Log> m_log;
	std::shared_ptr<AigerModel> m_model;
	std::shared_ptr<State> m_initialState;
	std::shared_ptr<ISolver> m_mainSolver;
	std::shared_ptr<ISolver> m_invSolver;
	std::vector<std::shared_ptr<std::vector<int> > > m_rotation;
	std::shared_ptr<Restart> m_restart;
	int m_repeat_state_num = 0;
};

}//namespace car

#endif