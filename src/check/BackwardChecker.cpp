#include "BackwardChecker.h"
#include <stack>
#include <string>

namespace car
{
	BackwardChecker::BackwardChecker(Settings settings, std::shared_ptr<AigerModel> model) : m_settings(settings)
	{
		m_model = model;
		State::numInputs = model->GetNumInputs();
		State::numLatches = model->GetNumLatches(); 
		m_log.reset(new Log(settings, model));
		const std::vector<int>& init = model->GetInitialState();
		std::shared_ptr<std::vector<int> > inputs(new std::vector<int>(State::numInputs, 0));
		std::shared_ptr<std::vector<int> > latches(new std::vector<int>());
		latches->reserve(State::numLatches);
		
		for (int i = 0; i < State::numLatches; ++i)
		{
			latches->push_back(init[i]);
		}
		m_initialState.reset(new State(nullptr, inputs, latches, 0));
	}

	bool BackwardChecker::Run()
	{
		for (int i = 0, maxI = m_model->GetOutputs().size(); i < maxI; ++i)
		{
			int badId = m_model->GetOutputs().at(i);
			bool result = Check(badId);
			//PrintUC();
			if (result)
			{
				m_log->PrintSafe(i);
			}
			else //unsafe
			{
				m_log->PrintCounterExample(i);
			}
			if (m_settings.Visualization) {
				m_vis->OutputGML(false);
			}
			std::string ps = "block querry/blocked/imply querry/imply: ";
			for (auto i: m_overSequence->GetStat())
				ps += std::to_string(i) + "/";
			m_log->PrintSth(ps);
			m_log->PrintStatistics();
		}
		return true;
	}

	bool BackwardChecker::Check(int badId)
	{
#pragma region early stage
		if (m_model->GetTrueId() == badId)
		{
			//placeholder
			return false;
		}
		else if (m_model->GetFalseId() == badId)
		{
			//placeholder
			return true;
		}

 		Init();

		if (ImmediateSatisfiable(badId))
		{
			CAR_DEBUG("Result >>> SAT <<<");
			std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > > pair;
			pair = m_mainSolver->GetAssignment();
			CAR_DEBUG_v("Get Assignment:", *pair.second);

			std::shared_ptr<State> newState(new State (m_initialState, pair.first, pair.second, 1));
			m_log->lastState = newState;
			return false;
		}
		CAR_DEBUG("Result >>> UNSAT <<<");
		m_log->Tick();
		auto uc = m_mainSolver->GetUnsatisfiableCoreFromBad(badId);
		m_log->StatMainSolver();
		if (uc->empty()) //uc is empty when Bad by itself is unsatisfying
		{
			//placeholder
			return true;
		}
		CAR_DEBUG_v("Get UC:", *uc);
		m_overSequence->Insert(uc, 0, m_model);
		if (m_settings.empi){
			updateLitOrder(*uc);
		}
		
		std::vector<std::shared_ptr<std::vector<int> > > frame;
		m_overSequence->GetFrame(0, frame);
		m_mainSolver->AddNewFrame(frame, 0);
		m_overSequence->effectiveLevel = 0;
		CAR_DEBUG_o("Frames: ", m_overSequence.get());
# pragma endregion 


		//main stage
		int frameStep = 0;
		std::stack<Task> workingStack;
		while (true)
		{
			m_log->PrintFramesInfo(m_overSequence.get());
			m_minUpdateLevel = m_overSequence->GetLength();
			if (m_settings.end)
			{
				for (int i = 0; i < m_underSequence.size(); ++i)
				{
					for (int j = m_underSequence[i].size()-1; j>=0; --j)
					{
						workingStack.emplace(m_underSequence[i][j], frameStep, false);
					}
				}
			}
			else
			{
				// to do, not from end just not implemented
				workingStack.emplace(m_underSequence[0][0], frameStep, false);
			}
			
			while (!workingStack.empty())
			{
				if ( (m_settings.timelimit > 0 && m_log->IsTimeout()) || (m_settings.Visualization && m_vis->isEnoughNodesForVis()))
				{
					if (m_settings.Visualization) {
						m_vis->OutputGML(true);
					}
					std::string ps = "block querry/blocked/imply querry/imply: ";
					for (auto i: m_overSequence->GetStat())
					  ps += std::to_string(i) + "/";
					m_log->PrintSth(ps);
					m_log->PrintSth("time out!!!");
					m_log->Timeout();
				}
				

				Task& task = workingStack.top();
				if (m_settings.restart && m_restart->RestartCheck(task.state.get()))
				{
					m_restart->DoRestart(workingStack);
					m_log->CountRestartTimes();
					continue;
				}
				
				if (!task.isLocated)
				{
					m_log->Tick();
					task.frameLevel = GetNewLevel(task.state, task.frameLevel+1);
					CAR_DEBUG("state get new level " + std::to_string(task.frameLevel));
					m_log->StatGetNewLevel();
					if (task.frameLevel > m_overSequence->effectiveLevel)
					{
						workingStack.pop();
						continue;
					}
				}
				task.isLocated = false;

 				if (task.frameLevel == -1)
				{ 
					m_log->Tick();
					std::vector<int> assumption;
					std::vector<int> assumption_pine;
					GetAssumption(task.state, task.frameLevel, assumption);
					// GetAssumptionByPine(task.state, task.frameLevel, assumption_pine);
					CAR_DEBUG("\nSAT CHECK on frame: " + std::to_string(task.frameLevel));
					CAR_DEBUG_s("From state: ", task.state);
					CAR_DEBUG_v("Assumption: ", assumption);
					bool result = m_mainSolver->SolveWithAssumptionAndBad(assumption, badId);
					m_log->StatMainSolver();
					if (result)
					{
						CAR_DEBUG("Result >>> SAT <<<");
						std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > > pair;
						pair = m_mainSolver->GetAssignment();
						CAR_DEBUG_v("Get Assignment:", *pair.second);
						std::shared_ptr<State> newState(new State (task.state, pair.first, pair.second, task.state->depth+1));
						m_log->lastState = newState;
						return false;
					}
					else
					{
						CAR_DEBUG("Result >>> UNSAT <<<");
						if (m_settings.rotate) PushToRotation(task.state, task.frameLevel);
						auto uc = m_mainSolver->GetUnsatisfiableCore();
						if (uc->empty())
						{
							//placeholder, uc is empty => safe
						}
						CAR_DEBUG_v("Get UC:", *uc);
						m_log->Tick();
						AddUnsatisfiableCore(uc, task.frameLevel+1);
						m_log->StatUpdateUc();
						m_restart->UcCountsPlus1();
 						task.frameLevel++;
						 //notes 4
						 /*
						if (task.frameLevel+1 >= m_overSequence->GetLength() || !m_overSequence->IsBlockedByFrame(*(task.state->latches), task.frameLevel+1))
						{
							task.isLocated = true;
						}
						else
						{
							workingStack.pop();
						}
						*/
						//end notes 4
						continue;
					}
				}
				m_log->Tick();

	
				std::vector<int> assumption;
				GetAssumption(task.state, task.frameLevel, assumption);
				CAR_DEBUG("\nSAT CHECK on frame: " + std::to_string(task.frameLevel));
				CAR_DEBUG_s("From state: ", task.state);
				CAR_DEBUG_v("Assumption: ", assumption);
				bool result = m_mainSolver->SolveWithAssumption(assumption, task.frameLevel);
				m_log->StatMainSolver();
				if (result)
				{
					//Solver return SAT, get a new State, then continue
					CAR_DEBUG("Result >>> SAT <<<");
					std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > > pair;
					pair = m_mainSolver->GetAssignment();
					std::shared_ptr<State> newState(new State (task.state, pair.first, pair.second, task.state->depth+1));
					CAR_DEBUG_s("Get state: ", newState);
					m_underSequence.push(newState);

					if (m_settings.Visualization) {
						m_vis->addState(newState);
					}                              
					int newFrameLevel = GetNewLevel(newState);
					CAR_DEBUG("state get new level " + std::to_string(newFrameLevel));
					workingStack.emplace(newState, newFrameLevel, true);
					continue;
				}
				else
				{
					//Solver return UNSAT, get uc, then continue
					CAR_DEBUG("Result >>> UNSAT <<<");
					PushToRotation(task.state, task.frameLevel);
					auto uc = m_mainSolver->GetUnsatisfiableCore();
					if (uc->empty())
					{
						//placeholder, uc is empty => safe
					}
					// if (m_settings.pine){
					// 	// maybe in a low framelevel
					// 	std::vector<int> assumption_pine;
					// 	m_log->Tick();
					// 	GetAssumptionByPine(task.state, task.frameLevel, assumption_pine);
					// 	m_log->PrintSAT(assumption_pine, task.frameLevel);
					// 	m_log->StatPine();
					// 	// all length pine progress
					// 	if (task.state->pine_state_type == 1){
					// 		// redo the sat
					// 		m_mainSolver->SolveWithAssumption(assumption_pine, task.frameLevel);
					// 		auto uc_pine = m_mainSolver->GetUnsatisfiableCore();
					// 		m_log->StatPineInfo(task.state, uc_pine, uc);
					// 		if (m_settings.debug){
					// 			m_log->DebugPrintVector(*uc_pine, "pine uc:");
					// 			m_log->PrintPineInfo(task.state, uc_pine);
					// 		}
					// 	}
					// 	// part length pine
					// 	if (m_settings.debug && task.state->pine_state_type == 1){
					// 		for (int l: *task.state->pine_l_index){
					// 			m_log->DebugPrintSth("== pine part sat check ==");
					// 			//get part pine
					// 			std::vector<int> part_pine(l);
					// 			std::copy(assumption_pine.begin(), assumption_pine.begin()+l, part_pine.begin());
					// 			m_log->DebugPrintVector(part_pine, "part pine:");
					// 			//redo the sat
					// 			bool part_result = m_mainSolver->SolveWithAssumption(part_pine, task.frameLevel);
					// 			if (part_result){
					// 				m_log->DebugPrintSth("sat");
					// 			}else{
					// 				auto uc_part = m_mainSolver->GetUnsatisfiableCore();
					// 				m_log->DebugPrintVector(*uc_part, "part uc:");
					// 				m_log->PrintPineInfo(task.state, uc_part);
					// 			}
					// 		}
					// 	}
					// }
					
					CAR_DEBUG_v("Get UC:", *uc);
					m_log->Tick();
					AddUnsatisfiableCore(uc, task.frameLevel+1);
					m_log->StatUpdateUc();
					m_restart->UcCountsPlus1();
					CAR_DEBUG_o("Frames: ", m_overSequence.get());
					task.frameLevel++;
					//notes 4
					/*
					if (task.frameLevel+1 < m_overSequence->GetLength() && !m_overSequence->IsBlockedByFrame(*(task.state->latches), task.frameLevel+1))
					{
						task.isLocated = true;
					}
					else
					{
						workingStack.pop();
					}
					*/
					//end notes 4
					continue;
				}
			}
			CAR_DEBUG("\nNew Frame Added");
			if (m_settings.propagation)
			{
				Propagation();
			}
			std::vector<std::shared_ptr<std::vector<int> > > lastFrame;
			frameStep++;
			m_overSequence->GetFrame(frameStep, lastFrame);
			m_mainSolver->AddNewFrame(lastFrame, frameStep);
			m_overSequence->effectiveLevel++;
			m_restart->ResetUcCounts();
			

			m_log->Tick();
			if (isInvExisted())
			{
				m_log->StatInvSolver();
				return true;
			}
			m_log->StatInvSolver();
		}
	}
		


	void BackwardChecker::Init()
	{
		// if (m_settings.propagation)
		// {
		// 	m_overSequence.reset(new OverSequenceForProp(m_model->GetNumInputs()));
		// }
		// else
		// {
		// 	m_overSequence.reset(new OverSequence(m_model->GetNumInputs()));
		// }
		m_overSequence.reset(new OverSequenceNI());
		// m_overSequence.reset(new OverSequence(m_model->GetNumInputs()));
		if (m_settings.empi){
			slimLitOrder.heuristicLitOrder = &litOrder;
		}
		m_underSequence = UnderSequence();
		m_underSequence.push(m_initialState);
		if (m_settings.Visualization) {
			m_vis.reset(new Vis(m_settings, m_model));
			m_vis->addState(m_initialState);
		}
		m_mainSolver.reset(new MainSolver(m_model, false));
		m_invSolver.reset(new InvSolver(m_model));
		m_log->ResetClock();
		m_restart.reset(new Restart(m_settings));
		m_repeat_state_num = 0;
	}

	void BackwardChecker::AddUnsatisfiableCore(std::shared_ptr<std::vector<int> > uc, int frameLevel)
	{
		// if (frameLevel <= m_overSequence->effectiveLevel)
		// {
			m_mainSolver->AddUnsatisfiableCore(*uc, frameLevel);
		// }
		m_overSequence->Insert(uc, frameLevel, m_model);
		if (m_settings.empi){
			updateLitOrder(*uc);
		}
		if(frameLevel < m_minUpdateLevel)
		{
			m_minUpdateLevel = frameLevel;
		}
	}

	bool BackwardChecker::ImmediateSatisfiable(int badId)
	{
		std::vector<int>& init = *(m_initialState->latches);
		std::vector<int> assumptions;
		// GetAssumptionByPine(m_initialState, -1, assumptions);
		assumptions.resize((init.size()));
		std::copy(init.begin(), init.end(), assumptions.begin());
		// assumptions[assumptions.size()-1] = badId;
		bool result = m_mainSolver->SolveWithAssumptionAndBad(assumptions, badId);
		return result;
	}

	bool BackwardChecker::isInvExisted()
	{
		if (m_invSolver == nullptr)
		{
			m_invSolver.reset(new InvSolver(m_model));
		}
		bool result = false;
		for (int i = 0; i < m_overSequence->GetLength(); ++i)
		{
			if (IsInvariant(i))
			{
				result = true;
			}
		}
		m_invSolver = nullptr;
		return result;
	}

	int BackwardChecker::GetNewLevel(std::shared_ptr<State> state, int start)
	{
		for (int i = start; i < m_overSequence->GetLength(); ++i)
		{
			if (!m_overSequence->IsBlockedByFrame(*(state->latches), i))
			// if (!m_overSequence->IsBlockedByFrame(*(state->latches), i))
			{
				return i-1;
			}
		}
		return m_overSequence->GetLength()-1; //placeholder
	}

	bool BackwardChecker::IsInvariant(int frameLevel)
	{
		std::vector<std::shared_ptr<std::vector<int> > > frame;
		m_overSequence->GetFrame(frameLevel, frame);

		if (frameLevel < m_minUpdateLevel)
		{
			m_invSolver->AddConstraintOr(frame);
			return false;
		}

		m_invSolver->AddConstraintAnd(frame);
 		bool result = !m_invSolver->SolveWithAssumption();
		m_invSolver->FlipLastConstrain();
		m_invSolver->AddConstraintOr(frame);
		return result;
	}



}//namespace car