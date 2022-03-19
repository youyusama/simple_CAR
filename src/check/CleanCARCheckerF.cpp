#include "CleanCARCheckerF.h"
#include <stack>
#include <string>

namespace car
{
	CleanCARCheckerF::CleanCARCheckerF(Settings settings, std::shared_ptr<AigerModel> model) : m_settings(settings)
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
			latches->push_back(init[i]);
    
		m_initialState.reset(new State(nullptr, inputs, latches, 0));
	}

	bool CleanCARCheckerF::Run()
	{
		for (int i = 0, maxI = m_model->GetOutputs().size(); i < maxI; ++i)	{
			int badId = m_model->GetOutputs().at(i);
			bool result = Check(badId);

			if (result) m_log->PrintSafe(i); //safe
			else m_log->PrintCounterExample(i, true); //unsafe

			m_log->PrintStatistics();
		}
		return true;
	}

	bool CleanCARCheckerF::Check(int badId)
	{
#pragma region early stage
    // bad is constantly true or false
		if (m_model->GetTrueId() == badId) return false;
		else if (m_model->GetFalseId() == badId) return true;

 		Init(badId);

		if (ImmediateSatisfiable(badId)){
#ifdef __DEBUG__
			auto pair = m_mainSolver->GetAssignment(m_log->m_res);
#else
			auto pair = m_mainSolver->GetAssignment();
#endif
			m_initialState->inputs = pair.first;
			return false;
		}

		for(auto latch:*(m_initialState->latches)){
			std::shared_ptr<std::vector<int>> puc(new std::vector<int> {-latch});
			m_overSequence->Insert(puc, 0);
		}

#ifdef __DEBUG__
		m_log->PrintUcNums(uc, m_overSequence);
#endif

		std::vector<std::shared_ptr<std::vector<int> > > frame;
		m_overSequence->GetFrame(0, frame);
		m_mainSolver->AddNewFrame(frame, 0);
		m_overSequence->effectiveLevel = 0;
		m_startSovler->UpdateStartSolverFlag();
# pragma endregion 

		//main stage
		int frameStep = 0;
		std::stack<Task> workingStack;
		
		while (true)
		{
			m_log->PrintFramesInfo(m_overSequence.get());
			m_minUpdateLevel = m_overSequence->GetLength();
			std::shared_ptr<State> startState = EnumerateStartState();
			while(startState != nullptr)
			{
				workingStack.push(Task(startState ,frameStep, true));
				while (!workingStack.empty())
				{
					if (m_settings.timelimit > 0 && m_log->IsTimeout())
					{
						m_log->Timeout();
					}

					Task& task = workingStack.top();
					
					if (!task.isLocated)
					{
						m_log->Tick();
						task.frameLevel = GetNewLevel(task.state, task.frameLevel+1);
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
						m_initialState->preState = task.state->preState;
						m_initialState->inputs = task.state->inputs;
						m_log->lastState = m_initialState;
						return false;
					}
					m_log->Tick();

					//bool result = m_mainSolver->SolveWithAssumption(*(task.state->latches), task.frameLevel);
					std::vector<int> assumption;
					GetAssumption(task.state, task.frameLevel, assumption);
					bool result = m_mainSolver->SolveWithAssumption(assumption, task.frameLevel);
					m_log->StatMainSolver();
					if (result)
					{
						//Solver return SAT, get a new State, then continue
						std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > > pair;
						pair = m_mainSolver->GetAssignment();
						std::shared_ptr<State> newState(new State (task.state, pair.first, pair.second, task.state->depth+1));
						m_underSequence.push(newState);
						int newFrameLevel = GetNewLevel(newState);
						workingStack.emplace(newState, newFrameLevel, true);
						continue;
					}
					else
					{
						//Solver return UNSAT, get uc, then continue
						auto uc = m_mainSolver->GetUnsatisfiableCore();
						if (uc->empty()){
							//placeholder, uc is empty => safe
						}
						m_log->Tick();
						AddUnsatisfiableCore(uc, task.frameLevel+1);
						m_log->StatUpdateUc();
	#ifdef __DEBUG__
						m_log->PrintUcNums(uc, m_overSequence);
	#endif
						task.frameLevel++;
						continue;
					}
				}// end while (!workingStack.empty())
				startState = EnumerateStartState();
			}
			
			std::vector<std::shared_ptr<std::vector<int> > > lastFrame;
			frameStep++;
			m_overSequence->GetFrame(frameStep, lastFrame);
			m_mainSolver->AddNewFrame(lastFrame, frameStep);
			m_overSequence->effectiveLevel++;
			m_startSovler->UpdateStartSolverFlag();

			m_log->Tick();
			if (isInvExisted())
			{
				m_log->StatInvSolver();
				return true;
			}
			m_log->StatInvSolver();
		}
	}
		


	void CleanCARCheckerF::Init(int badId)
	{
		m_overSequence.reset(new OverSequence(m_model->GetNumInputs()));
		m_overSequence->isForward = true;
		m_underSequence = UnderSequence();
		m_underSequence.push(m_initialState);
		m_mainSolver.reset(new MainSolver(m_model, true));
		m_invSolver.reset(new InvSolver(m_model));
		m_startSovler.reset(new StartSolver(m_model, badId));
		m_log->ResetClock();
	}

	void CleanCARCheckerF::AddUnsatisfiableCore(std::shared_ptr<std::vector<int> > uc, int frameLevel)
	{
		if (frameLevel <= m_overSequence->effectiveLevel)
		{
			m_mainSolver->AddUnsatisfiableCore(*uc, frameLevel);
		}
		else
		{
			m_startSovler->AddClause(-m_startSovler->GetFlag(), *uc);
		}
		m_overSequence->Insert(uc, frameLevel);
		if(frameLevel < m_minUpdateLevel)
		{
			m_minUpdateLevel = frameLevel;
		}
	}

	bool CleanCARCheckerF::ImmediateSatisfiable(int badId)
	{
		std::vector<int>& init = *(m_initialState->latches);
		std::vector<int> assumptions;
		assumptions.resize((init.size()));
		std::copy(init.begin(), init.end(), assumptions.begin());
		//assumptions[assumptions.size()-1] = badId;
		bool result = m_mainSolver->SolveWithAssumptionAndBad(assumptions, badId);
		return result;
	}

	bool CleanCARCheckerF::isInvExisted()
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

	int CleanCARCheckerF::GetNewLevel(std::shared_ptr<State> state, int start = 0)
	{
		for (int i = start; i < m_overSequence->GetLength(); ++i)
		{
			if (!m_overSequence->IsBlockedByFrame(*(state->latches), i))
			{
				return i-1;
			}
		}
		return m_overSequence->GetLength()-1; //placeholder
	}

	bool CleanCARCheckerF::IsInvariant(int frameLevel)
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