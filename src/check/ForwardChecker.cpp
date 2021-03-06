#include "ForwardChecker.h"
#include <stack>
#include <string>

namespace car {
ForwardChecker::ForwardChecker(Settings settings, std::shared_ptr<AigerModel> model) : m_settings(settings) {
  m_model = model;
  State::numInputs = model->GetNumInputs();
  State::numLatches = model->GetNumLatches();
  m_log.reset(new Log(settings, model));

  const std::vector<int> &init = model->GetInitialState();
  std::shared_ptr<std::vector<int>> inputs(new std::vector<int>(State::numInputs, 0));
  std::shared_ptr<std::vector<int>> latches(new std::vector<int>());
  latches->reserve(State::numLatches);

  for (int i = 0; i < State::numLatches; ++i) {
    latches->push_back(init[i]);
  }
  m_initialState.reset(new State(nullptr, inputs, latches, 0));
}

bool ForwardChecker::Run() {
  for (int i = 0, maxI = m_model->GetOutputs().size(); i < maxI; ++i) {
    int badId = m_model->GetOutputs().at(i);
    bool result = Check(badId);
    // PrintUC();
    if (result) {
      m_log->PrintSafe(i);
    } else // unsafe
    {
      m_log->PrintCounterExample(i, true);
    }
    if (m_settings.Visualization) {
      m_vis->OutputGML(false);
    }
    m_log->PrintStatistics();
  }
  return true;
}

bool ForwardChecker::Check(int badId) {
#pragma region early stage
  if (m_model->GetTrueId() == badId)
    return false;
  else if (m_model->GetFalseId() == badId)
    return true;

  Init(badId);

  if (ImmediateSatisfiable(badId)) {
    CAR_DEBUG("Result >>> SAT <<<");
    auto pair = m_mainSolver->GetAssignment();
    CAR_DEBUG_v("Get Assignment:", *pair.second);
    m_initialState->inputs = pair.first;
    return false;
  }

  CAR_DEBUG("Result >>> UNSAT <<<");
  // frame 0 is init state
  for (auto latch : *(m_initialState->latches)) {
    std::shared_ptr<std::vector<int>> puc(new std::vector<int>{-latch});
    m_overSequence->Insert(puc, 0);
  }

  std::vector<std::shared_ptr<std::vector<int>>> frame;
  m_overSequence->GetFrame(0, frame);
  m_mainSolver->AddNewFrame(frame, 0);
  m_overSequence->effectiveLevel = 0;
  m_startSovler->UpdateStartSolverFlag();
  CAR_DEBUG_o("Frames: ", m_overSequence.get());
#pragma endregion

  // main stage
  int frameStep = 0;
  std::stack<Task> workingStack;

  while (true) {
    m_log->PrintFramesInfo(m_overSequence.get());
    m_minUpdateLevel = m_overSequence->GetLength();
    std::shared_ptr<State> startState = EnumerateStartState();
    while (startState != nullptr) {
      workingStack.push(Task(startState, frameStep, true));
      if (m_settings.end) { // from the deep and the end
        for (int i = m_underSequence.size() - 1; i >= 0; i--) {
          for (int j = m_underSequence[i].size() - 1; j >= 0; j--) {
            workingStack.emplace(m_underSequence[i][j], frameStep, false);
          }
        }
      } else { // from the shallow and the start
        for (int i = 0; i < m_underSequence.size(); i++) {
          for (int j = 0; j < m_underSequence[i].size(); j++) {
            workingStack.emplace(m_underSequence[i][j], frameStep, false);
          }
        }
      }

      while (!workingStack.empty()) {
        if (m_settings.timelimit > 0 && m_log->IsTimeout()) {
          if (m_settings.Visualization) {
            m_vis->OutputGML(true);
          }
          m_log->PrintFramesInfo(m_overSequence.get());
          m_log->PrintSth("time out!!!");
          m_log->Timeout();
        }

        Task &task = workingStack.top();

        if (!task.isLocated) {
          m_log->Tick();
          task.frameLevel = GetNewLevel(task.state, task.frameLevel + 1);
          CAR_DEBUG("state get new level " + std::to_string(task.frameLevel));
          m_log->StatGetNewLevel();
          if (task.frameLevel > m_overSequence->effectiveLevel) {
            workingStack.pop();
            continue;
          }
        }
        task.isLocated = false;

        if (task.frameLevel == -1) {
          m_initialState->preState = task.state->preState;
          m_initialState->inputs = task.state->inputs;
          m_log->lastState = m_initialState;
          return false;
        }
        m_log->Tick();
        // bool result = m_mainSolver->SolveWithAssumption(*(task.state->latches), task.frameLevel);
        std::vector<int> assumption;
        GetAssumption(task.state, task.frameLevel, assumption);
        CAR_DEBUG("\nSAT CHECK on frame: " + std::to_string(task.frameLevel));
        CAR_DEBUG_s("From state: ", task.state);
        CAR_DEBUG_v("Assumption: ", assumption);
        bool result = m_mainSolver->SolveWithAssumption(assumption, task.frameLevel);
        m_log->StatMainSolver();
        if (result) {
          // Solver return SAT, get a new State, then continue
          CAR_DEBUG("Result >>> SAT <<<");
          std::pair<std::shared_ptr<std::vector<int>>, std::shared_ptr<std::vector<int>>> pair;
          pair = m_mainSolver->GetAssignment();
          CAR_DEBUG_v("Get Assignment:", *pair.second);
          std::shared_ptr<State> newState(new State(task.state, pair.first, pair.second, task.state->depth + 1));
          m_underSequence.push(newState);
          if (m_settings.Visualization) {
            m_vis->addState(newState);
          }
          int newFrameLevel = GetNewLevel(newState);
          workingStack.emplace(newState, newFrameLevel, true);
          continue;
        } else {
          // Solver return UNSAT, get uc, then continue
          CAR_DEBUG("Result >>> UNSAT <<<");
          auto uc = m_mainSolver->GetUnsatisfiableCore();
          if (uc->empty()) {
            // placeholder, uc is empty => safe
          }
          CAR_DEBUG_v("Get UC:", *uc);
          m_log->Tick();
          AddUnsatisfiableCore(uc, task.frameLevel + 1);
          m_log->StatUpdateUc();
          CAR_DEBUG_o("Frames: ", m_overSequence.get());
          task.frameLevel++;
          continue;
        }
      } // end while (!workingStack.empty())
      startState = EnumerateStartState();
    }

    CAR_DEBUG("\nNew Frame Added");
    if (m_settings.propagation) {
      Propagation();
    }
    std::vector<std::shared_ptr<std::vector<int>>> lastFrame;
    frameStep++;
    m_overSequence->GetFrame(frameStep, lastFrame);
    m_mainSolver->AddNewFrame(lastFrame, frameStep);
    m_overSequence->effectiveLevel++;
    m_startSovler->UpdateStartSolverFlag();


    m_log->Tick();
    if (isInvExisted()) {
      m_log->StatInvSolver();
      return true;
    }
    m_log->StatInvSolver();
  }
}


void ForwardChecker::Init(int badId) {
  // if (m_settings.propagation) {
  //   m_overSequence.reset(new OverSequenceForProp(m_model->GetNumInputs()));
  // } else {
  //   m_overSequence.reset(new OverSequence(m_model->GetNumInputs()));
  // }
  m_overSequence.reset(new OverSequenceNI(m_model));
  if (m_settings.empi) {
    slimLitOrder.heuristicLitOrder = &litOrder;
  }
  if (m_settings.Visualization) {
    m_vis.reset(new Vis(m_settings, m_model));
    m_vis->addState(m_initialState);
  }
  m_overSequence->isForward = true;
  m_underSequence = UnderSequence();
  m_underSequence.push(m_initialState);
  m_mainSolver.reset(new MainSolver(m_model, true));
  m_invSolver.reset(new InvSolver(m_model));
  m_startSovler.reset(new StartSolver(m_model, badId));
  m_log->ResetClock();
}

void ForwardChecker::AddUnsatisfiableCore(std::shared_ptr<std::vector<int>> uc, int frameLevel) {
  if (frameLevel <= m_overSequence->effectiveLevel) {
    m_mainSolver->AddUnsatisfiableCore(*uc, frameLevel);
  } else {
    m_startSovler->AddClause(-m_startSovler->GetFlag(), *uc);
  }
  m_overSequence->Insert(uc, frameLevel);
  if (m_settings.empi) {
    updateLitOrder(*uc);
  }
  if (frameLevel < m_minUpdateLevel) {
    m_minUpdateLevel = frameLevel;
  }
}

bool ForwardChecker::ImmediateSatisfiable(int badId) {
  std::vector<int> &init = *(m_initialState->latches);
  std::vector<int> assumptions;
  assumptions.resize((init.size()));
  std::copy(init.begin(), init.end(), assumptions.begin());
  // assumptions[assumptions.size()-1] = badId;
  bool result = m_mainSolver->SolveWithAssumptionAndBad(assumptions, badId);
  return result;
}

bool ForwardChecker::isInvExisted() {
  if (m_invSolver == nullptr) {
    m_invSolver.reset(new InvSolver(m_model));
  }
  bool result = false;
  for (int i = 0; i < m_overSequence->GetLength(); ++i) {
    if (IsInvariant(i)) {
      result = true;
    }
  }
  m_invSolver = nullptr;
  return result;
}

int ForwardChecker::GetNewLevel(std::shared_ptr<State> state, int start) {
  for (int i = start; i < m_overSequence->GetLength(); ++i) {
    if (!m_overSequence->IsBlockedByFrame_lazy(*(state->latches), i)) {
      return i - 1;
    }
  }
  return m_overSequence->GetLength() - 1; // placeholder
}

bool ForwardChecker::IsInvariant(int frameLevel) {
  std::vector<std::shared_ptr<std::vector<int>>> frame;
  m_overSequence->GetFrame(frameLevel, frame);

  if (frameLevel < m_minUpdateLevel) {
    m_invSolver->AddConstraintOr(frame);
    return false;
  }

  m_invSolver->AddConstraintAnd(frame);
  bool result = !m_invSolver->SolveWithAssumption();
  m_invSolver->FlipLastConstrain();
  m_invSolver->AddConstraintOr(frame);
  return result;
}


} // namespace car