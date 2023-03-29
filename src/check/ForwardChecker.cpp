#include "ForwardChecker.h"
#include <stack>
#include <string>
namespace car {

sptr<Log> GLOBAL_LOG;
sptr<OverSequenceSet> GLOBAL_OS;

void signalHandler(int signum) {
  if (GLOBAL_LOG->need_same_stat() && GLOBAL_OS != nullptr)
    GLOBAL_OS->compute_monotone_degree_frame();
  GLOBAL_LOG->PrintStatistics();
  exit(signum);
}


ForwardChecker::ForwardChecker(Settings settings, std::shared_ptr<AigerModel> model) : m_settings(settings) {
  m_model = model;
  State::numInputs = model->GetNumInputs();
  State::numLatches = model->GetNumLatches();
  m_log.reset(new Log(settings, model));
  GLOBAL_LOG = m_log;

  const std::vector<int> &init = model->GetInitialState();
  std::shared_ptr<std::vector<int>> inputs(new std::vector<int>(State::numInputs, 0));
  std::shared_ptr<std::vector<int>> latches(new std::vector<int>());
  latches->reserve(init.size());

  for (int i = 0; i < init.size(); ++i) {
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
    if (m_log->need_same_stat() && GLOBAL_OS != nullptr)
      m_overSequence->compute_monotone_degree_frame();
    m_log->PrintStatistics();
  }
  return true;
}

bool ForwardChecker::Check(int badId) {
  signal(SIGINT, signalHandler);
#pragma region early stage
  if (m_model->GetTrueId() == badId)
    return false;
  else if (m_model->GetFalseId() == badId)
    return true;

  Init(badId);
  if (ImmediateSatisfiable(badId)) {
    CAR_DEBUG("Result >>> SAT <<<\n");
    auto pair = m_mainSolver->GetAssignment();
    CAR_DEBUG_v("Get Assignment:", *pair.second);
    m_initialState->inputs = pair.first;
    m_log->lastState = m_initialState;
    return false;
  }

  m_mainSolver->add_negation_bad();
  CAR_DEBUG("Result >>> UNSAT <<<\n");
  // frame 0 is init state
  m_overSequence->Init_Frame_0(m_initialState->latches);

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
    m_overSequence->PrintFramesInfo();
    m_minUpdateLevel = m_overSequence->GetLength();
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
    m_log->Tick();
    std::shared_ptr<State> startState = EnumerateStartState();
    m_log->StatStartSolver();
    if (startState == nullptr) return true;
    CAR_DEBUG("\nstate from start solver\n");
    while (startState != nullptr) {
      workingStack.push(Task(startState, frameStep, true));

      while (!workingStack.empty()) {
        if (m_settings.timelimit > 0 && m_log->IsTimeout()) {
          if (m_settings.Visualization) {
            m_vis->OutputGML(true);
          }
          m_overSequence->PrintFramesInfo();
          m_log->PrintSth("time out!!!");
          m_log->Timeout();
        }

        Task &task = workingStack.top();

        if (!task.isLocated) {
          m_log->Tick();
          task.frameLevel = GetNewLevel(task.state, task.frameLevel + 1);
          CAR_DEBUG("state get new level " + std::to_string(task.frameLevel) + "\n");
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
        CAR_DEBUG("\nSAT CHECK on frame: " + std::to_string(task.frameLevel) + "\n");
        CAR_DEBUG_s("From state: ", task.state);
        CAR_DEBUG_v("State Detail: ", *task.state->latches);
        std::vector<int> assumption;
        GetAssumption(task.state, task.frameLevel, assumption);
        // CAR_DEBUG_v("Primed Assumption: ", assumption);
        m_log->Tick();
        bool result = m_mainSolver->SolveWithAssumption(assumption, task.frameLevel);
        m_log->StatMainSolver();
        if (result) {
          // Solver return SAT, get a new State, then continue
          CAR_DEBUG("Result >>> SAT <<<\n");
          std::pair<std::shared_ptr<std::vector<int>>, std::shared_ptr<std::vector<int>>> pair, partial_pair;
          pair = m_mainSolver->GetAssignment();
          // CAR_DEBUG_v("Get Assignment:", *pair.second);
          m_log->Tick();
          partial_pair = get_predecessor(pair, task.state);
          m_log->Statpartial();
          std::shared_ptr<State> newState(new State(task.state, pair.first, partial_pair.second, task.state->depth + 1));
          m_underSequence.push(newState);
          CAR_DEBUG_s("Get state: ", newState);
          if (m_settings.Visualization) {
            m_vis->addState(newState);
          }
          int newFrameLevel = GetNewLevel(newState);
          workingStack.emplace(newState, newFrameLevel, true);
          continue;
        } else {
          // Solver return UNSAT, get uc, then continue
          CAR_DEBUG("Result >>> UNSAT <<<\n");
          auto uc = m_mainSolver->Getuc(m_settings.minimal_uc);
          if (uc->empty()) {
            // placeholder, uc is empty => safe
          }
          CAR_DEBUG_v("Get UC:", *uc);
          m_log->Tick();
          if (m_settings.ctg)
            if (generalize_ctg(uc, task.frameLevel))
              updateLitOrder(*uc);
          m_log->Statmuc();
          CAR_DEBUG_v("Get UC:", *uc);
          m_log->Tick();
          if (AddUnsatisfiableCore(uc, task.frameLevel + 1))
            m_overSequence->propagate_uc_from_lvl(uc, task.frameLevel + 1, m_branching);
          m_log->StatUpdateUc();
          CAR_DEBUG_o("Frames: ", m_overSequence.get());
          task.frameLevel++;
          continue;
        }
      } // end while (!workingStack.empty())
      m_log->Tick();
      startState = EnumerateStartState();
      m_log->StatStartSolver();
      CAR_DEBUG("\nstate from start solver\n");
    }

    CAR_DEBUG("\nNew Frame Added\n");
    std::vector<std::shared_ptr<std::vector<int>>> lastFrame;
    frameStep++;

    if (m_settings.propagation) {
      m_log->Tick();
      Propagation();
      m_log->StatPropagation();
    }
    CAR_DEBUG_o("Frames: ", m_overSequence.get());
    m_mainSolver->simplify();
    m_overSequence->effectiveLevel++;
    m_startSovler->UpdateStartSolverFlag();

    if (m_settings.pVisualization) {
      m_vis->clear_tree();
      for (int k = 1; k < m_overSequence->GetLength(); k++) {
        std::vector<std::shared_ptr<std::vector<int>>> frame;
        m_overSequence->GetFrame(k, frame);
        for (auto uc : frame) {
          m_vis->add_tree_node(*uc, k);
        }
      }
      m_vis->print_tree(frameStep);
    }

    m_log->Tick();
    if (isInvExisted()) {
      m_log->StatInvSolver();
      return true;
    }
    m_log->StatInvSolver();
  }
}


void ForwardChecker::Init(int badId) {
  m_log->ResetClock();
  m_log->Tick();
  m_badId = badId;
  // m_overSequence.reset(new OverSequenceNI(m_model));
  m_overSequence.reset(new OverSequenceSet(m_model));
  m_overSequence->set_log(m_log);
  GLOBAL_OS = m_overSequence;
  m_branching.reset(new Branching(m_settings.Branching));
  litOrder.branching = m_branching;
  blockerOrder.branching = m_branching;
  if (m_settings.Visualization) {
    m_vis.reset(new Vis(m_settings, m_model));
    m_vis->addState(m_initialState);
  }
  if (m_settings.pVisualization) {
    m_vis.reset(new Vis(m_settings, m_model));
  }
  m_overSequence->isForward = true;
  m_underSequence = UnderSequence();
  m_mainSolver.reset(new MainSolver(m_model, true, true));
  m_lifts.reset(new MainSolver(m_model, true, true));
  m_invSolver.reset(new InvSolver(m_model));
  m_startSovler.reset(new StartSolver(m_model, badId));
  m_overSequence->set_solver(m_mainSolver);
  m_log->StatInit();
}

bool ForwardChecker::AddUnsatisfiableCore(std::shared_ptr<std::vector<int>> uc, int frameLevel) {
  m_mainSolver->AddUnsatisfiableCore(*uc, frameLevel);
  if (frameLevel > m_overSequence->effectiveLevel) {
    m_startSovler->AddClause(-m_startSovler->GetFlag(), *uc);
  }
  if (frameLevel < m_minUpdateLevel) {
    m_minUpdateLevel = frameLevel;
  }

  m_overSequence->Insert(uc, frameLevel);
  return true;
}

bool ForwardChecker::ImmediateSatisfiable(int badId) {
  std::vector<int> &init = *(m_initialState->latches);
  std::vector<int> assumptions;
  assumptions.resize((init.size()));
  std::copy(init.begin(), init.end(), assumptions.begin());
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
      m_log->PrintSth("Proof at frame " + std::to_string(i) + "\n");
      m_overSequence->compute_cls_in_fixpoint_ratio(i);
      m_overSequence->PrintFramesInfo();
      result = true;
      break;
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