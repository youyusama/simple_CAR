#include "BackwardChecker.h"
#include <stack>
#include <string>

namespace car {
BackwardChecker::BackwardChecker(Settings settings, shared_ptr<AigerModel> model) : m_settings(settings) {
    m_model = model;
    State::numInputs = model->GetNumInputs();
    State::numLatches = model->GetNumLatches();
    m_log.reset(new Log(settings, model));
    const vector<int> &init = model->GetInitialState();
    shared_ptr<vector<int>> inputs(new vector<int>(State::numInputs, 0));
    shared_ptr<vector<int>> latches(new vector<int>());
    latches->reserve(init.size());

    for (int i = 0; i < init.size(); ++i) {
        latches->push_back(init[i]);
    }
    m_initialState.reset(new State(nullptr, inputs, latches, 0));
}

bool BackwardChecker::Run() {
    for (int i = 0; i < m_model->GetNumBad(); ++i) {
        bool result = Check(m_model->GetBad());
        if (result) {
            m_log->PrintSafe(i);
        } else {
            m_log->PrintCounterExample(i, false);
        }
        m_log->PrintStatistics();
    }
    return true;
}

bool BackwardChecker::Check(int badId) {
#pragma region early stage
    if (m_model->GetTrueId() == badId)
        return false;
    else if (m_model->GetFalseId() == badId)
        return true;

    Init();

    if (ImmediateSatisfiable(badId)) {
        CAR_DEBUG("Result >>> SAT <<<\n");
        pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> pair;
        pair = m_mainSolver->GetAssignment();
        CAR_DEBUG_v("Get Assignment:", *pair.second);

        shared_ptr<State> newState(new State(m_initialState, pair.first, pair.second, 1));
        m_log->lastState = newState;
        return false;
    }
    CAR_DEBUG("Result >>> UNSAT <<<\n");
    m_log->Tick();
    auto uc = m_mainSolver->GetUnsatisfiableCoreFromBad(badId);
    m_log->StatMainSolver();
    if (uc->empty())
        return true;
    CAR_DEBUG_v("Get UC:", *uc);
    AddUnsatisfiableCore(uc, 0);
    m_overSequence->effectiveLevel = 0;
    CAR_DEBUG_o("Frames: ", m_overSequence.get());
#pragma endregion

    // main stage
    int frameStep = 0;
    stack<Task> workingStack;
    while (true) {
        m_overSequence->PrintFramesInfo();
        m_minUpdateLevel = m_overSequence->GetLength();
        if (m_settings.end) // from the deep and the end
        {
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
            Task &task = workingStack.top();

            if (!task.isLocated) {
                m_log->Tick();
                task.frameLevel = GetNewLevel(task.state, task.frameLevel + 1);
                CAR_DEBUG("state get new level " + to_string(task.frameLevel) + "\n");
                m_log->StatGetNewLevel();
                if (task.frameLevel > m_overSequence->effectiveLevel) {
                    workingStack.pop();
                    continue;
                }
            }
            task.isLocated = false;

            if (task.frameLevel == -1) {
                m_log->Tick();
                vector<int> assumption;
                GetAssumption(task.state, task.frameLevel, assumption);
                CAR_DEBUG("\nSAT CHECK on frame: " + to_string(task.frameLevel) + "\n");
                CAR_DEBUG_s("From state: ", task.state);
                // CAR_DEBUG_v("Assumption: ", assumption);
                bool result = m_mainSolver->SolveWithAssumptionAndBad(assumption, badId);
                m_log->StatMainSolver();
                if (result) {
                    CAR_DEBUG("Result >>> SAT <<<\n");
                    pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> pair;
                    pair = m_mainSolver->GetAssignment();
                    CAR_DEBUG_v("Get Assignment:", *pair.second);
                    shared_ptr<State> newState(new State(task.state, pair.first, pair.second, task.state->depth + 1));
                    m_log->lastState = newState;
                    m_overSequence->PrintFramesInfo();
                    return false;
                } else {
                    CAR_DEBUG("Result >>> UNSAT <<<\n");
                    auto uc = m_mainSolver->GetUnsatisfiableCore();
                    CAR_DEBUG_v("Get UC:", *uc);
                    updateLitOrder(*uc);
                    m_log->Tick();
                    AddUnsatisfiableCore(uc, task.frameLevel + 1);
                    m_log->StatUpdateUc();
                    task.frameLevel++;
                    continue;
                }
            }

            vector<int> assumption;
            GetAssumption(task.state, task.frameLevel, assumption);
            CAR_DEBUG("\nSAT CHECK on frame: " + to_string(task.frameLevel) + "\n");
            CAR_DEBUG_s("From state: ", task.state);
            // CAR_DEBUG_v("Assumption: ", assumption);
            m_log->Tick();
            bool result = m_mainSolver->SolveWithAssumption(assumption, task.frameLevel);
            m_log->StatMainSolver();
            if (result) {
                // Solver return SAT, get a new State, then continue
                CAR_DEBUG("Result >>> SAT <<<\n");
                pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> pair;
                pair = m_mainSolver->GetAssignment();
                shared_ptr<State> newState(new State(task.state, pair.first, pair.second, task.state->depth + 1));
                CAR_DEBUG_s("Get state: ", newState);
                m_underSequence.push(newState);
                m_log->Tick();
                int newFrameLevel = GetNewLevel(newState);
                CAR_DEBUG("state get new level " + to_string(newFrameLevel) + "\n");
                m_log->StatGetNewLevel();
                workingStack.emplace(newState, newFrameLevel, true);
                continue;
            } else {
                // Solver return UNSAT, get uc, then continue
                CAR_DEBUG("Result >>> UNSAT <<<\n");
                auto uc = m_mainSolver->Getuc(false);
                CAR_DEBUG_v("Get UC:", *uc);
                m_log->Tick();
                // if (m_settings.ctg)
                //   if (generalize_ctg(uc, task.frameLevel))
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
        }
        CAR_DEBUG("\nNew Frame Added\n");
        if (m_settings.propagation) {
            m_log->Tick();
            Propagation();
            m_log->StatPropagation();
        }
        vector<shared_ptr<vector<int>>> lastFrame;
        frameStep++;
        m_mainSolver->simplify();
        m_overSequence->effectiveLevel++;


        m_log->Tick();
        if (isInvExisted()) {
            m_log->StatInvSolver();
            return true;
        }
        m_log->StatInvSolver();
    }
}


void BackwardChecker::Init() {
    m_overSequence.reset(new OverSequenceSet(m_model));
    m_overSequence->set_log(m_log);
    m_branching.reset(new Branching(m_settings.Branching));
    litOrder.branching = m_branching;
    blockerOrder.branching = m_branching;
    m_underSequence = UnderSequence();
    m_underSequence.push(m_initialState);
    m_mainSolver.reset(new MainSolver(m_model, false, true));
    m_invSolver.reset(new InvSolver(m_model));
    m_overSequence->set_solver(m_mainSolver);
    m_log->ResetClock();
}

bool BackwardChecker::AddUnsatisfiableCore(shared_ptr<vector<int>> uc, int frameLevel) {
    m_mainSolver->AddUnsatisfiableCore(*uc, frameLevel);
    if (frameLevel < m_minUpdateLevel) {
        m_minUpdateLevel = frameLevel;
    }
    m_overSequence->Insert(uc, frameLevel);
    return true;
}

bool BackwardChecker::ImmediateSatisfiable(int badId) {
    vector<int> &init = *(m_initialState->latches);
    vector<int> assumptions;
    assumptions.resize((init.size()));
    copy(init.begin(), init.end(), assumptions.begin());
    bool result = m_mainSolver->SolveWithAssumptionAndBad(assumptions, badId);
    return result;
}

bool BackwardChecker::isInvExisted() {
    if (m_invSolver == nullptr) {
        m_invSolver.reset(new InvSolver(m_model));
    }
    bool result = false;
    for (int i = 0; i < m_overSequence->GetLength(); ++i) {
        if (IsInvariant(i)) {
            m_log->PrintSth("Proof at frame " + to_string(i) + "\n");
            m_overSequence->PrintFramesInfo();
            result = true;
            break;
        }
    }
    m_invSolver = nullptr;
    return result;
}

int BackwardChecker::GetNewLevel(shared_ptr<State> state, int start) {
    for (int i = start; i < m_overSequence->GetLength(); ++i) {
        if (!m_overSequence->IsBlockedByFrame_lazy(*(state->latches), i)) {
            return i - 1;
        }
    }
    return m_overSequence->GetLength() - 1; // placeholder
}

bool BackwardChecker::IsInvariant(int frameLevel) {
    vector<shared_ptr<vector<int>>> frame;
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