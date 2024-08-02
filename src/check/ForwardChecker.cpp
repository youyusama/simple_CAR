#include "ForwardChecker.h"
#include <stack>
#include <string>
namespace car {

ForwardChecker::ForwardChecker(Settings settings,
                               shared_ptr<AigerModel> model,
                               shared_ptr<Log> log) : m_settings(settings),
                                                      m_model(model),
                                                      m_log(log) {
    State::numInputs = model->GetNumInputs();
    State::numLatches = model->GetNumLatches();
    m_lastState = nullptr;
    GLOBAL_LOG = m_log;
}

bool ForwardChecker::Run() {
    signal(SIGINT, signalHandler);

    bool result = Check(m_model->GetBad());
    if (result) {
        m_log->L(0, "Safe");
        if (m_settings.witness)
            OutputWitness(m_model->GetBad());
    } else {
        m_log->L(0, "Unsafe");
        if (m_settings.witness)
            OutputCounterExample(m_model->GetBad());
    }
    m_log->PrintStatistics();
    return true;
}

bool ForwardChecker::Check(int badId) {
    if (m_model->GetTrueId() == badId)
        return false;
    else if (m_model->GetFalseId() == badId)
        return true;

    Init(badId);
    if (ImmediateSatisfiable(badId)) {
        m_log->L(3, "Result >>> SAT <<<");
        auto pair = m_mainSolver->GetAssignment();
        m_log->L(3, "Get Assignment:", CubeToStr(*pair.second));
        m_initialState->inputs = pair.first;
        m_lastState = m_initialState;
        return false;
    }

    m_mainSolver->add_negation_bad();
    m_log->L(3, "Result >>> UNSAT <<<");
    // frame 0 is init state
    m_overSequence->Init_Frame_0(m_initialState->latches);

    vector<shared_ptr<vector<int>>> frame;
    m_overSequence->GetFrame(0, frame);
    m_mainSolver->AddNewFrame(frame, 0);
    m_overSequence->effectiveLevel = 0;
    m_startSovler->UpdateStartSolverFlag();
    m_log->L(3, "Frames: ", m_overSequence->FramesInfo());
#pragma endregion

    // main stage
    int frameStep = 0;
    stack<Task> workingStack;
    while (true) {
        m_log->L(1, m_overSequence->FramesInfo());
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
        shared_ptr<State> startState = EnumerateStartState();
        m_log->StatStartSolver();
        if (startState == nullptr) {
            m_overSequence->SetInvariantLevel(frameStep);
            return true;
        }
        m_log->L(3, "\nstate from start solver");
        while (startState != nullptr) {
            workingStack.push(Task(startState, frameStep, true));

            while (!workingStack.empty()) {
                Task &task = workingStack.top();

                if (!task.isLocated) {
                    task.frameLevel = GetNewLevel(task.state, task.frameLevel + 1);
                    m_log->L(3, "state get new level ", task.frameLevel);
                    if (task.frameLevel > m_overSequence->effectiveLevel) {
                        workingStack.pop();
                        continue;
                    }
                }
                task.isLocated = false;

                if (task.frameLevel == -1) {
                    m_initialState->preState = task.state->preState;
                    m_initialState->inputs = task.state->inputs;
                    m_lastState = m_initialState;
                    return false;
                }
                m_log->L(3, "SAT CHECK on frame: ", task.frameLevel);
                m_log->L(3, "From state: ", CubeToStrShort(*task.state->latches));
                m_log->L(3, "State Detail: ", CubeToStr(*task.state->latches));
                vector<int> assumption;
                GetAssumption(task.state, task.frameLevel, assumption);
                m_log->Tick();
                bool result = m_mainSolver->SolveWithAssumption(assumption, task.frameLevel);
                m_log->StatMainSolver();
                if (result) {
                    // Solver return SAT, get a new State, then continue
                    m_log->L(3, "Result >>> SAT <<<");
                    pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> pair, partial_pair;
                    pair = m_mainSolver->GetAssignment();
                    partial_pair = get_predecessor(pair, task.state);
                    shared_ptr<State> newState(new State(task.state, pair.first, partial_pair.second, task.state->depth + 1));
                    m_underSequence.push(newState);
                    m_log->L(3, "Get state: ", CubeToStrShort(*newState->latches));
                    int newFrameLevel = GetNewLevel(newState);
                    workingStack.emplace(newState, newFrameLevel, true);
                    continue;
                } else {
                    // Solver return UNSAT, get uc, then continue
                    m_log->L(3, "Result >>> UNSAT <<<");
                    auto uc = m_mainSolver->Getuc(false);
                    assert(uc->size() > 0);
                    m_log->L(3, "Get UC: ", CubeToStr(*uc));
                    if (generalize_ctg(uc, task.frameLevel))
                        updateLitOrder(*uc);
                    m_log->L(3, "Get UC: ", CubeToStr(*uc));
                    if (AddUnsatisfiableCore(uc, task.frameLevel + 1)) {
                        m_log->Tick();
                        m_overSequence->propagate_uc_from_lvl(uc, task.frameLevel + 1, m_branching);
                        m_log->StatPropagation();
                    }
                    m_log->StatUpdateUc();
                    m_log->L(3, "Frames: ", m_overSequence->FramesInfo());
                    task.frameLevel++;
                    continue;
                }
            } // end while (!workingStack.empty())
            m_log->Tick();
            startState = EnumerateStartState();
            m_log->StatStartSolver();
            m_log->L(3, "\nstate from start solver");
        }

        frameStep++;
        m_log->L(3, "\nNew Frame Added");

        m_log->Tick();
        Propagation();
        m_log->StatPropagation();

        m_log->L(3, m_overSequence->FramesDetail());
        m_mainSolver->simplify();
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
    const cube &init = m_model->GetInitialState();
    shared_ptr<cube> inputs(new cube(State::numInputs, 0));
    shared_ptr<cube> latches(new cube());
    latches->reserve(init.size());
    latches->insert(latches->end(), init.begin(), init.end());
    m_initialState.reset(new State(nullptr, inputs, latches, 0));

    m_badId = badId;
    m_overSequence.reset(new OverSequenceSet(m_model));
    m_branching.reset(new Branching(m_settings.Branching));
    litOrder.branching = m_branching;
    blockerOrder.branching = m_branching;
    m_overSequence->isForward = true;
    m_underSequence = UnderSequence();
    m_mainSolver.reset(new MainSolver(m_model, true, true));
    m_lifts.reset(new MainSolver(m_model, true, true));
    m_invSolver.reset(new InvSolver(m_model));
    m_startSovler.reset(new StartSolver(m_model, badId));
    m_overSequence->set_solver(m_mainSolver);
}

bool ForwardChecker::AddUnsatisfiableCore(shared_ptr<vector<int>> uc, int frameLevel) {
    m_log->Tick();

    m_mainSolver->AddUnsatisfiableCore(*uc, frameLevel);
    if (frameLevel > m_overSequence->effectiveLevel) {
        m_startSovler->AddClause(-m_startSovler->GetFlag(), *uc);
    }
    if (frameLevel < m_minUpdateLevel) {
        m_minUpdateLevel = frameLevel;
    }
    m_overSequence->Insert(uc, frameLevel);

    m_log->StatUpdateUc();
    return true;
}

bool ForwardChecker::ImmediateSatisfiable(int badId) {
    vector<int> &init = *(m_initialState->latches);
    vector<int> assumptions;
    assumptions.resize((init.size()));
    copy(init.begin(), init.end(), assumptions.begin());
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
            m_log->L(1, "Proof at frame ", i);
            m_log->L(1, m_overSequence->FramesInfo());
            result = true;
            m_overSequence->SetInvariantLevel(i - 1);
            break;
        }
    }
    m_invSolver = nullptr;
    return result;
}

int ForwardChecker::GetNewLevel(shared_ptr<State> state, int start) {
    m_log->Tick();

    for (int i = start; i < m_overSequence->GetLength(); ++i) {
        if (!m_overSequence->IsBlockedByFrame_lazy(*(state->latches), i)) {
            return i - 1;
        }
    }

    m_log->StatGetNewLevel();
    return m_overSequence->GetLength() - 1; // placeholder
}

bool ForwardChecker::IsInvariant(int frameLevel) {
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


void ForwardChecker::OutputWitness(int bad) {
    // get outputfile
    auto startIndex = m_settings.aigFilePath.find_last_of("/");
    if (startIndex == string::npos) {
        startIndex = 0;
    } else {
        startIndex++;
    }
    auto endIndex = m_settings.aigFilePath.find_last_of(".");
    assert(endIndex != string::npos);
    string aigName = m_settings.aigFilePath.substr(startIndex, endIndex - startIndex);
    string outPath = m_settings.outputDir + aigName + ".w.aag";
    aiger *model_aig = m_model->GetAig();

    unsigned lvl_i;
    if (m_overSequence == nullptr)
        lvl_i = 0;
    else
        lvl_i = m_overSequence->GetInvariantLevel();
    if (lvl_i == 0) {
        aiger_open_and_write_to_file(model_aig, outPath.c_str());
        return;
    }

    aiger *witness_aig = aiger_init();
    // copy inputs
    for (unsigned i = 0; i < model_aig->num_inputs; i++) {
        aiger_symbol &input = model_aig->inputs[i];
        aiger_add_input(witness_aig, input.lit, input.name);
    }
    // copy latches
    for (unsigned i = 0; i < model_aig->num_latches; i++) {
        aiger_symbol &latch = model_aig->latches[i];
        aiger_add_latch(witness_aig, latch.lit, latch.next, latch.name);
        aiger_add_reset(witness_aig, latch.lit, latch.reset);
    }
    // copy and gates
    for (unsigned i = 0; i < model_aig->num_ands; i++) {
        aiger_and &gate = model_aig->ands[i];
        aiger_add_and(witness_aig, gate.lhs, gate.rhs0, gate.rhs1);
    }
    // copy constraints
    for (unsigned i = 0; i < model_aig->num_constraints; i++) {
        aiger_symbol &cons = model_aig->constraints[i];
        aiger_add_constraint(witness_aig, cons.lit, cons.name);
    }

    assert(model_aig->maxvar == witness_aig->maxvar);

    // P' = P & invariant
    // P' = !bad & ( O_0 | O_1 | ... | O_i )
    //             !( !O_0 & !O_1 & ...  & !O_i )
    //                 O_i = c_1 & c_2 & ... & c_j
    //                       c_j = !( l_1 & l_2 & .. & l_k )

    vector<unsigned> inv_lits;
    for (unsigned i = 0; i <= lvl_i; i++) {
        vector<shared_ptr<cube>> frame_i;
        m_overSequence->GetFrame(i, frame_i);
        vector<unsigned> frame_i_lits;
        for (unsigned j = 0; j < frame_i.size(); j++) {
            vector<unsigned> cube_j;
            for (int l : *frame_i[j]) cube_j.push_back(l > 0 ? (2 * l) : (2 * -l + 1));
            unsigned c_j = addCubeToANDGates(witness_aig, cube_j) ^ 1;
            frame_i_lits.push_back(c_j);
        }
        unsigned O_i = addCubeToANDGates(witness_aig, frame_i_lits);
        inv_lits.push_back(O_i ^ 1);
    }
    unsigned inv = addCubeToANDGates(witness_aig, inv_lits) ^ 1;
    unsigned bad_lit = bad > 0 ? (2 * bad) : (2 * -bad) + 1;
    unsigned p = aiger_not(bad_lit);
    unsigned p_prime = addCubeToANDGates(witness_aig, {p, inv});

    if (model_aig->num_bad == 1) {
        aiger_add_bad(witness_aig, aiger_not(p_prime), model_aig->bad[0].name);
    } else if (model_aig->num_outputs == 1) {
        aiger_add_output(witness_aig, aiger_not(p_prime), model_aig->outputs[0].name);
    } else {
        assert(false);
    }

    aiger_reencode(witness_aig);
    aiger_open_and_write_to_file(witness_aig, outPath.c_str());
}


void ForwardChecker::OutputCounterExample(int bad) {
    // get outputfile
    auto startIndex = m_settings.aigFilePath.find_last_of("/");
    if (startIndex == string::npos) {
        startIndex = 0;
    } else {
        startIndex++;
    }
    auto endIndex = m_settings.aigFilePath.find_last_of(".");
    assert(endIndex != string::npos);
    string aigName = m_settings.aigFilePath.substr(startIndex, endIndex - startIndex);
    string cexPath = m_settings.outputDir + aigName + ".cex";
    std::ofstream cexFile;
    cexFile.open(cexPath);

    assert(m_lastState != nullptr);

    cexFile << "." << endl;
    cexFile << "1" << endl
            << "b0" << endl;

    shared_ptr<State> state = m_lastState;
    cexFile << state->GetValueOfLatches() << endl;
    cexFile << state->GetValueOfInputs() << endl;
    while (state->preState != nullptr) {
        state = state->preState;
        cexFile << state->GetValueOfInputs() << endl;
    }

    cexFile << "." << endl;
    cexFile.close();
}
} // namespace car