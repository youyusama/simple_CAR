#include "BackwardChecker.h"
#include <stack>
#include <string>

namespace car {

BackwardChecker::BackwardChecker(Settings settings,
                                 shared_ptr<AigerModel> model,
                                 shared_ptr<Log> log) : m_settings(settings),
                                                        m_model(model),
                                                        m_log(log) {
    State::numInputs = model->GetNumInputs();
    State::numLatches = model->GetNumLatches();
    m_lastState = nullptr;
    GLOBAL_LOG = m_log;
}

bool BackwardChecker::Run() {
    signal(SIGINT, signalHandler);

    bool result = Check(m_model->GetBad());

    m_log->PrintStatistics();
    if (result) {
        m_log->L(0, "Safe");
        if (m_settings.witness)
            OutputWitness(m_model->GetBad());
    } else {
        m_log->L(0, "Unsafe");
        if (m_settings.witness)
            OutputCounterExample(m_model->GetBad());
    }

    return true;
}

bool BackwardChecker::Check(int badId) {
    if (m_model->GetTrueId() == badId)
        return false;
    else if (m_model->GetFalseId() == badId)
        return true;

    Init();

    if (ImmediateSatisfiable(badId)) {
        m_log->L(3, "Result >>> SAT <<<");
        pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> pair;
        pair = m_mainSolver->GetAssignment();
        m_log->L(3, "Get Assignment:", CubeToStr(*pair.second));

        shared_ptr<State> newState(new State(m_initialState, pair.first, pair.second, 1));
        m_lastState = newState;
        return false;
    }
    m_log->L(3, "Result >>> UNSAT <<<");
    m_log->Tick();
    auto uc = m_mainSolver->GetUnsatisfiableCoreFromBad(badId);
    if (uc->size() == 0) {
        m_overSequence->SetInvariantLevel(-1);
        return true;
    }
    m_log->StatMainSolver();
    m_log->L(3, "Get UC:", CubeToStr(*uc));
    AddUnsatisfiableCore(uc, 0);
    m_overSequence->effectiveLevel = 0;
    m_log->L(3, m_overSequence->FramesInfo());

    // main stage
    int frameStep = 0;
    stack<Task> workingStack;
    while (true) {
        m_log->L(1, m_overSequence->FramesInfo());
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
                task.frameLevel = GetNewLevel(task.state, task.frameLevel + 1);
                m_log->L(3, "state get new level ", task.frameLevel);
                if (task.frameLevel > m_overSequence->effectiveLevel) {
                    workingStack.pop();
                    continue;
                }
            }
            task.isLocated = false;

            if (task.frameLevel == -1) {
                vector<int> assumption;
                GetAssumption(task.state, task.frameLevel, assumption);
                m_log->L(3, "\nSAT CHECK on frame: ", task.frameLevel);
                m_log->L(3, "From state: ", CubeToStr(*task.state->latches));
                m_log->Tick();
                bool result = m_mainSolver->SolveWithAssumptionAndBad(assumption, badId);
                m_log->StatMainSolver();
                if (result) {
                    m_log->L(3, "Result >>> SAT <<<");
                    pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> pair;
                    pair = m_mainSolver->GetAssignment();
                    m_log->L(3, "Get Assignment:", CubeToStr(*pair.second));
                    shared_ptr<State> newState(new State(task.state, pair.first, pair.second, task.state->depth + 1));
                    m_lastState = newState;
                    m_log->L(3, m_overSequence->FramesInfo());
                    return false;
                } else {
                    m_log->L(3, "Result >>> UNSAT <<<");
                    auto uc = m_mainSolver->GetUnsatisfiableCore();
                    m_log->L(3, "Get UC:", CubeToStr(*uc));
                    updateLitOrder(*uc);
                    AddUnsatisfiableCore(uc, task.frameLevel + 1);
                    task.frameLevel++;
                    continue;
                }
            }

            vector<int> assumption;
            GetAssumption(task.state, task.frameLevel, assumption);
            m_log->L(3, "\nSAT CHECK on frame: ", task.frameLevel);
            m_log->L(3, "From state: ", CubeToStr(*task.state->latches));
            m_log->Tick();
            bool result = m_mainSolver->SolveWithAssumption(assumption, task.frameLevel);
            m_log->StatMainSolver();
            if (result) {
                // Solver return SAT, get a new State, then continue
                m_log->L(3, "Result >>> SAT <<<");
                pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> pair;
                pair = m_mainSolver->GetAssignment();
                shared_ptr<State> newState(new State(task.state, pair.first, pair.second, task.state->depth + 1));
                m_log->L(3, "Get state: ", CubeToStr(*newState->latches));
                m_underSequence.push(newState);
                int newFrameLevel = GetNewLevel(newState);
                m_log->L(3, "state get new level ", newFrameLevel);
                workingStack.emplace(newState, newFrameLevel, true);
                continue;
            } else {
                // Solver return UNSAT, get uc, then continue
                m_log->L(3, "Result >>> UNSAT <<<");
                auto uc = m_mainSolver->Getuc(false);
                m_log->L(3, "Get UC:", CubeToStr(*uc));
                if (generalize_ctg(uc, task.frameLevel))
                    updateLitOrder(*uc);
                m_log->L(3, "Get UC:", CubeToStr(*uc));
                if (AddUnsatisfiableCore(uc, task.frameLevel + 1)) {
                    m_log->Tick();
                    m_overSequence->propagate_uc_from_lvl(uc, task.frameLevel + 1, m_branching);
                    m_log->StatPropagation();
                }
                m_log->L(3, m_overSequence->FramesInfo());
                task.frameLevel++;
                continue;
            }
        }
        m_log->L(3, "\nNew Frame Added");
        m_log->Tick();
        Propagation();
        m_log->StatPropagation();

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
    const cube &init = m_model->GetInitialState();
    shared_ptr<cube> inputs(new cube(State::numInputs, 0));
    shared_ptr<cube> latches(new cube());
    latches->reserve(init.size());
    latches->insert(latches->end(), init.begin(), init.end());
    m_initialState.reset(new State(nullptr, inputs, latches, 0));

    m_overSequence.reset(new OverSequenceSet(m_model));
    m_branching.reset(new Branching(m_settings.Branching));
    litOrder.branching = m_branching;
    blockerOrder.branching = m_branching;
    m_underSequence = UnderSequence();
    m_underSequence.push(m_initialState);
    m_mainSolver.reset(new MainSolver(m_model, false, true));
    m_invSolver.reset(new InvSolver(m_model));
    m_overSequence->set_solver(m_mainSolver);
}

bool BackwardChecker::AddUnsatisfiableCore(shared_ptr<vector<int>> uc, int frameLevel) {
    m_log->Tick();

    m_mainSolver->AddUnsatisfiableCore(*uc, frameLevel);
    if (frameLevel < m_minUpdateLevel) {
        m_minUpdateLevel = frameLevel;
    }
    m_overSequence->Insert(uc, frameLevel);

    m_log->StatUpdateUc();
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

int BackwardChecker::GetNewLevel(shared_ptr<State> state, int start) {
    m_log->Tick();

    for (int i = start; i < m_overSequence->GetLength(); ++i) {
        if (!m_overSequence->IsBlockedByFrame_lazy(*(state->latches), i)) {
            return i - 1;
        }
    }

    m_log->StatGetNewLevel();
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


void BackwardChecker::OutputWitness(int bad) {
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
    if (m_overSequence == nullptr || m_overSequence->GetInvariantLevel() < 0) {
        aiger_open_and_write_to_file(model_aig, outPath.c_str());
        return;
    }
    lvl_i = m_overSequence->GetInvariantLevel();

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
    // P' = !bad & !( O_0 | O_1 | ... | O_i )
    //             ( !O_0 & !O_1 & ...  & !O_i )
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
    unsigned inv = addCubeToANDGates(witness_aig, inv_lits);
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


void BackwardChecker::OutputCounterExample(int bad) {
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

    cexFile << "1" << endl
            << "b0" << endl;

    stack<shared_ptr<State>> trace;
    shared_ptr<State> state = m_lastState;
    while (state != nullptr) {
        trace.push(state);
        state = state->preState;
    }
    cexFile << trace.top()->GetValueOfLatches() << endl;
    trace.pop();
    while (!trace.empty()) {
        cexFile << trace.top()->GetValueOfInputs() << endl;
        trace.pop();
    }
    cexFile << "." << endl;
    cexFile.close();
}
} // namespace car