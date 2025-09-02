#include "BackwardChecker.h"
#include <stack>
#include <string>

namespace car {

BackwardChecker::BackwardChecker(Settings settings,
                                 shared_ptr<Model> model,
                                 shared_ptr<Log> log) : m_settings(settings),
                                                        m_model(model),
                                                        m_log(log) {
    State::numInputs = model->GetNumInputs();
    State::numLatches = model->GetNumLatches();
    m_lastState = nullptr;
    GLOBAL_LOG = m_log;
}

CheckResult BackwardChecker::Run() {
    signal(SIGINT, signalHandler);

    if (Check(m_model->GetBad()))
        m_checkResult = CheckResult::Safe;
    else
        m_checkResult = CheckResult::Unsafe;

    m_log->PrintStatistics();

    return m_checkResult;
}

void BackwardChecker::Witness() {
    if (m_checkResult == CheckResult::Safe) {
        OutputWitness(m_model->GetBad());
    } else if (m_checkResult == CheckResult::Unsafe) {
        OutputCounterExample(m_model->GetBad());
    }
}

bool BackwardChecker::Check(int badId) {
    if (m_model->GetFalseId() == badId)
        return true;

    Init();
    m_log->L(2, "Initialized");

    m_log->L(2, "Initial States Check");
    if (ImmediateSatisfiable(badId)) {
        m_log->L(2, "Result >>> SAT <<<");
        auto p = m_startSolver->GetAssignment(false);
        m_log->L(3, "Get Assignment:", CubeToStr(p.second));

        shared_ptr<State> newState(new State(nullptr, p.first, p.second, 1));
        m_lastState = newState;
        return false;
    }
    m_log->L(2, "Result >>> UNSAT <<<");
    shared_ptr<cube> bad_assumption(new cube({badId}));
    if (!m_transSolvers[0]->Solve(bad_assumption)) {
        m_overSequence->SetInvariantLevel(-1);
        return true;
    }
    m_startSolver->UpdateStartSolverFlag();

    // main stage
    m_k = 0;
    stack<Task> workingStack;
    while (true) {
        m_minUpdateLevel = m_k + 1;
        if (m_settings.dt) { // Dynamic Traversal
            shared_ptr<vector<shared_ptr<State>>> dtseq = m_underSequence.GetSeqDT();
            for (auto state : *dtseq) {
                workingStack.emplace(state, m_k - 1, false);
            }
        } else { // from the shallow and the start
            for (int i = m_underSequence.size() - 1; i >= 0; i--) {
                for (int j = m_underSequence[i].size() - 1; j >= 0; j--) {
                    workingStack.emplace(m_underSequence[i][j], m_k - 1, false);
                }
            }
        }

        m_log->Tick();
        shared_ptr<State> startState = EnumerateStartState();
        m_log->StatStartSolver();

        while (startState != nullptr) {
            m_log->L(2, "State from StartSolver: ", CubeToStrShort(startState->latches));
            m_log->L(3, "State Detail: ", CubeToStr(startState->latches));
            workingStack.push(Task(startState, m_k - 1, true));

            while (!workingStack.empty()) {
                Task &task = workingStack.top();

                if (m_settings.restart && m_restart->RestartCheck()) {
                    m_log->L(1, "Restarting...");
                    m_underSequence = UnderSequence();
                    while (!workingStack.empty()) workingStack.pop();
                    m_restart->UpdateThreshold();
                    m_restart->ResetUcCounts();
                    continue;
                }

                if (!task.isLocated) {
                    task.frameLevel = GetNewLevel(task.state, task.frameLevel + 1);
                    m_log->L(3, "State get new level ", task.frameLevel);
                    if (task.frameLevel >= m_k) {
                        workingStack.pop();
                        continue;
                    }
                }
                task.isLocated = false;

                if (task.frameLevel == -1) {
                    if (CheckBad(task.state))
                        return false;
                    else
                        continue;
                }

                shared_ptr<cube> assumption(new cube(*task.state->latches));
                OrderAssumption(assumption);
                m_log->L(2, "\nSAT Check on frame: ", task.frameLevel);
                m_log->L(3, "From state: ", CubeToStr(task.state->latches));
                m_log->Tick();
                bool result = IsReachable(task.frameLevel, assumption);
                m_log->StatMainSolver();
                if (result) {
                    // Solver return SAT, get a new State, then continue
                    m_log->L(2, "Result >>> SAT <<<");
                    auto p = GetInputAndState(task.frameLevel);
                    shared_ptr<State> newState(new State(task.state, p.first, p.second, task.state->depth + 1));
                    m_log->L(3, "Get state: ", CubeToStr(newState->latches));
                    m_underSequence.push(newState);
                    if (m_settings.dt) task.state->HasSucc();
                    int newFrameLevel = GetNewLevel(newState);
                    m_log->L(3, "State get new level ", newFrameLevel);
                    workingStack.emplace(newState, newFrameLevel, true);
                    continue;
                } else {
                    // Solver return UNSAT, get uc, then continue
                    m_log->L(2, "Result >>> UNSAT <<<");
                    auto uc = GetUnsatCore(task.frameLevel);
                    m_log->L(3, "Get UC:", CubeToStr(uc));
                    if (Generalize(uc, task.frameLevel))
                        m_branching->Update(uc);
                    m_log->L(2, "Get Generalized UC:", CubeToStr(uc));
                    AddUnsatisfiableCore(uc, task.frameLevel + 1);
                    if (m_settings.dt) task.state->HasUC();
                    PropagateUp(uc, task.frameLevel + 1);
                    m_log->L(3, m_overSequence->FramesInfo());
                    task.frameLevel++;
                    continue;
                }
            }
            m_log->Tick();
            startState = EnumerateStartState();
            m_log->StatStartSolver();
        }

        if (m_invSolver == nullptr) {
            m_invSolver.reset(new SATSolver(m_model, m_settings.solver));
        }
        IsInvariant(0);
        for (int i = 0; i < m_k; ++i) {
            // propagation
            if (i >= m_minUpdateLevel) {
                shared_ptr<frame> fi = m_overSequence->GetFrame(i);
                shared_ptr<frame> fi_plus_1 = m_overSequence->GetFrame(i + 1);
                frame::iterator iter;
                for (shared_ptr<cube> uc : *fi) {
                    iter = fi_plus_1->find(uc);
                    if (iter != fi_plus_1->end()) continue; // propagated
                    if (Propagate(uc, i)) m_branching->Update(uc);
                }
            }

            // invariant check
            if (IsInvariant(i + 1)) {
                m_log->L(1, "Proof at frame ", i + 1);
                m_log->L(1, m_overSequence->FramesInfo());
                m_overSequence->SetInvariantLevel(i);
                OverSequenceRefine(i);
                return true;
            }
        }
        m_invSolver = nullptr;

        m_log->L(1, m_overSequence->FramesInfo());
        m_log->L(3, m_overSequence->FramesDetail());
        m_startSolver->UpdateStartSolverFlag();

        m_k++;
        m_restart->ResetUcCounts();
        m_log->L(2, "\nNew Frame Added");
    }
}


void BackwardChecker::Init() {
    m_overSequence = make_shared<OverSequenceSet>(m_model);
    m_underSequence = UnderSequence();
    m_branching = make_shared<Branching>(m_settings.branching);
    litOrder.branching = m_branching;
    blockerOrder.branching = m_branching;
    innOrder.m = m_model;

    // s & T & c & O_i'
    m_transSolvers.emplace_back(make_shared<SATSolver>(m_model, m_settings.solver));
    m_transSolvers[0]->AddTrans();
    m_transSolvers[0]->AddConstraints();
    // I & T & c
    m_startSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    m_startSolver->AddTrans();
    m_startSolver->AddConstraints();
    for (auto l : m_model->GetInitialState()) {
        m_startSolver->AddClause({l});
    }
    m_startSolver->AddInitialClauses();
    m_invSolver = make_shared<SATSolver>(m_model, MCSATSolver::cadical);
    m_restart.reset(new Restart(m_settings));
}

bool BackwardChecker::AddUnsatisfiableCore(shared_ptr<vector<int>> uc, int frameLevel) {
    m_restart->UcCountsPlus1();
    m_log->Tick();

    shared_ptr<cube> puc(new cube(*uc));
    GetPrimed(puc);
    if (m_settings.multipleSolvers) {
        if (frameLevel >= m_transSolvers.size()) {
            m_transSolvers.emplace_back(make_shared<SATSolver>(m_model, m_settings.solver));
            m_transSolvers.back()->AddTrans();
            m_transSolvers.back()->AddConstraints();
            if (m_settings.solveInProperty) m_transSolvers.back()->AddProperty();
        }
        m_transSolvers[frameLevel]->AddUC(puc);
    } else
        m_transSolvers[0]->AddUC(puc, frameLevel);

    if (frameLevel >= m_k) {
        m_startSolver->AddUC(puc, frameLevel);
    }
    if (frameLevel < m_minUpdateLevel) {
        m_minUpdateLevel = frameLevel;
    }
    m_overSequence->Insert(uc, frameLevel);

    m_log->StatUpdateUc();
    return true;
}

bool BackwardChecker::ImmediateSatisfiable(int badId) {
    shared_ptr<cube> assumptions = make_shared<cube>();
    assumptions->push_back(badId);
    m_log->Tick();
    bool result = m_startSolver->Solve(assumptions);
    m_log->StatMainSolver();
    m_startSolver->ClearAssumption();
    return result;
}


shared_ptr<State> BackwardChecker::EnumerateStartState() {
    if (!m_startSolver->Solve()) return nullptr;
    auto p = m_startSolver->GetAssignment(true);
    shared_ptr<State> startState(new State(nullptr, p.first, p.second, 0));
    return startState;
}


void BackwardChecker::OverSequenceRefine(int lvl) {
    // sometimes we have I & T & c & (O_0 | O_1 | ... | O_i+1)' is UNSAT,
    // but to output a correct witness,
    // we need I & c & (O_0 | O_1 | ... | O_i) is UNSAT.
    //         I & c & !P   is UNSAT.
    // get model s of I & c & (O_0 | O_1 | ... | O_i),
    // assert( s & T & c & (O_0 | O_1 | ... | O_i+1)' is UNSAT)
    // add uc to O_0 ... O_i

    // solver: I & c & (O_0 | O_1 | ... | O_i)
    shared_ptr<SATSolver> refine_solver = make_shared<SATSolver>(m_model, m_settings.solver);
    refine_solver->AddConstraints();
    for (auto l : m_model->GetInitialState()) refine_solver->AddClause({l});
    refine_solver->AddInitialClauses();
    clause inv;
    for (int i = 0; i <= lvl; i++) {
        shared_ptr<frame> frame_i = m_overSequence->GetFrame(i);
        int f = refine_solver->GetNewVar();
        for (shared_ptr<cube> uc : *frame_i) {
            clause cls;
            for (int i = 0; i < uc->size(); i++) {
                cls.emplace_back(-uc->at(i));
            }
            cls.emplace_back(-f);
            refine_solver->AddClause(cls);
        }
        inv.emplace_back(f);
    }
    refine_solver->AddClause(inv);

    // if I & c & (O_0 | O_1 | ... | O_i) is SAT, get model s
    while (refine_solver->Solve()) {
        m_log->L(1, "Refine OverSequence");
        auto s = refine_solver->GetAssignment(false);

        bool sat = IsReachable(lvl + 1, s.second);
        assert(!sat);
        auto uc = GetUnsatCore(lvl + 1);
        for (int i = 0; i <= lvl; i++)
            AddUnsatisfiableCore(uc, i);
        clause cls;
        for (auto ci : *uc) cls.emplace_back(-ci);
        refine_solver->AddClause(cls);
    }
}


int BackwardChecker::GetNewLevel(shared_ptr<State> state, int start) {
    m_log->Tick();

    for (int i = start; i <= m_k; ++i) {
        if (!m_overSequence->IsBlockedByFrame_lazy(state->latches, i)) {
            m_log->StatGetNewLevel();
            return i - 1;
        }
    }

    m_log->StatGetNewLevel();
    return m_k;
}


bool BackwardChecker::IsInvariant(int frameLevel) {
    m_log->Tick();

    shared_ptr<frame> frame_i = m_overSequence->GetFrame(frameLevel);

    if (frameLevel < m_minUpdateLevel) {
        AddConstraintOr(frame_i);
        return false;
    }

    AddConstraintAnd(frame_i);
    bool result = !m_invSolver->Solve();
    m_invSolver->FlipLastConstrain();
    AddConstraintOr(frame_i);

    m_log->StatInvSolver();
    return result;
}


// ================================================================================
// @brief: add constraint | O_i
// @input:
// @output:
// ================================================================================
void BackwardChecker::AddConstraintOr(const shared_ptr<frame> f) {
    cube cls;
    for (shared_ptr<cube> frame_cube : *f) {
        int flag = m_invSolver->GetNewVar();
        cls.push_back(flag);
        for (int i = 0; i < frame_cube->size(); i++) {
            m_invSolver->AddClause(cube{-flag, frame_cube->at(i)});
        }
    }
    m_invSolver->AddClause(cls);
}


// ================================================================================
// @brief: add constraint & !O_i
// @input:
// @output:
// ================================================================================
void BackwardChecker::AddConstraintAnd(const shared_ptr<frame> f) {
    int flag = m_invSolver->GetNewVar();
    for (shared_ptr<cube> frame_cube : *f) {
        cube cls;
        for (int i = 0; i < frame_cube->size(); i++) {
            cls.push_back(-frame_cube->at(i));
        }
        cls.push_back(-flag);
        m_invSolver->AddClause(cls);
    }
    m_invSolver->AddAssumption(make_shared<cube>(cube{flag}));
}


// ================================================================================
// @brief: counter-example to generalization
// @input:
// @output:
// ================================================================================
bool BackwardChecker::Generalize(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl) {
    unordered_set<int> required_lits;

    vector<shared_ptr<cube>> uc_blockers;
    m_overSequence->GetBlockers(uc, frame_lvl, uc_blockers);
    shared_ptr<cube> uc_blocker;
    if (uc_blockers.size() > 0) {
        if (m_settings.branching > 0)
            stable_sort(uc_blockers.begin(), uc_blockers.end(), blockerOrder);
        uc_blocker = uc_blockers[0];
    } else {
        uc_blocker = make_shared<cube>();
    }

    if (m_settings.referSkipping)
        for (auto b : *uc_blocker) required_lits.emplace(b);
    shared_ptr<vector<cube>> failed_ctses = make_shared<vector<cube>>();
    OrderAssumption(uc);
    for (int i = uc->size() - 1; i > 0; i--) {
        if (required_lits.find(uc->at(i)) != required_lits.end()) continue;
        shared_ptr<cube> temp_uc(new cube());
        temp_uc->reserve(uc->size());
        for (auto ll : *uc)
            if (ll != uc->at(i)) temp_uc->emplace_back(ll);
        if (Down(temp_uc, frame_lvl, rec_lvl, failed_ctses)) {
            uc->swap(*temp_uc);
            OrderAssumption(uc);
            i = uc->size();
        } else {
            required_lits.emplace(uc->at(i));
        }
    }
    sort(uc->begin(), uc->end(), cmp);
    if (uc->size() > uc_blocker->size() && frame_lvl != 0) {
        return false;
    } else
        return true;
}


bool BackwardChecker::Down(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl, shared_ptr<vector<cube>> failed_ctses) {
    int ctgs = 0;
    shared_ptr<cube> assumption(new cube(*uc));
    shared_ptr<State> p_ucs(new State(nullptr, nullptr, uc, 0));
    while (true) {
        m_log->Tick();
        // F_i & T & temp_uc'
        if (!IsReachable(frame_lvl, assumption)) {
            m_log->StatMainSolver();
            auto uc_ctg = GetUnsatCore(frame_lvl);
            if (uc->size() < uc_ctg->size()) return false; // there are cases that uc_ctg longer than uc
            uc->swap(*uc_ctg);
            return true;
        } else if (rec_lvl > m_settings.ctgMaxRecursionDepth) {
            m_log->StatMainSolver();
            return false;
        } else {
            m_log->StatMainSolver();
            auto p = GetInputAndState(frame_lvl);
            shared_ptr<State> cts(new State(nullptr, p.first, p.second, 0));
            if (DownHasFailed(cts->latches, failed_ctses)) return false;
            int cts_lvl = GetNewLevel(cts);
            shared_ptr<cube> cts_ass(new cube(*cts->latches));
            OrderAssumption(cts_ass);
            m_log->Tick();
            // F_i-1 & T & cts'
            if (ctgs < m_settings.ctgMaxStates && cts_lvl >= 0 && !IsReachable(cts_lvl, cts_ass)) {
                m_log->StatMainSolver();
                ctgs++;
                auto uc_cts = GetUnsatCore(cts_lvl);
                m_log->L(3, "CTG Get UC:", CubeToStr(uc_cts));
                if (Generalize(uc_cts, cts_lvl, rec_lvl + 1)) m_branching->Update(uc_cts);
                m_log->L(3, "CTG Get Generalized UC:", CubeToStr(uc_cts));
                AddUnsatisfiableCore(uc_cts, cts_lvl + 1);
                PropagateUp(uc_cts, cts_lvl + 1);
            } else {
                m_log->StatMainSolver();
                failed_ctses->emplace_back(*cts->latches);
                return false;
            }
        }
    }
}


bool BackwardChecker::DownHasFailed(const shared_ptr<cube> s, const shared_ptr<vector<cube>> failed_ctses) {
    for (auto f : *failed_ctses) {
        // if f->s , return true
        if (f.size() > s->size()) continue;
        if (includes(s->begin(), s->end(), f.begin(), f.end(), cmp)) return true;
    }
    return false;
}


bool BackwardChecker::CheckBad(shared_ptr<State> s) {
    m_log->L(2, "\nSAT CHECK on bad");
    m_log->L(3, "From state: ", CubeToStr(s->latches));
    shared_ptr<cube> assumption(new cube(*s->latches));
    OrderAssumption(assumption);
    assumption->push_back(m_model->GetBad());
    m_log->Tick();
    bool result = m_transSolvers[0]->Solve(assumption);
    m_log->StatMainSolver();
    if (result) {
        m_log->L(2, "Result >>> SAT <<<");
        auto p = m_transSolvers[0]->GetAssignment(false);
        m_log->L(3, "Get Assignment:", CubeToStr(p.second));
        shared_ptr<State> newState(new State(s, p.first, p.second, s->depth + 1));
        m_lastState = newState;
        m_log->L(3, m_overSequence->FramesInfo());
        return true;
    } else {
        m_log->L(2, "Result >>> UNSAT <<<");
        auto uc = m_transSolvers[0]->GetUC(false);
        // Generalization
        unordered_set<int> required_lits;
        for (int i = uc->size() - 1; i >= 0; i--) {
            if (uc->size() < 3) break;
            if (required_lits.find(uc->at(i)) != required_lits.end()) continue;
            shared_ptr<cube> temp_uc(new cube());
            temp_uc->reserve(uc->size());
            for (auto ll : *uc)
                if (ll != uc->at(i)) temp_uc->emplace_back(ll);
            assumption->clear();
            copy(temp_uc->begin(), temp_uc->end(), back_inserter(*assumption));
            OrderAssumption(assumption);
            assumption->push_back(m_model->GetBad());
            m_log->Tick();
            bool result = m_transSolvers[0]->Solve(assumption);
            m_log->StatMainSolver();
            if (!result) {
                auto new_uc = m_transSolvers[0]->GetUC(false);
                uc->swap(*new_uc);
                OrderAssumption(uc);
                i = uc->size();
            } else {
                required_lits.emplace(uc->at(i));
            }
        }
        sort(uc->begin(), uc->end(), cmp);
        m_log->L(2, "Get UC:", CubeToStr(uc));
        m_branching->Update(uc);
        AddUnsatisfiableCore(uc, 0);
        PropagateUp(uc, 0);
        return false;
    }
}


bool BackwardChecker::IsReachable(int lvl, const shared_ptr<cube> assumption) {
    if (m_settings.multipleSolvers)
        return m_transSolvers[lvl]->SolveFrame(assumption, 0);
    else
        return m_transSolvers[0]->SolveFrame(assumption, lvl);
}


pair<shared_ptr<cube>, shared_ptr<cube>> BackwardChecker::GetInputAndState(int lvl) {
    if (m_settings.multipleSolvers)
        return m_transSolvers[lvl]->GetAssignment(true);
    else
        return m_transSolvers[0]->GetAssignment(true);
}


shared_ptr<cube> BackwardChecker::GetUnsatCore(int lvl) {
    if (m_settings.multipleSolvers)
        return m_transSolvers[lvl]->GetUC(false);
    else
        return m_transSolvers[0]->GetUC(false);
}


// ================================================================================
// @brief: add the cube as and gates to the aiger model
// @input:
// @output:
// ================================================================================
unsigned BackwardChecker::addCubeToANDGates(aiger *circuit, vector<unsigned> cube) {
    assert(cube.size() > 0);
    unsigned res = cube[0];
    assert(res / 2 <= circuit->maxvar);
    for (unsigned i = 1; i < cube.size(); i++) {
        assert(cube[i] / 2 <= circuit->maxvar);
        unsigned new_gate = (circuit->maxvar + 1) * 2;
        aiger_add_and(circuit, new_gate, res, cube[i]);
        res = new_gate;
    }
    return res;
}


bool BackwardChecker::Propagate(shared_ptr<cube> c, int lvl) {
    m_log->Tick();

    bool result;
    if (!IsReachable(lvl, c)) {
        m_log->StatPropagation();
        AddUnsatisfiableCore(c, lvl + 1);
        result = true;
    } else {
        m_log->StatPropagation();
        result = false;
    }
    return result;
}


int BackwardChecker::PropagateUp(shared_ptr<cube> c, int lvl) {
    while (lvl < m_k) {
        if (Propagate(c, lvl))
            m_branching->Update(c);
        else
            break;
        lvl++;
    }
    return lvl + 1;
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
    string outPath = m_settings.witnessOutputDir + aigName + ".w.aig";
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
        shared_ptr<frame> frame_i = m_overSequence->GetFrame(i);
        vector<unsigned> frame_i_lits;
        for (shared_ptr<cube> frame_cube : *frame_i) {
            vector<unsigned> cube_j;
            for (int l : *frame_cube) cube_j.push_back(l > 0 ? (2 * l) : (2 * -l + 1));
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
    auto startIndex = m_settings.aigFilePath.find_last_of("/\\");
    if (startIndex == string::npos) {
        startIndex = 0;
    } else {
        startIndex++;
    }
    auto endIndex = m_settings.aigFilePath.find_last_of(".");
    assert(endIndex != string::npos);
    string aigName = m_settings.aigFilePath.substr(startIndex, endIndex - startIndex);
    string cexPath = m_settings.witnessOutputDir + aigName + ".cex";
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

    // get determined initial state
    if (trace.size() > 1) {
        shared_ptr<cube> assumption(new cube());
        shared_ptr<cube> succ(new cube(*trace.top()->latches));
        shared_ptr<cube> input(new cube(*trace.top()->inputs));
        GetPrimed(succ);
        assumption->insert(assumption->end(), succ->begin(), succ->end());
        assumption->insert(assumption->end(), input->begin(), input->end());
        m_startSolver->UpdateStartSolverFlag();
        bool sat = m_startSolver->Solve(assumption);
        assert(sat);
        auto p = m_startSolver->GetAssignment(false);
        shared_ptr<State> initState(new State(nullptr, p.first, p.second, 0));
        cexFile << initState->GetLatchesString() << endl;
    } else {
        cexFile << trace.top()->GetLatchesString() << endl;
    }

    while (!trace.empty()) {
        cexFile << trace.top()->GetInputsString() << endl;
        trace.pop();
    }
    cexFile << "." << endl;
    cexFile.close();
}
} // namespace car