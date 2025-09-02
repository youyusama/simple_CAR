#include "ForwardChecker.h"
#include <stack>
#include <string>
namespace car {

ForwardChecker::ForwardChecker(Settings settings,
                               shared_ptr<Model> model,
                               shared_ptr<Log> log) : m_settings(settings),
                                                      m_model(model),
                                                      m_log(log) {
    State::numInputs = model->GetNumInputs();
    State::numLatches = model->GetNumLatches();
    m_lastState = nullptr;
    GLOBAL_LOG = m_log;
    m_checkResult = CheckResult::Unknown;

    m_settings.satSolveInDomain = m_settings.satSolveInDomain && m_settings.solver == MCSATSolver::minicore;
}

CheckResult ForwardChecker::Run() {
    signal(SIGINT, signalHandler);

    if (Check(m_model->GetBad()))
        m_checkResult = CheckResult::Safe;
    else
        m_checkResult = CheckResult::Unsafe;

    m_log->PrintStatistics();

    return m_checkResult;
}

void ForwardChecker::Witness() {
    if (m_checkResult == CheckResult::Safe) {
        OutputWitness(m_model->GetBad());
    } else if (m_checkResult == CheckResult::Unsafe) {
        OutputCounterExample(m_model->GetBad());
    }
}

bool ForwardChecker::Check(int badId) {
    if (m_model->GetFalseId() == badId)
        return true;

    Init(badId);
    m_log->L(2, "Initialized");

    m_log->L(2, "Initial States Check");
    if (ImmediateSatisfiable(badId)) {
        m_log->L(2, "Result >>> SAT <<<");
        auto p = m_transSolvers[0]->GetAssignment(false);
        m_log->L(3, "Get Assignment:", CubeToStr(p.second));
        m_initialState->inputs = p.first;
        m_initialState->latches = p.second;
        m_lastState = m_initialState;
        return false;
    }
    m_log->L(2, "Result >>> UNSAT <<<");

    // initialize frame 0
    m_startSolver->UpdateStartSolverFlag();
    for (int l : *m_initialState->latches) {
        shared_ptr<cube> neg_init_l(new cube{-l});
        m_overSequence->Insert(neg_init_l, 0, false);
        m_transSolvers[0]->AddUC(neg_init_l);
        m_startSolver->AddUC(neg_init_l, 0);
    }
    if (m_settings.solveInProperty) m_transSolvers[0]->AddProperty();

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
        // T & c & P & T' & c' & bad' is unsat
        if (m_k > 0 && startState == nullptr && m_overSequence->IsEmpty(m_k)) {
            m_overSequence->SetInvariantLevel(-1);
            return true;
        }

        while (startState != nullptr) {
            m_log->L(2, "State from StartSolver: ", CubeToStrShort(startState->latches));
            m_log->L(3, "State Detail: ", CubeToStr(startState->latches));
            workingStack.push(Task(startState, m_k - 1, true));

            while (!workingStack.empty()) {
                Task &task = workingStack.top();

                if (m_settings.restart && m_restart->RestartCheck()) {
                    m_log->L(1, "Restarting...");
                    m_underSequence = UnderSequence();
                    while (workingStack.size() > 1) workingStack.pop();
                    m_restart->UpdateThreshold();
                    m_restart->ResetUcCounts();
                    continue;
                }

                if (!task.isLocated) {
                    task.frameLevel = GetNewLevel(task.state, task.frameLevel + 1);
                    m_log->L(3, "State get new Level ", task.frameLevel);
                    if (task.frameLevel >= m_k) {
                        workingStack.pop();
                        continue;
                    }
                }
                task.isLocated = false;

                if (task.frameLevel == -1) {
                    if (CheckInit(task.state)) {
                        m_initialState->preState = task.state->preState;
                        m_initialState->inputs = task.state->inputs;
                        m_initialState->latches = task.state->latches;
                        m_lastState = m_initialState;
                        return false;
                    } else
                        continue;
                }
                m_log->L(2, "SAT Check on Frame: ", task.frameLevel);
                m_log->L(2, "From State: ", CubeToStrShort(task.state->latches));
                m_log->L(3, "State Detail: ", CubeToStr(task.state->latches));
                shared_ptr<cube> assumption(new cube(*task.state->latches));
                OrderAssumption(assumption);
                GetPrimed(assumption);
                if (m_settings.satSolveInDomain) GetAndPushDomain(assumption);
                m_log->Tick();
                bool result = IsReachable(task.frameLevel, assumption);
                m_log->StatMainSolver();
                if (result) {
                    // Solver return SAT, get a new State, then continue
                    m_log->L(2, "Result >>> SAT <<<");
                    auto p = GetInputAndState(task.frameLevel);
                    m_log->L(3, "Input Detail: ", CubeToStr(p.first));
                    m_log->L(3, "State Detail: ", CubeToStr(p.second));
                    GeneralizePredecessor(p, task.state);
                    shared_ptr<State> newState(new State(task.state, p.first, p.second, task.state->depth + 1));
                    m_underSequence.push(newState);
                    if (m_settings.dt) task.state->HasSucc();
                    m_log->L(3, "Get State: ", CubeToStrShort(newState->latches));
                    m_log->L(3, "State Detail: ", CubeToStr(newState->latches));
                    int newFrameLevel = GetNewLevel(newState);
                    workingStack.emplace(newState, newFrameLevel, true);
                    if (m_settings.satSolveInDomain) PopDomain();
                    continue;
                } else {
                    // Solver return UNSAT, get uc, then continue
                    m_log->L(2, "Result >>> UNSAT <<<");
                    auto uc = GetUnsatCore(task.frameLevel, task.state->latches);
                    assert(uc->size() > 0);
                    m_log->L(3, "Get UC: ", CubeToStr(uc));
                    if (Generalize(uc, task.frameLevel))
                        m_branching->Update(uc);
                    m_log->L(2, "Get Generalized UC: ", CubeToStr(uc));
                    AddUnsatisfiableCore(uc, task.frameLevel + 1);
                    if (m_settings.dt) task.state->HasUC();
                    PropagateUp(uc, task.frameLevel + 1);
                    m_log->L(3, "Frames: ", m_overSequence->FramesInfo());
                    task.frameLevel++;
                    if (m_settings.satSolveInDomain) PopDomain();
                    continue;
                }
            } // end while (!workingStack.empty())
            m_log->Tick();
            startState = EnumerateStartState();
            m_log->StatStartSolver();
        }

        if (m_invSolver == nullptr) {
            m_invSolver.reset(new SATSolver(m_model, MCSATSolver::minisat));
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
                m_log->L(1, "Proof at Frame ", i + 1);
                m_log->L(1, m_overSequence->FramesInfo());
                m_overSequence->SetInvariantLevel(i);
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


void ForwardChecker::Init(int badId) {
    shared_ptr<cube> inputs(new cube(State::numInputs, 0));
    shared_ptr<cube> latches(new cube(m_model->GetInitialState()));
    m_initialState.reset(new State(nullptr, inputs, latches, 0));

    m_badId = badId;
    m_overSequence = make_shared<OverSequenceSet>(m_model);
    m_underSequence = UnderSequence();
    m_branching = make_shared<Branching>(m_settings.branching);
    litOrder.branching = m_branching;
    blockerOrder.branching = m_branching;
    innOrder.m = m_model;
    m_domainStack = make_shared<vector<shared_ptr<cube>>>();

    // O_i & T & c & s'
    m_transSolvers.emplace_back(make_shared<SATSolver>(m_model, m_settings.solver));
    if (m_settings.satSolveInDomain) m_transSolvers[0]->SetSolveInDomain();
    m_transSolvers[0]->AddTrans();
    m_transSolvers[0]->AddConstraints();
    m_transSolvers[0]->AddInitialClauses();
    AddSamePrimeConstraints(m_transSolvers[0]);
    // lift
    m_liftSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    if (m_settings.satSolveInDomain) m_liftSolver->SetSolveInDomain();
    m_liftSolver->AddTrans();
    if (m_settings.satSolveInDomain)
        m_liftSolver->SetDomainCOI(make_shared<cube>(m_model->GetConstraints()));
    // inv
    m_invSolver = make_shared<SATSolver>(m_model, MCSATSolver::cadical);
    // s & T & c & P & T' & c' & bad'
    m_startSolver = make_shared<SATSolver>(m_model, MCSATSolver::cadical);
    m_startSolver->AddTrans();
    m_startSolver->AddTransK(1);
    m_startSolver->AddBadk(1);
    m_startSolver->AddProperty();
    m_startSolver->AddConstraints();
    m_startSolver->AddConstraintsK(1);
    AddSamePrimeConstraints(m_startSolver);
    // bad predecessor lift
    m_badPredLiftSolver = make_shared<SATSolver>(m_model, MCSATSolver::cadical);
    m_badPredLiftSolver->AddTrans();
    m_badPredLiftSolver->AddTransK(1);

    m_restart.reset(new Restart(m_settings));
}

bool ForwardChecker::AddUnsatisfiableCore(shared_ptr<vector<int>> uc, int frameLevel) {
    m_restart->UcCountsPlus1();
    m_log->Tick();

    if (m_settings.multipleSolvers) {
        if (frameLevel >= m_transSolvers.size()) {
            m_transSolvers.emplace_back(make_shared<SATSolver>(m_model, m_settings.solver));
            if (m_settings.satSolveInDomain) m_transSolvers.back()->SetSolveInDomain();
            m_transSolvers.back()->AddTrans();
            m_transSolvers.back()->AddConstraints();
            AddSamePrimeConstraints(m_transSolvers.back());
            if (m_settings.solveInProperty) m_transSolvers.back()->AddProperty();
        }
        m_transSolvers[frameLevel]->AddUC(uc);
    } else
        m_transSolvers[0]->AddUC(uc, frameLevel);

    if (frameLevel >= m_k) {
        m_startSolver->AddUC(uc, frameLevel);
    }
    if (frameLevel < m_minUpdateLevel) {
        m_minUpdateLevel = frameLevel;
    }
    m_overSequence->Insert(uc, frameLevel);

    m_log->StatUpdateUc();
    return true;
}

bool ForwardChecker::ImmediateSatisfiable(int badId) {
    shared_ptr<cube> assumptions(new cube(*m_initialState->latches));
    assumptions->push_back(badId);
    m_log->Tick();
    if (m_settings.satSolveInDomain) m_transSolvers[0]->SetTempDomainCOI(assumptions);
    bool result = m_transSolvers[0]->Solve(assumptions);
    m_log->StatMainSolver();
    return result;
}


shared_ptr<State> ForwardChecker::EnumerateStartState() {
    if (m_startSolver->Solve()) {
        auto p = m_startSolver->GetAssignment(false);

        cube inputs_prime;
        shared_ptr<vector<int>> coi_inputs = m_model->GetCOIInputs();
        for (int i : *coi_inputs) {
            int i_p = m_model->GetPrimeK(i, 1);
            if (m_startSolver->GetModel(i_p))
                inputs_prime.push_back(i_p);
            else
                inputs_prime.push_back(-i_p);
        }

        // (p) & input & T & input' & T' -> (bad' & c' & c)
        // (p) & input & T & input' & T' & (!bad' | !c' | !c) is unsat
        shared_ptr<cube> partial_latch = make_shared<cube>(*p.second);

        // (!bad' | !c' | !c)
        clause cls;
        cls.push_back(-m_model->GetPrimeK(m_badId, 1));
        for (auto cons : m_model->GetConstraints())
            cls.push_back(-m_model->GetPrimeK(cons, 1));
        for (auto cons : m_model->GetConstraints())
            cls.push_back(-cons);
        m_badPredLiftSolver->AddTempClause(cls);

        while (true) {
            shared_ptr<cube> assumption(new cube());
            copy(partial_latch->begin(), partial_latch->end(), back_inserter(*assumption));
            OrderAssumption(assumption);
            copy(p.first->begin(), p.first->end(), back_inserter(*assumption));
            copy(inputs_prime.begin(), inputs_prime.end(), back_inserter(*assumption));

            bool res = m_badPredLiftSolver->Solve(assumption);
            assert(!res);
            shared_ptr<cube> temp_p = m_badPredLiftSolver->GetUC(false);
            if (temp_p->size() == partial_latch->size() &&
                equal(temp_p->begin(), temp_p->end(), partial_latch->begin()))
                break;
            else {
                partial_latch = temp_p;
            }
        }
        m_badPredLiftSolver->ReleaseTempClause();
        p.second = partial_latch;

        cube inputs_bad;
        for (int i : *coi_inputs) {
            int i_p = m_model->GetPrimeK(i, 1);
            if (m_startSolver->GetModel(i_p))
                inputs_bad.push_back(i);
            else
                inputs_bad.push_back(-i);
        }
        shared_ptr<State> badState(new State(nullptr, make_shared<cube>(inputs_bad), nullptr, 0));
        shared_ptr<State> badPredState(new State(badState, p.first, p.second, 0));
        return badPredState;
    } else {
        return nullptr;
    }
}


int ForwardChecker::GetNewLevel(shared_ptr<State> state, int start) {
    m_log->Tick();

    for (int i = start; i <= m_k; i++) {
        if (!m_overSequence->IsBlockedByFrame_lazy(state->latches, i)) {
            m_log->StatGetNewLevel();
            return i - 1;
        }
    }

    m_log->StatGetNewLevel();
    return m_k;
}


bool ForwardChecker::IsInvariant(int frameLevel) {
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
void ForwardChecker::AddConstraintOr(const shared_ptr<frame> f) {
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
void ForwardChecker::AddConstraintAnd(const shared_ptr<frame> f) {
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
// @brief: s & input & T -> (t' & c)  =>  (s) & input & T & (!t' | !c) is unsat
// @input: pair<input, latch>
// @output: pair<input, partial latch>
// ================================================================================
void ForwardChecker::GeneralizePredecessor(pair<shared_ptr<cube>, shared_ptr<cube>> &s, shared_ptr<State> t) {
    m_log->Tick();

    shared_ptr<cube> partial_latch = make_shared<cube>(*s.second);

    // (!t' | !c)
    clause cls;
    cls.reserve(t->latches->size());
    for (auto l : *t->latches) {
        cls.emplace_back(m_model->GetPrime(-l));
    }
    for (auto cons : m_model->GetConstraints()) cls.push_back(-cons);
    m_liftSolver->AddTempClause(cls);
    if (m_settings.satSolveInDomain) {
        m_liftSolver->ResetTempDomain();
        m_liftSolver->SetTempDomain(TopDomain());
    }

    while (true) {
        shared_ptr<cube> assumption(new cube());
        copy(partial_latch->begin(), partial_latch->end(), back_inserter(*assumption));
        OrderAssumption(assumption);
        copy(s.first->begin(), s.first->end(), back_inserter(*assumption));

        bool res = m_liftSolver->Solve(assumption);
        assert(!res);
        shared_ptr<cube> temp_p = m_liftSolver->GetUC(false);
        if (*temp_p == *partial_latch)
            break;
        else {
            partial_latch = temp_p;
        }
    }
    m_liftSolver->ReleaseTempClause();
    s.second = partial_latch;

    m_log->StatLiftSolver();
}


// ================================================================================
// @brief: counter-example to generalization
// @input:
// @output:
// ================================================================================
bool ForwardChecker::Generalize(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl) {
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
    if (m_settings.satSolveInDomain) {
        shared_ptr<cube> puc(new cube(*uc));
        GetPrimed(puc);
        GetAndPushDomain(puc);
    }
    for (int i = uc->size() - 1; i >= 0; i--) {
        if (uc->size() < 2) break;
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
    if (m_settings.satSolveInDomain) PopDomain();
    sort(uc->begin(), uc->end(), cmp);
    if (uc->size() > uc_blocker->size() && frame_lvl != 0) {
        return false;
    } else {
        return true;
    }
}


bool ForwardChecker::Down(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl, shared_ptr<vector<cube>> failed_ctses) {
    int ctgs = 0;
    m_log->L(3, "Down:", CubeToStr(uc));
    shared_ptr<cube> assumption(new cube(*uc));
    GetPrimed(assumption);
    shared_ptr<State> p_ucs(new State(nullptr, nullptr, uc, 0));
    while (true) {
        // F_i & T & temp_uc'
        m_log->Tick();
        if (!IsReachable(frame_lvl, assumption)) {
            m_log->StatMainSolver();
            sort(uc->begin(), uc->end(), cmp);
            auto uc_ctg = GetUnsatCore(frame_lvl, uc);
            if (uc->size() < uc_ctg->size()) return false; // there are cases that uc_ctg longer than uc
            uc->swap(*uc_ctg);
            return true;
        } else if (rec_lvl > m_settings.ctgMaxRecursionDepth) {
            m_log->StatMainSolver();
            return false;
        } else {
            m_log->StatMainSolver();
            auto p = GetInputAndState(frame_lvl);
            GeneralizePredecessor(p, p_ucs);
            shared_ptr<State> cts(new State(nullptr, p.first, p.second, 0));
            if (DownHasFailed(cts->latches, failed_ctses)) return false;
            int cts_lvl = GetNewLevel(cts);
            shared_ptr<cube> cts_ass(new cube(*cts->latches));
            OrderAssumption(cts_ass);
            GetPrimed(cts_ass);
            if (m_settings.satSolveInDomain) GetAndPushDomain(cts_ass);
            // F_i-1 & T & cts'
            m_log->L(3, "Try ctg:", CubeToStr(cts->latches));
            m_log->Tick();
            if (ctgs < m_settings.ctgMaxStates && cts_lvl >= 0 && !IsReachable(cts_lvl, cts_ass)) {
                m_log->StatMainSolver();
                ctgs++;
                auto uc_cts = GetUnsatCore(cts_lvl, cts->latches);
                m_log->L(3, "CTG Get UC:", CubeToStr(uc_cts));
                if (Generalize(uc_cts, cts_lvl, rec_lvl + 1))
                    m_branching->Update(uc_cts);
                m_log->L(3, "CTG Get Generalized UC:", CubeToStr(uc_cts));
                AddUnsatisfiableCore(uc_cts, cts_lvl + 1);
                PropagateUp(uc_cts, cts_lvl + 1);
                if (m_settings.satSolveInDomain) PopDomain();
            } else {
                m_log->StatMainSolver();
                failed_ctses->emplace_back(*cts->latches);
                if (m_settings.satSolveInDomain) PopDomain();
                return false;
            }
        }
    }
}


bool ForwardChecker::DownHasFailed(const shared_ptr<cube> s, const shared_ptr<vector<cube>> failed_ctses) {
    for (auto f : *failed_ctses) {
        // if f->s , return true
        if (f.size() > s->size()) continue;
        if (includes(s->begin(), s->end(), f.begin(), f.end(), cmp)) return true;
    }
    return false;
}


bool ForwardChecker::CheckInit(shared_ptr<State> s) {
    m_log->L(2, "SAT Check Init ");
    m_log->L(2, "From State: ", CubeToStrShort(s->latches));
    m_log->L(3, "State Detail: ", CubeToStr(s->latches));
    shared_ptr<cube> assumption(new cube(*s->latches));
    OrderAssumption(assumption);
    if (m_settings.satSolveInDomain) GetAndPushDomain(assumption);
    m_log->Tick();
    bool result = IsReachable(0, assumption);
    m_log->StatMainSolver();
    if (result) {
        // Solver return SAT
        m_log->L(2, "Result >>> SAT <<<");
        auto p = m_transSolvers[0]->GetAssignment(false);
        s->latches = p.second;
        if (m_settings.satSolveInDomain) PopDomain();
        return true;
    } else {
        // Solver return UNSAT, get uc, then continue
        m_log->L(2, "Result >>> UNSAT <<<");
        auto uc = m_transSolvers[0]->GetUC(false);
        assert(uc->size() > 0);
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
            m_log->Tick();
            bool result = IsReachable(0, assumption);
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
        m_log->L(2, "Get UC: ", CubeToStr(uc));
        AddUnsatisfiableCore(uc, 0);
        PropagateUp(uc, 0);
        m_log->L(3, "Frames: ", m_overSequence->FramesInfo());
        if (m_settings.satSolveInDomain) PopDomain();
        return false;
    }
}


bool ForwardChecker::Propagate(shared_ptr<cube> c, int lvl) {
    bool result;
    shared_ptr<cube> assumption(new cube(*c));
    GetPrimed(assumption);
    if (m_settings.satSolveInDomain) GetAndPushDomain(assumption);
    m_log->Tick();
    if (!IsReachable(lvl, assumption)) {
        m_log->StatPropagation();
        AddUnsatisfiableCore(c, lvl + 1);
        result = true;
    } else {
        m_log->StatPropagation();
        result = false;
    }
    if (m_settings.satSolveInDomain) PopDomain();
    return result;
}


int ForwardChecker::PropagateUp(shared_ptr<cube> c, int lvl) {
    while (lvl < m_k) {
        if (Propagate(c, lvl))
            m_branching->Update(c);
        else
            break;
        lvl++;
    }
    return lvl + 1;
}


void ForwardChecker::AddSamePrimeConstraints(shared_ptr<SATSolver> slv) {
    // if l_1 and l_2 have the same primed value l',
    // then l_1 and l_2 shoud have same value, except the initial states
    int init = slv->GetNewVar();
    int cons = slv->GetNewVar();

    // init | cons
    slv->AddClause(clause{init, cons});

    unordered_map<int, vector<int>> preValueMap;
    m_model->GetPreValueOfLatchMap(preValueMap);
    for (auto it = preValueMap.begin(); it != preValueMap.end(); it++) {
        if (it->second.size() > 1) {
            // cons -> ( p <-> v )
            int v = slv->GetNewVar();
            for (int p : it->second) {
                slv->AddClause(clause{-cons, -p, v});
                slv->AddClause(clause{-cons, p, -v});
            }
        }
    }
    // init -> i
    for (int i : m_model->GetInitialState()) {
        slv->AddClause(clause{-init, i});
    }
}


bool ForwardChecker::IsReachable(int lvl, const shared_ptr<cube> assumption) {
    if (m_settings.multipleSolvers) {
        if (m_settings.satSolveInDomain) {
            m_transSolvers[lvl]->ResetTempDomain();
            m_transSolvers[lvl]->SetTempDomain(TopDomain());
        }
        return m_transSolvers[lvl]->SolveFrame(assumption, 0);
    } else
        return m_transSolvers[0]->SolveFrame(assumption, lvl);
}


pair<shared_ptr<cube>, shared_ptr<cube>> ForwardChecker::GetInputAndState(int lvl) {
    if (m_settings.multipleSolvers)
        return m_transSolvers[lvl]->GetAssignment(false);
    else
        return m_transSolvers[0]->GetAssignment(false);
}


shared_ptr<cube> ForwardChecker::GetUnsatCore(int lvl, const shared_ptr<cube> state) {
    shared_ptr<cube> uc;
    if (m_settings.multipleSolvers)
        uc = m_transSolvers[lvl]->GetUC(true);
    else
        uc = m_transSolvers[0]->GetUC(true);

    MakeSubset(uc, state);
    return uc;
}


shared_ptr<cube> ForwardChecker::TopDomain() {
    return m_domainStack->back();
}


shared_ptr<cube> ForwardChecker::GetAndPushDomain(shared_ptr<cube> c) {
    shared_ptr<cube> domain = make_shared<cube>();
    domain = m_model->GetCOIDomain(c);
    m_domainStack->emplace_back(domain);
    return domain;
}


void ForwardChecker::PopDomain() {
    m_domainStack->pop_back();
}


// ================================================================================
// @brief: let c1 be a subset of c2
// @input: c1 and c2 are both sorted and without repeated elements
// @output:
// ================================================================================
void ForwardChecker::MakeSubset(shared_ptr<cube> c1, shared_ptr<cube> c2) {
    size_t i = 0, j = 0, k = 0;

    while (i < c1->size() && j < c2->size()) {
        if (c1->operator[](i) == c2->operator[](j)) {
            if (k != i) {
                c1->operator[](k) = std::move(c1->operator[](i));
            }
            k++;
            i++;
            j++;
        } else if (cmp(c1->operator[](i), c2->operator[](j))) {
            i++;
        } else {
            j++;
        }
    }
    c1->resize(k);
}


// ================================================================================
// @brief: add the cube as and gates to the aiger model
// @input:
// @output:
// ================================================================================
unsigned ForwardChecker::addCubeToANDGates(aiger *circuit, vector<unsigned> cube) {
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

    // same prime constraint
    // if l1 and l2 have same prime l', then l1 and l2 shoud have same value, except the initial states
    // sp_cons = init | cons
    // init = l1 & l2 & ... & lk
    // cons = ( x1 <-> x2 ) & ( x1 <-> x3 ) & ( ... )
    unordered_map<int, vector<int>> map;
    m_model->GetPreValueOfLatchMap(map);
    vector<unsigned> cons_lits;
    for (auto it = map.begin(); it != map.end(); it++) {
        if (it->second.size() > 1) {
            unsigned x0 = it->second[0] > 0 ? (2 * it->second[0]) : (2 * -it->second[0] + 1);
            for (int i = 1; i < it->second.size(); i++) {
                unsigned xi = it->second[i] > 0 ? (2 * it->second[i]) : (2 * -it->second[i] + 1);
                cons_lits.push_back(addCubeToANDGates(witness_aig, {x0, xi ^ 1}) ^ 1);
                cons_lits.push_back(addCubeToANDGates(witness_aig, {x0 ^ 1, xi}) ^ 1);
            }
        }
    }
    unsigned sp_cons;
    if (cons_lits.size() > 0) {
        vector<unsigned> init_lits;
        for (auto l : m_model->GetInitialState()) {
            int ll = l > 0 ? (2 * l) : (2 * -l + 1);
            init_lits.push_back(ll);
        }
        unsigned init = addCubeToANDGates(witness_aig, init_lits);
        unsigned cons = addCubeToANDGates(witness_aig, cons_lits);
        sp_cons = addCubeToANDGates(witness_aig, {init ^ 1, cons ^ 1}) ^ 1;
    }

    // P' = P & invariant
    // P' = !bad & ( O_0 | O_1 | ... | O_i )
    //             !( !O_0 & !O_1 & ...  & !O_i )
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
    unsigned inv = addCubeToANDGates(witness_aig, inv_lits) ^ 1;
    int bad_lit_int = bad > 0 ? (2 * bad) : (2 * -bad) + 1;
    unsigned bad_lit = bad_lit_int;
    unsigned p = aiger_not(bad_lit);
    unsigned p_prime = addCubeToANDGates(witness_aig, {p, inv});

    if (cons_lits.size() > 0) {
        p_prime = addCubeToANDGates(witness_aig, {p_prime, sp_cons});
    }

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

    shared_ptr<State> state = m_lastState;
    cexFile << state->GetLatchesString() << endl;
    cexFile << state->GetInputsString() << endl;
    while (state->preState != nullptr) {
        state = state->preState;
        cexFile << state->GetInputsString() << endl;
    }

    cexFile << "." << endl;
    cexFile.close();
}
} // namespace car