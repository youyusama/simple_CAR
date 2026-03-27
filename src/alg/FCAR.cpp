#include "FCAR.h"
#include <stack>
#include <string>

namespace car {
FCAR::FCAR(Settings settings,
           Model &model,
           Log &log) : m_settings(settings),
                       m_model(model),
                       m_log(log),
                       m_innOrder(model) {
    State::num_inputs = model.GetNumInputs();
    State::num_latches = model.GetNumLatches();
    m_lastState = nullptr;
    global_log = &m_log;
    m_checkResult = CheckResult::Unknown;

    m_settings.satSolveInDomain = m_settings.satSolveInDomain && m_settings.solver == MCSATSolver::minicore;
}

CheckResult FCAR::Run() {
    signal(SIGINT, SignalHandler);

    if (Check())
        m_checkResult = CheckResult::Safe;
    else
        m_checkResult = CheckResult::Unsafe;

    m_log.PrintCustomStatistics();
    return m_checkResult;
}

void FCAR::Witness() {
    if (m_checkResult == CheckResult::Safe) {
        OutputWitness();
    } else if (m_checkResult == CheckResult::Unsafe) {
        OutputCounterExample();
    }
}

bool FCAR::Check() {
    [[maybe_unused]] auto check_scope = m_log.Section("FC_Check");

    if (!m_initialized) {
        Init();
        LOG_L(m_log, 2, "Initialized");
    } else {
        Reset();
        LOG_L(m_log, 2, "Reset");
    }

    // check if initial state is bad
    if (ImmediateSatisfiable()) return false;

    // main stage
    stack<Task> working_stack;
    while (true) {
        [[maybe_unused]] auto frame_scope = m_log.Section("FC_Frame");
        m_minUpdateLevel = m_k;
        if (m_settings.dt) { // Dynamic Traversal
            auto dtseq = m_underSequence.GetSeqDT();
            for (auto state : dtseq) {
                working_stack.emplace(state, m_k - 1, false);
            }
        } else { // from the shallow and the start
            for (int i = m_underSequence.Size() - 1; i >= 0; i--) {
                for (int j = m_underSequence[i].size() - 1; j >= 0; j--) {
                    working_stack.emplace(m_underSequence[i][j], m_k - 1, false);
                }
            }
        }
        LOG_L(m_log, 2, "Start Frame: ", m_k);
        LOG_L(m_log, 2, "Working Stack Size: ", working_stack.size());

        shared_ptr<State> start_state = EnumerateStartState();
        // T & c & P & T' & c' & bad' is unsat
        if (m_k > 0 && start_state == nullptr && m_overSequence->IsEmpty(m_k)) {
            m_overSequence->SetInvariantLevel(-1);
            return true;
        }

        CreateTransSolver(m_k);

        while (start_state != nullptr) {
            LOG_L(m_log, 2, "State from StartSolver: ", CubeToStrShort(start_state->latches));
            LOG_L(m_log, 3, "State Detail: ", CubeToStr(start_state->latches));
            working_stack.emplace(start_state, m_k - 1, true);

            while (!working_stack.empty()) {
                [[maybe_unused]] auto task_scope = m_log.Section("FC_Task");
                Task &task = working_stack.top();

                if (m_settings.restart && m_restart->RestartCheck()) {
                    LOG_L(m_log, 1, "Restarting...");
                    m_underSequence = UnderSequence();
                    while (working_stack.size() > 1) working_stack.pop();
                    m_restart->UpdateThreshold();
                    m_restart->ResetUcCounts();
                    continue;
                }

                if (!task.isLocated) {
                    task.frameLevel = GetNewLevel(task.state->latches, task.frameLevel + 1);
                    LOG_L(m_log, 3, "State get new Level ", task.frameLevel);
                }

                if (task.frameLevel >= m_k) {
                    working_stack.pop();
                    continue;
                }

                task.isLocated = false;

                if (task.frameLevel == (m_searchFromInitSucc ? 0 : -1)) {
                    if (!m_searchFromInitSucc) {
                        m_lastState = task.state;
                        return false;
                    } else if (CheckInit(task.state)) {
                        return false;
                    } else
                        continue;
                }
                LOG_L(m_log, 2, "SAT Check on Frame: ", task.frameLevel);
                LOG_L(m_log, 2, "From State: ", CubeToStrShort(task.state->latches));
                LOG_L(m_log, 3, "State Detail: ", CubeToStr(task.state->latches));
                Cube assumption(task.state->latches);
                OrderAssumption(assumption);
                GetPrimed(assumption);
                m_transSolvers[task.frameLevel]->SetTempDomainCOI(assumption);
                bool result = IsReachable(task.frameLevel, assumption, "SAT_R_Main");
                if (result) {
                    // Solver return SAT, get a new State, then continue
                    LOG_L(m_log, 2, "Result >>> SAT <<<");
                    auto p = GetInputAndState(task.frameLevel);
                    LOG_L(m_log, 3, "Input Detail: ", CubeToStr(p.first));
                    LOG_L(m_log, 3, "State Detail: ", CubeToStr(p.second));
                    GeneralizePredecessor(p, task.state);
                    shared_ptr<State> new_state =
                        make_shared<State>(task.state, p.first, p.second, task.state->depth + 1);
                    m_underSequence.Push(new_state);
                    if (m_settings.dt) task.state->HasSucc();
                    LOG_L(m_log, 3, "Get State: ", CubeToStrShort(new_state->latches));
                    LOG_L(m_log, 3, "State Detail: ", CubeToStr(new_state->latches));
                    int new_frame_level = GetNewLevel(new_state->latches);
                    working_stack.emplace(new_state, new_frame_level, true);
                } else {
                    // Solver return UNSAT, get uc, then continue
                    LOG_L(m_log, 2, "Result >>> UNSAT <<<");
                    auto uc = GetUnsatCore(task.frameLevel, task.state->latches);
                    assert(uc.size() > 0);
                    LOG_L(m_log, 3, "Get UC: ", CubeToStr(uc));
                    if (Generalize(uc, task.frameLevel))
                        m_branching->Update(uc);
                    LOG_L(m_log, 2, "Get Generalized UC: ", CubeToStr(uc));
                    AddUnsatisfiableCore(uc, task.frameLevel + 1);
                    if (m_settings.dt) task.state->HasUC();
                    task.frameLevel = PropagateUp(uc, task.frameLevel + 1);
                    LOG_L(m_log, 3, m_overSequence->FramesInfo());
                }
            } // end while (!workingStack.empty())
            start_state = EnumerateStartState();
        }

        // inv
        m_invSolver = make_shared<SATSolver>(m_model, MCSATSolver::cadical);
        if (!m_searchFromInitSucc) IsInvariant(0);
        for (int i = 0; i < m_k; ++i) {
            // propagation
            if (i >= m_minUpdateLevel) {
                shared_ptr<OverSequenceSet::FrameSet> fi = m_overSequence->GetFrame(i);
                shared_ptr<OverSequenceSet::FrameSet> fi_plus_1 = m_overSequence->GetFrame(i + 1);
                OverSequenceSet::FrameSet::iterator iter;
                for (const Cube &uc : *fi) {
                    if (fi_plus_1->find(uc) != fi_plus_1->end()) continue; // propagated
                    if (Propagate(uc, i))
                        m_branching->Update(uc);
                }
            }

            // invariant check
            if (IsInvariant(i + 1)) {
                LOG_L(m_log, 1, "Proof at Frame ", i + 1);
                LOG_L(m_log, 1, m_overSequence->FramesInfo());
                m_overSequence->SetInvariantLevel(i);
                return true;
            }
        }

        LOG_L(m_log, 1, m_overSequence->FramesInfo());
        LOG_L(m_log, 3, m_overSequence->FramesDetail());
        InitializeStartSolver();

        m_k++;
        m_restart->ResetUcCounts();
        LOG_L(m_log, 2, "\nNew Frame Added");
    }
}


void FCAR::Init() {
    [[maybe_unused]] auto init_scope = m_log.Section("FC_Init");

    // start solver search state in frame k
    m_k = m_searchFromInitSucc ? 2 : 1;

    if (m_searchFromInitSucc) {
        m_initStateImplyBad = IsInitStateImplyBad();
        if (!m_initStateImplyBad)
            LOG_L(m_log, 1, "Initial state does not imply bad");
    }

    // initial states
    Cube init_latches;
    if (m_customInit.empty())
        init_latches = m_model.GetInitialState();
    else
        init_latches = m_customInit;
    m_initialState = make_shared<State>(nullptr, Cube{}, init_latches, 0);

    m_overSequence = make_shared<OverSequenceSet>(m_model);
    m_underSequence = UnderSequence();
    m_branching = make_shared<Branching>(m_settings.branching);
    m_litOrder.branching = m_branching;
    m_blockerOrder.branching = m_branching;
    m_transSolvers.clear();
    m_lastState = nullptr;

    CreateTransSolver(0);

    // lift
    m_liftSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    if (m_settings.satSolveInDomain) m_liftSolver->SetSolveInDomain();
    m_liftSolver->AddTrans();
    m_liftSolver->SetDomainCOI(m_model.GetConstraints());

    InitializeStartSolver();
    if (m_settings.searchFromBadPred) {
        // bad predecessor lift
        m_badLiftSolver = make_shared<SATSolver>(m_model, MCSATSolver::cadical);
        m_badLiftSolver->AddTrans();
        m_badLiftSolver->AddTransK(1);
    } else {
        // bad lift
        m_badLiftSolver = make_shared<SATSolver>(m_model, m_settings.solver);
        if (m_settings.satSolveInDomain) m_badLiftSolver->SetSolveInDomain();
        m_badLiftSolver->AddTrans();
        m_badLiftSolver->SetDomainCOI(m_model.GetConstraints());
        m_badLiftSolver->SetDomainCOI({m_model.GetBad()});
        m_shoalsLabels = m_badLiftSolver->AddShoalConstraintsAsLabels(m_shoals, m_dead);
        m_wallsLabels = m_badLiftSolver->AddWallConstraintsAsLabels(m_walls);
    }

    m_restart.reset(new Restart(m_settings));

    // initialize frame 0
    for (Lit l : m_initialState->latches) {
        Cube uc{~l};
        m_transSolvers[0]->AddUC(uc);
        m_overSequence->Insert(uc, 0);

        if (m_searchFromInitSucc && !m_initStateImplyBad)
            m_transSolvers[0]->AddBad();
    }

    m_initialized = true;
}


void FCAR::CreateTransSolver(int k) {
    while (m_transSolvers.size() <= k) {
        // O_i & T & c & s'
        m_transSolvers.emplace_back(make_shared<SATSolver>(m_model, m_settings.solver));
        if (m_settings.satSolveInDomain) m_transSolvers.back()->SetSolveInDomain();
        m_transSolvers.back()->AddTrans();
        m_transSolvers.back()->AddConstraints();
        if (m_settings.solveInProperty && k > 0) m_transSolvers.back()->AddProperty();
        // liveness: T = T & ( W <-> W' )
        m_transSolvers.back()->AddWallConstraints(m_walls);
    }
}


void FCAR::Reset() {
    m_underSequence.Clear();
    m_lastState = nullptr;
    m_cexTrace.clear();

    InitializeStartSolver();
    // add exist lemma
    auto frame_i = m_overSequence->GetFrame(m_k);
    for (auto l : *frame_i) m_startSolver->AddUC(l);
    m_wallsLabels = m_badLiftSolver->AddWallConstraintsAsLabels(m_walls);
}


void FCAR::InitializeStartSolver() {
    if (m_settings.searchFromBadPred) {
        // s & T & c & P & T' & c' & bad'
        m_startSolver = make_shared<SATSolver>(m_model, MCSATSolver::cadical);
        m_startSolver->AddTrans();
        m_startSolver->AddConstraints();
        m_startSolver->AddTransK(1);
        m_startSolver->AddBadk(1);
        m_startSolver->AddProperty();
        m_startSolver->AddConstraintsK(1);
    } else {
        // s & c & bad
        m_startSolver = make_shared<SATSolver>(m_model, m_settings.solver);
        if (m_settings.satSolveInDomain) m_startSolver->SetSolveInDomain();
        m_startSolver->AddTrans();
        m_startSolver->AddConstraints();
        if (m_loopRefuting) {
            for (auto lit : m_initialState->latches) m_startSolver->AddClause({lit});
        } else {
            m_startSolver->AddBad();
        }
        m_startSolver->SetDomainCOI(m_model.GetConstraints());
        m_startSolver->SetDomainCOI({m_model.GetBad()});
        // liveness: T = T & !C'
        //           T = T & ( W <-> W' )
        m_startSolver->AddShoalConstraints(m_shoals, m_dead);
        m_startSolver->AddWallConstraints(m_walls);
    }
}


bool FCAR::AddUnsatisfiableCore(const Cube &uc, int frameLevel) {
    [[maybe_unused]] auto scoped = m_log.Section("DS_AddUC");
    m_restart->UcCountsPlus1();
    if (!m_overSequence->Insert(uc, frameLevel)) return false;

    m_transSolvers[frameLevel]->AddUC(uc);

    if (frameLevel == m_k) {
        m_startSolver->AddUC(uc);
    }
    if (frameLevel < m_minUpdateLevel) {
        m_minUpdateLevel = frameLevel;
    }

    return true;
}


shared_ptr<State> FCAR::EnumerateStartState() {
    [[maybe_unused]] auto scoped = m_log.Section("FC_StartEnum");
    bool sat = false;
    {
        [[maybe_unused]] auto sat_scope = m_log.Section("SAT_Start");
        sat = m_startSolver->Solve();
    }
    if (sat) {
        if (m_loopRefuting) {
            shared_ptr<State> bad_state(new State(nullptr, {}, m_customInit, 0));
            return bad_state;
        }

        auto p = m_startSolver->GetAssignment(false);

        if (m_settings.searchFromBadPred) {
            // start state is the predecessor of a bad state
            Cube inputs_prime;
            for (int i : m_model.GetPropertyCOIInputs()) {
                Lit i_p = m_model.EnsurePrimeK(MkLit(i), 1);
                if (m_startSolver->GetModel(VarOf(i_p)) == T_TRUE)
                    inputs_prime.push_back(i_p);
                else if (m_startSolver->GetModel(VarOf(i_p)) == T_FALSE)
                    inputs_prime.push_back(~i_p);
            }

            // (p) & input & T & input' & T' -> (bad' & c' & c)
            // (p) & input & T & input' & T' & (!bad' | !c' | !c) is unsat
            Cube partial_latch = p.second;

            // (!bad' | !c' | !c)
            Clause cls;
            cls.push_back(~m_model.EnsurePrimeK(m_model.GetBad(), 1));
            for (auto cons : m_model.GetConstraints())
                cls.push_back(~m_model.EnsurePrimeK(cons, 1));
            for (auto cons : m_model.GetConstraints())
                cls.push_back(~cons);
            m_badLiftSolver->AddTempClause(cls);

            int gen_tried = 0;

            while (true) {
                Cube assumption;
                copy(partial_latch.begin(), partial_latch.end(), back_inserter(assumption));
                OrderAssumption(assumption);

                if (gen_tried == 1) reverse(assumption.begin(), assumption.end());
                if (gen_tried > 1) random_shuffle(assumption.begin(), assumption.end());
                gen_tried++;

                copy(p.first.begin(), p.first.end(), back_inserter(assumption));
                copy(inputs_prime.begin(), inputs_prime.end(), back_inserter(assumption));

                bool res;
                {
                    [[maybe_unused]] auto sat_bad_pred = m_log.Section("SAT_BadLift");
                    res = m_badLiftSolver->Solve(assumption);
                }
                assert(!res);
                Cube temp_p = GetUnsatAssumption(m_badLiftSolver, partial_latch);
                if (temp_p.size() >= partial_latch.size())
                    break;
                else {
                    partial_latch.swap(temp_p);
                }
            }
            m_badLiftSolver->ReleaseTempClause();
            p.second = partial_latch;

            Cube inputs_bad;
            for (int i : m_model.GetPropertyCOIInputs()) {
                Lit i_p = m_model.EnsurePrimeK(MkLit(i), 1);
                if (m_startSolver->GetModel(VarOf(i_p)) == T_TRUE)
                    inputs_bad.push_back(MkLit(i));
                else if (m_startSolver->GetModel(VarOf(i_p)) == T_FALSE)
                    inputs_bad.push_back(~MkLit(i));
            }
            shared_ptr<State> bad_state(new State(nullptr, inputs_bad, Cube(), 0));
            shared_ptr<State> bad_pred_state(new State(bad_state, p.first, p.second, 0));
            return bad_pred_state;
        } else {
            // start state is a bad state
            // (p) -> (bad & c)
            // (p) & (!bad | !c) is unsat
            Cube partial_latch = p.second;
            LOG_L(m_log, 3, "Bad State Latches Before Lifting: ", CubeToStr(partial_latch));

            // (!bad | !c)
            Clause cls;
            cls.push_back(~m_model.GetBad());
            for (auto cons : m_model.GetConstraints())
                cls.push_back(~cons);
            for (auto l : m_shoalsLabels) cls.push_back(~l);
            for (auto l : m_wallsLabels) cls.push_back(~l);
            m_badLiftSolver->AddTempClause(cls);
            LOG_L(m_log, 3, "lift assume: ", CubeToStr(cls));

            int gen_tried = 0;

            while (true) {
                Cube assumption;
                copy(partial_latch.begin(), partial_latch.end(), back_inserter(assumption));
                OrderAssumption(assumption);

                if (gen_tried == 1) reverse(assumption.begin(), assumption.end());
                if (gen_tried > 1) random_shuffle(assumption.begin(), assumption.end());
                gen_tried++;

                copy(p.first.begin(), p.first.end(), back_inserter(assumption));

                bool res;
                {
                    [[maybe_unused]] auto sat_bad_pred = m_log.Section("SAT_BadLift");
                    res = m_badLiftSolver->Solve(assumption);
                }
                assert(!res);
                Cube temp_p = GetUnsatAssumption(m_badLiftSolver, partial_latch);
                if (temp_p.size() >= partial_latch.size())
                    break;
                else {
                    partial_latch.swap(temp_p);
                }
            }
            m_badLiftSolver->ReleaseTempClause();
            p.second = partial_latch;

            shared_ptr<State> bad_state(new State(nullptr, p.first, p.second, 0));
            return bad_state;
        }
    } else {
        return nullptr;
    }
}


int FCAR::GetNewLevel(const Cube &states, int start) {
    [[maybe_unused]] auto scoped = m_log.Section("FC_GetNewLevel");
    if (m_searchFromInitSucc) start = (start == 0) ? 1 : start;

    for (int i = start; i <= m_k; i++) {
        if (!m_overSequence->IsBlockedByFrame(states, i)) {
            return i - 1;
        }
    }

    return m_k;
}


bool FCAR::IsInvariant(int frameLevel) {
    [[maybe_unused]] auto scoped = m_log.Section("FC_Invariant");
    shared_ptr<OverSequenceSet::FrameSet> frame_i = m_overSequence->GetFrame(frameLevel);

    if (frameLevel < m_minUpdateLevel) {
        AddConstraintOr(frame_i);
        return false;
    }

    AddConstraintAnd(frame_i);
    bool result;
    {
        [[maybe_unused]] auto sat_inv = m_log.Section("SAT_Inv");
        result = !m_invSolver->Solve();
    }
    m_invSolver->FlipLastConstrain();
    AddConstraintOr(frame_i);

    return result;
}


// ================================================================================
// @brief: add constraint | O_i
// @input:
// @output:
// ================================================================================
void FCAR::AddConstraintOr(const shared_ptr<OverSequenceSet::FrameSet> f) {
    Cube cls;
    for (const Cube &frame_cube : *f) {
        Var flag = m_invSolver->GetNewVar();
        Lit flag_lit = MkLit(flag);
        cls.push_back(flag_lit);
        for (size_t i = 0; i < frame_cube.size(); i++) {
            m_invSolver->AddClause(Cube{~flag_lit, frame_cube[i]});
        }
    }
    m_invSolver->AddClause(cls);
}


// ================================================================================
// @brief: add constraint & !O_i
// @input:
// @output:
// ================================================================================
void FCAR::AddConstraintAnd(const shared_ptr<OverSequenceSet::FrameSet> f) {
    Var flag = m_invSolver->GetNewVar();
    Lit flag_lit = MkLit(flag);
    for (const Cube &frame_cube : *f) {
        Cube cls;
        for (size_t i = 0; i < frame_cube.size(); i++) {
            cls.push_back(~frame_cube[i]);
        }
        cls.push_back(~flag_lit);
        m_invSolver->AddClause(cls);
    }
    m_invSolver->AddAssumption(Cube{flag_lit});
}


// ================================================================================
// @brief: s & input & T -> (t' & c)  =>  (s) & input & T & (!t' | !c) is unsat
// @input: pair<input, latch>
// @output: pair<input, partial latch>
// ================================================================================
void FCAR::GeneralizePredecessor(pair<Cube, Cube> &s, shared_ptr<State> t) {
    [[maybe_unused]] auto scoped = m_log.Section("FC_GenPred");
    Cube partial_latch = s.second;

    // (!t' | !c)
    Clause cls;
    cls.reserve(t->latches.size());
    for (auto l : t->latches) {
        cls.emplace_back(m_model.LookupPrime(~l));
    }
    for (auto cons : m_model.GetConstraints()) cls.push_back(~cons);
    m_liftSolver->AddTempClause(cls);
    m_liftSolver->SetTempDomainCOI(cls);
    int gen_tried = 0;

    while (true) {
        Cube assumption;
        copy(partial_latch.begin(), partial_latch.end(), back_inserter(assumption));
        OrderAssumption(assumption);

        if (gen_tried == 1) reverse(assumption.begin(), assumption.end());
        if (gen_tried > 1) random_shuffle(assumption.begin(), assumption.end());
        gen_tried++;

        copy(s.first.begin(), s.first.end(), back_inserter(assumption));

        bool res;
        {
            [[maybe_unused]] auto sat_lift = m_log.Section("SAT_Lift");
            res = m_liftSolver->Solve(assumption);
        }
        assert(!res);
        Cube temp_p = GetUnsatAssumption(m_liftSolver, partial_latch);
        if (temp_p.size() == 0) break; // happens when t->latches' are all inputs
        if (temp_p.size() >= partial_latch.size())
            break;
        else {
            partial_latch.swap(temp_p);
        }
    }
    m_liftSolver->ReleaseTempClause();
    s.second.swap(partial_latch);
}


// ================================================================================
// @brief: counter-example to generalization
// @input:
// @output:
// ================================================================================
bool FCAR::Generalize(Cube &uc, int frameLvl, int recLvl) {
    [[maybe_unused]] auto setup_scope = m_log.Section("FC_Gen_Set");
    unordered_set<Lit, LitHash> required_lits;

    vector<Cube> uc_blockers;
    m_overSequence->GetBlockers(uc, frameLvl, uc_blockers);
    Cube uc_blocker;
    if (uc_blockers.size() > 0) {
        if (m_settings.branching > 0)
            stable_sort(uc_blockers.begin(), uc_blockers.end(), m_blockerOrder);
        uc_blocker = uc_blockers[0];
    }

    if (m_settings.referSkipping)
        for (auto b : uc_blocker) required_lits.emplace(b);
    vector<Cube> failed_ctses;
    int attempts = m_settings.ctgMaxAttempts;
    OrderAssumption(uc);
    setup_scope = m_log.Section("FC_Gen_Loop");
    for (int i = uc.size() - 1; i >= 0; i--) {
        if (uc.size() < 2) break;
        if (required_lits.find(uc.at(i)) != required_lits.end()) continue;
        [[maybe_unused]] auto iter_scope = m_log.Section("FC_Gen_Try");
        Cube temp_uc;
        temp_uc.reserve(uc.size());
        for (auto ll : uc)
            if (ll != uc.at(i)) temp_uc.emplace_back(ll);
        if (Down(temp_uc, frameLvl, recLvl, failed_ctses)) {
            uc.swap(temp_uc);
            i = uc.size();
            attempts = m_settings.ctgMaxAttempts;
        } else {
            if (--attempts == 0) break;
            required_lits.emplace(uc.at(i));
        }
    }
    setup_scope = m_log.Section("FC_Gen_Post");
    sort(uc.begin(), uc.end());
    if (uc.size() > uc_blocker.size() && frameLvl != 0) {
        return false;
    } else {
        return true;
    }
}


bool FCAR::Down(Cube &uc, int frameLvl, int recLvl, vector<Cube> &failedCtses) {
    [[maybe_unused]] auto down_setup = m_log.Section("FC_Dn_Set");
    int ctgs = 0;
    LOG_L(m_log, 3, "Down:", CubeToStr(uc));
    Cube assumption(uc);
    GetPrimed(assumption);
    shared_ptr<State> p_ucs(new State(nullptr, Cube(), uc, 0));
    down_setup = m_log.Section("FC_Dn_Loop");
    while (true) {
        m_transSolvers[frameLvl]->SetTempDomainCOI(assumption);
        // F_i & T & temp_uc'
        if (!IsReachable(frameLvl, assumption, "SAT_R_Down")) {
            uc = GetUnsatCore(frameLvl, uc);
            return true;
        }

        if (recLvl >= m_settings.ctgMaxRecursionDepth ||
            ctgs >= m_settings.ctgMaxStates ||
            frameLvl < 1) {
            return false;
        }

        [[maybe_unused]] auto ctg_scope = m_log.Section("FC_Dn_CTG");
        auto p = GetInputAndState(frameLvl);
        GeneralizePredecessor(p, p_ucs);
        shared_ptr<State> cts(new State(nullptr, p.first, p.second, 0));
        if (DownHasFailed(cts->latches, failedCtses)) return false;

        if (CTSBlock(cts, frameLvl - 1, recLvl, failedCtses)) {
            ctgs++;
        } else {
            failedCtses.emplace_back(cts->latches);
            return false;
        }
    }
}


bool FCAR::CTSBlock(shared_ptr<State> cts, int frameLvl, int recLvl, vector<Cube> &failedCtses, int ctsCount) {
    if (ctsCount >= m_settings.ctgMaxBlocks) return false;

    // F_i & T & cts'
    LOG_L(m_log, 3, "Try cts:", CubeToStr(cts->latches));
    Cube cts_ass(cts->latches);
    OrderAssumption(cts_ass);
    GetPrimed(cts_ass);

    while (true) {
        m_transSolvers[frameLvl]->SetTempDomainCOI(cts_ass);
        if (!IsReachable(frameLvl, cts_ass, "SAT_R_CTS_B")) {
            auto uc_cts = GetUnsatCore(frameLvl, cts->latches);
            LOG_L(m_log, 3, "CTG Get UC:", CubeToStr(uc_cts));
            if (Generalize(uc_cts, frameLvl, recLvl + 1))
                m_branching->Update(uc_cts);
            LOG_L(m_log, 3, "CTG Get Generalized UC:", CubeToStr(uc_cts));
            AddUnsatisfiableCore(uc_cts, frameLvl + 1);
            PropagateUp(uc_cts, frameLvl + 1);
            return true;
        } else {
            auto p = GetInputAndState(frameLvl);
            GeneralizePredecessor(p, cts);
            shared_ptr<State> pre_cts(new State(nullptr, p.first, p.second, 0));
            if (DownHasFailed(pre_cts->latches, failedCtses)) return false;

            int pre_cts_lvl = frameLvl - 1;
            if (pre_cts_lvl < 0 ||
                !CTSBlock(pre_cts, pre_cts_lvl, recLvl, failedCtses, ctsCount + 1)) {

                failedCtses.emplace_back(cts->latches);
                return false;
            }
        }
    }
}


bool FCAR::DownHasFailed(const Cube &s, const vector<Cube> &failedCtses) {
    for (auto f : failedCtses) {
        // if f->s , return true
        if (f.size() > s.size()) continue;
        if (includes(s.begin(), s.end(), f.begin(), f.end())) return true;
    }
    return false;
}


bool FCAR::ImmediateSatisfiable() {
    [[maybe_unused]] auto scoped = m_log.Section("FC_InitSat");
    auto slv = make_unique<SATSolver>(m_model, MCSATSolver::cadical);
    slv->AddTrans();
    slv->AddConstraints();
    slv->AddInitialClauses();
    for (auto i : m_model.GetInitialState()) {
        slv->AddClause(Cube{i});
    }
    Cube assumptions = Cube{m_model.GetBad()};
    bool sat = slv->Solve(assumptions);
    if (sat) {
        auto p = slv->GetAssignment(false);
        m_lastState = make_shared<State>(nullptr, p.first, p.second, 0);
        return true;
    } else if (m_settings.searchFromBadPred) {
        slv->AddTransK(1);
        slv->AddConstraintsK(1);
        slv->AddBadk(1);
        sat = slv->Solve(Cube{});
        if (sat) {
            Cube inputs_bad;
            for (int i : m_model.GetPropertyCOIInputs()) {
                Lit i_p = m_model.EnsurePrimeK(MkLit(i), 1);
                if (slv->GetModel(VarOf(i_p)) == T_TRUE)
                    inputs_bad.push_back(MkLit(i));
                else if (slv->GetModel(VarOf(i_p)) == T_FALSE)
                    inputs_bad.push_back(~MkLit(i));
            }
            shared_ptr<State> bad_state(new State(nullptr, inputs_bad, Cube(), 0));
            auto p = slv->GetAssignment(false);
            m_lastState = make_shared<State>(bad_state, p.first, p.second, 0);
            return true;
        }
    }
    return false;
}


bool FCAR::CheckInit(shared_ptr<State> s) {
    [[maybe_unused]] auto scoped = m_log.Section("FC_InitChk");
    assert(!m_searchFromInitSucc);
    LOG_L(m_log, 2, "SAT Check Init ");
    LOG_L(m_log, 2, "From State: ", CubeToStrShort(s->latches));
    LOG_L(m_log, 3, "State Detail: ", CubeToStr(s->latches));
    Cube assumption(s->latches);
    OrderAssumption(assumption);
    if (m_searchFromInitSucc) {
        GetPrimed(assumption);
    }
    m_transSolvers[0]->SetTempDomain(assumption);
    bool result = IsReachable(0, assumption, "SAT_R_Init");
    if (result) {
        // Solver return SAT
        LOG_L(m_log, 2, "Result >>> SAT <<<");
        auto p = m_transSolvers[0]->GetAssignment(false);

        if (m_searchFromInitSucc) {
            m_initialState->inputs = p.first;
            m_initialState->latches = p.second;
            m_initialState->preState = s;
        } else {
            s->latches = p.second;
            m_initialState->preState = s->preState;
            m_initialState->inputs = s->inputs;
            m_initialState->latches = s->latches;
        }
        m_lastState = m_initialState;
        return true;
    } else {
        // Solver return UNSAT, get uc, refine frame 0
        LOG_L(m_log, 2, "Result >>> UNSAT <<<");
        Cube uc;
        if (m_searchFromInitSucc)
            uc = GetUnsatCore(0, s->latches);
        else
            uc = GetUnsatAssumption(m_transSolvers[0], assumption);
        assert(uc.size() > 0);
        OrderAssumption(uc);

        // Generalization
        unordered_set<Lit, LitHash> required_lits;
        for (int i = uc.size() - 1; i >= 0; i--) {
            if (uc.size() < 3) break;
            if (required_lits.find(uc.at(i)) != required_lits.end()) continue;
            assumption.clear();
            for (auto ll : uc)
                if (ll != uc.at(i)) assumption.emplace_back(ll);
            if (m_searchFromInitSucc) {
                GetPrimed(assumption);
            }
            bool result = IsReachable(0, assumption, "SAT_R_Init");
            if (!result) {
                Cube new_uc;
                if (m_searchFromInitSucc)
                    new_uc = GetUnsatCore(0, uc);
                else
                    new_uc = GetUnsatAssumption(m_transSolvers[0], assumption);
                uc.swap(new_uc);
                i = uc.size();
            } else {
                required_lits.emplace(uc.at(i));
            }
        }
        sort(uc.begin(), uc.end());
        LOG_L(m_log, 2, "Get UC: ", CubeToStr(uc));
        if (m_searchFromInitSucc) {
            AddUnsatisfiableCore(uc, 1);
            PropagateUp(uc, 1);
        } else {
            AddUnsatisfiableCore(uc, 0);
            PropagateUp(uc, 0);
        }
        LOG_L(m_log, 3, "Frames: ", m_overSequence->FramesInfo());
        return false;
    }
}


bool FCAR::Propagate(const Cube &c, int lvl) {
    [[maybe_unused]] auto scoped = m_log.Section("FC_Prop");
    bool result;
    Cube assumption(c);
    GetPrimed(assumption);
    m_transSolvers[lvl]->SetTempDomainCOI(assumption);
    if (!IsReachable(lvl, assumption, "SAT_R_Prop")) {
        auto uc = GetUnsatCore(lvl, c);
        AddUnsatisfiableCore(uc, lvl + 1);
        result = true;
    } else {
        result = false;
    }
    return result;
}


int FCAR::PropagateUp(const Cube &c, int lvl) {
    [[maybe_unused]] auto scoped = m_log.Section("FC_PropUp");
    while (lvl < m_k) {
        if (Propagate(c, lvl))
            m_branching->Update(c);
        else
            break;
        lvl++;
    }
    return lvl;
}


bool FCAR::IsReachable(int lvl, const Cube &assumption, const string &label) {
    [[maybe_unused]] auto scoped = m_log.Section(label);
    return m_transSolvers[lvl]->Solve(assumption);
}


pair<Cube, Cube> FCAR::GetInputAndState(int lvl) {
    return m_transSolvers[lvl]->GetAssignment(false);
}


Cube FCAR::GetUnsatCore(int lvl, const Cube &state) {
    [[maybe_unused]] auto scoped = m_log.Section("DS_UCore");
    const unordered_set<Lit, LitHash> &conflict = m_transSolvers[lvl]->GetConflict();
    Cube res;
    for (auto l : state) {
        Lit p = m_model.LookupPrime(l);
        if (conflict.find(p) != conflict.end())
            res.emplace_back(l);
    }
    return res;
}


Cube FCAR::GetUnsatAssumption(shared_ptr<SATSolver> solver, const Cube &assumptions) {
    [[maybe_unused]] auto scoped = m_log.Section("DS_UAssump");
    const unordered_set<Lit, LitHash> &conflict = solver->GetConflict();
    Cube res;
    for (auto a : assumptions) {
        if (conflict.find(a) != conflict.end())
            res.emplace_back(a);
    }
    return res;
}


void FCAR::BuildCEXTrace() {
    assert(m_checkResult == CheckResult::Unsafe);
    assert(m_lastState != nullptr);
    m_cexTrace.clear();

    auto state = m_lastState;
    while (state->preState != nullptr) {
        m_cexTrace.emplace_back(pair<Cube, Cube>(state->inputs, state->latches));
        state = state->preState;
    }
    m_cexTrace.emplace_back(pair<Cube, Cube>(state->inputs, state->latches));

    LOG_L(m_log, 3, "Build CEX Trace:");
    // simulate the concrete execution
    auto slv = make_shared<SATSolver>(m_model, MCSATSolver::minicore);
    slv->AddTrans();
    slv->AddConstraints();
    for (int i = 0; i < m_cexTrace.size() - 1; i++) {
        LOG_L(m_log, 3, "Inputs: ", CubeToStr(m_cexTrace[i].first));
        LOG_L(m_log, 3, "Latches: ", CubeToStr(m_cexTrace[i].second));
        Cube assumption = m_cexTrace[i].first;
        assumption.insert(assumption.end(),
                          m_cexTrace[i].second.begin(), m_cexTrace[i].second.end());
        bool sat = slv->Solve(assumption);
        assert(sat);
        auto p = slv->GetAssignment(true);
        bool included = includes(p.second.begin(), p.second.end(),
                                 m_cexTrace[i + 1].second.begin(),
                                 m_cexTrace[i + 1].second.end());
        assert(included);
        m_cexTrace[i + 1].second = p.second;
    }
    LOG_L(m_log, 3, "Inputs: ", CubeToStr(m_cexTrace.back().first));
    LOG_L(m_log, 3, "Latches: ", CubeToStr(m_cexTrace.back().second));
}


std::vector<std::pair<Cube, Cube>> FCAR::GetCexTrace() {
    assert(m_checkResult == CheckResult::Unsafe);
    if (m_cexTrace.empty()) BuildCEXTrace();

    return m_cexTrace;
}


FrameList FCAR::GetInv() {
    FrameList inv;
    if (!m_overSequence) return inv;
    int lvl = m_overSequence->GetInvariantLevel();
    if (lvl < 0) return inv;
    for (int i = m_searchFromInitSucc ? 1 : 0; i <= lvl; ++i) {
        inv.emplace_back(m_overSequence->FrameSetToFrame(*m_overSequence->GetFrame(i)));
    }
    LOG_L(m_log, 3, "Get Invariant:\n", m_overSequence->FramesDetail());
    return inv;
}


void FCAR::KLiveIncr() {
    int k_step = m_model.KLivenessIncrement();
    vector<Clause> k_clauses = m_model.GetKLiveClauses(k_step);

    // add trans
    for (auto slv : m_transSolvers) {
        for (auto cls : k_clauses) slv->AddClause(cls);
    }
    for (auto cls : k_clauses) m_liftSolver->AddClause(cls);
    for (auto cls : k_clauses) m_badLiftSolver->AddClause(cls);

    // add init
    m_transSolvers[0]->AddClause({~m_model.GetKLiveSignal(k_step)});

    // start solver will be reset
}


bool FCAR::IsInitStateImplyBad() {
    // init -> bad
    if (m_customInit.empty()) return false;
    auto slv = make_shared<SATSolver>(m_model, m_settings.solver);
    slv->AddTrans();
    slv->AddConstraints();
    Cube assumptions = m_customInit;
    assumptions.push_back(~m_model.GetBad());
    bool sat = slv->Solve(assumptions);
    return !sat;
}


// ================================================================================
// @brief: add the Cube as and gates to the aiger model
// @input:
// @output:
// ================================================================================
unsigned FCAR::AddCubeToAndGates(aiger *circuit, vector<unsigned> cb) {
    assert(cb.size() > 0);
    unsigned res = cb[0];
    assert(res / 2 <= circuit->maxvar);
    for (unsigned i = 1; i < cb.size(); i++) {
        assert(cb[i] / 2 <= circuit->maxvar);
        unsigned new_gate = (circuit->maxvar + 1) * 2;
        aiger_add_and(circuit, new_gate, res, cb[i]);
        res = new_gate;
    }
    return res;
}


void FCAR::OutputWitness() {
    // get outputfile
    auto start_index = m_settings.aigFilePath.find_last_of("/");
    if (start_index == string::npos) {
        start_index = 0;
    } else {
        start_index++;
    }
    auto end_index = m_settings.aigFilePath.find_last_of(".");
    assert(end_index != string::npos);
    string aig_name = m_settings.aigFilePath.substr(start_index, end_index - start_index);
    string out_path = m_settings.witnessOutputDir + aig_name + ".w.aig";
    aiger *model_aig = m_model.GetAiger().get();

    if (m_overSequence->GetInvariantLevel() < 0 && m_model.GetEquivalenceMap().size() == 0) {
        aiger_open_and_write_to_file(model_aig, out_path.c_str());
        return;
    }

    shared_ptr<aiger> witness_aig_ptr(aiger_init(), AigerDeleter);
    aiger *witness_aig = witness_aig_ptr.get();
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

    // add equivalence
    // (l1 <-> l2) & (l1 <-> l3) & ( ... ) & l_true
    // ! ( !l1 & l2 ) & ! ( l1 & !l2 )
    auto &eq_map = m_model.GetEquivalenceMap();
    vector<unsigned> eq_lits;
    for (auto itr = eq_map.begin(); itr != eq_map.end(); itr++) {
        if (IsConst(itr->second)) {
            unsigned true_eq_lit = ToAigerLit(itr->second);
            eq_lits.emplace_back(true_eq_lit);
            continue;
        }
        assert(itr->first <= witness_aig->maxvar);
        assert(VarOf(itr->second) <= witness_aig->maxvar);
        unsigned l1 = ToAigerLit(MkLit(itr->first));
        unsigned l2 = ToAigerLit(itr->second);
        eq_lits.emplace_back(AddCubeToAndGates(witness_aig, {l1, l2 ^ 1}) ^ 1);
        eq_lits.emplace_back(AddCubeToAndGates(witness_aig, {l1 ^ 1, l2}) ^ 1);
    }

    unsigned eq_cons;
    if (eq_lits.size() > 0) {
        eq_cons = AddCubeToAndGates(witness_aig, eq_lits);
    }

    // prove on lvl 0
    if (m_overSequence == nullptr || m_overSequence->GetInvariantLevel() < 0) {
        unsigned bad_lit = ToAigerLit(m_model.GetBad());
        unsigned p = aiger_not(bad_lit);
        unsigned p_prime = p;
        if (eq_lits.size() > 0) {
            p_prime = AddCubeToAndGates(witness_aig, {p, eq_cons});
        }

        if (model_aig->num_bad == 1) {
            aiger_add_bad(witness_aig, aiger_not(p_prime), model_aig->bad[0].name);
        } else if (model_aig->num_outputs == 1) {
            aiger_add_output(witness_aig, aiger_not(p_prime), model_aig->outputs[0].name);
        } else {
            assert(false);
        }

        aiger_reencode(witness_aig);
        aiger_open_and_write_to_file(witness_aig, out_path.c_str());
        return;
    }
    unsigned lvl_i = m_overSequence->GetInvariantLevel();

    // P' = P & invariant
    // P' = !bad & ( O_0 | O_1 | ... | O_i )
    //             !( !O_0 & !O_1 & ...  & !O_i )
    //                 O_i = c_1 & c_2 & ... & c_j
    //                       c_j = !( l_1 & l_2 & .. & l_k )

    vector<unsigned> inv_lits;
    for (unsigned i = 0; i <= lvl_i; i++) {
        shared_ptr<OverSequenceSet::FrameSet> frame_i = m_overSequence->GetFrame(i);
        vector<unsigned> frame_i_lits;
        for (const Cube &frame_cube : *frame_i) {
            vector<unsigned> cube_j;
            for (Lit l : frame_cube) cube_j.push_back(ToAigerLit(l));
            unsigned c_j = AddCubeToAndGates(witness_aig, cube_j) ^ 1;
            frame_i_lits.push_back(c_j);
        }
        unsigned o_i = AddCubeToAndGates(witness_aig, frame_i_lits);
        inv_lits.push_back(o_i ^ 1);
    }
    unsigned inv = AddCubeToAndGates(witness_aig, inv_lits) ^ 1;
    unsigned bad_lit = ToAigerLit(m_model.GetBad());
    unsigned p = aiger_not(bad_lit);
    unsigned p_prime = AddCubeToAndGates(witness_aig, {p, inv});

    if (eq_lits.size() > 0) {
        p_prime = AddCubeToAndGates(witness_aig, {p_prime, eq_cons});
    }

    if (model_aig->num_bad == 1) {
        aiger_add_bad(witness_aig, aiger_not(p_prime), model_aig->bad[0].name);
    } else if (model_aig->num_outputs == 1) {
        aiger_add_output(witness_aig, aiger_not(p_prime), model_aig->outputs[0].name);
    } else {
        assert(false);
    }

    aiger_reencode(witness_aig);
    aiger_open_and_write_to_file(witness_aig, out_path.c_str());
}


void FCAR::OutputCounterExample() {
    // get outputfile
    auto start_index = m_settings.aigFilePath.find_last_of("/\\");
    if (start_index == string::npos) {
        start_index = 0;
    } else {
        start_index++;
    }
    auto end_index = m_settings.aigFilePath.find_last_of(".");
    assert(end_index != string::npos);
    string aig_name = m_settings.aigFilePath.substr(start_index, end_index - start_index);
    string cex_path = m_settings.witnessOutputDir + aig_name + ".cex";
    std::ofstream cex_file;
    cex_file.open(cex_path);

    assert(m_lastState != nullptr);

    cex_file << "1" << endl
             << "b0" << endl;

    shared_ptr<State> state = m_lastState;
    cex_file << state->GetLatchesString() << endl;
    cex_file << state->GetInputsString() << endl;
    while (state->preState != nullptr) {
        state = state->preState;
        cex_file << state->GetInputsString() << endl;
    }

    cex_file << "." << endl;
    cex_file.close();
}
} // namespace car
