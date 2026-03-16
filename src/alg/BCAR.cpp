#include "BCAR.h"
#include <stack>
#include <string>

namespace car {
BCAR::BCAR(Settings settings,
           Model &model,
           Log &log) : m_settings(settings),
                       m_model(model),
                       m_log(log),
                       m_innOrder(model) {
    State::num_inputs = model.GetNumInputs();
    State::num_latches = model.GetNumLatches();
    m_lastState = nullptr;
    global_log = &m_log;
}

CheckResult BCAR::Run() {
    signal(SIGINT, SignalHandler);

    if (Check())
        m_checkResult = CheckResult::Safe;
    else
        m_checkResult = CheckResult::Unsafe;

    m_log.PrintCustomStatistics();

    return m_checkResult;
}

void BCAR::Witness() {
    if (m_checkResult == CheckResult::Safe) {
        OutputWitness();
    } else if (m_checkResult == CheckResult::Unsafe) {
        OutputCounterExample();
    }
}


std::vector<std::pair<Cube, Cube>> BCAR::GetCexTrace() {
    assert(m_checkResult == CheckResult::Unsafe);
    if (m_cexTrace.empty()) BuildCEXTrace();

    return m_cexTrace;
}


void BCAR::BuildCEXTrace() {
    assert(m_checkResult == CheckResult::Unsafe);
    assert(m_lastState != nullptr);
    m_cexTrace.clear();

    vector<shared_ptr<State>> trace;
    shared_ptr<State> state = m_lastState;
    while (state != nullptr) {
        trace.push_back(state);
        state = state->preState;
    }
    reverse(trace.begin(), trace.end());
    for (int i = 0; i < trace.size() - 1; ++i) {
        m_cexTrace.emplace_back(pair<Cube, Cube>(trace[i + 1]->inputs, trace[i]->latches));
    }
}


FrameList BCAR::GetInv() {
    FrameList inv;
    if (!m_overSequence) return inv;
    int lvl = m_overSequence->GetInvariantLevel();
    if (lvl < 0) return inv;
    for (int i = 0; i <= lvl; ++i) {
        inv.emplace_back(m_overSequence->FrameSetToFrame(*m_overSequence->GetFrame(i)));
    }
    return inv;
}

void BCAR::KLiveIncr() {
    int k_step = m_model.KLivenessIncrement();
    vector<Clause> k_clauses = m_model.GetKLiveClauses(k_step);

    // add trans
    for (auto slv : m_transSolvers) {
        for (auto cls : k_clauses) slv->AddClause(cls);
    }

    ResetBadSolver();
    // start solver will be reset
}


void BCAR::ResetBadSolver() {
    // reset bad solver

    // bad & T & c
    m_badSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    m_badSolver->AddTrans();
    m_badSolver->AddConstraints();
    if (m_loopRefuting) {
        for (auto l : m_customInit)
            m_badSolver->AddClause({l});
    } else {
        m_badSolver->AddBad();
    }
    // liveness: T = T & (W <-> W')
    //           T = T & C'
    //           T = T & !d'
    m_badSolver->AddWallConstraints(m_walls);
    for (auto inv : m_shoals) m_badSolver->AddInvAsClauseK(inv, false, 1);
    for (auto d : m_dead) m_badSolver->AddCubeAsClauseK(d, true, 1);
}


bool BCAR::Check() {
    [[maybe_unused]] auto check_scope = m_log.Section("FC_Check");

    if (!m_initialized) {
        Init();
        LOG_L(m_log, 2, "Initialized");
    } else {
        Reset();
        LOG_L(m_log, 2, "Reset");
    }

    if (!CheckBad(make_shared<State>(nullptr, Cube{}, Cube{}, 0))) {
        LOG_L(m_log, 1, "Bad State not reachable");
        m_overSequence->SetInvariantLevel(-1);
        return true;
    }

    if (ImmediateSatisfiable()) {
        LOG_L(m_log, 1, "Immediate Satisfiable");
        return false;
    }

    // main stage
    stack<Task> working_stack;
    while (true) {
        [[maybe_unused]] auto frame_scope = m_log.Section("FC_Frame");
        m_minUpdateLevel = m_k + 1;
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
                    while (!working_stack.empty()) working_stack.pop();
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

                if (task.frameLevel == -1) {
                    if (CheckBad(task.state))
                        return false;
                    else
                        continue;
                }

                Cube assumption(task.state->latches);
                OrderAssumption(assumption);
                LOG_L(m_log, 2, "SAT Check on frame: ", task.frameLevel);
                LOG_L(m_log, 3, "From state: ", CubeToStr(task.state->latches));
                bool result = IsReachable(task.frameLevel, assumption, "SAT_BC_Main");
                if (result) {
                    // Solver return SAT, get a new State, then continue
                    LOG_L(m_log, 2, "Result >>> SAT <<<");
                    auto p = m_transSolvers[task.frameLevel]->GetAssignment(true);
                    auto new_state =
                        make_shared<State>(task.state, p.first, p.second, task.state->depth + 1);
                    LOG_L(m_log, 3, "Get state: ", CubeToStr(new_state->latches));
                    m_underSequence.Push(new_state);
                    if (m_settings.dt) task.state->HasSucc();
                    int new_frame_level = GetNewLevel(new_state->latches);
                    working_stack.emplace(new_state, new_frame_level, true);
                } else {
                    // Solver return UNSAT, get uc, then continue
                    LOG_L(m_log, 2, "Result >>> UNSAT <<<");
                    auto uc = GetUnsatAssumption(m_transSolvers[task.frameLevel], task.state->latches);
                    LOG_L(m_log, 3, "Get UC:", CubeToStr(uc));
                    if (Generalize(uc, task.frameLevel))
                        m_branching->Update(uc);
                    LOG_L(m_log, 2, "Get Generalized UC:", CubeToStr(uc));
                    AddUnsatisfiableCore(uc, task.frameLevel + 1);
                    if (m_settings.dt) task.state->HasUC();
                    task.frameLevel = PropagateUp(uc, task.frameLevel + 1);
                    LOG_L(m_log, 3, m_overSequence->FramesInfo());
                }
            }
            start_state = EnumerateStartState();
        }

        // inv
        m_invSolver = make_shared<SATSolver>(m_model, MCSATSolver::cadical);
        IsInvariant(0);
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
                LOG_L(m_log, 1, "Proof at frame ", i + 1);
                LOG_L(m_log, 1, m_overSequence->FramesInfo());
                m_overSequence->SetInvariantLevel(i);
                OverSequenceRefine(i);
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


void BCAR::Init() {
    [[maybe_unused]] auto init_scope = m_log.Section("FC_Init");

    m_k = 0;

    m_initStateImplyBad = IsInitStateImplyBad();
    if (!m_initStateImplyBad)
        LOG_L(m_log, 1, "Initial state does not imply bad");

    m_overSequence = make_shared<OverSequenceSet>(m_model);
    m_underSequence = UnderSequence();
    m_branching = make_shared<Branching>(m_settings.branching);
    m_litOrder.branching = m_branching;
    m_blockerOrder.branching = m_branching;
    m_transSolvers.clear();
    m_lastState = nullptr;

    // s & T & c & O_i'
    m_transSolvers.emplace_back(make_shared<SATSolver>(m_model, m_settings.solver));
    m_transSolvers[0]->AddTrans();
    m_transSolvers[0]->AddConstraints();
    // liveness: T = T & (W <-> W')
    //           T = T & C'
    //           T = T & !d'
    if (m_loopRefuting) m_transSolvers[0]->AddWallConstraints(m_walls);
    for (auto inv : m_shoals) m_transSolvers[0]->AddInvAsClauseK(inv, false, 1);
    for (auto d : m_dead) m_transSolvers[0]->AddCubeAsClauseK(d, true, 1);

    InitializeStartSolver();
    ResetBadSolver();

    m_restart.reset(new Restart(m_settings));

    m_initialized = true;
}


void BCAR::InitializeStartSolver() {
    // I & T & c
    m_startSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    m_startSolver->AddTrans();
    m_startSolver->AddConstraints();
    if (!m_customInit.empty()) {
        for (int lit : m_customInit) {
            m_startSolver->AddClause({lit});
        }
        if (!m_initStateImplyBad) m_startSolver->AddBad();
    } else {
        for (auto l : m_model.GetInitialState()) {
            m_startSolver->AddClause({l});
        }
        m_startSolver->AddInitialClauses();
    }
    // liveness: T = T & (W <-> W')
    //           T = T & C'
    //           T = T & !d'
    if (m_loopRefuting) m_startSolver->AddWallConstraints(m_walls);
    for (auto inv : m_shoals) m_startSolver->AddInvAsClauseK(inv, false, 1);
    for (auto d : m_dead) m_startSolver->AddCubeAsClauseK(d, true, 1);
}


void BCAR::Reset() {
    m_underSequence.Clear();
    m_lastState = nullptr;
    m_cexTrace.clear();

    InitializeStartSolver();
    // add exist lemma
    auto frame_i = m_overSequence->GetFrame(m_k);
    for (auto l : *frame_i) {
        Cube pl(l);
        GetPrimed(pl);
        m_startSolver->AddUC(pl);
    }

    ResetBadSolver();
}


bool BCAR::IsInitStateImplyBad() {
    if (m_customInit.empty()) return false;
    auto slv = make_shared<SATSolver>(m_model, m_settings.solver);
    slv->AddTrans();
    slv->AddConstraints();
    Cube assumptions = m_customInit;
    assumptions.push_back(m_model.GetBad());
    bool sat = slv->Solve(assumptions);
    return !sat;
}


bool BCAR::AddUnsatisfiableCore(const Cube &uc, int frameLevel) {
    [[maybe_unused]] auto scoped = m_log.Section("DS_AddUC");
    m_restart->UcCountsPlus1();

    if (!m_overSequence->Insert(uc, frameLevel)) return false;

    Cube puc(uc);
    GetPrimed(puc);
    if (frameLevel >= m_transSolvers.size()) {
        m_transSolvers.emplace_back(make_shared<SATSolver>(m_model, m_settings.solver));
        m_transSolvers.back()->AddTrans();
        m_transSolvers.back()->AddConstraints();
        if (m_settings.solveInProperty) m_transSolvers.back()->AddProperty();
        // liveness: T = T & (W <-> W')
        //           T = T & C'
        //           T = T & !d'
        if (m_loopRefuting) m_transSolvers.back()->AddWallConstraints(m_walls);
        for (auto inv : m_shoals) m_transSolvers.back()->AddInvAsClauseK(inv, false, 1);
        for (auto d : m_dead) m_transSolvers.back()->AddCubeAsClauseK(d, true, 1);
    }
    m_transSolvers[frameLevel]->AddUC(puc);

    if (frameLevel >= m_k) {
        m_startSolver->AddUC(puc);
    }
    if (frameLevel < m_minUpdateLevel) {
        m_minUpdateLevel = frameLevel;
    }

    return true;
}


bool BCAR::ImmediateSatisfiable() {
    [[maybe_unused]] auto imm_scope = m_log.Section("FC_ImmSAT");
    if (!m_customInit.empty()) {
        return false;
    }

    Cube assumptions;
    assumptions.push_back(m_model.GetBad());
    bool result = false;
    {
        [[maybe_unused]] auto sat_scope = m_log.Section("SAT_BC_Imm");
        result = m_startSolver->Solve(assumptions);
    }
    if (result) {
        // initial state
        auto init = m_startSolver->GetAssignment(false);
        auto init_state = make_shared<State>(nullptr, Cube{}, init.second, 0);
        // start state is a successor of initial state
        auto p = m_startSolver->GetAssignment(true);
        auto start_state = make_shared<State>(init_state, p.first, p.second, 1);
        m_lastState = start_state;
    }
    m_startSolver->ClearAssumption();
    return result;
}


shared_ptr<State> BCAR::EnumerateStartState() {
    [[maybe_unused]] auto scoped = m_log.Section("FC_StartEnum");
    {
        [[maybe_unused]] auto sat_scope = m_log.Section("SAT_BC_Start");
        if (!m_startSolver->Solve()) return nullptr;
    }

    // initial state
    auto init = m_startSolver->GetAssignment(false);
    auto init_state = make_shared<State>(nullptr, Cube{}, init.second, 0);
    // start state is a successor of initial state
    auto p = m_startSolver->GetAssignment(true);
    auto start_state = make_shared<State>(init_state, p.first, p.second, 1);
    return start_state;
}


void BCAR::OverSequenceRefine(int lvl) {
    [[maybe_unused]] auto refine_scope = m_log.Section("FC_Refine");
    // sometimes we have I & T & c & (O_0 | O_1 | ... | O_i+1)' is UNSAT,
    // but to output a correct witness,
    // we need I & c & (O_0 | O_1 | ... | O_i) is UNSAT.
    //         I & c & !P   is UNSAT.
    // get model s of I & c & (O_0 | O_1 | ... | O_i),
    // assert( s & T & c & (O_0 | O_1 | ... | O_i+1)' is UNSAT)
    // add uc to O_0 ... O_i

    if (m_loopRefuting) return;

    // solver: I & c & (O_0 | O_1 | ... | O_i)
    shared_ptr<SATSolver> refine_solver = make_shared<SATSolver>(m_model, m_settings.solver);
    refine_solver->AddConstraints();
    if (!m_customInit.empty()) {
        for (int lit : m_customInit) refine_solver->AddClause({lit});
    } else {
        for (auto l : m_model.GetInitialState()) refine_solver->AddClause({l});
        refine_solver->AddInitialClauses();
    }
    refine_solver->AddInvAsClauseK(GetInv(), false, 0);

    // if I & c & (O_0 | O_1 | ... | O_i) is SAT, get model s
    while (true) {
        bool refine_sat = false;
        {
            [[maybe_unused]] auto sat_scope = m_log.Section("SAT_BC_Refine");
            refine_sat = refine_solver->Solve();
        }
        if (!refine_sat) break;
        LOG_L(m_log, 1, "Refine OverSequence");
        auto s = refine_solver->GetAssignment(false);

        bool sat = IsReachable(lvl + 1, s.second, "SAT_BC_RefineReach");
        assert(!sat);
        auto uc = GetUnsatAssumption(m_transSolvers[lvl + 1], s.second);
        for (int i = 0; i <= lvl; i++)
            AddUnsatisfiableCore(uc, i);
        Clause cls;
        for (auto ci : uc) cls.emplace_back(-ci);
        refine_solver->AddClause(cls);
    }
}


int BCAR::GetNewLevel(const Cube &states, int start) {
    [[maybe_unused]] auto scoped = m_log.Section("FC_GetNewLevel");

    for (int i = start; i <= m_k; i++) {
        if (!m_overSequence->IsBlockedByFrame(states, i)) {
            return i - 1;
        }
    }

    return m_k;
}


bool BCAR::IsInvariant(int frameLevel) {
    [[maybe_unused]] auto inv_scope = m_log.Section("FC_Invariant");
    shared_ptr<OverSequenceSet::FrameSet> frame_i = m_overSequence->GetFrame(frameLevel);

    if (frameLevel < m_minUpdateLevel) {
        AddConstraintOr(frame_i);
        return false;
    }

    AddConstraintAnd(frame_i);
    bool result = false;
    {
        [[maybe_unused]] auto sat_scope = m_log.Section("SAT_BC_Inv");
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
void BCAR::AddConstraintOr(const shared_ptr<OverSequenceSet::FrameSet> f) {
    Cube cls;
    for (const Cube &frame_cube : *f) {
        int flag = m_invSolver->GetNewVar();
        cls.push_back(flag);
        for (int i = 0; i < frame_cube.size(); i++) {
            m_invSolver->AddClause(Cube{-flag, frame_cube[i]});
        }
    }
    m_invSolver->AddClause(cls);
}


// ================================================================================
// @brief: add constraint & !O_i
// @input:
// @output:
// ================================================================================
void BCAR::AddConstraintAnd(const shared_ptr<OverSequenceSet::FrameSet> f) {
    int flag = m_invSolver->GetNewVar();
    for (const Cube &frame_cube : *f) {
        Cube cls;
        for (int i = 0; i < frame_cube.size(); i++) {
            cls.push_back(-frame_cube[i]);
        }
        cls.push_back(-flag);
        m_invSolver->AddClause(cls);
    }
    m_invSolver->AddAssumption(Cube{flag});
}


// ================================================================================
// @brief: counter-example to generalization
// @input:
// @output:
// ================================================================================
bool BCAR::Generalize(Cube &uc, int frameLvl, int recLvl) {
    [[maybe_unused]] auto gen_scope = m_log.Section("FC_Gen");
    unordered_set<int> required_lits;

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
    for (int i = uc.size() - 1; i > 0; i--) {
        if (required_lits.find(uc[i]) != required_lits.end()) continue;
        Cube temp_uc;
        temp_uc.reserve(uc.size());
        for (auto ll : uc)
            if (ll != uc[i]) temp_uc.emplace_back(ll);
        if (Down(temp_uc, frameLvl, recLvl, failed_ctses)) {
            uc.swap(temp_uc);
            i = uc.size();
            attempts = m_settings.ctgMaxAttempts;
        } else {
            if (--attempts == 0) break;
            required_lits.emplace(uc[i]);
        }
    }
    sort(uc.begin(), uc.end(), Cmp);
    if (uc.size() > uc_blocker.size() && frameLvl != 0) {
        return false;
    } else
        return true;
}


bool BCAR::Down(Cube &uc, int frameLvl, int recLvl, vector<Cube> &failedCtses) {
    [[maybe_unused]] auto down_scope = m_log.Section("FC_Dn");
    int ctgs = 0;
    Cube assumption(uc);
    while (true) {
        // F_i & T & temp_uc'
        bool reachable = IsReachable(frameLvl, assumption, "SAT_BC_Down");
        if (!reachable) {
            uc = GetUnsatAssumption(m_transSolvers[frameLvl], uc);
            return true;
        }

        if (recLvl >= m_settings.ctgMaxRecursionDepth ||
            ctgs >= m_settings.ctgMaxStates ||
            frameLvl < 1) {
            return false;
        }

        auto p = m_transSolvers[frameLvl]->GetAssignment(false);
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


bool BCAR::CTSBlock(shared_ptr<State> cts, int frameLvl, int recLvl, vector<Cube> &failedCtses, int ctsCount) {
    if (ctsCount >= m_settings.ctgMaxBlocks) return false;

    LOG_L(m_log, 3, "Try cts:", CubeToStr(cts->latches));
    Cube cts_ass(cts->latches);
    OrderAssumption(cts_ass);

    while (true) {
        if (!IsReachable(frameLvl, cts_ass, "SAT_R_CTS_B")) {
            auto uc_cts = GetUnsatAssumption(m_transSolvers[frameLvl], cts->latches);
            LOG_L(m_log, 3, "CTG Get UC:", CubeToStr(uc_cts));
            if (Generalize(uc_cts, frameLvl, recLvl + 1)) m_branching->Update(uc_cts);
            LOG_L(m_log, 3, "CTG Get Generalized UC:", CubeToStr(uc_cts));
            AddUnsatisfiableCore(uc_cts, frameLvl + 1);
            PropagateUp(uc_cts, frameLvl + 1);
            return true;
        } else {
            auto p = m_transSolvers[frameLvl]->GetAssignment(false);
            shared_ptr<State> post_cts(new State(nullptr, p.first, p.second, 0));
            if (DownHasFailed(post_cts->latches, failedCtses)) return false;

            int pre_cts_lvl = frameLvl - 1;
            if (pre_cts_lvl < 0 ||
                !CTSBlock(post_cts, pre_cts_lvl, recLvl, failedCtses, ctsCount + 1)) {

                failedCtses.emplace_back(cts->latches);
                return false;
            }
        }
    }
}


bool BCAR::DownHasFailed(const Cube &s, const vector<Cube> &failedCtses) {
    for (auto f : failedCtses) {
        // if f->s , return true
        if (f.size() > s.size()) continue;
        if (includes(s.begin(), s.end(), f.begin(), f.end(), Cmp)) return true;
    }
    return false;
}


bool BCAR::CheckBad(shared_ptr<State> s) {
    [[maybe_unused]] auto check_scope = m_log.Section("FC_CheckBad");

    LOG_L(m_log, 2, "\nSAT CHECK on bad");
    LOG_L(m_log, 3, "From state: ", CubeToStr(s->latches));
    Cube assumption(s->latches);
    OrderAssumption(assumption);
    bool result = false;
    {
        [[maybe_unused]] auto sat_scope = m_log.Section("SAT_BC_Bad");
        result = m_badSolver->Solve(assumption);
    }
    if (result) {
        LOG_L(m_log, 2, "Result >>> SAT <<<");
        auto p = m_badSolver->GetAssignment(false);
        LOG_L(m_log, 3, "Get Assignment:", CubeToStr(p.second));
        shared_ptr<State> new_state(new State(s, p.first, p.second, s->depth + 1));
        m_lastState = new_state;
        LOG_L(m_log, 3, m_overSequence->FramesInfo());
        return true;
    } else {
        LOG_L(m_log, 2, "Result >>> UNSAT <<<");
        auto uc = GetUnsatAssumption(m_badSolver, assumption);
        // Generalization
        unordered_set<int> required_lits;
        for (int i = uc.size() - 1; i >= 0; i--) {
            if (uc.size() < 3) break;
            if (required_lits.find(uc[i]) != required_lits.end()) continue;
            Cube temp_uc;
            temp_uc.reserve(uc.size());
            for (auto ll : uc)
                if (ll != uc[i]) temp_uc.emplace_back(ll);

            bool result = false;
            {
                [[maybe_unused]] auto sat_gen_scope = m_log.Section("SAT_BC_BadGen");
                result = m_badSolver->Solve(temp_uc);
            }
            if (!result) {
                auto new_uc = GetUnsatAssumption(m_badSolver, temp_uc);
                uc.swap(new_uc);
                i = uc.size();
            } else {
                required_lits.emplace(uc[i]);
            }
        }
        sort(uc.begin(), uc.end(), Cmp);
        LOG_L(m_log, 2, "Get UC:", CubeToStr(uc));
        m_branching->Update(uc);
        AddUnsatisfiableCore(uc, 0);
        PropagateUp(uc, 0);
        return false;
    }
}


bool BCAR::IsReachable(int lvl, const Cube &assumption, const string &label) {
    [[maybe_unused]] auto scoped = m_log.Section(label);
    return m_transSolvers[lvl]->Solve(assumption);
}


Cube BCAR::GetUnsatAssumption(shared_ptr<SATSolver> solver, const Cube &assumptions) {
    const unordered_set<int> &conflict = solver->GetConflict();
    Cube res;

    for (auto a : assumptions) {
        if (conflict.find(a) != conflict.end())
            res.emplace_back(a);
    }

    sort(res.begin(), res.end(), Cmp);
    return res;
}


// ================================================================================
// @brief: add the Cube as and gates to the aiger model
// @input:
// @output:
// ================================================================================
unsigned BCAR::AddCubeToAndGates(aiger *circuit, vector<unsigned> cb) {
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


bool BCAR::Propagate(const Cube &c, int lvl) {
    [[maybe_unused]] auto prop_scope = m_log.Section("FC_Prop");
    bool result = !IsReachable(lvl, c, "SAT_BC_Prop");
    if (result) {
        auto uc = GetUnsatAssumption(m_transSolvers[lvl], c);
        AddUnsatisfiableCore(uc, lvl + 1);
    }
    return result;
}


int BCAR::PropagateUp(const Cube &c, int lvl) {
    while (lvl < m_k) {
        if (Propagate(c, lvl))
            m_branching->Update(c);
        else
            break;
        lvl++;
    }
    return lvl;
}


void BCAR::OutputWitness() {
    [[maybe_unused]] auto wit_scope = m_log.Section("FC_Witness");
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
    // (l1 <-> l2) & (l1 <-> l3) & ( ... )
    auto &eq_map = m_model.GetEquivalenceMap();
    vector<unsigned> eq_lits;
    for (auto itr = eq_map.begin(); itr != eq_map.end(); itr++) {
        if (itr->first == m_model.TrueId() || itr->second == m_model.TrueId()) {
            unsigned true_eq_lit = m_model.GetAigerLit(itr->second);
            eq_lits.emplace_back(true_eq_lit);
            continue;
        }
        assert(abs(itr->first) < witness_aig->maxvar);
        assert(abs(itr->second) < witness_aig->maxvar);
        unsigned l1 = m_model.GetAigerLit(itr->first);
        unsigned l2 = m_model.GetAigerLit(itr->second);
        eq_lits.emplace_back(AddCubeToAndGates(witness_aig, {l1, l2 ^ 1}) ^ 1);
        eq_lits.emplace_back(AddCubeToAndGates(witness_aig, {l1 ^ 1, l2}) ^ 1);
    }

    unsigned eq_cons;
    if (eq_lits.size() > 0) {
        eq_cons = AddCubeToAndGates(witness_aig, eq_lits);
    }

    // prove on lvl 0
    if (m_overSequence == nullptr || m_overSequence->GetInvariantLevel() < 0) {
        unsigned bad_lit = m_model.GetAigerLit(m_model.GetBad());
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
    // P' = !bad & !( O_0 | O_1 | ... | O_i )
    //             ( !O_0 & !O_1 & ...  & !O_i )
    //                 O_i = c_1 & c_2 & ... & c_j
    //                       c_j = !( l_1 & l_2 & .. & l_k )

    vector<unsigned> inv_lits;
    for (unsigned i = 0; i <= lvl_i; i++) {
        shared_ptr<OverSequenceSet::FrameSet> frame_i = m_overSequence->GetFrame(i);
        vector<unsigned> frame_i_lits;
        for (const Cube &frame_cube : *frame_i) {
            vector<unsigned> cube_j;
            for (int l : frame_cube) cube_j.push_back(l > 0 ? (2 * l) : (2 * -l + 1));
            unsigned c_j = AddCubeToAndGates(witness_aig, cube_j) ^ 1;
            frame_i_lits.push_back(c_j);
        }
        unsigned o_i = AddCubeToAndGates(witness_aig, frame_i_lits);
        inv_lits.push_back(o_i ^ 1);
    }
    unsigned inv = AddCubeToAndGates(witness_aig, inv_lits);
    unsigned bad_lit = m_model.GetAigerLit(m_model.GetBad());
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


void BCAR::OutputCounterExample() {
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

    vector<shared_ptr<State>> trace;
    shared_ptr<State> state = m_lastState;
    while (state != nullptr) {
        trace.push_back(state);
        state = state->preState;
    }
    reverse(trace.begin(), trace.end());
    cex_file << trace[0]->GetLatchesString() << endl;

    for (int i = 1; i < trace.size(); i++) {
        cex_file << trace[i]->GetInputsString() << endl;
    }
    cex_file << "." << endl;
    cex_file.close();
}
} // namespace car
