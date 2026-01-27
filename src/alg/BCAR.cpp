#include "BCAR.h"
#include <stack>
#include <string>

namespace car {
BCAR::BCAR(Settings settings,
           Model &model,
           Log &log) : m_settings(settings),
                       m_model(model),
                       m_log(log),
                       innOrder(model) {
    State::numInputs = model.GetNumInputs();
    State::numLatches = model.GetNumLatches();
    m_lastState = nullptr;
    GLOBAL_LOG = &m_log;
}

CheckResult BCAR::Run() {
    signal(SIGINT, signalHandler);

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


std::vector<std::pair<cube, cube>> BCAR::GetCexTrace() {
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
        m_cexTrace.emplace_back(pair<cube, cube>(trace[i + 1]->inputs, trace[i]->latches));
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
    vector<clause> k_clauses = m_model.GetKLiveClauses(k_step);

    for (auto &slv : m_transSolvers) {
        for (auto &cls : k_clauses) slv->AddClause(cls);
    }
    if (m_startSolver) {
        for (auto &cls : k_clauses) m_startSolver->AddClause(cls);
    }
    if (m_badSolver) {
        for (auto &cls : k_clauses) m_badSolver->AddClause(cls);
    }
    if (m_invSolver) {
        for (auto &cls : k_clauses) m_invSolver->AddClause(cls);
    }

    if (!m_transSolvers.empty()) {
        m_transSolvers[0]->AddClause({-m_model.GetKLiveSignal(k_step)});
    }
    if (m_startSolver) {
        m_startSolver->AddClause({-m_model.GetKLiveSignal(k_step)});
    }
}

bool BCAR::Check() {
    [[maybe_unused]] auto checkScope = m_log.Section("FC_Check");

    if (!m_initialized) {
        Init();
        m_log.L(2, "Initialized");
    } else {
        Reset();
        m_log.L(2, "Reset");
    }

    if (!CheckBad(make_shared<State>(nullptr, cube{}, cube{}, 0))) {
        m_log.L(1, "Bad State not reachable");
        m_overSequence->SetInvariantLevel(-1);
        return true;
    }

    if (ImmediateSatisfiable()) {
        m_log.L(1, "Immediate Satisfiable");
        return false;
    }

    // main stage
    stack<Task> workingStack;
    while (true) {
        [[maybe_unused]] auto frameScope = m_log.Section("FC_Frame");
        m_minUpdateLevel = m_k + 1;
        if (m_settings.dt) { // Dynamic Traversal
            auto dtseq = m_underSequence.GetSeqDT();
            for (auto state : dtseq) {
                workingStack.emplace(state, m_k - 1, false);
            }
        } else { // from the shallow and the start
            for (int i = m_underSequence.size() - 1; i >= 0; i--) {
                for (int j = m_underSequence[i].size() - 1; j >= 0; j--) {
                    workingStack.emplace(m_underSequence[i][j], m_k - 1, false);
                }
            }
        }

        m_log.L(2, "Start Frame: ", m_k);
        m_log.L(2, "Working Stack Size: ", workingStack.size());

        shared_ptr<State> startState = EnumerateStartState();

        while (startState != nullptr) {
            m_log.L(2, "State from StartSolver: ", CubeToStrShort(startState->latches));
            m_log.L(3, "State Detail: ", CubeToStr(startState->latches));
            workingStack.emplace(startState, m_k - 1, true);

            while (!workingStack.empty()) {
                [[maybe_unused]] auto taskScope = m_log.Section("FC_Task");
                Task &task = workingStack.top();

                if (m_settings.restart && m_restart->RestartCheck()) {
                    m_log.L(1, "Restarting...");
                    m_underSequence = UnderSequence();
                    while (!workingStack.empty()) workingStack.pop();
                    m_restart->UpdateThreshold();
                    m_restart->ResetUcCounts();
                    continue;
                }

                if (!task.isLocated) {
                    task.frameLevel = GetNewLevel(task.state->latches, task.frameLevel + 1);
                    m_log.L(3, "State get new Level ", task.frameLevel);
                }

                if (task.frameLevel >= m_k) {
                    workingStack.pop();
                    continue;
                }

                task.isLocated = false;

                if (task.frameLevel == -1) {
                    if (CheckBad(task.state))
                        return false;
                    else
                        continue;
                }

                cube assumption(task.state->latches);
                OrderAssumption(assumption);
                m_log.L(2, "SAT Check on frame: ", task.frameLevel);
                m_log.L(3, "From state: ", CubeToStr(task.state->latches));
                bool result = IsReachable(task.frameLevel, assumption, "SAT_BC_Main");
                if (result) {
                    // Solver return SAT, get a new State, then continue
                    m_log.L(2, "Result >>> SAT <<<");
                    auto p = m_transSolvers[task.frameLevel]->GetAssignment(true);
                    auto newState =
                        make_shared<State>(task.state, p.first, p.second, task.state->depth + 1);
                    m_log.L(3, "Get state: ", CubeToStr(newState->latches));
                    m_underSequence.push(newState);
                    if (m_settings.dt) task.state->HasSucc();
                    int newFrameLevel = GetNewLevel(newState->latches);
                    workingStack.emplace(newState, newFrameLevel, true);
                } else {
                    // Solver return UNSAT, get uc, then continue
                    m_log.L(2, "Result >>> UNSAT <<<");
                    auto uc = GetUnsatAssumption(m_transSolvers[task.frameLevel], task.state->latches);
                    m_log.L(3, "Get UC:", CubeToStr(uc));
                    if (Generalize(uc, task.frameLevel))
                        m_branching->Update(uc);
                    m_log.L(2, "Get Generalized UC:", CubeToStr(uc));
                    AddUnsatisfiableCore(uc, task.frameLevel + 1);
                    if (m_settings.dt) task.state->HasUC();
                    task.frameLevel = PropagateUp(uc, task.frameLevel + 1);
                    m_log.L(3, m_overSequence->FramesInfo());
                }
            }
            startState = EnumerateStartState();
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
                for (const cube &uc : *fi) {
                    if (fi_plus_1->find(uc) != fi_plus_1->end()) continue; // propagated
                    if (Propagate(uc, i))
                        m_branching->Update(uc);
                }
            }

            // invariant check
            if (IsInvariant(i + 1)) {
                m_log.L(1, "Proof at frame ", i + 1);
                m_log.L(1, m_overSequence->FramesInfo());
                m_overSequence->SetInvariantLevel(i);
                OverSequenceRefine(i);
                return true;
            }
        }

        m_log.L(1, m_overSequence->FramesInfo());
        m_log.L(3, m_overSequence->FramesDetail());
        InitializeStartSolver();

        m_k++;
        m_restart->ResetUcCounts();
        m_log.L(2, "\nNew Frame Added");
    }
}


void BCAR::Init() {
    [[maybe_unused]] auto initScope = m_log.Section("FC_Init");

    m_k = 0;

    m_initStateImplyBad = IsInitStateImplyBad();
    if (!m_initStateImplyBad)
        m_log.L(1, "Initial state does not imply bad");

    m_overSequence = make_shared<OverSequenceSet>(m_model);
    m_underSequence = UnderSequence();
    m_branching = make_shared<Branching>(m_settings.branching);
    litOrder.branching = m_branching;
    blockerOrder.branching = m_branching;
    m_transSolvers.clear();
    m_lastState = nullptr;

    // s & T & c & O_i'
    m_transSolvers.emplace_back(make_shared<SATSolver>(m_model, m_settings.solver));
    m_transSolvers[0]->AddTrans();
    m_transSolvers[0]->AddConstraints();
    // liveness: T = T & (W <-> W')
    //           T = T & C'
    //           T = T & !d'
    m_transSolvers[0]->AddWallConstraints(m_walls);
    for (auto inv : m_shoals) m_transSolvers[0]->AddInvAsClauseK(inv, false, 1);
    for (auto d : m_dead) m_transSolvers[0]->AddCubeAsClauseK(d, true, 1);

    InitializeStartSolver();

    // bad & T & c
    m_badSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    m_badSolver->AddTrans();
    m_badSolver->AddConstraints();
    m_badSolver->AddBad();
    // liveness: T = T & (W <-> W')
    //           T = T & C'
    //           T = T & !d'
    m_badSolver->AddWallConstraints(m_walls);
    for (auto inv : m_shoals) m_badSolver->AddInvAsClauseK(inv, false, 1);
    for (auto d : m_dead) m_badSolver->AddCubeAsClauseK(d, true, 1);

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
    m_startSolver->AddWallConstraints(m_walls);
    for (auto inv : m_shoals) m_startSolver->AddInvAsClauseK(inv, false, 1);
    for (auto d : m_dead) m_startSolver->AddCubeAsClauseK(d, true, 1);
}


void BCAR::Reset() {
    m_underSequence.clear();
    m_lastState = nullptr;
    m_cexTrace.clear();

    // add exist lemma
    auto frame_i = m_overSequence->GetFrame(m_k);
    for (auto l : *frame_i) m_startSolver->AddUC(l);
}


bool BCAR::IsInitStateImplyBad() {
    if (m_customInit.empty()) return false;
    auto slv = make_shared<SATSolver>(m_model, m_settings.solver);
    slv->AddTrans();
    slv->AddConstraints();
    cube assumptions = m_customInit;
    assumptions.push_back(m_model.GetBad());
    bool sat = slv->Solve(assumptions);
    return !sat;
}


bool BCAR::AddUnsatisfiableCore(const cube &uc, int frameLevel) {
    [[maybe_unused]] auto scoped = m_log.Section("DS_AddUC");
    m_restart->UcCountsPlus1();

    if (!m_overSequence->Insert(uc, frameLevel)) return false;

    cube puc(uc);
    GetPrimed(puc);
    if (frameLevel >= m_transSolvers.size()) {
        m_transSolvers.emplace_back(make_shared<SATSolver>(m_model, m_settings.solver));
        m_transSolvers.back()->AddTrans();
        m_transSolvers.back()->AddConstraints();
        if (m_settings.solveInProperty) m_transSolvers.back()->AddProperty();
        // liveness: T = T & (W <-> W')
        //           T = T & C'
        //           T = T & !d'
        m_transSolvers.back()->AddWallConstraints(m_walls);
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
    [[maybe_unused]] auto immScope = m_log.Section("FC_ImmSAT");
    if (!m_customInit.empty()) {
        return false;
    }

    cube assumptions;
    assumptions.push_back(m_model.GetBad());
    bool result = false;
    {
        [[maybe_unused]] auto satScope = m_log.Section("SAT_BC_Imm");
        result = m_startSolver->Solve(assumptions);
    }
    if (result) {
        // initial state
        auto init = m_startSolver->GetAssignment(false);
        auto initState = make_shared<State>(nullptr, cube{}, init.second, 0);
        // start state is a successor of initial state
        auto p = m_startSolver->GetAssignment(true);
        auto startState = make_shared<State>(initState, p.first, p.second, 1);
        m_lastState = startState;
    }
    m_startSolver->ClearAssumption();
    return result;
}


shared_ptr<State> BCAR::EnumerateStartState() {
    [[maybe_unused]] auto scoped = m_log.Section("FC_StartEnum");
    {
        [[maybe_unused]] auto satScope = m_log.Section("SAT_BC_Start");
        if (!m_startSolver->Solve()) return nullptr;
    }

    // initial state
    auto init = m_startSolver->GetAssignment(false);
    auto initState = make_shared<State>(nullptr, cube{}, init.second, 0);
    // start state is a successor of initial state
    auto p = m_startSolver->GetAssignment(true);
    auto startState = make_shared<State>(initState, p.first, p.second, 1);
    return startState;
}


void BCAR::OverSequenceRefine(int lvl) {
    [[maybe_unused]] auto refineScope = m_log.Section("FC_Refine");
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
    if (!m_customInit.empty()) {
        for (int lit : m_customInit) refine_solver->AddClause({lit});
    } else {
        for (auto l : m_model.GetInitialState()) refine_solver->AddClause({l});
        refine_solver->AddInitialClauses();
    }
    refine_solver->AddInvAsClauseK(GetInv(), false, 0);

    // if I & c & (O_0 | O_1 | ... | O_i) is SAT, get model s
    while (true) {
        bool refineSat = false;
        {
            [[maybe_unused]] auto satScope = m_log.Section("SAT_BC_Refine");
            refineSat = refine_solver->Solve();
        }
        if (!refineSat) break;
        m_log.L(1, "Refine OverSequence");
        auto s = refine_solver->GetAssignment(false);

        bool sat = IsReachable(lvl + 1, s.second, "SAT_BC_RefineReach");
        assert(!sat);
        auto uc = GetUnsatAssumption(m_transSolvers[lvl + 1], s.second);
        for (int i = 0; i <= lvl; i++)
            AddUnsatisfiableCore(uc, i);
        clause cls;
        for (auto ci : uc) cls.emplace_back(-ci);
        refine_solver->AddClause(cls);
    }
}


int BCAR::GetNewLevel(const cube &states, int start) {
    [[maybe_unused]] auto scoped = m_log.Section("FC_GetNewLevel");

    for (int i = start; i <= m_k; i++) {
        if (!m_overSequence->IsBlockedByFrame(states, i)) {
            return i - 1;
        }
    }

    return m_k;
}


bool BCAR::IsInvariant(int frameLevel) {
    [[maybe_unused]] auto invScope = m_log.Section("FC_Invariant");
    shared_ptr<OverSequenceSet::FrameSet> frame_i = m_overSequence->GetFrame(frameLevel);

    if (frameLevel < m_minUpdateLevel) {
        AddConstraintOr(frame_i);
        return false;
    }

    AddConstraintAnd(frame_i);
    bool result = false;
    {
        [[maybe_unused]] auto satScope = m_log.Section("SAT_BC_Inv");
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
    cube cls;
    for (const cube &frame_cube : *f) {
        int flag = m_invSolver->GetNewVar();
        cls.push_back(flag);
        for (int i = 0; i < frame_cube.size(); i++) {
            m_invSolver->AddClause(cube{-flag, frame_cube[i]});
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
    for (const cube &frame_cube : *f) {
        cube cls;
        for (int i = 0; i < frame_cube.size(); i++) {
            cls.push_back(-frame_cube[i]);
        }
        cls.push_back(-flag);
        m_invSolver->AddClause(cls);
    }
    m_invSolver->AddAssumption(cube{flag});
}


// ================================================================================
// @brief: counter-example to generalization
// @input:
// @output:
// ================================================================================
bool BCAR::Generalize(cube &uc, int frame_lvl, int rec_lvl) {
    [[maybe_unused]] auto genScope = m_log.Section("FC_Gen");
    unordered_set<int> required_lits;

    vector<cube> uc_blockers;
    m_overSequence->GetBlockers(uc, frame_lvl, uc_blockers);
    cube uc_blocker;
    if (uc_blockers.size() > 0) {
        if (m_settings.branching > 0)
            stable_sort(uc_blockers.begin(), uc_blockers.end(), blockerOrder);
        uc_blocker = uc_blockers[0];
    }

    if (m_settings.referSkipping)
        for (auto b : uc_blocker) required_lits.emplace(b);
    vector<cube> failed_ctses;
    OrderAssumption(uc);
    for (int i = uc.size() - 1; i > 0; i--) {
        if (required_lits.find(uc[i]) != required_lits.end()) continue;
        cube temp_uc;
        temp_uc.reserve(uc.size());
        for (auto ll : uc)
            if (ll != uc[i]) temp_uc.emplace_back(ll);
        if (Down(temp_uc, frame_lvl, rec_lvl, failed_ctses)) {
            uc.swap(temp_uc);
            i = uc.size();
        } else {
            required_lits.emplace(uc[i]);
        }
    }
    sort(uc.begin(), uc.end(), cmp);
    if (uc.size() > uc_blocker.size() && frame_lvl != 0) {
        return false;
    } else
        return true;
}


bool BCAR::Down(cube &uc, int frame_lvl, int rec_lvl, vector<cube> &failed_ctses) {
    [[maybe_unused]] auto downScope = m_log.Section("FC_Dn");
    int ctgs = 0;
    cube assumption(uc);
    while (true) {
        // F_i & T & temp_uc'
        bool reachable = IsReachable(frame_lvl, assumption, "SAT_BC_Down");
        if (!reachable) {
            uc = GetUnsatAssumption(m_transSolvers[frame_lvl], uc);
            return true;
        }

        if (rec_lvl >= m_settings.ctgMaxRecursionDepth ||
            ctgs >= m_settings.ctgMaxStates ||
            frame_lvl < 1) {
            return false;
        }

        auto p = m_transSolvers[frame_lvl]->GetAssignment(false);
        shared_ptr<State> cts(new State(nullptr, p.first, p.second, 0));
        if (DownHasFailed(cts->latches, failed_ctses)) return false;

        if (CTSBlock(cts, frame_lvl - 1, rec_lvl, failed_ctses)) {
            ctgs++;
        } else {
            failed_ctses.emplace_back(cts->latches);
            return false;
        }
    }
}


bool BCAR::CTSBlock(shared_ptr<State> cts, int frame_lvl, int rec_lvl, vector<cube> &failed_ctses, int cts_count) {
    if (cts_count >= m_settings.ctgMaxBlocks) return false;

    m_log.L(3, "Try cts:", CubeToStr(cts->latches));
    cube cts_ass(cts->latches);
    OrderAssumption(cts_ass);

    while (true) {
        if (!IsReachable(frame_lvl, cts_ass, "SAT_R_CTS_B")) {
            auto uc_cts = GetUnsatAssumption(m_transSolvers[frame_lvl], cts->latches);
            m_log.L(3, "CTG Get UC:", CubeToStr(uc_cts));
            if (Generalize(uc_cts, frame_lvl, rec_lvl + 1)) m_branching->Update(uc_cts);
            m_log.L(3, "CTG Get Generalized UC:", CubeToStr(uc_cts));
            AddUnsatisfiableCore(uc_cts, frame_lvl + 1);
            PropagateUp(uc_cts, frame_lvl + 1);
            return true;
        } else {
            auto p = m_transSolvers[frame_lvl]->GetAssignment(false);
            shared_ptr<State> post_cts(new State(nullptr, p.first, p.second, 0));
            if (DownHasFailed(post_cts->latches, failed_ctses)) return false;

            int pre_cts_lvl = frame_lvl - 1;
            if (pre_cts_lvl < 0 ||
                !CTSBlock(post_cts, pre_cts_lvl, rec_lvl, failed_ctses, cts_count + 1)) {

                failed_ctses.emplace_back(cts->latches);
                return false;
            }
        }
    }
}


bool BCAR::DownHasFailed(const cube &s, const vector<cube> &failed_ctses) {
    for (auto f : failed_ctses) {
        // if f->s , return true
        if (f.size() > s.size()) continue;
        if (includes(s.begin(), s.end(), f.begin(), f.end(), cmp)) return true;
    }
    return false;
}


bool BCAR::CheckBad(shared_ptr<State> s) {
    [[maybe_unused]] auto checkScope = m_log.Section("FC_CheckBad");

    m_log.L(2, "\nSAT CHECK on bad");
    m_log.L(3, "From state: ", CubeToStr(s->latches));
    cube assumption(s->latches);
    OrderAssumption(assumption);
    bool result = false;
    {
        [[maybe_unused]] auto satScope = m_log.Section("SAT_BC_Bad");
        result = m_badSolver->Solve(assumption);
    }
    if (result) {
        m_log.L(2, "Result >>> SAT <<<");
        auto p = m_badSolver->GetAssignment(false);
        m_log.L(3, "Get Assignment:", CubeToStr(p.second));
        shared_ptr<State> newState(new State(s, p.first, p.second, s->depth + 1));
        m_lastState = newState;
        m_log.L(3, m_overSequence->FramesInfo());
        return true;
    } else {
        m_log.L(2, "Result >>> UNSAT <<<");
        auto uc = GetUnsatAssumption(m_badSolver, assumption);
        // Generalization
        unordered_set<int> required_lits;
        for (int i = uc.size() - 1; i >= 0; i--) {
            if (uc.size() < 3) break;
            if (required_lits.find(uc[i]) != required_lits.end()) continue;
            cube temp_uc;
            temp_uc.reserve(uc.size());
            for (auto ll : uc)
                if (ll != uc[i]) temp_uc.emplace_back(ll);
            assumption.clear();
            copy(temp_uc.begin(), temp_uc.end(), back_inserter(assumption));
            OrderAssumption(assumption);
            assumption.push_back(m_model.GetBad());
            bool result = false;
            {
                [[maybe_unused]] auto satGenScope = m_log.Section("SAT_BC_BadGen");
                result = m_badSolver->Solve(assumption);
            }
            if (!result) {
                auto new_uc = GetUnsatAssumption(m_badSolver, assumption);
                uc.swap(new_uc);
                OrderAssumption(uc);
                i = uc.size();
            } else {
                required_lits.emplace(uc[i]);
            }
        }
        sort(uc.begin(), uc.end(), cmp);
        m_log.L(2, "Get UC:", CubeToStr(uc));
        m_branching->Update(uc);
        AddUnsatisfiableCore(uc, 0);
        PropagateUp(uc, 0);
        return false;
    }
}


bool BCAR::IsReachable(int lvl, const cube &assumption, const string &label) {
    [[maybe_unused]] auto scoped = m_log.Section(label);
    return m_transSolvers[lvl]->Solve(assumption);
}


cube BCAR::GetUnsatAssumption(shared_ptr<SATSolver> solver, const cube &assumptions) {
    const unordered_set<int> &conflict = solver->GetConflict();
    cube res;

    for (auto a : assumptions) {
        if (conflict.find(a) != conflict.end())
            res.emplace_back(a);
    }

    sort(res.begin(), res.end(), cmp);
    return res;
}


// ================================================================================
// @brief: add the cube as and gates to the aiger model
// @input:
// @output:
// ================================================================================
unsigned BCAR::addCubeToANDGates(aiger *circuit, vector<unsigned> cube) {
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


bool BCAR::Propagate(const cube &c, int lvl) {
    [[maybe_unused]] auto propScope = m_log.Section("FC_Prop");
    bool result = !IsReachable(lvl, c, "SAT_BC_Prop");
    if (result) {
        auto uc = GetUnsatAssumption(m_transSolvers[lvl], c);
        AddUnsatisfiableCore(uc, lvl + 1);
    }
    return result;
}


int BCAR::PropagateUp(const cube &c, int lvl) {
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
    [[maybe_unused]] auto witScope = m_log.Section("FC_Witness");
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
    aiger *model_aig = m_model.GetAiger().get();

    if (m_overSequence->GetInvariantLevel() < 0 && m_model.GetEquivalenceMap().size() == 0) {
        aiger_open_and_write_to_file(model_aig, outPath.c_str());
        return;
    }

    shared_ptr<aiger> witness_aig_ptr(aiger_init(), aigerDeleter);
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
        eq_lits.emplace_back(addCubeToANDGates(witness_aig, {l1, l2 ^ 1}) ^ 1);
        eq_lits.emplace_back(addCubeToANDGates(witness_aig, {l1 ^ 1, l2}) ^ 1);
    }

    unsigned eq_cons;
    if (eq_lits.size() > 0) {
        eq_cons = addCubeToANDGates(witness_aig, eq_lits);
    }

    // prove on lvl 0
    if (m_overSequence == nullptr || m_overSequence->GetInvariantLevel() < 0) {
        unsigned bad_lit = m_model.GetAigerLit(m_model.GetBad());
        unsigned p = aiger_not(bad_lit);
        unsigned p_prime = p;
        if (eq_lits.size() > 0) {
            p_prime = addCubeToANDGates(witness_aig, {p, eq_cons});
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
        for (const cube &frame_cube : *frame_i) {
            vector<unsigned> cube_j;
            for (int l : frame_cube) cube_j.push_back(l > 0 ? (2 * l) : (2 * -l + 1));
            unsigned c_j = addCubeToANDGates(witness_aig, cube_j) ^ 1;
            frame_i_lits.push_back(c_j);
        }
        unsigned O_i = addCubeToANDGates(witness_aig, frame_i_lits);
        inv_lits.push_back(O_i ^ 1);
    }
    unsigned inv = addCubeToANDGates(witness_aig, inv_lits);
    unsigned bad_lit = m_model.GetAigerLit(m_model.GetBad());
    unsigned p = aiger_not(bad_lit);
    unsigned p_prime = addCubeToANDGates(witness_aig, {p, inv});

    if (eq_lits.size() > 0) {
        p_prime = addCubeToANDGates(witness_aig, {p_prime, eq_cons});
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


void BCAR::OutputCounterExample() {
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

    vector<shared_ptr<State>> trace;
    shared_ptr<State> state = m_lastState;
    while (state != nullptr) {
        trace.push_back(state);
        state = state->preState;
    }
    reverse(trace.begin(), trace.end());
    cexFile << trace[0]->GetLatchesString() << endl;

    for (int i = 1; i < trace.size(); i++) {
        cexFile << trace[i]->GetInputsString() << endl;
    }
    cexFile << "." << endl;
    cexFile.close();
}
} // namespace car
