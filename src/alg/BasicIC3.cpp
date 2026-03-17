#include "BasicIC3.h"
#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>

namespace car {

BasicIC3::BasicIC3(Settings settings,
                   Model &model,
                   Log &log) : m_settings(settings),
                               m_log(log),
                               m_model(model) {
    State::num_inputs = model.GetNumInputs();
    State::num_latches = model.GetNumLatches();
    m_cexStart = nullptr;
    global_log = &m_log;
    m_checkResult = CheckResult::Unknown;

    m_settings.satSolveInDomain = m_settings.satSolveInDomain && m_settings.solver == MCSATSolver::minicore;
}

BasicIC3::~BasicIC3() {
}

CheckResult BasicIC3::Run() {
    signal(SIGINT, SignalHandler);

    if (Check())
        m_checkResult = CheckResult::Safe;
    else
        m_checkResult = CheckResult::Unsafe;

    PrintALLStats();
    m_log.PrintCustomStatistics();

    return m_checkResult;
}

std::vector<std::pair<Cube, Cube>> BasicIC3::GetCexTrace() {
    std::vector<std::pair<Cube, Cube>> trace;
    if (!m_cexStart) return trace;

    vector<shared_ptr<State>> path;
    for (auto cur = m_cexStart; cur != nullptr; cur = cur->preState) {
        path.emplace_back(cur);
    }
    reverse(path.begin(), path.end());

    trace.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        Cube inputs;
        if (i > 0) {
            inputs = path[i]->inputs;
        }
        trace.emplace_back(path[i]->latches, std::move(inputs));
    }
    return trace;
}

FrameList BasicIC3::GetInv() {
    FrameList inv;
    if (m_invariantLevel <= 0) return inv;
    for (int i = 0; i < m_invariantLevel && i < static_cast<int>(m_transSolvers.size()); ++i) {
        Frame f;
        for (const Cube &cb : m_lfm.BorderCubes(i)) {
            f.emplace_back(cb);
        }
        inv.emplace_back(std::move(f));
    }
    return inv;
}

void BasicIC3::KLiveIncr() {
    int k_step = m_model.KLivenessIncrement();
    vector<Clause> k_clauses = m_model.GetKLiveClauses(k_step);
}


bool BasicIC3::ImmediateSatisfiable() {
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
        m_cexStart = make_shared<State>(nullptr, p.first, p.second, 0);
        return true;
    } else if (m_settings.searchFromBadPred) {
        slv->AddTransK(1);
        slv->AddConstraintsK(1);
        slv->AddBadk(1);
        sat = slv->Solve(Cube{});
        if (sat) {
            Cube inputs_bad;
            for (int i : m_model.GetPropertyCOIInputs()) {
                int i_p = m_model.GetPrimeK(i, 1);
                if (slv->GetModel(i_p) == T_TRUE)
                    inputs_bad.push_back(i);
                else if (slv->GetModel(i_p) == T_FALSE)
                    inputs_bad.push_back(-i);
            }
            shared_ptr<State> bad_state(new State(nullptr, inputs_bad, Cube(), 0));
            auto p = slv->GetAssignment(false);
            m_cexStart = make_shared<State>(bad_state, p.first, p.second, 0);
            return true;
        }
    }
    return false;
}


bool BasicIC3::IsInitStateImplyBad() {
    if (m_customInit.empty()) return false;
    auto slv = make_shared<SATSolver>(m_model, m_settings.solver);
    slv->AddTrans();
    slv->AddConstraints();
    Cube assumptions = m_customInit;
    assumptions.push_back(m_model.GetBad());
    bool sat = slv->Solve(assumptions);
    return !sat;
}


void BasicIC3::Extend() {
    // reserve k+1 frames
    while (m_transSolvers.size() <= m_k + 1) AddNewFrame();

    InitializeStartSolver();
    for (const Cube &cb : m_lfm.BorderCubes(m_k)) {
        m_startSolver->AddUC(cb);
    }
}

bool BasicIC3::Check() {

    if (!m_initialized) {
        Init();
        LOG_L(m_log, 2, "Initialized");
    } else {
        Reset();
        LOG_L(m_log, 2, "Reset");
    }

    // check if initial state is bad
    if (ImmediateSatisfiable()) return false;

    // The main IC3 loop.
    while (true) {
        Extend();
        LOG_L(m_log, 2, "==================== k = ", m_k, " ====================");

        if (!Strengthen()) {
            LOG_L(m_log, 2, "UNSAFE: CEX found during strengthening of F_", m_k);
            return false;
        }

        if (PropagateFrame()) {
            LOG_L(m_log, 1, "SAFE: Proof found at F_", m_invariantLevel);
            LOG_L(m_log, 1, FramesInfo());
            return true;
        }

        LOG_L(m_log, 1, FramesInfo());
        m_k++;
    }

    return true; // Should be unreachable
}


void BasicIC3::Init() {

    // start solver search state in frame k
    m_k = m_searchFromInitSucc ? 2 : 1;
    m_lfm.Reset();
    m_transSolvers.clear();

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
    m_initialStateSet.insert(init_latches.begin(), init_latches.end());

    m_invariantLevel = 0;
    m_branching = make_shared<Branching>(m_settings.branching);
    m_litOrder.branching = m_branching;
    m_blockerOrder.branching = m_branching;

    // create frame 0
    AddNewFrame();

    // lift
    m_liftSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    if (m_settings.satSolveInDomain) m_liftSolver->SetSolveInDomain();
    m_liftSolver->AddTrans();
    m_liftSolver->SetDomainCOI(m_model.GetConstraints());

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

    // initialize frame 0
    auto &init_slv = m_transSolvers[0];
    // F_0 is defined as exactly the initial states.
    for (const auto &lit : m_initialStateSet) {
        auto lemma = Clause{-lit};
        init_slv->AddUC(lemma);
    }

    m_initialized = true;
}


void BasicIC3::InitializeStartSolver() {
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


void BasicIC3::Reset() {
}


void BasicIC3::AddNewFrame() {
    int level = static_cast<int>(m_transSolvers.size());
    LOG_L(m_log, 2, "Adding new frame F_", level);

    m_lfm.EnsureLevel(level);

    auto solver = make_shared<SATSolver>(m_model, m_settings.solver);
    if (m_settings.satSolveInDomain) solver->SetSolveInDomain();
    solver->AddTrans();
    solver->AddConstraints();
    m_transSolvers.push_back(solver);
}


void BasicIC3::AddLemmaToSolvers(const Cube &blockingCube, int beginLevel, int endLevel) {
    // add lemma to trans solvers
    for (int i = beginLevel; i <= endLevel; ++i) {
        m_transSolvers[i]->AddUC(blockingCube);
    }

    // add lemma to start solver
    if (endLevel >= m_k) {
        m_startSolver->AddUC(blockingCube);
    }
}


int BasicIC3::AddLemma(const Cube &blockingCube, int frameLevel, bool fromCTI) {
    // add lemma to lemma forest
    auto res = m_lfm.AddLemma(blockingCube, frameLevel);

    // add lemma to solvers
    AddLemmaToSolvers(blockingCube, res.beginLevel, res.endLevel);

    // update minUpdateLevel
    if (res.beginLevel < m_minUpdateLevel) m_minUpdateLevel = res.beginLevel;

    // active lemma learning
    if (fromCTI && m_settings.activeLemmaLearning) {
        ActiveLemmaLearning(res.lemmaId);
    }
    return res.lemmaId;
}

void BasicIC3::ActiveLemmaLearning(int newLemmaId) {
    auto ancestor_chain = m_lfm.GetAncestorChain(newLemmaId);
    if (ancestor_chain.empty()) return;

    auto hot_spots = FindHotSpots(ancestor_chain);

    for (int hotspot_lemma_id : hot_spots) {
        if (!m_lfm.Alive(hotspot_lemma_id) ||
            m_lfm.Reachable(hotspot_lemma_id)) continue;

        m_allPushAttempted++;
        m_lfm.ResetRefineCountSinceALL(hotspot_lemma_id);

        auto status = ActiveProve(hotspot_lemma_id);

        if (status == ALLProveStatus::Invalidated) {
            m_allStatusInvalidated++;
            continue;
        }
        if (status == ALLProveStatus::Bailout) {
            m_allStatusBailout++;
            continue;
        }
        if (status == ALLProveStatus::Reachable) {
            m_allStatusReachable++;
            MarkReachable(hotspot_lemma_id);
            continue;
        }

        assert(status == ALLProveStatus::Proved);
        m_allStatusProved++;

        if (!m_lfm.Alive(hotspot_lemma_id)) continue;

        const int frame_level = m_lfm.FrameLevelOf(hotspot_lemma_id);
        PropagateUp(hotspot_lemma_id, frame_level);
    }
}

std::vector<int> BasicIC3::FindHotSpots(const std::vector<int> &ancestorChain) {
    std::vector<int> hot_spots;
    for (int lemma_id : ancestorChain) {
        if (!m_lfm.Alive(lemma_id)) continue;
        if (m_lfm.Reachable(lemma_id)) break;
        if (m_lfm.RefineCountSinceALL(lemma_id) >= m_settings.allThreshold) {
            hot_spots.push_back(lemma_id);
        }
    }
    std::reverse(hot_spots.begin(), hot_spots.end());
    return hot_spots;
}

void BasicIC3::MarkReachable(int lemmaId) {
    int cur = lemmaId;
    while (cur != -1) {
        if (!m_lfm.Alive(cur)) break;
        if (m_lfm.Reachable(cur)) break;
        m_lfm.SetReachable(cur, true);
        cur = m_lfm.ParentOf(cur);
    }
}

BasicIC3::ALLProveStatus BasicIC3::ActiveProve(int targetLemmaId) {
    if (!m_lfm.Alive(targetLemmaId)) return ALLProveStatus::Invalidated;

    int attempts_left = m_settings.allMaxStates;
    while (true) {
        if (!m_lfm.Alive(targetLemmaId)) return ALLProveStatus::Invalidated;

        int goal_level = m_lfm.FrameLevelOf(targetLemmaId);
        if (goal_level < 1 || goal_level >= static_cast<int>(m_transSolvers.size())) {
            return ALLProveStatus::Invalidated;
        }

        const Cube goal_cube = m_lfm.CubeOf(targetLemmaId);

        if (!m_lfm.HasCTPPreds(targetLemmaId)) {
            if (!IsReachable(goal_cube, m_transSolvers[goal_level])) {
                return ALLProveStatus::Proved;
            }
            if (attempts_left <= 0) return ALLProveStatus::Bailout;

            auto ctp_assignment = m_transSolvers[goal_level]->GetAssignment(false);
            auto ctp_state = make_shared<State>(nullptr, ctp_assignment.first, ctp_assignment.second, 0);
            auto succ_state = make_shared<State>(nullptr, Cube{}, goal_cube, 0);
            GeneralizePredecessor(ctp_state, succ_state);
            m_lfm.PushCTPPred(targetLemmaId, ctp_state->latches, goal_level);
        }

        if (attempts_left <= 0) return ALLProveStatus::Bailout;

        Cube ctp_cube;
        int ctp_level = -1;
        if (!m_lfm.PopCTPPred(targetLemmaId, ctp_cube, ctp_level)) {
            continue;
        }
        attempts_left--;

        if (ctp_level < 1 || ctp_level >= static_cast<int>(m_transSolvers.size())) {
            return ALLProveStatus::Invalidated;
        }

        if (GetSubsumeLevel(ctp_cube, ctp_level) != -1) {
            continue;
        }

        auto ctp_solver = m_transSolvers[ctp_level - 1];
        if (IsInductive(ctp_cube, ctp_solver)) {
            auto ctp_core = GetAndValidateCore(ctp_solver, ctp_cube);
            if (MIC(ctp_core, ctp_level, 0)) {
                m_branching->Update(ctp_core);
            }
            int ctp_lemma_id = AddLemma(ctp_core, ctp_level);
            PropagateUp(ctp_lemma_id, ctp_level);
            continue;
        }

        if (ctp_level == 1) return ALLProveStatus::Reachable;

        m_lfm.PushCTPPred(targetLemmaId, ctp_cube, ctp_level);
        auto new_ctp_assignment = ctp_solver->GetAssignment(false);
        auto new_ctp_state = make_shared<State>(nullptr, new_ctp_assignment.first, new_ctp_assignment.second, 0);
        auto pred_succ_state = make_shared<State>(nullptr, Cube{}, ctp_cube, 0);
        GeneralizePredecessor(new_ctp_state, pred_succ_state);
        m_lfm.PushCTPPred(targetLemmaId, new_ctp_state->latches, ctp_level - 1);
    }
}


void BasicIC3::PrintALLStats() const {
    if (!m_settings.activeLemmaLearning) return;
    m_log.L("ALL stats: pushAttempted=", m_allPushAttempted,
            ", statusProved=", m_allStatusProved,
            ", statusReachable=", m_allStatusReachable,
            ", statusBailout=", m_allStatusBailout,
            ", statusInvalidated=", m_allStatusInvalidated);
}


Cube BasicIC3::GetUnsatCore(const shared_ptr<SATSolver> &solver, const Cube &fallbackCube, bool prime) {
    unordered_set<int> conflict_set = solver->GetConflict();
    Cube core;
    if (!prime) {
        for (const auto &lit : fallbackCube) {
            if (conflict_set.count(lit)) {
                core.push_back(lit);
            }
        }
        return core;
    } else {
        for (const auto &lit : fallbackCube) {
            int lit_p = m_model.GetPrimeK(lit, 1);
            if (conflict_set.count(lit_p)) {
                core.push_back(lit);
            }
        }
        return core;
    }
}

shared_ptr<State> BasicIC3::EnumerateStartState() {
    LOG_L(m_log, 2, "Searching for a start state at level ", m_k);
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
                int i_p = m_model.GetPrimeK(i, 1);
                if (m_startSolver->GetModel(i_p) == T_TRUE)
                    inputs_prime.push_back(i_p);
                else if (m_startSolver->GetModel(i_p) == T_FALSE)
                    inputs_prime.push_back(-i_p);
            }

            // (p) & input & T & input' & T' -> (bad' & c' & c)
            // (p) & input & T & input' & T' & (!bad' | !c' | !c) is unsat
            Cube partial_latch = p.second;

            // (!bad' | !c' | !c)
            Clause cls;
            cls.push_back(-m_model.GetPrimeK(m_model.GetBad(), 1));
            for (auto cons : m_model.GetConstraints())
                cls.push_back(-m_model.GetPrimeK(cons, 1));
            for (auto cons : m_model.GetConstraints())
                cls.push_back(-cons);
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
                Cube temp_p = GetUnsatCore(m_badLiftSolver, partial_latch, false);
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
                int i_p = m_model.GetPrimeK(i, 1);
                if (m_startSolver->GetModel(i_p) == T_TRUE)
                    inputs_bad.push_back(i);
                else if (m_startSolver->GetModel(i_p) == T_FALSE)
                    inputs_bad.push_back(-i);
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
            cls.push_back(-m_model.GetBad());
            for (auto cons : m_model.GetConstraints())
                cls.push_back(-cons);
            for (auto l : m_shoalsLabels) cls.push_back(-l);
            for (auto l : m_wallsLabels) cls.push_back(-l);
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
                Cube temp_p = GetUnsatCore(m_badLiftSolver, partial_latch, false);
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

bool BasicIC3::Strengthen() {
    m_minUpdateLevel = m_k;

    while (true) {
        shared_ptr<State> start_state = EnumerateStartState();
        if (start_state != nullptr) {
            set<Obligation> obligations;
            obligations.emplace(start_state, m_k - 1, 1);

            if (!HandleObligations(obligations)) {
                return false;
            }
        } else {
            LOG_L(m_log, 2, "No more CTIs at level ", m_k, ". Frame is strengthened.");
            return true;
        }
    }
}


// ================================================================================
// @brief: Get a frame level where cb is subsumed
// @input:
// @output: -1 if not subsumed
// ================================================================================
int BasicIC3::GetSubsumeLevel(const Cube &cb, int startLvl) {
    if (startLvl < 0) startLvl = 0;
    if (startLvl > m_k + 1) return -1;

    for (int lvl = startLvl; lvl <= m_k + 1; ++lvl) {
        if (m_lfm.IsBlockedAtLevel(cb, lvl)) {
            return lvl;
        }
    }
    return -1;
}

bool BasicIC3::PopObligation(set<Obligation> &obligations, Obligation &ob) {
    if (obligations.empty()) return false;
    auto it = obligations.begin();
    if (it->level > m_k) return false;
    ob = *it;
    obligations.erase(it);
    ob.act += 1.0;
    return true;
}

void BasicIC3::PushObligation(set<Obligation> &obligations, Obligation ob, int newLevel) {
    while (ob.level < newLevel) {
        ob.act *= 0.6;
        ob.level++;
    }
    obligations.insert(ob);
}

bool BasicIC3::HandleObligations(set<Obligation> &obligations) {
    Obligation ob(nullptr, 0, 0);
    while (PopObligation(obligations, ob)) {

        if (ob.act >= m_settings.maxObligationAct) {
            LOG_L(m_log, 2, "Obligation at level ", ob.level, " depth ", ob.depth, " reached max activity. Skipped.");
            continue;
        }

        int subsume_lvl = GetSubsumeLevel(ob.state->latches, ob.level + 1);
        if (subsume_lvl != -1) {
            LOG_L(m_log, 2, "Obligation at level ", ob.level + 1, " depth ", ob.depth, " is subsumed at level ", subsume_lvl, ". Skipped.");
            PushObligation(obligations, ob, subsume_lvl + 1);
            continue;
        }

        // Query: F_{ob.level} & T & cti'
        LOG_L(m_log, 2, "Handling obligation at level ", ob.level);

        auto &trans_slv = m_transSolvers[ob.level];
        auto &cti_cube = ob.state->latches;

        if (!IsReachable(cti_cube, trans_slv)) {
            auto uc = GetAndValidateCore(trans_slv, cti_cube);

            size_t push_level = Generalize(uc, ob.level);

            if (push_level <= m_k) {
                LOG_L(m_log, 2, "Creating new obligation for same state at higher level ", push_level);
                PushObligation(obligations, ob, static_cast<int>(push_level));
            }
        } else {
            auto p = trans_slv->GetAssignment(false);
            auto predecessor_state =
                make_shared<State>(ob.state, p.first, p.second, ob.depth + 1);

            if (ob.level == 0) {
                LOG_L(m_log, 2, "UNSAFE: Found a path from the initial state.");
                m_cexStart = predecessor_state;
                return false;
            }

            GeneralizePredecessor(predecessor_state, ob.state);

            LOG_L(m_log, 2, "Found predecessor for CTI. New obligation at level ", ob.level - 1);
            PushObligation(obligations, ob, ob.level);
            PushObligation(obligations, Obligation(predecessor_state, ob.level - 1, ob.depth + 1), ob.level - 1);
        }
    }
    return true;
}

bool BasicIC3::IsInductive(const Cube &cb, const shared_ptr<SATSolver> &slv) {
    Clause cls;
    cls.reserve(cb.size());
    for (const auto &lit : cb) {
        cls.push_back(-lit);
    }
    slv->AddTempClause(cls);
    Cube assumption(cb);
    OrderAssumption(assumption);
    GetPrimed(assumption);
    slv->SetTempDomainCOI(assumption);
    bool result = !slv->Solve(assumption);
    slv->ReleaseTempClause();
    return result;
}

bool BasicIC3::Down(Cube &downCube, int frameLvl, int recLvl, const set<int> &triedLits) {
    LOG_L(m_log, 3, "Down: ", CubeToStr(downCube), " at frame level ", frameLvl, " and recursion level ", recLvl);
    int ctgs = 0;
    int joins = 0;
    auto &trans_slv = m_transSolvers[frameLvl];

    while (true) {
        LOG_L(m_log, 3, "Down attempt: ", CubeToStr(downCube));
        if (!InitiationCheck(downCube)) {
            return false;
        }
        if (IsInductive(downCube, trans_slv)) {
            Cube down_core = GetAndValidateCore(trans_slv, downCube);
            downCube.swap(down_core);
            return true;
        }

        if (recLvl > m_settings.ctgMaxRecursionDepth)
            return false;

        shared_ptr<State> down_state = make_shared<State>(nullptr, Cube(), downCube, 0);
        auto p = trans_slv->GetAssignment(false);
        auto ctg_state = make_shared<State>(down_state, p.first, p.second, 0);
        GeneralizePredecessor(ctg_state, down_state);

        const Cube &ctg_cube = ctg_state->latches;
        LOG_L(m_log, 3, "CTG Cube: ", CubeToStr(ctg_cube));

        if (!InitiationCheck(ctg_cube)) {
            return false;
        }

        if (ctgs < m_settings.ctgMaxStates &&
            frameLvl > 0 &&
            IsInductive(ctg_cube, m_transSolvers[frameLvl - 1])) {

            ctgs++;
            LOG_L(m_log, 3, "CTG is inductive at level ", frameLvl - 1);
            Cube ctg_core = GetAndValidateCore(m_transSolvers[frameLvl - 1], ctg_cube);

            if (MIC(ctg_core, frameLvl - 1, recLvl + 1)) {
                m_branching->Update(ctg_core);
            }
            int ctg_lemma_id = AddLemma(ctg_core, frameLvl - 1);
            PropagateUp(ctg_lemma_id, frameLvl - 1);
        } else {
            ctgs = 0;
            Cube join_cube;
            for (int i = 0; i < downCube.size(); i++) {
                if (binary_search(ctg_cube.begin(), ctg_cube.end(), downCube[i], Cmp)) {
                    join_cube.push_back(downCube[i]);
                } else if (triedLits.count(downCube[i])) {
                    return false;
                }
            }
            LOG_L(m_log, 3, "Joint Cube: ", CubeToStr(join_cube));
            downCube.swap(join_cube);
        }
    }
}


size_t BasicIC3::Generalize(Cube &cb, int frameLvl) {
    LOG_L(m_log, 3, "Generalizing Cube: ", CubeToStr(cb), ", at frameLvl: ", frameLvl);
    if (MIC(cb, frameLvl, 0)) {
        m_branching->Update(cb);
    }
    int lemma_id = AddLemma(cb, frameLvl, true);
    size_t push_level = PropagateUp(lemma_id, frameLvl);
    return push_level;
}


bool BasicIC3::MIC(Cube &cb, int frameLvl, int recLvl) {
    LOG_L(m_log, 3, "MIC: ", CubeToStr(cb), ", at frameLvl: ", frameLvl, ", recLvl: ", recLvl);

    vector<Cube> blockers;
    Cube blocker;
    set<int> tried_lits;

    if (m_settings.referSkipping && frameLvl > 0) {
        m_lfm.GetBlockers(cb, frameLvl, blockers);
        if (!blockers.empty()) {
            if (m_settings.branching > 0) {
                sort(blockers.begin(), blockers.end(), m_blockerOrder);
            }
            blocker = blockers[0];
        }
        for (const auto &lit : blocker) {
            tried_lits.insert(lit);
        }
    }

    int attempts = m_settings.ctgMaxAttempts;

    OrderAssumption(cb);
    // Iterate backwards to handle the shrinking Cube size gracefully.
    for (int i = cb.size() - 1; i >= 0; --i) {
        if (cb.size() < 3) break;
        int lit_to_drop = cb[i];

        // If we have already tried and failed to drop this literal, skip.
        if (tried_lits.count(lit_to_drop)) {
            continue;
        }

        // Create a temporary Cube with one literal removed.
        Cube drop_cube;
        drop_cube.reserve(cb.size() - 1);
        for (int j = 0; j < cb.size(); ++j) {
            if (i == j) continue;
            drop_cube.push_back(cb[j]);
        }

        if (Down(drop_cube, frameLvl, recLvl, tried_lits)) {
            // dropCube is sorted
            cb.swap(drop_cube);
            i = cb.size();
            attempts = m_settings.ctgMaxAttempts;
        } else {
            if (--attempts == 0) break;
            tried_lits.insert(lit_to_drop);
        }
    }
    sort(cb.begin(), cb.end(), Cmp);
    if (cb.size() > blocker.size() && frameLvl != 0) {
        return false;
    } else {
        return true;
    }
}


void BasicIC3::GeneralizePredecessor(const shared_ptr<State> &predecessorState, const shared_ptr<State> &successorState) {
    LOG_L(m_log, 3, "Generalizing predecessor. Initial latch size: ", predecessorState->latches.size(), ", input size: ", predecessorState->inputs.size(), ", Successor state latch size: ", successorState->latches.size());

    Clause succ_negation_clause;
    succ_negation_clause.reserve(successorState->latches.size());
    for (const auto &lit : successorState->latches) {
        succ_negation_clause.push_back(-m_model.GetPrimeK(lit, 1));
    }
    for (auto cons : m_model.GetConstraints()) {
        succ_negation_clause.push_back(-cons);
    }
    m_liftSolver->AddTempClause(succ_negation_clause);
    m_liftSolver->SetTempDomainCOI(succ_negation_clause);

    auto &partial_latch = predecessorState->latches;

    while (true) {
        Cube assumption(partial_latch);
        OrderAssumption(assumption);
        assumption.insert(assumption.begin(), predecessorState->inputs.begin(), predecessorState->inputs.end());
        // There exist some successors whose predecessors are the entire set. (All latches are determined solely by the inputs.)

        bool result = m_liftSolver->Solve(assumption);
        assert(!result);

        auto core = GetUnsatCore(m_liftSolver, partial_latch, false);
        LOG_L(m_log, 3, "Core size: ", core.size(), ", Partial latch size: ", partial_latch.size());

        if (core.size() == 0) break;
        if (core.size() >= partial_latch.size()) {
            break;
        } else {
            partial_latch.swap(core);
        }
    }
    m_liftSolver->ReleaseTempClause();
    LOG_L(m_log, 3, "Generalized predecessor. Final latch size: ", predecessorState->latches.size());
}

bool BasicIC3::InitiationCheck(const Cube &cb) {
    for (const auto &lit : cb) {
        if (m_initialStateSet.count(-lit)) {
            return true; // Disjoint (UNSAT), check passes.
        }
    }
    LOG_L(m_log, 3, "Initiation check failed.");
    return false;
}


Cube BasicIC3::GetAndValidateCore(const shared_ptr<SATSolver> &solver, const Cube &fallbackCube) {
    // fallbackCube is sorted
    Cube core = GetUnsatCore(solver, fallbackCube, true);
    LOG_L(m_log, 3, "Got UNSAT core: ", CubeToStr(core));
    if (!InitiationCheck(core)) {
        LOG_L(m_log, 3, "GetAndValidateCore: core intersects with initial states. Reverting to fallback Cube.");
        core = fallbackCube;
    }
    return core;
}


string BasicIC3::FramesInfo() const {
    stringstream ss;
    ss << "Frames " << m_transSolvers.size() << endl;
    for (size_t i = 0; i < m_transSolvers.size(); ++i) {
        ss << m_lfm.BorderSize(static_cast<int>(i)) << " ";
    }
    return ss.str();
}


bool BasicIC3::IsReachable(const Cube &cb, const shared_ptr<SATSolver> &slv) {
    Cube assumption(cb);
    OrderAssumption(assumption);
    GetPrimed(assumption);
    slv->SetTempDomainCOI(assumption);

    return slv->Solve(assumption);
}


// ================================================================================
// @brief: ~cb is in F_i, check if F_i (& ~cb) & T & cb' is UNSAT?
// @input:
// @output:
// ================================================================================
bool BasicIC3::Propagate(int lemmaId, int lvl) {
    bool result;
    Cube cb = m_lfm.CubeOf(lemmaId);

    if (!IsReachable(cb, m_transSolvers[lvl])) {
        auto core = GetAndValidateCore(m_transSolvers[lvl], cb);
        if (core.size() < cb.size()) {
            AddLemma(core, lvl + 1);
        } else {
            int propagated_level = m_lfm.PropagateLemma(lemmaId, lvl + 1);
            AddLemmaToSolvers(cb, propagated_level, propagated_level);
        }
        m_branching->Update(core);
        result = true;
    } else {
        result = false;
    }
    return result;
}


int BasicIC3::PropagateUp(int LemmaId, int startLevel) {
    int lvl = startLevel;
    while (lvl <= m_k) {
        if (Propagate(LemmaId, lvl))
            m_branching->Update(m_lfm.CubeOf(LemmaId));
        else
            break;
        lvl++;
    }
    return lvl;
}


bool BasicIC3::PropagateFrame() {
    LOG_L(m_log, 2, "Propagating clauses.");

    for (int i = m_minUpdateLevel; i <= m_k; ++i) {
        int lemmas_kept = 0;
        int lemmas_propagated = 0;

        m_lfm.CleanDeadBorders(i);
        m_lfm.SortBorderByCubeSize(i);

        std::vector<int> lemmas_to_iterate = m_lfm.BorderIds(i);
        for (int lemma_id : lemmas_to_iterate) {
            if (!m_lfm.Alive(lemma_id) || m_lfm.Reachable(lemma_id)) continue;

            if (Propagate(lemma_id, i)) {
                lemmas_propagated++;
            } else {
                lemmas_kept++;
            }
        }

        LOG_L(m_log, 2, "Frame ", i, " propagation: ", lemmas_propagated, " propagated, ", lemmas_kept, " kept.");

        if (m_lfm.BorderEmpty(i)) {
            LOG_L(m_log, 2, "SAFE: Frame F_", i, " is empty.");
            m_invariantLevel = i + 1;
            LOG_L(m_log, 2, "m_invariantLevel: ", m_invariantLevel);
            LOG_L(m_log, 2, FramesInfo());
            return true; // Proof found
        }
    }

    return false;
}


void BasicIC3::OutputCounterExample() {
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
    cout << cex_path << endl;
    std::ofstream cex_file;
    cex_file.open(cex_path);

    assert(m_cexStart != nullptr);

    cex_file << "1" << endl
             << "b0" << endl;

    shared_ptr<State> state = m_cexStart;
    cex_file << state->GetLatchesString() << endl;
    cex_file << state->GetInputsString() << endl;
    while (state->preState != nullptr) {
        state = state->preState;
        cex_file << state->GetInputsString() << endl;
    }

    cex_file << "." << endl;
    cex_file.close();
}


unsigned BasicIC3::AddCubeToAndGates(aiger *circuit, vector<unsigned> cb) {
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

void BasicIC3::OutputWitness() {
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

    if (m_invariantLevel == 0 && m_model.GetEquivalenceMap().size() == 0) {
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
        if (itr->first == m_model.TrueId() || itr->second == m_model.TrueId()) {
            unsigned true_eq_lit = m_model.GetAigerLit(itr->second);
            eq_lits.emplace_back(true_eq_lit);
            continue;
        }
        assert(abs(itr->first) <= witness_aig->maxvar);
        assert(abs(itr->second) <= witness_aig->maxvar);
        unsigned l1 = m_model.GetAigerLit(itr->first);
        unsigned l2 = m_model.GetAigerLit(itr->second);
        eq_lits.emplace_back(AddCubeToAndGates(witness_aig, {l1, l2 ^ 1}) ^ 1);
        eq_lits.emplace_back(AddCubeToAndGates(witness_aig, {l1 ^ 1, l2}) ^ 1);
    }

    unsigned eq_cons;
    if (eq_lits.size() > 0) {
        eq_cons = AddCubeToAndGates(witness_aig, eq_lits);
    }

    bool empty_inv = true;
    for (int i = m_invariantLevel; i <= m_k + 1; i++) {
        if (!m_lfm.BorderEmpty(i)) {
            empty_inv = false;
            break;
        }
    }

    // prove on lvl 0
    if (m_invariantLevel == 0 || empty_inv) {
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

    // P' = P & invariant
    // P' = !bad & ( O_0 | O_1 | ... | O_i )
    //             !( !O_0 & !O_1 & ...  & !O_i )
    //                 O_i = c_1 & c_2 & ... & c_j
    //                       c_j = !( l_1 & l_2 & .. & l_k )

    // P' = P & invariant
    // P' = !bad & F_{i+1}
    //             F_{i+1} = c_1 & c_2 & ... & c_j
    //                       c_j = !( l_1 & l_2 & .. & l_k )

    std::set<Cube, bool (*)(const Cube &, const Cube &)> ind_inv(CubeComp);

    for (int i = m_invariantLevel; i <= m_k + 1; i++) {
        for (const Cube &cb : m_lfm.BorderCubes(i)) {
            ind_inv.insert(cb);
        }
    }
    assert(!ind_inv.empty());

    vector<unsigned> inv_lits;
    for (auto it = ind_inv.begin(); it != ind_inv.end(); it++) {
        vector<unsigned> cube_lits;
        for (int l : *it) cube_lits.push_back(l > 0 ? (2 * l) : (2 * -l + 1));
        unsigned cls = AddCubeToAndGates(witness_aig, cube_lits) ^ 1;
        inv_lits.push_back(cls);
    }
    unsigned inv = AddCubeToAndGates(witness_aig, inv_lits);

    unsigned bad_lit = m_model.GetAigerLit(m_model.GetBad());
    unsigned p = aiger_not(bad_lit);
    unsigned p_prime = AddCubeToAndGates(witness_aig, {p, inv});

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
}


void BasicIC3::Witness() {
    if (m_checkResult == CheckResult::Unsafe) {
        LOG_L(m_log, 2, "Generating counterexample.");
        OutputCounterExample();
    } else if (m_checkResult == CheckResult::Safe) {
        LOG_L(m_log, 2, "Generating proof.");
        OutputWitness();
    } else {
        LOG_L(m_log, 2, "Unknown check result.");
    }
}

} // namespace car
