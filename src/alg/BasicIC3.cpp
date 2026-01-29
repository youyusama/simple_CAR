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
    State::numInputs = model.GetNumInputs();
    State::numLatches = model.GetNumLatches();
    m_cexStart = nullptr;
    GLOBAL_LOG = &m_log;
    m_checkResult = CheckResult::Unknown;

    m_settings.satSolveInDomain = m_settings.satSolveInDomain && m_settings.solver == MCSATSolver::minicore;
}

BasicIC3::~BasicIC3() {
}

CheckResult BasicIC3::Run() {
    signal(SIGINT, signalHandler);

    if (Check())
        m_checkResult = CheckResult::Safe;
    else
        m_checkResult = CheckResult::Unsafe;

    m_log.PrintCustomStatistics();

    return m_checkResult;
}

std::vector<std::pair<cube, cube>> BasicIC3::GetCexTrace() {
    std::vector<std::pair<cube, cube>> trace;
    if (!m_cexStart) return trace;

    vector<shared_ptr<State>> path;
    for (auto cur = m_cexStart; cur != nullptr; cur = cur->preState) {
        path.emplace_back(cur);
    }
    reverse(path.begin(), path.end());

    trace.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        cube inputs;
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
    for (int i = 0; i < m_invariantLevel && i < static_cast<int>(m_frames.size()); ++i) {
        frame f;
        for (const auto &cb : m_frames[i].borderCubes) {
            f.emplace_back(cb);
        }
        inv.emplace_back(std::move(f));
    }
    return inv;
}

void BasicIC3::KLiveIncr() {
    int k_step = m_model.KLivenessIncrement();
    vector<clause> k_clauses = m_model.GetKLiveClauses(k_step);
}


bool BasicIC3::ImmediateSatisfiable() {
    auto &init_slv = m_frames[0].solver;
    cube assumptions;
    assumptions.push_back(m_model.GetBad());
    init_slv->SetTempDomainCOI(assumptions);
    bool sat = init_slv->Solve(assumptions);
    if (sat) {
        auto p = init_slv->GetAssignment(false);
        m_cexStart = make_shared<State>(nullptr, p.first, p.second, 0);
    }
    return sat;
}


bool BasicIC3::IsInitStateImplyBad() {
    if (m_customInit.empty()) return false;
    auto slv = make_shared<SATSolver>(m_model, m_settings.solver);
    slv->AddTrans();
    slv->AddConstraints();
    cube assumptions = m_customInit;
    assumptions.push_back(m_model.GetBad());
    bool sat = slv->Solve(assumptions);
    return !sat;
}


void BasicIC3::Extend() {
    // reserve k+1 frames
    while (m_frames.size() <= m_k + 1) AddNewFrame();

    InitializeStartSolver();
    for (const auto &lemma : m_frames[m_k].borderCubes) {
        m_startSolver->AddUC(lemma);
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

    if (m_searchFromInitSucc) {
        m_initStateImplyBad = IsInitStateImplyBad();
        if (!m_initStateImplyBad)
            LOG_L(m_log, 1, "Initial state does not imply bad");
    }

    // initial states
    cube init_latches;
    if (m_customInit.empty())
        init_latches = m_model.GetInitialState();
    else
        init_latches = m_customInit;
    m_initialState = make_shared<State>(nullptr, cube{}, init_latches, 0);
    m_initialStateSet.insert(init_latches.begin(), init_latches.end());

    m_invariantLevel = 0;
    m_branching = make_shared<Branching>(m_settings.branching);
    litOrder.branching = m_branching;
    blockerOrder.branching = m_branching;

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
    IC3Frame &frame0 = m_frames[0];
    // F_0 is defined as exactly the initial states.
    for (const auto &lit : m_initialStateSet) {
        auto lemma = clause{-lit};
        frame0.solver->AddUC(lemma);
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
    LOG_L(m_log, 2, "Adding new frame F_", m_frames.size());
    IC3Frame newFrame;
    newFrame.k = m_frames.size();
    newFrame.solver = make_shared<SATSolver>(m_model, m_settings.solver);
    if (m_settings.satSolveInDomain) newFrame.solver->SetSolveInDomain();
    newFrame.solver->AddTrans();
    newFrame.solver->AddConstraints();
    m_frames.push_back(newFrame);
}

void BasicIC3::AddBlockingCube(const cube &blockingCube, int frameLevel, bool toAll) {
    assert(frameLevel >= 1);
    auto insertionResult = m_frames[frameLevel].borderCubes.insert(blockingCube);
    if (!insertionResult.second) {
        return;
    }

    if (frameLevel < m_minUpdateLevel) m_minUpdateLevel = frameLevel;

    for (int i = toAll ? 1 : frameLevel; i <= frameLevel; ++i) {
        if (m_frames[i].solver) {
            m_frames[i].solver->AddUC(blockingCube);
        }
    }
    if (frameLevel >= m_k) {
        m_startSolver->AddUC(blockingCube);
    }
}

cube BasicIC3::GetUnsatCore(const shared_ptr<SATSolver> &solver, const cube &fallbackCube, bool prime) {
    unordered_set<int> conflictSet = solver->GetConflict();
    cube core;
    if (!prime) {
        for (const auto &lit : fallbackCube) {
            if (conflictSet.count(lit)) {
                core.push_back(lit);
            }
        }
        return core;
    } else {
        for (const auto &lit : fallbackCube) {
            int lit_p = m_model.GetPrimeK(lit, 1);
            if (conflictSet.count(lit_p)) {
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
        [[maybe_unused]] auto satScope = m_log.Section("SAT_Start");
        sat = m_startSolver->Solve();
    }
    if (sat) {
        if (m_loopRefuting) {
            shared_ptr<State> badState(new State(nullptr, {}, m_customInit, 0));
            return badState;
        }

        auto p = m_startSolver->GetAssignment(false);

        if (m_settings.searchFromBadPred) {
            // start state is the predecessor of a bad state
            cube inputs_prime;
            for (int i : m_model.GetPropertyCOIInputs()) {
                int i_p = m_model.GetPrimeK(i, 1);
                if (m_startSolver->GetModel(i_p) == t_True)
                    inputs_prime.push_back(i_p);
                else if (m_startSolver->GetModel(i_p) == t_False)
                    inputs_prime.push_back(-i_p);
            }

            // (p) & input & T & input' & T' -> (bad' & c' & c)
            // (p) & input & T & input' & T' & (!bad' | !c' | !c) is unsat
            cube partial_latch = p.second;

            // (!bad' | !c' | !c)
            clause cls;
            cls.push_back(-m_model.GetPrimeK(m_model.GetBad(), 1));
            for (auto cons : m_model.GetConstraints())
                cls.push_back(-m_model.GetPrimeK(cons, 1));
            for (auto cons : m_model.GetConstraints())
                cls.push_back(-cons);
            m_badLiftSolver->AddTempClause(cls);

            int gen_tried = 0;

            while (true) {
                cube assumption;
                copy(partial_latch.begin(), partial_latch.end(), back_inserter(assumption));
                OrderAssumption(assumption);

                if (gen_tried == 1) reverse(assumption.begin(), assumption.end());
                if (gen_tried > 1) random_shuffle(assumption.begin(), assumption.end());
                gen_tried++;

                copy(p.first.begin(), p.first.end(), back_inserter(assumption));
                copy(inputs_prime.begin(), inputs_prime.end(), back_inserter(assumption));

                bool res;
                {
                    [[maybe_unused]] auto satBadPred = m_log.Section("SAT_BadLift");
                    res = m_badLiftSolver->Solve(assumption);
                }
                assert(!res);
                cube temp_p = GetUnsatCore(m_badLiftSolver, partial_latch, false);
                if (temp_p.size() >= partial_latch.size())
                    break;
                else {
                    partial_latch.swap(temp_p);
                }
            }
            m_badLiftSolver->ReleaseTempClause();
            p.second = partial_latch;

            cube inputs_bad;
            for (int i : m_model.GetPropertyCOIInputs()) {
                int i_p = m_model.GetPrimeK(i, 1);
                if (m_startSolver->GetModel(i_p) == t_True)
                    inputs_bad.push_back(i);
                else if (m_startSolver->GetModel(i_p) == t_False)
                    inputs_bad.push_back(-i);
            }
            shared_ptr<State> badState(new State(nullptr, inputs_bad, cube(), 0));
            shared_ptr<State> badPredState(new State(badState, p.first, p.second, 0));
            return badPredState;
        } else {
            // start state is a bad state
            // (p) -> (bad & c)
            // (p) & (!bad | !c) is unsat
            cube partial_latch = p.second;
            LOG_L(m_log, 3, "Bad State Latches Before Lifting: ", CubeToStr(partial_latch));

            // (!bad | !c)
            clause cls;
            cls.push_back(-m_model.GetBad());
            for (auto cons : m_model.GetConstraints())
                cls.push_back(-cons);
            for (auto l : m_shoalsLabels) cls.push_back(-l);
            for (auto l : m_wallsLabels) cls.push_back(-l);
            m_badLiftSolver->AddTempClause(cls);
            LOG_L(m_log, 3, "lift assume: ", CubeToStr(cls));

            int gen_tried = 0;

            while (true) {
                cube assumption;
                copy(partial_latch.begin(), partial_latch.end(), back_inserter(assumption));
                OrderAssumption(assumption);

                if (gen_tried == 1) reverse(assumption.begin(), assumption.end());
                if (gen_tried > 1) random_shuffle(assumption.begin(), assumption.end());
                gen_tried++;

                copy(p.first.begin(), p.first.end(), back_inserter(assumption));

                bool res;
                {
                    [[maybe_unused]] auto satBadPred = m_log.Section("SAT_BadLift");
                    res = m_badLiftSolver->Solve(assumption);
                }
                assert(!res);
                cube temp_p = GetUnsatCore(m_badLiftSolver, partial_latch, false);
                if (temp_p.size() >= partial_latch.size())
                    break;
                else {
                    partial_latch.swap(temp_p);
                }
            }
            m_badLiftSolver->ReleaseTempClause();
            p.second = partial_latch;

            shared_ptr<State> badState(new State(nullptr, p.first, p.second, 0));
            return badState;
        }
    } else {
        return nullptr;
    }
}

bool BasicIC3::Strengthen() {
    m_minUpdateLevel = m_k;

    while (true) {
        shared_ptr<State> startState = EnumerateStartState();
        if (startState != nullptr) {
            set<Obligation> obligations;
            obligations.emplace(startState, m_k - 1, 1);

            if (!HandleObligations(obligations)) {
                return false;
            }
        } else {
            LOG_L(m_log, 2, "No more CTIs at level ", m_k, ". Frame is strengthened.");
            return true;
        }
    }
}


bool BasicIC3::HandleObligations(set<Obligation> &obligations) {
    while (!obligations.empty()) {
        Obligation ob = *obligations.begin();

        // Query: F_{ob.level} & T & cti'
        LOG_L(m_log, 2, "Handling obligation at level ", ob.level);

        auto &trans_slv = m_frames[ob.level].solver;
        auto &ctiCube = ob.state->latches;

        if (UnreachabilityCheck(ctiCube, trans_slv)) {
            obligations.erase(obligations.begin());
            auto uc = GetAndValidateCore(trans_slv, ctiCube);

            size_t pushLevel = Generalize(uc, ob.level);

            if (pushLevel <= m_k) {
                LOG_L(m_log, 2, "Creating new obligation for same state at higher level ", pushLevel);
                obligations.insert(Obligation(ob.state, pushLevel, ob.depth));
            }
        } else {
            auto p = trans_slv->GetAssignment(false);
            auto predecessorState =
                make_shared<State>(ob.state, p.first, p.second, ob.depth + 1);

            if (ob.level == 0) {
                LOG_L(m_log, 2, "UNSAFE: Found a path from the initial state.");
                m_cexStart = predecessorState;
                return false;
            }

            GeneralizePredecessor(predecessorState, ob.state);

            LOG_L(m_log, 2, "Found predecessor for CTI. New obligation at level ", ob.level - 1);
            obligations.insert(Obligation(predecessorState, ob.level - 1, ob.depth + 1));
        }
    }
    return true;
}

bool BasicIC3::InductionCheck(const cube &cb, const shared_ptr<SATSolver> &slv) {
    clause cls;
    cls.reserve(cb.size());
    for (const auto &lit : cb) {
        cls.push_back(-lit);
    }
    slv->AddTempClause(cls);
    cube assumption(cb);
    OrderAssumption(assumption);
    GetPrimed(assumption);
    slv->SetTempDomainCOI(assumption);
    bool result = !slv->Solve(assumption);
    slv->ReleaseTempClause();
    return result;
}

bool BasicIC3::Down(cube &downCube, int frameLvl, int recLvl, const set<int> &triedLits) {
    LOG_L(m_log, 3, "Down: ", CubeToStr(downCube), " at frame level ", frameLvl, " and recursion level ", recLvl);
    int ctgs = 0;
    int joins = 0;
    auto &trans_slv = m_frames[frameLvl].solver;
    shared_ptr<State> downState = make_shared<State>(nullptr, cube(), downCube, 0);

    while (true) {
        LOG_L(m_log, 3, "Down attempt: ", CubeToStr(downCube));
        if (!InitiationCheck(downCube)) {
            return false;
        }
        if (InductionCheck(downCube, trans_slv)) {
            cube downCore = GetAndValidateCore(trans_slv, downCube);
            downCube.swap(downCore);
            return true;
        }

        if (recLvl > m_settings.ctgMaxRecursionDepth || frameLvl > 0)
            return false;

        auto p = trans_slv->GetAssignment(false);
        auto ctgState = make_shared<State>(downState, p.first, p.second, 0);
        GeneralizePredecessor(ctgState, downState);

        const cube &ctgCube = ctgState->latches;
        LOG_L(m_log, 3, "CTG cube: ", CubeToStr(ctgCube));

        if (!InitiationCheck(ctgCube)) {
            return false;
        }

        auto &trans_slv_m1 = m_frames[frameLvl - 1].solver;

        if (ctgs < m_settings.ctgMaxStates &&
            InductionCheck(ctgCube, trans_slv_m1)) {

            ctgs++;
            LOG_L(m_log, 3, "CTG is inductive at level ", frameLvl - 1);
            cube ctgCore = GetAndValidateCore(trans_slv_m1, ctgCube);

            int pushLevel = PropagateUp(ctgCore, frameLvl);
            if (MIC(ctgCore, pushLevel - 1, recLvl + 1)) {
                m_branching->Update(ctgCore);
            }
            LOG_L(m_log, 2, "Learned ctg clause and pushed to frame ", pushLevel);
            AddBlockingCube(ctgCore, pushLevel, true);
        } else {
            ctgs = 0;
            cube joinCube;
            for (int i = 0; i < downCube.size(); i++) {
                if (binary_search(ctgCube.begin(), ctgCube.end(), downCube[i], cmp)) {
                    joinCube.push_back(downCube[i]);
                } else if (triedLits.count(downCube[i])) {
                    return false;
                }
            }
            LOG_L(m_log, 3, "Joint cube: ", CubeToStr(joinCube));
            downCube.swap(joinCube);
        }
    }
}

void BasicIC3::GetBlockers(const cube &blockingCube, int framelevel, vector<cube> &blockers) {
    int size = -1;
    for (const auto &cb : m_frames[framelevel].borderCubes) {
        if (size != -1 && size < cb.size()) break;
        if (includes(blockingCube.begin(), blockingCube.end(), cb.begin(), cb.end(), cmp)) {
            size = cb.size();
            blockers.push_back(cb);
        }
    }
}

size_t BasicIC3::Generalize(cube &cb, int frameLvl) {
    LOG_L(m_log, 3, "Generalizing cube: ", CubeToStr(cb), ", at frameLvl: ", frameLvl);
    if (MIC(cb, frameLvl, 0)) {
        m_branching->Update(cb);
    }
    int pushLevel = PropagateUp(cb, frameLvl + 1);
    LOG_L(m_log, 2, "Learned clause and pushed to frame ", pushLevel);
    AddBlockingCube(cb, pushLevel, true);
    return pushLevel;
}

bool BasicIC3::MIC(cube &cb, int frameLvl, int recLvl) {
    LOG_L(m_log, 3, "MIC: ", CubeToStr(cb), ", at frameLvl: ", frameLvl, ", recLvl: ", recLvl);

    vector<cube> blockers;
    cube blocker;
    set<int> triedLits;

    if (m_settings.referSkipping && frameLvl > 0) {
        GetBlockers(cb, frameLvl, blockers);
        if (!blockers.empty()) {
            if (m_settings.branching > 0) {
                sort(blockers.begin(), blockers.end(), blockerOrder);
            }
            blocker = blockers[0];
        }
        for (const auto &lit : blocker) {
            triedLits.insert(lit);
        }
    }


    const int maxMicAttempts = 3;
    size_t attempts = maxMicAttempts;

    OrderAssumption(cb);
    // Iterate backwards to handle the shrinking cube size gracefully.
    for (int i = cb.size() - 1; i >= 0; --i) {
        if (cb.size() < 3) break;
        int litToDrop = cb[i];

        // If we have already tried and failed to drop this literal, skip.
        if (triedLits.count(litToDrop)) {
            continue;
        }

        // Create a temporary cube with one literal removed.
        cube dropCube;
        dropCube.reserve(cb.size() - 1);
        for (int j = 0; j < cb.size(); ++j) {
            if (i == j) continue;
            dropCube.push_back(cb[j]);
        }

        if (Down(dropCube, frameLvl, recLvl, triedLits)) {
            // dropCube is sorted
            cb.swap(dropCube);
            i = cb.size();
            attempts = maxMicAttempts;
        } else {
            if (--attempts == 0) {
                LOG_L(m_log, 3, "Max MIC attempts reached, stopping generalization.");
                break;
            }
            triedLits.insert(litToDrop);
        }
    }
    sort(cb.begin(), cb.end(), cmp);
    if (cb.size() > blocker.size() && frameLvl != 0) {
        return false;
    } else {
        return true;
    }
}


void BasicIC3::GeneralizePredecessor(const shared_ptr<State> &predecessorState, const shared_ptr<State> &successorState) {
    LOG_L(m_log, 3, "Generalizing predecessor. Initial latch size: ", predecessorState->latches.size(), ", input size: ", predecessorState->inputs.size(), ", Successor state latch size: ", successorState->latches.size());

    clause succNegationClause;
    succNegationClause.reserve(successorState->latches.size());
    for (const auto &lit : successorState->latches) {
        succNegationClause.push_back(-m_model.GetPrimeK(lit, 1));
    }
    for (auto cons : m_model.GetConstraints()) {
        succNegationClause.push_back(-cons);
    }
    m_liftSolver->AddTempClause(succNegationClause);
    m_liftSolver->SetTempDomainCOI(succNegationClause);

    auto &partialLatch = predecessorState->latches;

    while (true) {
        cube assumption(partialLatch);
        OrderAssumption(assumption);
        assumption.insert(assumption.begin(), predecessorState->inputs.begin(), predecessorState->inputs.end());
        // There exist some successors whose predecessors are the entire set. (All latches are determined solely by the inputs.)

        bool result = m_liftSolver->Solve(assumption);
        assert(!result);

        auto core = GetUnsatCore(m_liftSolver, partialLatch, false);
        LOG_L(m_log, 3, "Core size: ", core.size(), ", Partial latch size: ", partialLatch.size());

        if (core.size() == 0) break;
        if (core.size() >= partialLatch.size()) {
            break;
        } else {
            partialLatch.swap(core);
        }
    }
    m_liftSolver->ReleaseTempClause();
    LOG_L(m_log, 3, "Generalized predecessor. Final latch size: ", predecessorState->latches.size());
}

bool BasicIC3::InitiationCheck(const cube &cb) {
    for (const auto &lit : cb) {
        if (m_initialStateSet.count(-lit)) {
            return true; // Disjoint (UNSAT), check passes.
        }
    }
    LOG_L(m_log, 3, "Initiation check failed.");
    return false;
}


cube BasicIC3::GetAndValidateCore(const shared_ptr<SATSolver> &solver, const cube &fallbackCube) {
    // fallbackCube is sorted
    cube core = GetUnsatCore(solver, fallbackCube, true);
    LOG_L(m_log, 3, "Got UNSAT core: ", CubeToStr(core));
    if (!InitiationCheck(core)) {
        LOG_L(m_log, 3, "GetAndValidateCore: core intersects with initial states. Reverting to fallback cube.");
        core = fallbackCube;
    }
    return core;
}

string BasicIC3::FramesInfo() const {
    stringstream ss;
    ss << "Frames " << m_frames.size() << endl;
    for (size_t i = 0; i < m_frames.size(); ++i) {
        ss << m_frames[i].borderCubes.size() << " ";
    }
    return ss.str();
}

bool BasicIC3::UnreachabilityCheck(const cube &cb, const shared_ptr<SATSolver> &slv) {
    cube assumption(cb);
    OrderAssumption(assumption);
    GetPrimed(assumption);
    slv->SetTempDomainCOI(assumption);

    bool sat = slv->Solve(assumption);
    return !sat;
}

int BasicIC3::PropagateUp(const cube &cb, int startLevel) {
    int pushLevel = startLevel;
    while (pushLevel <= m_k) {
        if (!UnreachabilityCheck(cb, m_frames[pushLevel].solver)) {
            break;
        }
        m_branching->Update(cb);
        pushLevel++;
    }
    return pushLevel;
}

bool BasicIC3::PropagateFrame() {
    LOG_L(m_log, 2, "Propagating clauses.");

    LOG_L(m_log, 2, "Cleaning up redundant clauses.");
    set<cube, IC3Frame::CubeComp> allCubes;
    for (int i = m_k + 1; i > m_minUpdateLevel; --i) {
        IC3Frame &frame = m_frames[i];
        if (frame.borderCubes.empty()) continue;
        size_t originalSize = frame.borderCubes.size();

        set<cube, IC3Frame::CubeComp> remainingCubes;
        set_difference(frame.borderCubes.begin(), frame.borderCubes.end(),
                       allCubes.begin(), allCubes.end(),
                       inserter(remainingCubes, remainingCubes.end()),
                       IC3Frame::CubeComp());

        if (originalSize != remainingCubes.size()) {
            LOG_L(m_log, 3, "Frame ", i, " cleanup: ", originalSize, " -> ", remainingCubes.size());
        }

        frame.borderCubes.swap(remainingCubes);
        allCubes.insert(frame.borderCubes.begin(), frame.borderCubes.end());
    }

    LOG_L(m_log, 2, "Propagating clauses forward.");
    for (int i = m_minUpdateLevel; i <= m_k; ++i) {
        IC3Frame &framei = m_frames[i];
        int cubesKept = 0;
        int cubesPropagated = 0;

        for (auto it = framei.borderCubes.begin(); it != framei.borderCubes.end();) {
            const auto &cb = *it;
            if (UnreachabilityCheck(cb, framei.solver)) {
                cubesPropagated++;
                auto core = GetAndValidateCore(framei.solver, cb);
                AddBlockingCube(core, i + 1, core.size() < cb.size());
                m_branching->Update(core);
                // Safely erase and advance the iterator
                it = framei.borderCubes.erase(it);
            } else {
                cubesKept++;
                ++it;
            }
        }

        LOG_L(m_log, 2, "Frame ", i, " propagation: ", cubesPropagated, " propagated, ", cubesKept, " kept.");


        if (framei.borderCubes.empty()) {
            LOG_L(m_log, 2, "SAFE: Frame F_", i, " is empty.");
            m_invariantLevel = i + 1;
            LOG_L(m_log, 2, "m_invariantLevel: ", m_invariantLevel);
            LOG_L(m_log, 2, FramesInfo());
            return true; // Proof found
        }
    }

    // No proof was found in this iteration.
    return false;
}


void BasicIC3::OutputCounterExample() {
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
    cout << cexPath << endl;
    std::ofstream cexFile;
    cexFile.open(cexPath);

    assert(m_cexStart != nullptr);

    cexFile << "1" << endl
            << "b0" << endl;

    shared_ptr<State> state = m_cexStart;
    cexFile << state->GetLatchesString() << endl;
    cexFile << state->GetInputsString() << endl;
    while (state->preState != nullptr) {
        state = state->preState;
        cexFile << state->GetInputsString() << endl;
    }

    cexFile << "." << endl;
    cexFile.close();
}


unsigned BasicIC3::addCubeToANDGates(aiger *circuit, vector<unsigned> cube) {
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

void BasicIC3::OutputWitness() {
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

    if (m_invariantLevel == 0 && m_model.GetEquivalenceMap().size() == 0) {
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
        eq_lits.emplace_back(addCubeToANDGates(witness_aig, {l1, l2 ^ 1}) ^ 1);
        eq_lits.emplace_back(addCubeToANDGates(witness_aig, {l1 ^ 1, l2}) ^ 1);
    }

    unsigned eq_cons;
    if (eq_lits.size() > 0) {
        eq_cons = addCubeToANDGates(witness_aig, eq_lits);
    }

    // prove on lvl 0
    if (m_invariantLevel == 0 || (m_frames[m_invariantLevel].borderCubes.empty())) {
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

    // P' = P & invariant
    // P' = !bad & ( O_0 | O_1 | ... | O_i )
    //             !( !O_0 & !O_1 & ...  & !O_i )
    //                 O_i = c_1 & c_2 & ... & c_j
    //                       c_j = !( l_1 & l_2 & .. & l_k )

    // P' = P & invariant
    // P' = !bad & F_{i+1}
    //             F_{i+1} = c_1 & c_2 & ... & c_j
    //                       c_j = !( l_1 & l_2 & .. & l_k )

    set<cube, IC3Frame::CubeComp> indInv;

    for (int i = m_invariantLevel; i <= m_k + 1; i++) {
        indInv.insert(m_frames[i].borderCubes.begin(), m_frames[i].borderCubes.end());
    }
    // if (indInv.empty()) {
    //     indInv.insert(infFrame.borderCubes.begin(), infFrame.borderCubes.end());
    // }
    assert(!indInv.empty());

    vector<unsigned> invLits;
    for (auto it = indInv.begin(); it != indInv.end(); it++) {
        vector<unsigned> cube_lits;
        for (int l : *it) cube_lits.push_back(l > 0 ? (2 * l) : (2 * -l + 1));
        unsigned cls = addCubeToANDGates(witness_aig, cube_lits) ^ 1;
        invLits.push_back(cls);
    }
    unsigned inv = addCubeToANDGates(witness_aig, invLits);

    unsigned bad_lit = m_model.GetAigerLit(m_model.GetBad());
    unsigned p = aiger_not(bad_lit);
    unsigned p_prime = addCubeToANDGates(witness_aig, {p, inv});

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
