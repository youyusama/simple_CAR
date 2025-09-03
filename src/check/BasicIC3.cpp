#include "BasicIC3.h"
#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>

namespace car {

BasicIC3::BasicIC3(Settings settings,
                   shared_ptr<Model> model,
                   shared_ptr<Log> log) : m_settings(settings),
                                          m_log(log),
                                          m_model(model) {
    State::numInputs = model->GetNumInputs();
    State::numLatches = model->GetNumLatches();
    m_cexStart = nullptr;

    // Initialize the dedicated solver for predecessor generalization (lifting).
    m_liftSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    if (m_settings.satSolveInDomain) m_liftSolver->SetSolveInDomain();
    m_liftSolver->AddTrans();
    // set permanent domain
    if (m_settings.satSolveInDomain)
        m_liftSolver->SetDomainCOI(make_shared<cube>(m_model->GetConstraints()));

    // no need to set domain for bad
    m_badPredLiftSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    m_badPredLiftSolver->AddTrans();
    m_badPredLiftSolver->AddTransK(1);
    clause cls;
    cls.push_back(-m_model->GetPrimeK(m_model->GetBad(), 1));
    for (auto cons : m_model->GetConstraints()) {
        cls.push_back(-m_model->GetPrimeK(cons, 1));
    }
    for (auto cons : m_model->GetConstraints()) {
        cls.push_back(-cons);
    }
    m_badPredLiftSolver->AddClause(cls);

    // Store the initial state literals in a set for efficient lookups.
    const auto &initState = m_model->GetInitialState();
    m_initialStateSet.insert(initState.begin(), initState.end());
    m_log->L(1, "BasicIC3 checker initialized.");

    GLOBAL_LOG = m_log;
    m_checkResult = CheckResult::Unknown;
    m_k = 0;
    m_invariantLevel = 0;
    lemmaCount = 0;

    m_branching = make_shared<Branching>(m_settings.branching);
    litOrder.branching = m_branching;
    blockerOrder.branching = m_branching;

    m_settings.satSolveInDomain = m_settings.satSolveInDomain && m_settings.solver == MCSATSolver::minicore;
    
}

BasicIC3::~BasicIC3() {
}

CheckResult BasicIC3::Run() {
    signal(SIGINT, signalHandler);

    if (Check(m_model->GetBad()))
        m_checkResult = CheckResult::Safe;
    else
        m_checkResult = CheckResult::Unsafe;

    m_log->PrintStatistics();

    return m_checkResult;
}

void BasicIC3::Extend() {
    AddNewFrames(); // Ensures frames up to F_{k+1} exist.

    m_startSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    m_startSolver->AddTrans();
    m_startSolver->AddTransK(1);
    m_startSolver->AddBadk(1);
    m_startSolver->AddProperty();
    m_startSolver->AddConstraints();
    m_startSolver->AddConstraintsK(1);
    AddSamePrimeConstraints(m_startSolver);

    auto blockingCubes = m_frames[m_k].borderCubes;
    for (const auto &blockingCube : blockingCubes) {
        clause lemma;
        lemma.reserve(blockingCube->size());
        for (const auto &lit : *blockingCube) {
            lemma.push_back(-lit);
        }
        m_startSolver->AddClause(lemma);
    }
}

bool BasicIC3::Check(int badId) {
    if (m_model->GetFalseId() == badId) {
        m_log->L(1, "SAFE: Constant bad.");
        return true;
    }
    if (!BaseCases()) {
        m_log->L(1, "UNSAFE: CEX found in base cases.");
        return false; // CEX found in 0 or 1 steps
    }

    m_log->L(1, "Base cases passed. Starting main IC3 loop.");

    AddNewFrame(); // This creates and adds F_0
    IC3Frame &frame0 = m_frames[0];

    // F_0 is defined as exactly the initial states.
    for (const auto &lit : m_initialStateSet) {
        frame0.solver->AddClause({lit});
        auto blockingCube = make_shared<cube>(cube{-lit});
        if (m_settings.satSolveInDomain) {
            frame0.solver->SetDomainCOI(blockingCube);
        }
    }

    // The main IC3 loop.
    for (m_k = 1;; ++m_k) {
        m_log->L(1, "==================== k=", m_k, " ====================");
        Extend();
        m_log->L(1, FramesInfo());

        if (!Strengthen()) {
            m_log->L(1, "UNSAFE: CEX found during strengthening of F_", m_k);
            return false;
        }

        if (Propagate()) {
            m_log->L(1, "SAFE: Proof found at F_", m_k);
            return true;
        }
    }

    return true; // Should be unreachable
}

bool BasicIC3::BaseCases() {
    // 0-step check: I & T & ~P
    // Check if any initial state is a bad state.
    auto baseSolver = make_shared<SATSolver>(m_model, m_settings.solver);
    baseSolver->AddTrans(); // Also load transition relation for combinational logic
    baseSolver->AddConstraints();
    baseSolver->AddBad();
    auto assumption = make_shared<cube>();
    assumption->insert(assumption->end(), m_initialStateSet.begin(), m_initialStateSet.end());

    if (baseSolver->Solve(assumption)) {
        m_log->L(1, "UNSAFE: Property fails in initial states.");
        pair<shared_ptr<cube>, shared_ptr<cube>> assignment = baseSolver->GetAssignment(false);
        m_cexStart = make_shared<State>(
            nullptr,
            assignment.first,
            assignment.second,
            0);
        return false;
    }

    // 1-step check: I & T & ~P'
    // Check if a bad state is reachable in one step.
    auto step1Solver = make_shared<SATSolver>(m_model, m_settings.solver);
    step1Solver->AddTrans();
    step1Solver->AddTransK(1);
    step1Solver->AddBadk(1);
    step1Solver->AddProperty();
    step1Solver->AddConstraints();
    step1Solver->AddConstraintsK(1);
    AddSamePrimeConstraints(step1Solver);

    if (step1Solver->Solve(assumption)) {
        m_log->L(1, "UNSAFE: Property fails at step 1.");
        cube primeInputs;
        shared_ptr<vector<int>> coiInputs = m_model->GetCOIInputs();
        for (int i : *coiInputs) {
            int i_p = m_model->GetPrimeK(i, 1);
            if (step1Solver->GetModel(i_p))
                primeInputs.push_back(i);
            else
                primeInputs.push_back(-i);
        }
        pair<shared_ptr<cube>, shared_ptr<cube>> assignment = step1Solver->GetAssignment(false);
        shared_ptr<State> badState(new State(nullptr, make_shared<cube>(primeInputs), nullptr, 0));
        m_cexStart = make_shared<State>(
            badState,
            assignment.first,
            assignment.second,
            1 // depth
        );
        return false;
    }
    return true;
}

void BasicIC3::AddNewFrames() {
    // We need frames up to index k+1. The size should be at least k+2.
    while (m_frames.size() <= m_k + 1) {
        AddNewFrame();
    }
}

void BasicIC3::AddNewFrame() {
    m_log->L(2, "Adding new frame F_", m_frames.size());
    IC3Frame newFrame;
    newFrame.k = m_frames.size();
    newFrame.solver = make_shared<SATSolver>(m_model, m_settings.solver);
    if (m_settings.satSolveInDomain) newFrame.solver->SetSolveInDomain();
    newFrame.solver->AddTrans();
    newFrame.solver->AddConstraints();
    newFrame.solver->AddProperty();
    AddSamePrimeConstraints(newFrame.solver);
    m_frames.push_back(newFrame);
}

void BasicIC3::AddBlockingCube(const shared_ptr<cube> &blockingCube, int frameLevel, bool toAll) {
    assert(frameLevel >= 1);
    auto insertionResult = m_frames[frameLevel].borderCubes.insert(blockingCube);
    if (!insertionResult.second) {
        return;
    }

    m_earliest = min(m_earliest, frameLevel);
    if (toAll) {
        lemmaCount++;
        m_log->L(2, "Frame ", frameLevel, ": ", CubeToStr(blockingCube));
    }


    clause lemma;
    lemma.reserve(blockingCube->size());
    for (const auto &lit : *blockingCube) {
        lemma.push_back(-lit);
    }
    for (int i = toAll ? 1 : frameLevel; i <= frameLevel; ++i) {
        if (m_frames[i].solver) {
            m_frames[i].solver->AddClause(lemma);
            if (m_settings.satSolveInDomain) {
                m_frames[i].solver->SetDomainCOI(blockingCube);
            }
        }
    }
    if (frameLevel >= m_k) {
        m_startSolver->AddClause(lemma);
    }
}

shared_ptr<cube> BasicIC3::GetCore(const shared_ptr<SATSolver> &solver, const shared_ptr<cube> &fallbackCube, bool prime) {
    unordered_set<int> conflictSet = solver->GetConflict();
    shared_ptr<cube> core = make_shared<cube>();
    if (!prime) {
        for (const auto &lit : *fallbackCube) {
            if (conflictSet.count(lit)) {
                core->push_back(lit);
            }
        }
        return core;
    } else {
        for (const auto &lit : *fallbackCube) {
            int lit_p = m_model->GetPrimeK(lit, 1);
            if (conflictSet.count(lit_p)) {
                core->push_back(lit);
            }
        }
        return core;
    }
}

shared_ptr<State> BasicIC3::EnumerateStartState() {
    m_log->L(2, "Searching for a start state at level ", m_k);
    if (m_startSolver->Solve()) {
        m_trivial = false;

        cube primeInputs;
        cube badInputs;
        shared_ptr<vector<int>> coiInputs = m_model->GetCOIInputs();
        for (int i : *coiInputs) {
            int i_p = m_model->GetPrimeK(i, 1);
            if (m_startSolver->GetModel(i_p)) {
                primeInputs.push_back(i_p);
                badInputs.push_back(i);
            } else {
                primeInputs.push_back(-i_p);
                badInputs.push_back(-i);
            }
        }

        pair<shared_ptr<cube>, shared_ptr<cube>> assignment = m_startSolver->GetAssignment(false);
        shared_ptr<cube> partialLatch = make_shared<cube>(*assignment.second);

        while (true) {
            shared_ptr<cube> assumps = make_shared<cube>();
            assumps->insert(assumps->end(), partialLatch->begin(), partialLatch->end());
            OrderAssumption(assumps);
            assumps->insert(assumps->end(), assignment.first->begin(), assignment.first->end());
            assumps->insert(assumps->end(), primeInputs.begin(), primeInputs.end());

            bool res = m_badPredLiftSolver->Solve(assumps);
            assert(!res);
            shared_ptr<cube> tempCore = GetCore(m_badPredLiftSolver, partialLatch, false);
            if (tempCore->size() == partialLatch->size()) {
                break;
            } else {
                partialLatch = tempCore;
            }
        }

        assignment.second = partialLatch;

        shared_ptr<State> badState(new State(nullptr, make_shared<cube>(badInputs), nullptr, 0));
        shared_ptr<State> ctiState = make_shared<State>(
            badState,
            assignment.first,
            assignment.second,
            1);
        m_log->L(2, "Found start state at level ", m_k, ": ", CubeToStr(ctiState->latches), ", input: ", CubeToStr(ctiState->inputs));
        return ctiState;
    } else {
        m_log->L(2, "No start state found at level ", m_k);
        return nullptr;
    }
}

bool BasicIC3::Strengthen() {
    m_trivial = true;
    m_earliest = m_k + 1;

    while (true) {
        shared_ptr<State> startState = EnumerateStartState();
        if (startState != nullptr) {
            set<Obligation> obligations;
            obligations.emplace(startState, m_k - 1, 1);

            if (!HandleObligations(obligations)) {
                return false;
            }
        } else {
            m_log->L(2, "No more CTIs at level ", m_k, ". Frame is strengthened.");
            return true;
        }
    }
}

bool BasicIC3::HandleObligations(set<Obligation> &obligations) {
    while (!obligations.empty()) {
        Obligation ob = *obligations.begin();

        // Query: F_{ob.level} & T & cti'
        m_log->L(2, "Handling obligation for state at level ", ob.level, " with depth ", ob.depth);

        const shared_ptr<SATSolver> &frameSolver = m_frames[ob.level].solver;
        const shared_ptr<cube> &ctiCube = ob.state->latches;

        if (UnreachabilityCheck(ctiCube, frameSolver)) {
            obligations.erase(obligations.begin());
            auto newBlockingCube = GetAndValidateCore(frameSolver, ctiCube);

            size_t pushLevel = Generalize(newBlockingCube, ob.level);

            if (pushLevel <= m_k) {
                m_log->L(2, "Creating new obligation for same state at higher level ", pushLevel);
                obligations.insert(Obligation(ob.state, pushLevel, ob.depth));
            }
        } else {
            pair<shared_ptr<cube>, shared_ptr<cube>> assignment = frameSolver->GetAssignment(false);
            auto predecessorState = make_shared<State>(ob.state, assignment.first, assignment.second, ob.depth + 1);
            if (ob.level == 0) {
                m_log->L(1, "UNSAFE: Found a path from the initial state.");
                m_cexStart = predecessorState;
                return false;
            }

            GeneralizePredecessor(predecessorState, ob.state);

            m_log->L(2, "Found predecessor for CTI. New obligation at level ", ob.level - 1);
            obligations.insert(Obligation(predecessorState, ob.level - 1, ob.depth + 1));
        }
    }
    return true;
}

bool BasicIC3::InductionCheck(const shared_ptr<cube> &cb, const shared_ptr<SATSolver> &slv) {
    clause cls;
    cls.reserve(cb->size());
    for (const auto &lit : *cb) {
        cls.push_back(-lit);
    }
    slv->AddTempClause(cls);
    auto assumption = make_shared<cube>(*cb);
    OrderAssumption(assumption);
    GetPrimed(assumption);
    if (m_settings.satSolveInDomain) {
        slv->ResetTempDomain();
        slv->SetTempDomainCOI(make_shared<cube>(*cb));
        slv->SetTempDomainCOI(assumption);
    }
    bool result = !slv->Solve(assumption);
    slv->ReleaseTempClause();
    return result;
}

bool BasicIC3::Down(const shared_ptr<cube> &downCube, int frameLvl, int recLvl, const set<int> &triedLits) {
    m_log->L(3, "Down: ", CubeToStr(downCube), " at frame level ", frameLvl, " and recursion level ", recLvl);
    int ctgs = 0;
    int joins = 0;
    const auto solverLvl = m_frames[frameLvl].solver;
    bool downRes = false;
    if (recLvl > m_settings.ctgMaxRecursionDepth) {
        m_log->L(3, "Max recursion depth reached, quick check");
        if (!InitiationCheck(downCube)) {
            return false;
        }
        if (InductionCheck(downCube, solverLvl)) {
            shared_ptr<cube> downCore = GetAndValidateCore(solverLvl, downCube);
            downCube->swap(*downCore);
            return true;
        }
        return false;
    }

    while (true) {
        m_log->L(3, "Down attempt: ", CubeToStr(downCube));
        if (!InitiationCheck(downCube)) {
            return false;
        }
        if (InductionCheck(downCube, solverLvl)) {
            shared_ptr<cube> downCore = GetAndValidateCore(solverLvl, downCube);
            downCube->swap(*downCore);
            return true;
        }

        shared_ptr<State> downState = make_shared<State>(nullptr, nullptr, downCube, 0);
        pair<shared_ptr<cube>, shared_ptr<cube>> ctgAssignment = solverLvl->GetAssignment(false);
        auto ctgState = make_shared<State>(downState, ctgAssignment.first, ctgAssignment.second, 0);
        GeneralizePredecessor(ctgState, downState);

        const shared_ptr<cube> &ctgCube = ctgState->latches;
        m_log->L(3, "CTG cube: ", CubeToStr(ctgCube));

        if (!InitiationCheck(ctgCube)) {
            return false;
        }

        const auto solverLvlMinus1 = m_frames[frameLvl - 1].solver;
        if (ctgs < m_settings.ctgMaxStates && frameLvl > 1 && InductionCheck(ctgCube, solverLvlMinus1)) {
            ctgs++;
            m_log->L(3, "CTG is inductive at level ", frameLvl - 1);
            shared_ptr<cube> ctgCore = GetAndValidateCore(solverLvlMinus1, ctgCube);

            int pushLevel = PushLemmaForward(ctgCore, frameLvl);
            if (MIC(ctgCore, pushLevel - 1, recLvl + 1)) {
                m_branching->Update(ctgCore);
            }
            m_log->L(2, "Learned ctg clause and pushed to frame ", pushLevel);
            AddBlockingCube(ctgCore, pushLevel, true);
        } else {
            ctgs = 0;
            shared_ptr<cube> joinCube = make_shared<cube>();
            for (int i = downCube->size() - 1; i >= 0; i--) {
                if (binary_search(ctgCube->begin(), ctgCube->end(), downCube->at(i))) {
                    joinCube->push_back(downCube->at(i));
                } else if (triedLits.count(downCube->at(i))) {
                    return false;
                }
            }
            m_log->L(3, "Joint cube: ", CubeToStr(joinCube));
            downCube->swap(*joinCube);
        }
    }
}

void BasicIC3::GetBlockers(const shared_ptr<cube> &blockingCube, int framelevel, vector<shared_ptr<cube>> &blockers) {
    int size = -1;
    for (auto it : m_frames[framelevel].borderCubes) {
        shared_ptr<cube> cb = make_shared<cube>(*it);
        if (size != -1 && size < cb->size()) break;
        if (includes(blockingCube->begin(), blockingCube->end(), cb->begin(), cb->end(), cmp)) {
            size = cb->size();
            blockers.push_back(cb);
        }
    }
}

size_t BasicIC3::Generalize(const shared_ptr<cube> &cb, int frameLvl) {
    m_log->L(3, "Generalizing cube: ", CubeToStr(cb), ", at frameLvl: ", frameLvl);
    if (MIC(cb, frameLvl, 1)) {
        m_branching->Update(cb);
    }
    int pushLevel = PushLemmaForward(cb, frameLvl + 1);
    m_log->L(2, "Learned clause and pushed to frame ", pushLevel);
    AddBlockingCube(cb, pushLevel, true);
    return pushLevel;
}

bool BasicIC3::MIC(const shared_ptr<cube> &cb, int frameLvl, int recLvl) {
    m_log->L(3, "MIC: ", CubeToStr(cb), ", at frameLvl: ", frameLvl, ", recLvl: ", recLvl);

    vector<shared_ptr<cube>> blockers;
    shared_ptr<cube> blocker = make_shared<cube>();
    set<int> triedLits;

    if (m_settings.referSkipping && frameLvl > 0) {
        GetBlockers(cb, frameLvl, blockers);
        if (blockers.size() > 0) {
            if (m_settings.branching > 0) {
                sort(blockers.begin(), blockers.end(), blockerOrder);
            }
            *blocker = *blockers[0];
        }
        for (const auto &lit : *blocker) {
            triedLits.insert(lit);
        }
    }


    const int maxMicAttempts = 3;
    size_t attempts = maxMicAttempts;

    OrderAssumption(cb);
    // Iterate backwards to handle the shrinking cube size gracefully.
    for (int i = cb->size() - 1; i >= 0; --i) {
        if (cb->size() < 2) break;
        int litToDrop = cb->at(i);

        // If we have already tried and failed to drop this literal, skip.
        if (triedLits.count(litToDrop)) {
            continue;
        }

        // Create a temporary cube with one literal removed.
        shared_ptr<cube> dropCube = make_shared<cube>();
        dropCube->reserve(cb->size() - 1);
        for (int j = 0; j < cb->size(); ++j) {
            if (i == j) continue;
            dropCube->push_back(cb->at(j));
        }

        if (Down(dropCube, frameLvl, recLvl, triedLits)) {
            // dropCube is sorted
            cb->swap(*dropCube);
            OrderAssumption(cb);
            i = cb->size();
            attempts = maxMicAttempts;
        } else {
            if (--attempts == 0) {
                m_log->L(3, "Max MIC attempts reached, stopping generalization.");
                break;
            }
            triedLits.insert(litToDrop);
        }
    }
    sort(cb->begin(), cb->end(), cmp);
    if (cb->size() > blocker->size() && frameLvl != 0) {
        return false;
    } else {
        return true;
    }
}


void BasicIC3::GeneralizePredecessor(const shared_ptr<State> &predecessorState, const shared_ptr<State> &successorState) {
    m_log->L(3, "Generalizing predecessor. Initial latch size: ", predecessorState->latches->size(), ", input size: ", predecessorState->inputs->size(), ", Successor state latch size: ", successorState->latches->size());

    clause succNegationClause;
    succNegationClause.reserve(successorState->latches->size());
    for (const auto &lit : *(successorState->latches)) {
        succNegationClause.push_back(-m_model->GetPrimeK(lit, 1));
    }
    for (auto cons : m_model->GetConstraints()) {
        succNegationClause.push_back(-cons);
    }
    m_liftSolver->AddTempClause(succNegationClause);
    if (m_settings.satSolveInDomain) {
        m_liftSolver->ResetTempDomain();
        shared_ptr<cube> primeLatches = make_shared<cube>();
        for (const auto &lit : *(successorState->latches)) {
            primeLatches->push_back(m_model->GetPrimeK(lit, 1));
        }
        m_liftSolver->SetTempDomainCOI(primeLatches);
    }

    const auto &partialLatch = predecessorState->latches;

    while (true) {
        auto assumption = make_shared<cube>(*partialLatch);
        OrderAssumption(assumption);
        assumption->insert(assumption->begin(), predecessorState->inputs->begin(), predecessorState->inputs->end());
        // There exist some successors whose predecessors are the entire set. (All latches are determined solely by the inputs.)

        bool result = m_liftSolver->Solve(assumption);
        assert(!result);

        auto core = GetCore(m_liftSolver, partialLatch, false);
        m_log->L(3, "Core size: ", core->size(), ", Partial latch size: ", partialLatch->size());

        if (core->size() >= partialLatch->size()) {
            break;
        } else {
            partialLatch->swap(*core);
        }
    }
    m_liftSolver->ReleaseTempClause();
    m_log->L(3, "Generalized predecessor. Final latch size: ", predecessorState->latches->size());
}

bool BasicIC3::InitiationCheck(const shared_ptr<cube> &cb) {
    for (const auto &lit : *cb) {
        if (m_initialStateSet.count(-lit)) {
            return true; // Disjoint (UNSAT), check passes.
        }
    }
    m_log->L(3, "Initiation check failed.");
    return false;
}


// try to add negated literals from initial states
void BasicIC3::InitiationAugmentation(const shared_ptr<cube> &failureCube, const shared_ptr<cube> &fallbackCube) {
    for (const auto &lit : *fallbackCube) {
        if (m_initialStateSet.count(-lit)) {
            failureCube->push_back(lit);
            break;
        }
    }
    sort(failureCube->begin(), failureCube->end(), cmp);
}

// fallbackCube is sorted
shared_ptr<cube> BasicIC3::GetAndValidateCore(const shared_ptr<SATSolver> &solver, const shared_ptr<cube> &fallbackCube) {
    shared_ptr<cube> core = GetCore(solver, fallbackCube, true);
    m_log->L(3, "Got UNSAT core: ", CubeToStr(core));
    if (!InitiationCheck(core)) {
        m_log->L(3, "GetAndValidateCore: core intersects with initial states. Reverting to fallback cube.");
        // InitiationAugmentation(core, fallbackCube);
        *core = *fallbackCube;
    }
    return core;
}

string BasicIC3::FramesInfo() const {
    stringstream ss;
    ss << "Frames: ";
    for (size_t i = 0; i < m_frames.size(); ++i) {
        ss << "F" << i << "[" << m_frames[i].borderCubes.size() << "] ";
    }
    return ss.str();
}

bool BasicIC3::UnreachabilityCheck(const shared_ptr<cube> &cb, const shared_ptr<SATSolver> &slv) {
    auto assumption = make_shared<cube>(*cb);
    OrderAssumption(assumption);
    GetPrimed(assumption);
    if (m_settings.satSolveInDomain) {
        slv->ResetTempDomain();
        slv->SetTempDomainCOI(assumption);
    }
    bool result = !slv->Solve(assumption);
    return result;
}

int BasicIC3::PushLemmaForward(const shared_ptr<cube> &cb, int startLevel) {
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

bool BasicIC3::Propagate() {
    m_log->L(1, "Propagating clauses.");

    m_log->L(2, "Cleaning up redundant clauses.");
    set<shared_ptr<cube>, cubePtrComp> allCubes;
    for (int i = m_k + 1; i >= m_earliest; --i) {
        IC3Frame &frame = m_frames[i];
        if (frame.borderCubes.empty()) continue;
        size_t originalSize = frame.borderCubes.size();

        set<shared_ptr<cube>, cubePtrComp> remainingCubes;
        set_difference(frame.borderCubes.begin(), frame.borderCubes.end(),
                       allCubes.begin(), allCubes.end(),
                       inserter(remainingCubes, remainingCubes.end()),
                       cubePtrComp());

        if (originalSize != remainingCubes.size()) {
            m_log->L(3, "Frame ", i, " cleanup: ", originalSize, " -> ", remainingCubes.size());
        }

        frame.borderCubes.swap(remainingCubes);
        allCubes.insert(frame.borderCubes.begin(), frame.borderCubes.end());
    }

    m_log->L(2, "Propagating clauses forward.");
    for (int i = m_trivial ? m_k : 1; i <= m_k; ++i) {
        IC3Frame &framei = m_frames[i];
        int cubesKept = 0;
        int cubesPropagated = 0;

        for (auto it = framei.borderCubes.begin(); it != framei.borderCubes.end();) {
            const auto &cb = *it;
            if (UnreachabilityCheck(cb, framei.solver)) {
                cubesPropagated++;
                auto core = GetAndValidateCore(framei.solver, cb);
                AddBlockingCube(core, i + 1, core->size() < cb->size());
                m_branching->Update(core);
                // Safely erase and advance the iterator
                it = framei.borderCubes.erase(it);
            } else {
                cubesKept++;
                ++it;
            }
        }

        m_log->L(2, "Frame ", i, " propagation: ", cubesPropagated, " propagated, ", cubesKept, " kept.");


        if (framei.borderCubes.empty()) {
            m_log->L(1, "SAFE: Frame F_", i, " is empty.");
            m_invariantLevel = i + 1;
            m_log->L(1, "m_invariantLevel: ", m_invariantLevel);
            m_log->L(1, FramesInfo());
            m_log->L(1, "lemmaCount: ", lemmaCount);
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

void BasicIC3::OutputWitness(int bad) {
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
    // bad is constant
    // if (m_invariantLevel == 0 || (m_frames[m_invariantLevel].borderCubes.empty() && infFrame.borderCubes.empty())) {
    if (m_invariantLevel == 0 || (m_frames[m_invariantLevel].borderCubes.empty())) {
        aiger_open_and_write_to_file(model_aig, outPath.c_str());
        return;
    }
    lvl_i = m_k;

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

    // P' = P & invariant
    // P' = !bad & F_{i+1}
    //             F_{i+1} = c_1 & c_2 & ... & c_j
    //                       c_j = !( l_1 & l_2 & .. & l_k )

    set<shared_ptr<cube>, cubePtrComp> indInv;

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
        for (int l : **it) cube_lits.push_back(l > 0 ? (2 * l) : (2 * -l + 1));
        unsigned cls = addCubeToANDGates(witness_aig, cube_lits) ^ 1;
        invLits.push_back(cls);
    }
    unsigned inv = addCubeToANDGates(witness_aig, invLits);

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


void BasicIC3::Witness() {
    if (m_checkResult == CheckResult::Unsafe) {
        m_log->L(1, "Generating counterexample.");
        OutputCounterExample();
    } else if (m_checkResult == CheckResult::Safe) {
        m_log->L(1, "Generating proof.");
        OutputWitness(m_model->GetBad());
    } else {
        m_log->L(1, "Unknown check result.");
    }
}

void BasicIC3::AddSamePrimeConstraints(shared_ptr<SATSolver> slv) {
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

} // namespace car