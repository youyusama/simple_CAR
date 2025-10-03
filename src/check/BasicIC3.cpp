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
    m_rng = make_shared<RNG>(m_settings.randomSeed);
    micMaxAttempts = m_settings.micMaxAttempts ? m_settings.micMaxAttempts : INT32_MAX;

    badCube = make_shared<cube>();
    badCube->push_back(m_model->GetBad());

    if (m_settings.solver == MCSATSolver::cadical) {
        // initialize m_T and m_TT
        m_log->Tick();
        m_T = make_shared<SATSolver>(m_model, MCSATSolver::cadical);
        m_T->AddTrans();
        m_log->StatSolverCreate();
        SetPreprocessingOptions(m_T);
        m_log->Tick();
        if (m_settings.cadicalSimplify)
            m_T->Simplify();
        m_log->StatCadicalSimplify();
        SetSolvingOptions(m_T);
        if (m_settings.bad_pred) {
            m_log->Tick();
            m_TT = make_shared<SATSolver>(m_T);
            m_TT->AddTransK(1);
            m_log->StatSolverCreate();
            SetPreprocessingOptions(m_TT);
            m_log->Tick();
            if (m_settings.cadicalSimplify)
                m_TT->Simplify();
            m_log->StatCadicalSimplify();
            SetSolvingOptions(m_TT);
        }
    }

    // Initialize the dedicated solver for predecessor generalization (lifting).
    m_liftSolver = NewT();
    if (m_settings.satSolveInDomain) m_liftSolver->SetSolveInDomain();
    // set permanent domain
    if (m_settings.satSolveInDomain)
        m_liftSolver->SetDomainCOI(make_shared<cube>(m_model->GetConstraints()));

    if (m_settings.bad_pred) {
        m_badPredLiftSolver = NewTT();
        clause cls;
        cls.push_back(-m_model->GetPrimeK(m_model->GetBad(), 1));
        for (auto cons : m_model->GetConstraints()) {
            cls.push_back(-m_model->GetPrimeK(cons, 1));
        }
        for (auto cons : m_model->GetConstraints()) {
            cls.push_back(-cons);
        }
        m_badPredLiftSolver->AddClause(cls);
    }

    // Store the initial state literals in a set for efficient lookups.
    const auto &initState = m_model->GetInitialState();
    m_initialStateSet.insert(initState.begin(), initState.end());
    m_log->L(1, "BasicIC3 checker initialized.");

    GLOBAL_LOG = m_log;
    m_checkResult = CheckResult::Unknown;
    m_invariantLevel = 0;
    lemmaCount = 0;

    m_branching = make_shared<Branching>(m_settings.branching);
    litOrder.branching = m_branching;

    m_settings.satSolveInDomain = m_settings.satSolveInDomain && m_settings.solver == MCSATSolver::minicore;
}

BasicIC3::~BasicIC3() {
}

void BasicIC3::SetPreprocessingOptions(const shared_ptr<SATSolver> &slv) {
    if (m_settings.solver == MCSATSolver::cadical && m_settings.cadicalOptionsPre) {
        slv->SetOption("block", 1);
        slv->SetOption("cover", 1);
        slv->SetOption("condition", 1);
    }
}

void BasicIC3::SetSolvingOptions(const shared_ptr<SATSolver> &slv) {
    if (m_settings.solver == MCSATSolver::cadical && m_settings.cadicalOptionsPre) {
        slv->SetOption("block", 0);
        slv->SetOption("cover", 0);
        slv->SetOption("condition", 0);
    }
    if (m_settings.solver == MCSATSolver::cadical && m_settings.cadicalOptionsSol) {
        slv->SetOption("ilb", 1);
        slv->SetOption("ilbassumptions", 1);

        slv->SetOption("prob", 0);
        slv->SetOption("compact", 0);

        slv->SetOption("flush", 1);
        slv->SetOption("walk", 0);
        slv->SetOption("score", 0);
    }
}

shared_ptr<SATSolver> BasicIC3::NewT() {
    m_log->Tick();
    shared_ptr<SATSolver> slv;
    if (m_settings.solver == MCSATSolver::cadical) {
        slv = make_shared<SATSolver>(m_T);
    } else {
        slv = make_shared<SATSolver>(m_model, m_settings.solver);
        slv->AddTrans();
    }
    m_log->StatSolverCreate();
    return slv;
}

shared_ptr<SATSolver> BasicIC3::NewTT() {
    m_log->Tick();
    shared_ptr<SATSolver> slv;
    if (m_settings.solver == MCSATSolver::cadical) {
        slv = make_shared<SATSolver>(m_TT);
    } else {
        slv = make_shared<SATSolver>(m_model, m_settings.solver);
        slv->AddTrans();
        slv->AddTransK(1);
    }
    m_log->StatSolverCreate();
    return slv;
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

bool BasicIC3::BaseCheck() {
    AddNewFrame(); // This creates and adds F_0
    IC3Frame &frame0 = m_frames[0];

    // F_0 is defined as exactly the initial states.
    for (const auto &lit : m_initialStateSet) {
        frame0.solver->AddClause({lit});
        if (m_settings.satSolveInDomain) {
            auto blockingCube = make_shared<cube>(cube{-lit});
            frame0.solver->SetDomainCOI(blockingCube);
        }
    }
    frame0.solver->AddInitialClauses();

    if (m_settings.satSolveInDomain) {
        frame0.solver->ResetTempDomain();
        frame0.solver->SetTempDomainCOI(badCube);
    }
    if (frame0.solver->Solve(badCube)) {
        pair<shared_ptr<cube>, shared_ptr<cube>> assignment = frame0.solver->GetAssignment(false);
        auto badState = make_shared<State>(nullptr, assignment.first, assignment.second, 0);
        m_log->L(1, "UNSAFE: Found a path from the initial state.");
        m_cexStart = badState;
        return false;
    }

    if (m_settings.bad_pred) {
        auto assumption = make_shared<cube>();
        assumption->insert(assumption->end(), m_initialStateSet.begin(), m_initialStateSet.end());
        auto step1Solver = NewTT();
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
        AddNewFrame(); // This creates and adds F_1
    }
    return true;
}

// Create start solver, should be equivalent to F_{MaxLevel()-1}
void BasicIC3::NewStartSolver() {
    m_startSolver = NewTT();
    m_startSolver->AddBadk(1);
    m_startSolver->AddProperty();
    m_startSolver->AddConstraints();
    m_startSolver->AddConstraintsK(1);
    AddSamePrimeConstraints(m_startSolver);

    assert(MaxLevel() >= 2);

    auto blockingCubes = m_frames[MaxLevel() - 1].borderCubes;
    blockingCubes.insert(blockingCubes.end(),
                         m_frames[MaxLevel()].borderCubes.begin(),
                         m_frames[MaxLevel()].borderCubes.end());
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

    if (!BaseCheck()) {
        m_log->L(1, "UNSAFE: Base case failed.");
        return false;
    }

    AddNewFrame(); // This creates and adds F_1 (or F_2 if bad_pred is set)

    // The main IC3 loop.
    while (true) {
        m_log->L(1, "============================================", " Starting IC3 iteration at F_", MaxLevel(), " ============================================");
        m_log->L(1, FramesInfo());
        m_log->L(1, "lemmaCount: ", lemmaCount);

        if (m_settings.bad_pred)
            NewStartSolver();

        if (!Strengthen()) {
            m_log->L(1, "UNSAFE: CEX found during strengthening of F_", MaxLevel());
            return false;
        }

        if (m_invariantLevel) {
            m_log->L(1, "m_invariantLevel: ", m_invariantLevel);
            m_log->L(1, FramesInfo());
            m_log->L(1, "lemmaCount: ", lemmaCount);
            return true; // Proof found
        }

        AddNewFrame();

        if (Propagate()) {
            m_log->L(1, "SAFE: Proof found when propagating lemmas at F_", MaxLevel());
            return true;
        }
    }

    return true; // Should be unreachable
}

void BasicIC3::AddNewFrame() {
    m_log->L(2, "Adding new frame F_", m_frames.size());
    IC3Frame newFrame;
    newFrame.k = m_frames.size();
    newFrame.solver = NewT();
    if (m_settings.satSolveInDomain) newFrame.solver->SetSolveInDomain();
    newFrame.solver->AddConstraints();
    if (m_settings.bad_pred && newFrame.k >= 1)
        newFrame.solver->AddProperty();
    AddSamePrimeConstraints(newFrame.solver);
    m_frames.push_back(newFrame);
}

void BasicIC3::AddBlockingCube(const shared_ptr<cube> &blockingCube, int frameLevel, bool lazyCheck) {
    assert(frameLevel >= 1);
    if (!lazyCheck) {
        lemmaCount++;
        m_log->L(2, "Frame ", frameLevel, ": ", CubeToStr(blockingCube));
    }

    if (lazyCheck && LazyCheck(blockingCube, frameLevel) != -1) {
        return;
    }

    m_log->Tick();
    clause lemma;
    lemma.reserve(blockingCube->size());
    for (const auto &lit : *blockingCube) {
        lemma.push_back(-lit);
    }

    if (m_settings.bad_pred && !lazyCheck && frameLevel >= MaxLevel() - 1) {
        // Add to start solver as well
        m_startSolver->AddClause(lemma);
    }

    int beginLevel = -1;
    for (int i = frameLevel; i >= 1; --i) {
        int j = 0;
        while (j < m_frames[i].borderCubes.size()) {
            shared_ptr<cube> existingCube = m_frames[i].borderCubes[j];
            // if existingCube subsumes or equals blockingCube(all literals in existingCube are in blockingCube)
            if (includes(blockingCube->begin(), blockingCube->end(), existingCube->begin(), existingCube->end())) {
                // if equal
                if (blockingCube->size() == existingCube->size()) {
                    // swap and pop_back for efficiency
                    m_frames[i].borderCubes[j] = m_frames[i].borderCubes.back();
                    m_frames[i].borderCubes.pop_back();
                    for (int k = i + 1; k <= frameLevel; ++k) {
                        m_frames[k].solver->AddClause(lemma);
                        if (m_settings.satSolveInDomain) {
                            m_frames[k].solver->SetDomainCOI(blockingCube);
                        }
                    }
                    m_frames[frameLevel].borderCubes.push_back(blockingCube);
                    m_earliest = min(m_earliest, i + 1);
                    m_log->StatAddBlockingCube();
                    return;
                }

                beginLevel = i + 1;
                break;
            }
            // if blockingCube subsumes existingCube (all literals in blockingCube are in existingCube)
            if (includes(existingCube->begin(), existingCube->end(), blockingCube->begin(), blockingCube->end())) {
                // swap and pop_back for efficiency
                m_frames[i].borderCubes[j] = m_frames[i].borderCubes.back();
                m_frames[i].borderCubes.pop_back();
                continue;
            }
            j++;
        }
        if (i != frameLevel && m_frames[i].borderCubes.empty()) {
            if (!m_settings.bad_pred || i != MaxLevel() - 1) {
                m_invariantLevel = i + 1;
                m_log->L(1, "SAFE: Invariant found at F_", m_invariantLevel, " when adding blocking cube.");
            }
        }
        if (beginLevel != -1) break;
    }


    if (beginLevel == -1) beginLevel = 1;
    for (int i = beginLevel; i <= frameLevel; ++i) {
        m_frames[i].solver->AddClause(lemma);
        if (m_settings.satSolveInDomain) {
            m_frames[i].solver->SetDomainCOI(blockingCube);
        }
    }
    m_frames[frameLevel].borderCubes.push_back(blockingCube);
    m_earliest = min(m_earliest, beginLevel);
    m_log->StatAddBlockingCube();
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

bool BasicIC3::EnumerateStartState() {
    m_log->L(3, "Searching for a start state at level ", m_settings.bad_pred ? MaxLevel() - 1 : MaxLevel());

    if (!m_settings.bad_pred) {
        shared_ptr<SATSolver> startSolver = m_frames[MaxLevel()].solver;

        if (m_settings.satSolveInDomain) {
            startSolver->ResetTempDomain();
            startSolver->SetTempDomainCOI(badCube);
        }
        m_log->Tick();
        if (!startSolver->Solve(badCube)) {
            m_log->StatStartSolver();
            return false;
        } else {
            m_log->StatStartSolver();
            pair<shared_ptr<cube>, shared_ptr<cube>> assignment = startSolver->GetAssignment(false);
            auto badState = make_shared<State>(nullptr, assignment.first, assignment.second, 0);

            m_log->L(3, "Found start state at level ", MaxLevel());
            GeneralizePredecessor(badState, badCube);
            m_log->L(2, "Found bad State. New obligation at level ", MaxLevel());
            PushObligation(make_shared<Obligation>(badState, MaxLevel(), 0), MaxLevel());

            return true;
        }
    } else {
        m_log->Tick();
        if (m_startSolver->Solve()) {
            m_log->StatStartSolver();
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
            const shared_ptr<cube> &partialLatch = assignment.second;

            int strategy = 1;
            while (true) {
                shared_ptr<cube> assumps = make_shared<cube>(*partialLatch);
                if (!m_settings.liftRand)
                    OrderAssumption(assumps);
                else {
                    OrderAssumptionWithStrategy(assumps, strategy);
                    strategy++;
                }
                assumps->insert(assumps->end(), assignment.first->begin(), assignment.first->end());
                assumps->insert(assumps->end(), primeInputs.begin(), primeInputs.end());

                bool res = m_badPredLiftSolver->Solve(assumps);
                assert(!res);
                shared_ptr<cube> tempCore = GetCore(m_badPredLiftSolver, partialLatch, false);
                if (tempCore->size() >= partialLatch->size()) {
                    break;
                } else {
                    partialLatch->swap(*tempCore);
                }
            }

            shared_ptr<State> badState(new State(nullptr, make_shared<cube>(badInputs), nullptr, 0));
            shared_ptr<State> ctiState = make_shared<State>(
                badState,
                assignment.first,
                assignment.second,
                1);
            m_log->L(2, "Found start state at level ", MaxLevel() - 1, ": ", CubeToStr(ctiState->latches), ", input: ", CubeToStr(ctiState->inputs));
            PushObligation(make_shared<Obligation>(ctiState, MaxLevel() - 1, 1), MaxLevel() - 1);
            return true;
        } else {
            m_log->StatStartSolver();
            return false;
        }
    }
}

bool BasicIC3::Strengthen() {
    m_earliest = m_settings.bad_pred ? MaxLevel() - 1 : MaxLevel();

    bool result = true;
    while (true) {
        if (!HandleObligations()) {
            result = false;
            break;
        }
        if (m_invariantLevel) {
            result = true;
            break;
        }
        if (!EnumerateStartState()) {
            m_log->L(2, "No more CTIs at level ", m_settings.bad_pred ? MaxLevel() - 1 : MaxLevel(), ". Strengthening done.");
            result = true;
            break;
        }
        if (m_cexStart != nullptr) {
            result = false;
            break;
        }
    }
    return result;
}

shared_ptr<Obligation> BasicIC3::PopObligation() {
    if (m_obligations.empty()) return nullptr;
    shared_ptr<Obligation> ob = *m_obligations.begin();
    if (ob->level > MaxLevel()) return nullptr;
    m_obligations.erase(m_obligations.begin());
    ob->act += 1.0;
    m_log->L(2, "Popping obligation for state at level ", ob->level, " with depth ", ob->depth, " and activity ", ob->act);
    return ob;
}

void BasicIC3::PushObligation(const shared_ptr<Obligation> &ob, int newLevel) {
    while (ob->level < newLevel) {
        ob->act *= 0.6;
        ob->level++;
    }
    m_log->L(2, "Pushing obligation for state at level ", ob->level, " with depth ", ob->depth, " and activity ", ob->act);
    m_obligations.insert(ob);
    m_log->L(2, "Total obligations: ", m_obligations.size());
}

// check if cube a subsumes b (i.e., a is a subset of b)
bool BasicIC3::SubsumeSet(const shared_ptr<cube> &a, const LitSet &b) {
    if (a->size() > b.size()) {
        return false;
    }
    for (const auto &lit : *a) {
        if (!b.has(lit)) {
            return false;
        }
    }
    return true;
}

int BasicIC3::LazyCheck(const shared_ptr<cube> &cb, int startLvl) {
    m_log->Tick();
    m_log->L(3, "LazyCheck: ", CubeToStr(cb), " starting from frame level ", startLvl);
    int result = -1;
    m_tmpLitSet.clear();
    for (const auto &lit : *cb) {
        m_tmpLitSet.insert(lit);
    }
    for (int i = startLvl; i <= MaxLevel(); ++i) {
        for (const auto &blockingCube : m_frames[i].borderCubes) {
            if (SubsumeSet(blockingCube, m_tmpLitSet)) {
                m_log->L(3, "LazyCheck: subsumed by blocking cube ", CubeToStr(blockingCube), " at frame level ", i);
                m_tmpLitSet.clear();
                result = i;
                m_log->StatLazyCheck();
                return result;
            }
        }
    }
    m_log->StatLazyCheck();
    return result;
}

bool BasicIC3::HandleObligations() {
    shared_ptr<Obligation> ob;
    while ((ob = PopObligation()) != nullptr) {
        assert(ob->level <= MaxLevel() && ob->level >= 1);
        if (ob->act >= m_settings.maxObligationAct) {
            m_log->L(2, "Obligation for state at level ", ob->level, " with depth ", ob->depth, " reached max activity. Skipped.");
            continue;
        }

        // Query: F_{ob.level} & T & cti'
        m_log->L(2, "Handling obligation for state at level ", ob->level, " with depth ", ob->depth);

        // need to add trivial contained optimization here
        int subsumeLvl = LazyCheck(ob->state->latches, ob->level);
        if (subsumeLvl != -1) {
            m_log->L(2, "Obligation subsumed by existing blocking cube at level ", subsumeLvl, ". Obligation removed.");
            ob->level = subsumeLvl + 1;
            PushObligation(ob, subsumeLvl + 1);
            continue;
        }

        const shared_ptr<SATSolver> &frameSolver = m_frames[ob->level - 1].solver;
        const shared_ptr<cube> &ctiCube = ob->state->latches;

        if (UnreachabilityCheck(ctiCube, frameSolver)) {
            auto newBlockingCube = GetAndValidateCore(frameSolver, ctiCube);
            size_t pushLevel = Generalize(newBlockingCube, ob->level);
            if (m_invariantLevel) return true;
            PushObligation(ob, pushLevel + 1);
        } else {
            pair<shared_ptr<cube>, shared_ptr<cube>> assignment = frameSolver->GetAssignment(false);
            auto predecessorState = make_shared<State>(ob->state, assignment.first, assignment.second, ob->depth + 1);
            if (ob->level == 1) {
                m_log->L(1, "UNSAFE: Found a path from the initial state.");
                m_cexStart = predecessorState;
                return false;
            }

            shared_ptr<cube> succ = make_shared<cube>(*ctiCube);
            GetPrimed(succ);
            GeneralizePredecessor(predecessorState, succ);

            m_log->L(2, "Found predecessor for CTI. New obligation at level ", ob->level - 1);
            PushObligation(ob, ob->level);
            PushObligation(make_shared<Obligation>(predecessorState, ob->level - 1, ob->depth + 1), ob->level - 1);
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
    m_log->Tick();
    bool result = !slv->Solve(assumption);
    m_log->StatFrameSolver();
    slv->ReleaseTempClause();
    return result;
}

bool BasicIC3::Down(const shared_ptr<cube> &downCube, int frameLvl, int recLvl, const unordered_set<int> &triedLits) {
    m_log->L(3, "Down: ", CubeToStr(downCube), " at frame level ", frameLvl, " and recursion level ", recLvl);
    int ctgs = 0;
    int joins = 0;
    assert(frameLvl >= 1);
    const auto downCubeSolver = m_frames[frameLvl - 1].solver;
    // if frameLvl == 1, there is no need to extract ctgCube in F_0, since F_0 is exact the initial states and cannot be blocked
    if (recLvl > m_settings.ctgMaxRecursionDepth || frameLvl == 1) {
        m_log->L(3, "Max recursion depth reached, quick check");
        if (!InitiationCheck(downCube)) {
            return false;
        }
        if (InductionCheck(downCube, downCubeSolver)) {
            shared_ptr<cube> downCore = GetAndValidateCore(downCubeSolver, downCube);
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
        if (InductionCheck(downCube, downCubeSolver)) {
            shared_ptr<cube> downCore = GetAndValidateCore(downCubeSolver, downCube);
            downCube->swap(*downCore);
            return true;
        }

        pair<shared_ptr<cube>, shared_ptr<cube>> ctgAssignment = downCubeSolver->GetAssignment(false);
        auto ctgState = make_shared<State>(nullptr, ctgAssignment.first, ctgAssignment.second, 0);
        shared_ptr<cube> succ = make_shared<cube>(*downCube);
        GetPrimed(succ);
        GeneralizePredecessor(ctgState, succ);

        const shared_ptr<cube> &ctgCube = ctgState->latches;
        m_log->L(3, "CTG cube: ", CubeToStr(ctgCube));

        // no need to calculate joinCube, since the joinCube is larger than ctgCube
        if (!InitiationCheck(ctgCube)) {
            return false;
        }

        assert(frameLvl >= 2); // frameLvl == 1 has been handled at the beginning of the function
        const auto ctgCubeSolver = m_frames[frameLvl - 2].solver;
        if (ctgs < m_settings.ctgMaxStates && InductionCheck(ctgCube, ctgCubeSolver)) {
            ctgs++;
            m_log->L(3, "CTG can be blocked at frame level ", frameLvl - 1);
            shared_ptr<cube> ctgCore = GetAndValidateCore(ctgCubeSolver, ctgCube);

            int pushLevel = PushLemmaForward(ctgCore, frameLvl - 1);
            MIC(ctgCore, pushLevel, recLvl + 1);
            m_log->L(2, "Learned ctg clause and pushed to frame ", pushLevel);
            AddBlockingCube(ctgCore, pushLevel, false);
        } else {
            ctgs = 0;
            shared_ptr<cube> joinCube = make_shared<cube>();
            unordered_set<int> ctgCubeSet(ctgCube->begin(), ctgCube->end());
            for (int i = downCube->size() - 1; i >= 0; i--) {
                if (ctgCubeSet.count(downCube->at(i))) {
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

shared_ptr<cube> BasicIC3::GetBlocker(const shared_ptr<cube> &blockingCube, int framelevel) {
    m_log->Tick();
    for (auto it : m_frames[framelevel].borderCubes) {
        shared_ptr<cube> cb = make_shared<cube>(*it);
        if (includes(blockingCube->begin(), blockingCube->end(), cb->begin(), cb->end(), cmp)) {
            m_log->StatGetBlocker();
            return cb;
        }
    }
    m_log->StatGetBlocker();
    return nullptr;
}

size_t BasicIC3::Generalize(const shared_ptr<cube> &cb, int frameLvl) {
    m_log->L(3, "Generalizing cube: ", CubeToStr(cb), ", at frameLvl: ", frameLvl);
    if (cb->size() >= 2) {
        MIC(cb, frameLvl, 1);
    }
    int pushLevel = PushLemmaForward(cb, frameLvl);
    m_log->L(2, "Learned clause and pushed to frame ", pushLevel);
    AddBlockingCube(cb, pushLevel, false);
    return pushLevel;
}

void BasicIC3::MIC(const shared_ptr<cube> &cb, int frameLvl, int recLvl) {
    m_log->L(3, "MIC: ", CubeToStr(cb), ", at frameLvl: ", frameLvl, ", recLvl: ", recLvl);

    shared_ptr<cube> blocker;
    unordered_set<int> triedLits;
    unordered_set<int> blockerSet;

    // refer skipping needs to find blockers at frameLvl - 1, so frameLvl must be at least 2 (F_0 does not have border cubes)
    if (m_settings.referSkipping && frameLvl >= 2) {
        blocker = GetBlocker(cb, frameLvl - 1);
        if (blocker) {
            for (const auto &lit : *blocker) {
                blockerSet.insert(lit);
            }
        }
    }

    size_t attempts = micMaxAttempts;

    if (m_settings.referSkipping && blocker) {
        OrderAssumptionMIC(cb, blockerSet);
    } else {
        OrderAssumptionMIC(cb);
    }

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
            cb->swap(*dropCube);
            if (m_settings.referSkipping && blocker) {
                OrderAssumptionMIC(cb, blockerSet);
            } else {
                OrderAssumptionMIC(cb);
            }
            i = cb->size();
            attempts = micMaxAttempts;
        } else {
            if (--attempts == 0) {
                m_log->L(3, "Max MIC attempts reached, stopping generalization.");
                break;
            }
            triedLits.insert(litToDrop);
        }
    }
    sort(cb->begin(), cb->end(), cmp);
}


void BasicIC3::GeneralizePredecessor(const shared_ptr<State> &predecessorState, const shared_ptr<cube> &succ) {
    m_log->L(3, "Generalizing predecessor. Initial latch size: ", predecessorState->latches->size(), ", input size: ", predecessorState->inputs->size(), ", Successor state latch size: ", succ->size());

    clause succNegationClause;
    succNegationClause.reserve(succ->size());
    for (const auto &lit : *succ) {
        succNegationClause.push_back(-lit);
    }
    for (auto cons : m_model->GetConstraints()) {
        succNegationClause.push_back(-cons);
    }
    m_liftSolver->AddTempClause(succNegationClause);

    const auto &partialLatch = predecessorState->latches;

    int strategy = 1;
    while (true) {
        auto assumption = make_shared<cube>(*partialLatch);
        if (!m_settings.liftRand)
            OrderAssumption(assumption);
        else {
            OrderAssumptionWithStrategy(assumption, strategy);
            strategy++;
        }

        assumption->insert(assumption->begin(), predecessorState->inputs->begin(), predecessorState->inputs->end());
        // There exist some successors whose predecessors are the entire set. (All latches are determined solely by the inputs.)
        if (m_settings.satSolveInDomain) {
            m_liftSolver->ResetTempDomain();
            m_liftSolver->SetTempDomainCOI(succ);
            m_liftSolver->SetTempDomainCOI(partialLatch);
        }

        m_log->Tick();
        bool result = m_liftSolver->Solve(assumption);
        m_log->StatLiftSolver();
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


// // try to add negated literals from initial states
// void BasicIC3::InitiationAugmentation(const shared_ptr<cube> &failureCube, const shared_ptr<cube> &fallbackCube) {
//     for (const auto &lit : *fallbackCube) {
//         if (m_initialStateSet.count(-lit)) {
//             failureCube->push_back(lit);
//             break;
//         }
//     }
//     sort(failureCube->begin(), failureCube->end(), cmp);
// }

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
    m_log->Tick();
    bool result = !slv->Solve(assumption);
    m_log->StatFrameSolver();
    return result;
}

// lemma can be blocked at startLevel (not reachable from F_{startLevel-1})
// returns the highest level the lemma can be blocked at (induction check using F_{pushLevel-1} passes, but using F_{pushLevel} fails)
// The max pushLevel is MaxLevel()
int BasicIC3::PushLemmaForward(const shared_ptr<cube> &cb, int startLevel) {
    int pushLevel = startLevel;
    while (pushLevel < MaxLevel()) {
        if (!InductionCheck(cb, m_frames[pushLevel].solver)) {
            break;
        }
        m_branching->Update(cb);
        pushLevel++;
    }
    return pushLevel;
}

bool BasicIC3::Propagate() {
    m_log->L(1, "Propagating clauses.");

    for (int i = m_earliest; i < (m_settings.bad_pred ? MaxLevel() - 1 : MaxLevel()); ++i) {
        m_log->L(2, "Propagating from F_", i, " to F_", i + 1);
        // Sort the border cubes by size (smallest first) to improve the chances of pushing more clauses.
        sort(m_frames[i].borderCubes.begin(), m_frames[i].borderCubes.end(), [](const shared_ptr<cube> &a, const shared_ptr<cube> &b) {
            return a->size() < b->size();
        });

        size_t initialBorderSize = m_frames[i].borderCubes.size();
        size_t pushedCount = 0;

        // Save a snapshot of the current border cubes to iterate over.
        vector<shared_ptr<cube>> currentBorderCubes = m_frames[i].borderCubes;
        for (const auto &lemma : currentBorderCubes) {
            if (m_frames[i].borderCubes.end() == find(m_frames[i].borderCubes.begin(), m_frames[i].borderCubes.end(), lemma)) {
                continue; // This lemma has already been pushed and removed.
            }
            if (UnreachabilityCheck(lemma, m_frames[i].solver)) {
                pushedCount++;
                auto core = GetAndValidateCore(m_frames[i].solver, lemma);
                AddBlockingCube(core, i + 1, true);
                m_branching->Update(core);
                // AddBlockingCube already removes the lemma from F_i
            }
        }
        if (m_frames[i].borderCubes.empty()) {
            if (!m_settings.bad_pred || i != MaxLevel() - 1) {
                m_log->L(1, "SAFE: Invariant found at F_", i + 1, " during propagation.");
                m_invariantLevel = i + 1;
                m_log->L(1, "m_invariantLevel: ", m_invariantLevel);
                m_log->L(1, FramesInfo());
                m_log->L(1, "lemmaCount: ", lemmaCount);
                return true; // Proof found
            }
        }
    }
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

    // bad is inductive invariant
    if (m_invariantLevel == 0 || (m_frames[m_invariantLevel].borderCubes.empty())) {
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

    vector<shared_ptr<cube>> indInv;

    for (int i = m_invariantLevel; i <= MaxLevel(); i++) {
        indInv.insert(indInv.end(), m_frames[i].borderCubes.begin(), m_frames[i].borderCubes.end());
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