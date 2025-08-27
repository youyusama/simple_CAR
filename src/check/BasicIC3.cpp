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
    m_liftSolver->AddTrans();
    // Store the initial state literals in a set for efficient lookups.
    const auto &initState = m_model->GetInitialState();
    m_initialStateSet.insert(initState.begin(), initState.end());
    m_log->L(1, "BasicIC3 checker initialized.");

    GLOBAL_LOG = m_log;
    m_checkResult = CheckResult::Unknown;
}

BasicIC3::~BasicIC3() {
    // Destructor logic, if needed
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

    // Initialize F_0. It represents the initial states I.
    AddNewFrame(); // This creates and adds F_0
    IC3Frame &frame0 = m_frames[0];

    // F_0 is defined as exactly the initial states.
    for (const auto &lit : m_initialStateSet) {
        // Add initial state literals as unit clauses to the solver
        frame0.solver->AddClause({lit});
        // Also add blocking cubes to define F_0 logically, which will be propagated.
        auto blockingCube = make_shared<cube>(cube{-lit});
        AddBlockingCube(blockingCube, 0);
    }

    // The main IC3 loop.
    // Start with k=1, trying to prove P holds for 1 step, then 2, etc.
    for (m_k = 1;; ++m_k) {
        m_log->L(1, "==================== k=", m_k, " ====================");
        m_log->L(1, FramesInfo());
        AddNewFrames(); // Ensures frames up to F_{k+1} exist.

        // Phase 2: Block bad states recursively by strengthening F_k
        if (!Strengthen()) {
            // A real counterexample was found and confirmed
            m_log->L(1, "UNSAFE: CEX found during strengthening of F_", m_k);
            return false;
        }

        // Phase 3: Push clauses forward from F_k to F_{k+1} and check for proof
        if (Propagate()) {
            // Proof found, the property is safe
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
        auto p = baseSolver->GetAssignment(false);
        m_cexStart->inputs = p.first;
        m_cexStart->latches = p.second;
        return false;
    }

    // 1-step check: I & T & ~P'
    // Check if a bad state is reachable in one step.
    auto step1Solver = make_shared<SATSolver>(m_model, m_settings.solver);
    step1Solver->AddTrans();
    step1Solver->AddTransK(1);
    step1Solver->AddBadk(1);
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
                primeInputs.push_back(i_p);
            else
                primeInputs.push_back(-i_p);
        }
        pair<shared_ptr<cube>, shared_ptr<cube>> assignment = step1Solver->GetAssignment(false);
        shared_ptr<State> badState(new State(nullptr, make_shared<cube>(primeInputs), nullptr, 0));
        shared_ptr<State> m_cexStart = make_shared<State>(
            badState, // nextState is null for the last state in the trace
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
    newFrame.solver->AddTrans();
    newFrame.solver->AddConstraints();
    if (m_frames.size() > 0) {
        newFrame.solver->AddTransK(1);
        newFrame.solver->AddConstraintsK(1);
    }
    newFrame.solver->AddProperty();
    AddSamePrimeConstraints(newFrame.solver);
    m_frames.push_back(newFrame);
}

void BasicIC3::AddBlockingCube(shared_ptr<cube> blockingCube, int frameLevel) {
    // 1. Store the CUBE in the frame's set of border cubes.
    //    This represents the part of the state space that is now blocked.
    auto insertionResult = m_frames[frameLevel].borderCubes.insert(blockingCube);

    // If the cube was already present, do nothing.
    if (!insertionResult.second) {
        return;
    }

    // A new cube was added, so update the earliest modified frame.
    m_earliest = min(m_earliest, frameLevel);

    m_log->L(2, "Frame ", frameLevel, ": ", CubeToStr(blockingCube));

    // 2. Create the corresponding CLAUSE (lemma) by negating the cube.
    //    The clause is the logical dual of the cube: (~l1 | ~l2 | ...)
    clause lemma;
    lemma.reserve(blockingCube->size());
    for (const auto &lit : *blockingCube) {
        lemma.push_back(-lit);
    }

    // 3. Add the learned CLAUSE (lemma) to the SAT solvers of all relevant frames.
    //    This enforces the lemma in future SAT checks.
    //    F_0 is special
    for (int i = 1; i <= frameLevel; ++i) {
        if (m_frames[i].solver) {
            m_frames[i].solver->AddClause(lemma);
        }
    }
}

bool BasicIC3::Strengthen() {
    m_trivial = true;
    m_earliest = m_k + 1;

    while (true) {
        // Check for a CTI: a state `s` in F_k such that T(s, s') and P(s') is false.
        auto solver = m_frames[m_k].solver;
        auto assumption = make_shared<cube>(cube{m_model->GetPrimeK(m_model->GetBad(), 1)});
        if (solver->Solve(assumption)) {
            // CTI found. This iteration is not trivial.
            m_trivial = false;

            cube badInputs;
            shared_ptr<vector<int>> coiInputs = m_model->GetCOIInputs();
            for (int i : *coiInputs) {
                int i_p = m_model->GetPrimeK(i, 1);
                if (solver->GetModel(i_p))
                    badInputs.push_back(i);
                else
                    badInputs.push_back(-i);
            }
            pair<shared_ptr<cube>, shared_ptr<cube>> assignment = solver->GetAssignment(false);
            shared_ptr<State> badState(new State(nullptr, make_shared<cube>(badInputs), nullptr, 0));
            shared_ptr<State> ctiState = make_shared<State>(
                badState, // nextState is null for the last state in the trace
                assignment.first,
                assignment.second,
                1 // depth
            );
            m_log->L(3, "Found CTI at level ", m_k, ": ", CubeToStr(ctiState->latches), ", input: ", CubeToStr(ctiState->inputs));

            // Create a priority queue for proof obligations.
            set<Obligation> obligations;
            // Add the initial CTI as the first obligation.
            // We need to block it relative to frame k-1.
            obligations.emplace(ctiState, m_k - 1, 1);

            if (!HandleObligations(obligations)) {
                // A real counterexample was found.
                // m_cexState is now set by HandleObligations to the start of the trace.
                return false;
            }
        } else {
            // No more CTIs found at this level. F_k is relatively inductive.
            m_log->L(1, "No more CTIs at level ", m_k, ". Frame is strengthened.");
            return true;
        }
    }
}

// todo: initiation check when GetUC
bool BasicIC3::HandleObligations(set<Obligation> &obligations) {
    while (!obligations.empty()) {
        // Get the highest priority obligation (lowest level).
        Obligation ob = *obligations.begin();

        // Check if the CTI is reachable from the previous frame.
        // Query: F_{ob.level} & T & cti'
        m_log->L(2, "Handling obligation for state at level ", ob.level, " with depth ", ob.depth);
        auto solver = m_frames[ob.level].solver;
        auto ctiPrime = GetPrimeCube(ob.state->latches);

        if (!solver->Solve(ctiPrime)) {
            // UNSAT: cti is blocked relative to F_{ob.level}. The obligation is fulfilled.
            obligations.erase(obligations.begin()); // Remove the now-handled obligation.

            // Get the UNSAT core, but validate it against the initial states.
            // The safe fallback is the original CTI state that was just blocked.
            auto newBlockingCube = GetAndValidateUC(solver, ob.state->latches);

            // Generalize the cube further using MIC.
            Generalize(newBlockingCube, ob.level);

            // Push the clause forward as far as possible by calling the helper function.
            int pushLevel = PushLemmaForward(newBlockingCube, ob.level + 1);

            m_log->L(2, "Learned clause and pushed to frame ", pushLevel);
            AddBlockingCube(newBlockingCube, pushLevel);

            // If the clause was pushed to a level within the frontier,
            // we must re-check the original state against this new, higher level.
            if (pushLevel <= m_k) {
                m_log->L(2, "Creating new obligation for same state at higher level ", pushLevel);
                obligations.insert(Obligation(ob.state, pushLevel, ob.depth));
            }

        } else {
            // SAT: A predecessor `t` exists in F_{ob.level} for the CTI `s = ob.state`.
            pair<shared_ptr<cube>, shared_ptr<cube>> assignment = solver->GetAssignment(false);

            // Create the predecessor state object first with the concrete assignment.
            auto predecessorState = make_shared<State>(
                ob.state,
                assignment.first,
                assignment.second,
                ob.depth + 1);

            if (ob.level == 0) {
                // Base case: The predecessor `t` is in F_0 (initial states). A real CEX is found.
                m_log->L(1, "UNSAFE: Found a path from the initial state.");
                m_cexStart = predecessorState;
                return false; // Signal that CEX is found.
            }

            // Now, generalize the predecessor state in-place by minimizing its latch cube.
            GeneralizePredecessor(predecessorState, ob.state);

            // Link t -> s (t.nextState = s) to build the forward trace
            predecessorState->preState = ob.state;


            // Inductive step: Predecessor `t` is in F_{ob.level}.
            // Create a new, more urgent obligation for `t` relative to F_{ob.level-1}.
            m_log->L(2, "Found predecessor for CTI. New obligation at level ", ob.level - 1);

            // Add the new obligation. The old one remains until this one is resolved.
            obligations.insert(Obligation(predecessorState, ob.level - 1, ob.depth + 1));
        }
    }
    return true; // All obligations handled without finding a CEX
}

void BasicIC3::Generalize(shared_ptr<cube> c, int level) {
    // This function minimizes a given cube `c` (which represents a clause !c)
    // by attempting to remove literals one by one, while maintaining that
    // the resulting clause is still inductive relative to F_level.
    // The incoming cube `c` is assumed to be sorted.
    m_log->L(3, "Generalizing cube: ", CubeToStr(c), ", at level ", level);
    auto generalizedCube = make_shared<cube>(*c);
    set<int> triedLits;

    // order assumption for innards
    OrderAssumption(generalizedCube);

    // Iterate backwards to handle the shrinking cube size gracefully.
    for (int i = generalizedCube->size() - 1; i >= 0; --i) {
        // If the cube shrank, the index might be out of bounds. Adjust it.
        if (i >= generalizedCube->size()) {
            i = generalizedCube->size() - 1;
        }

        int litToDrop = generalizedCube->at(i);

        // If we have already tried and failed to drop this literal, skip.
        if (triedLits.count(litToDrop)) {
            continue;
        }

        // Create a temporary cube with one literal removed.
        auto tempCube = make_shared<cube>();
        tempCube->reserve(generalizedCube->size() - 1);
        for (int j = 0; j < generalizedCube->size(); ++j) {
            if (i == j) continue;
            tempCube->push_back(generalizedCube->at(j));
        }

        // 1. Initiation Check: I & tempCube must be UNSAT.
        if (!InitiationCheck(tempCube)) {
            // SAT: The smaller cube intersects with the initial states. Cannot drop.
            triedLits.insert(litToDrop);
            continue;
        }

        // 2. Inductivity Check for IC3: F_level & !tempCube & T & tempCube' must be UNSAT.
        auto solver = m_frames[level].solver;

        // Add !tempCube as a temporary clause
        clause tempClause;
        tempClause.reserve(tempCube->size());
        for (const auto &lit : *tempCube) {
            tempClause.push_back(-lit);
        }
        solver->AddTempClause(tempClause);

        // order assumption for innards
        OrderAssumption(tempCube);

        m_log->L(3, "Checking inductivity for ordered cube: ", CubeToStr(tempCube));

        auto tempCubePrime = GetPrimeCube(tempCube);
        bool isInductive = !solver->Solve(tempCubePrime);

        if (isInductive) {
            // UNSAT: The smaller cube is inductive. We can drop the literal.
            // The UNSAT core might allow for an even more aggressive reduction.
            // We must validate the core against the initial states.
            generalizedCube = GetAndValidateUC(solver, tempCube);
            OrderAssumption(generalizedCube);
        } else {
            // SAT: The smaller cube is not inductive. We cannot drop the literal.
            triedLits.insert(litToDrop);
        }
        solver->ReleaseTempClause(); // Always release the temporary clause
    }

    m_log->L(3, "Generalized cube: ", CubeToStr(generalizedCube));
    *c = *generalizedCube; // Modify the original cube's content
}

void BasicIC3::GeneralizePredecessor(shared_ptr<State> predecessorState, shared_ptr<State> successorState) {
    // This function "lifts" a concrete predecessor state to a more general cube
    // of states by iteratively minimizing the latch set using UNSAT cores.
    m_log->L(3, "Generalizing predecessor. Initial latch size: ", predecessorState->latches->size());
    // 1. Create the temporary clause !s' based on the successor.
    clause succNegationClause;

    // Successor is a state cube.
    succNegationClause.reserve(successorState->latches->size());
    for (const auto &lit : *(successorState->latches)) {
        succNegationClause.push_back(-m_model->GetPrimeK(lit, 1));
    }

    for (auto cons : m_model->GetConstraints()) {
        succNegationClause.push_back(-cons);
    }
    
    m_liftSolver->AddTempClause(succNegationClause);
    // 2. Iteratively minimize the latch part of the predecessor.
    auto partialLatch = make_shared<cube>(*predecessorState->latches);

    while (true) {
        // 2a. Build assumption: current partial latch cube + fixed inputs.
        auto assumption = make_shared<cube>();
        assumption->insert(assumption->end(), partialLatch->begin(), partialLatch->end());
        assumption->insert(assumption->end(), predecessorState->inputs->begin(), predecessorState->inputs->end());

        // 2b. Solve. This query must be UNSAT.
        bool result = m_liftSolver->Solve(assumption);
        assert(!result && "Lifting query resulted in SAT, which should be impossible.");

        // 2c. Get core. GetUC(false) returns a sorted cube of latches/innards.
        auto core = m_liftSolver->GetUC(false);
        m_log->L(3, "Core size: ", core->size(), ", Partial latch size: ", partialLatch->size());

        // 2d. Check for fixed point. If the core is not strictly smaller, we are done.
        if (core->size() >= partialLatch->size()) {
            break;
        } else {
            // 2e. Update partial latch with the new, smaller core.
            partialLatch = core;
        }
    }

    // 3. Release the temporary clause from the solver.
    m_liftSolver->ReleaseTempClause();

    // 4. Update the predecessor state's latch cube in-place.
    predecessorState->latches = partialLatch;
    m_log->L(3, "Generalized predecessor. Final latch size: ", predecessorState->latches->size());
}

// --- Helper Implementations ---

bool BasicIC3::InitiationCheck(const shared_ptr<cube> &c) {
    m_log->L(3, "Initiation check for cube: ", CubeToStr(c));
    if (m_model->GetConstraints().empty()) {
        // Simple case: No internal signals. Check for direct conflict with initial state literals.
        // This is true if c contains a literal l such that -l is in I.
        for (const auto &lit : *c) {
            if (m_initialStateSet.count(-lit)) {
                return true; // Disjoint (UNSAT), check passes.
            }
        }
        return false; // Intersects with I (potentially SAT), check fails.
    } else {
        // Complex case: Cube may contain internal signals.
        // We must use a SAT solver to check if I & c is satisfiable.
        // F_0's solver contains the clauses for I.
        auto solver = m_frames[0].solver;
        return !solver->Solve(c); // Return true if UNSAT (disjoint).
    }
}

shared_ptr<cube> BasicIC3::GetAndValidateUC(const shared_ptr<SATSolver> solver, const shared_ptr<cube> fallbackCube) {
    shared_ptr<cube> core = solver->GetUC(true);
    m_log->L(3, "Got UNSAT core: ", CubeToStr(core));
    if (!InitiationCheck(core)) {
        m_log->L(3, "GetAndValidateUC: core intersects with initial states. Reverting to fallback cube.");
        // return fallbackCube; this is not safe, we need to create a new cube
        core = make_shared<cube>(*fallbackCube); // Create a new cube based
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

shared_ptr<cube> BasicIC3::GetPrimeCube(const shared_ptr<cube> &c) {
    // m_log->L(3, "Getting prime cube for: ", CubeToStr(c));
    auto primeCube = make_shared<cube>();
    primeCube->reserve(c->size());
    for (const auto &lit : *c) {
        primeCube->push_back(m_model->GetPrimeK(lit, 1));
    }
    // m_log->L(3, "Prime cube: ", CubeToStr(primeCube));
    return primeCube;
}

int BasicIC3::PushLemmaForward(shared_ptr<cube> c, int startLevel) {
    int pushLevel = startLevel;
    while (pushLevel <= m_k) {
        auto nextFrameSolver = m_frames[pushLevel].solver;
        auto cubePrime = GetPrimeCube(c);
        // If F_{pushLevel} & T & c' is SAT, we can't push further.
        if (nextFrameSolver->Solve(cubePrime)) {
            break;
        }
        pushLevel++;
    }
    return pushLevel;
}

// --- Placeholder Implementations for Subsequent Phases ---

bool BasicIC3::Propagate() {
    m_log->L(1, "Phase 3: Propagating clauses.");

    // 1. Clean up: remove redundant clauses from lower frames.
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

    // 2. Propagate clauses forward.
    m_log->L(2, "Propagating clauses forward.");
    // Optimization: if the last strengthening was trivial, only propagate from F_k.
    for (int i = m_trivial ? m_k : 1; i <= m_k; ++i) {
        IC3Frame &frame = m_frames[i];
        int cubesKept = 0;
        int cubesPropagated = 0;

        // Use the safe manual iteration pattern from IC3.cpp
        for (auto it = frame.borderCubes.begin(); it != frame.borderCubes.end();) {
            const auto &c = *it;
            // A clause !c is inductive relative to F_i if F_i & T & c' is UNSAT.
            auto solver = frame.solver;
            auto prime_c = GetPrimeCube(c);
            if (!solver->Solve(prime_c)) {
                // UNSAT -> Inductive.
                cubesPropagated++;
                // Get the UNSAT core, which might be a smaller, more general cube.
                // The fallback is the original cube 'c'.
                auto smaller_c = GetAndValidateUC(solver, c);

                // Push the (potentially smaller) blocking cube to the next frame.
                AddBlockingCube(smaller_c, i + 1);

                // And remove the original cube 'c' from the current frame's border.
                // Safely erase and advance the iterator
                it = frame.borderCubes.erase(it);
            } else {
                // SAT -> Not inductive, keep the cube in this frame.
                cubesKept++;
                ++it;
            }
        }

        m_log->L(2, "Frame ", i, " propagation: ", cubesPropagated, " propagated, ", cubesKept, " kept.");

        // After attempting to push all clauses, check for proof.
        // A proof is found if F_i becomes empty, meaning all its clauses
        // were propagated to F_{i+1}, making F_i an inductive invariant.
        if (frame.borderCubes.empty()) {
            m_log->L(1, "SAFE: Frame F_", i, " is empty.");
            m_invariantLevel = i+1;
            m_log->L(1, "Border cubes in Frame F_", m_invariantLevel, " are an inductive invariant.");
            return true; // Proof found
        }
    }

    // 3. Simplify solvers to clean up internal states.
    // for (int i = 1; i <= m_k + 1; ++i) {
    //     m_frames[i].solver->Simplify();
    // }
    // m_liftSolver->Simplify();

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
    string outPath = m_settings.witnessOutputDir + aigName + ".w.aag";
    aiger *model_aig = m_model->GetAig();

    unsigned lvl_i;
    // bad is constant
    if (m_frames.size() == 0 || m_frames[m_invariantLevel].borderCubes.empty()) {
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

    
    set<shared_ptr<cube>, cubePtrComp> &indInv = m_frames[m_invariantLevel].borderCubes;
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