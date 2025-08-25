#include "BasicIC3.h"
#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>

namespace car {

BasicIC3::BasicIC3(Settings settings, std::shared_ptr<Model> model, std::shared_ptr<Log> log)
    : m_settings(settings), m_log(log), m_model(model), m_k(0), m_cexState(nullptr) {
    State::numInputs = model->GetNumInputs();
    State::numLatches = model->GetNumLatches();
    innOrder.m = m_model; // Initialize the model pointer for the sorter

    // Initialize the dedicated solver for predecessor generalization (lifting).
    m_liftSolver = std::make_shared<SATSolver>(m_model);
    m_liftSolver->AddTrans();
    // Store the initial state literals in a set for efficient lookups.
    const auto &initState = m_model->GetInitialState();
    m_initialStateSet.insert(initState.begin(), initState.end());
    m_log->L(1, "BasicIC3 checker initialized.");
}

BasicIC3::~BasicIC3() {
    // Destructor logic, if needed
}

bool BasicIC3::Run() {
    m_log->Tick();
    bool result = Check();
    m_log->PrintStatistics();

    if (result) {
        m_log->L(0, "Safe");
    } else {
        m_log->L(0, "Unsafe");
        if (m_settings.witness) // If witness generation is enabled
            GenerateCounterExample();
    }
    return result;
}

bool BasicIC3::Check() {
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
        auto blockingCube = std::make_shared<cube>(cube{-lit});
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
    auto baseSolver = std::make_shared<SATSolver>(m_model);
    baseSolver->AddTrans(); // Also load transition relation for combinational logic
    auto assumption = std::make_shared<cube>();
    assumption->insert(assumption->end(), m_initialStateSet.begin(), m_initialStateSet.end());
    assumption->push_back(m_model->GetBadLit());

    if (baseSolver->Solve(assumption)) {
        m_log->L(0, "UNSAFE: Property fails in initial states.");
        return false;
    }

    // 1-step check: I & T & ~P'
    // Check if a bad state is reachable in one step.
    auto step1Solver = std::make_shared<SATSolver>(m_model);
    step1Solver->AddTrans();
    assumption->clear();
    assumption->insert(assumption->end(), m_initialStateSet.begin(), m_initialStateSet.end());
    assumption->push_back(m_model->GetPrime(m_model->GetBadLit()));

    if (step1Solver->Solve(assumption)) {
        m_log->L(0, "UNSAFE: Property fails at step 1.");
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
    newFrame.solver = std::make_shared<SATSolver>(m_model);
    newFrame.solver->AddTrans();
    newFrame.solver->AddProperty();
    m_frames.push_back(newFrame);
}

void BasicIC3::AddBlockingCube(std::shared_ptr<cube> blockingCube, int frameLevel) {
    // 1. Store the CUBE in the frame's set of border cubes.
    //    This represents the part of the state space that is now blocked.
    auto insertionResult = m_frames[frameLevel].borderCubes.insert(blockingCube);

    // If the cube was already present, do nothing.
    if (!insertionResult.second) {
        return;
    }

    // A new cube was added, so update the earliest modified frame.
    m_earliest = std::min(m_earliest, frameLevel);

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
        auto assumption = std::make_shared<cube>(cube{m_model->GetPrime(m_model->GetBadLit())});

        if (solver->Solve(assumption)) {
            // CTI found. This iteration is not trivial.
            m_trivial = false;

            // Extract the CTI state `s` (the unprimed latch values) from the model.
            Assignment assignment = solver->GetAssignment(false);
            auto ctiState = std::make_shared<State>(
                nullptr, // nextState is null for the last state in the trace
                assignment.inputs,
                assignment.latches,
                assignment.innards,
                assignment.primeInputs,
                1 // depth
            );

            // Generalize (lift) this predecessor of the bad state.
            // The successor is implicitly 'bad', so we pass nullptr.
            GeneralizePredecessor(ctiState);

            // Create a priority queue for proof obligations.
            std::set<Obligation> obligations;
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
bool BasicIC3::HandleObligations(std::set<Obligation> &obligations) {
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

            // 更新SAT solver的variable activities
            m_solverVarActivities = solver->GetVariableActivities();

            // Get the UNSAT core, but validate it against the initial states.
            // The safe fallback is the original CTI state that was just blocked.
            auto newBlockingCube = GetAndValidateUC(solver, ob.state->latches);

            // Generalize the cube further using MIC.
            // OrderAssumption现在会自动根据设置选择排序方式
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

            // internalSignals逻辑（现在OrderAssumption会自动处理activity-driven或传统排序）
            if (m_settings.internalSignals) {
                std::shared_ptr<cube> generalizedCubeWithInnards(new cube(*newBlockingCube));
                shared_ptr<cube> innards = m_model->GetRelevantInnards(newBlockingCube, ob.state->innards);
                if (innards->size() > 0) {
                    std::string method = m_settings.activityDriven ? "Activity-driven" : "Traditional";
                    m_log->L(3, method, ": Innards: ", CubeToStr(innards));
                    generalizedCubeWithInnards->insert(generalizedCubeWithInnards->end(), innards->begin(), innards->end());
                    
                    // OrderAssumption会自动根据设置选择排序方式
                    Generalize(generalizedCubeWithInnards, pushLevel - 1);
                    
                    // New condition: Extract latches from the generalized cube and compare its size
                    // to the original latch-only cube's size.
                    auto generalizedLatches = std::make_shared<cube>();
                    for (const auto& lit : *generalizedCubeWithInnards) {
                        // A literal is a latch if it's not an innard (in this context).
                        if (m_model->IsLatch(lit)) {
                            generalizedLatches->push_back(lit);
                        }
                    }

                    // We proceed only if the generalization with innards resulted in a smaller latch cube,
                    // meaning it's a more abstract lemma in terms of state variables.
                    if (generalizedLatches->size() < newBlockingCube->size()) {
                        m_log->L(3, method, ": Generalized Lemma with Innards is more abstract. Old latch count: ", newBlockingCube->size(), ", New: ", generalizedLatches->size());
                        m_log->L(3, method, ": Full lemma with innards: ", CubeToStr(generalizedCubeWithInnards));
                        int newPushLevel = PushLemmaForward(generalizedCubeWithInnards, pushLevel);
                        m_log->L(3, method, ": Pushed Lemma with Innards to frame ", newPushLevel);
                        AddBlockingCube(generalizedCubeWithInnards, newPushLevel);
                    }
                }
            }
        } else {
            // SAT: A predecessor `t` exists in F_{ob.level} for the CTI `s = ob.state`.
            Assignment assignment = solver->GetAssignment(false);

            // Create the predecessor state object first with the concrete assignment.
            auto predecessorState = std::make_shared<State>(
                ob.state,
                assignment.inputs,
                assignment.latches,
                assignment.innards,
                assignment.primeInputs,
                ob.depth + 1);

            if (ob.level == 0) {
                // Base case: The predecessor `t` is in F_0 (initial states). A real CEX is found.
                m_log->L(0, "UNSAFE: Found a path from the initial state.");
                m_cexState = predecessorState;
                return false; // Signal that CEX is found.
            }

            // Now, generalize the predecessor state in-place by minimizing its latch cube.
            GeneralizePredecessor(predecessorState, ob.state);

            // Link t -> s (t.nextState = s) to build the forward trace
            predecessorState->nextState = ob.state;


            // Inductive step: Predecessor `t` is in F_{ob.level}.
            // Create a new, more urgent obligation for `t` relative to F_{ob.level-1}.
            m_log->L(2, "Found predecessor for CTI. New obligation at level ", ob.level - 1);

            // Add the new obligation. The old one remains until this one is resolved.
            obligations.insert(Obligation(predecessorState, ob.level - 1, ob.depth + 1));
        }
    }
    return true; // All obligations handled without finding a CEX
}

void BasicIC3::Generalize(std::shared_ptr<cube> c, int level) {
    // This function minimizes a given cube `c` (which represents a clause !c)
    // by attempting to remove literals one by one, while maintaining that
    // the resulting clause is still inductive relative to F_level.
    // The incoming cube `c` is assumed to be sorted.
    m_log->L(3, "Generalizing cube: ", CubeToStr(c), ", at level ", level);
    auto generalizedCube = std::make_shared<cube>(*c);
    std::set<int> triedLits;

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
        auto tempCube = std::make_shared<cube>();
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

void BasicIC3::GeneralizePredecessor(std::shared_ptr<State> predecessorState, std::shared_ptr<State> successorState) {
    // This function "lifts" a concrete predecessor state to a more general cube
    // of states by iteratively minimizing the latch set using UNSAT cores.
    m_log->L(3, "Generalizing predecessor. Initial latch size: ", predecessorState->latches->size());

    // 1. Create the temporary clause !s' based on the successor.
    clause succNegationClause;
    if (successorState) {
        // Successor is a state cube.
        succNegationClause.reserve(successorState->latches->size());
        for (const auto &lit : *(successorState->latches)) {
            succNegationClause.push_back(-m_model->GetPrime(lit));
        }
    } else {
        // Successor is the bad state.
        succNegationClause.push_back(-m_model->GetPrime(m_model->GetBadLit()));
        m_log->L(3, "Generalizing predecessor: No successor state provided, using bad state.");
    }
    m_liftSolver->AddTempClause(succNegationClause);

    // 2. Iteratively minimize the latch part of the predecessor.
    auto partialLatch = std::make_shared<cube>(*predecessorState->latches);

    while (true) {
        // 2a. Build assumption: current partial latch cube + fixed inputs.
        auto assumption = std::make_shared<cube>();
        assumption->insert(assumption->end(), partialLatch->begin(), partialLatch->end());
        assumption->insert(assumption->end(), predecessorState->inputs->begin(), predecessorState->inputs->end());
        assumption->insert(assumption->end(), predecessorState->primeInputs->begin(), predecessorState->primeInputs->end());

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

bool BasicIC3::InitiationCheck(const std::shared_ptr<cube> &c) {
    // Checks if the cube c is disjoint from the initial states I.
    // This is true if I => !c, or equivalently, I & c is UNSAT.
    bool hasInnards = false;
    // for (const auto &lit : *c) {
    //     if (m_model->IsInnard(lit)) {
    //         hasInnards = true;
    //         break; // No need to check further, we have an innard.
    //     }
    // }
    // Innards must at front
    m_log->L(3, "Initiation check for cube: ", CubeToStr(c));
    if (!c->empty() && m_model->IsInnard(c->front())) {
        hasInnards = true;
    }
    if (!hasInnards) {
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

std::shared_ptr<cube> BasicIC3::GetAndValidateUC(const std::shared_ptr<SATSolver> solver, const std::shared_ptr<cube> fallbackCube) {
    std::shared_ptr<cube> core = solver->GetUC(true);
    m_log->L(3, "Got UNSAT core: ", CubeToStr(core));
    if (!InitiationCheck(core)) {
        m_log->L(3, "GetAndValidateUC: core intersects with initial states. Reverting to fallback cube.");
        // return fallbackCube; this is not safe, we need to create a new cube
        core = std::make_shared<cube>(*fallbackCube); // Create a new cube based
    }
    return core;
}

std::string BasicIC3::FramesInfo() const {
    std::stringstream ss;
    ss << "Frames: ";
    for (size_t i = 0; i < m_frames.size(); ++i) {
        ss << "F" << i << "[" << m_frames[i].borderCubes.size() << "] ";
    }
    return ss.str();
}

std::shared_ptr<cube> BasicIC3::GetPrimeCube(const std::shared_ptr<cube> &c) {
    // m_log->L(3, "Getting prime cube for: ", CubeToStr(c));
    auto primeCube = std::make_shared<cube>();
    primeCube->reserve(c->size());
    for (const auto &lit : *c) {
        primeCube->push_back(m_model->GetPrime(lit));
    }
    // m_log->L(3, "Prime cube: ", CubeToStr(primeCube));
    return primeCube;
}

int BasicIC3::PushLemmaForward(std::shared_ptr<cube> c, int startLevel) {
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
    std::set<std::shared_ptr<cube>, cubePtrComp> allCubes;
    for (int i = m_k + 1; i >= m_earliest; --i) {
        IC3Frame &frame = m_frames[i];
        if (frame.borderCubes.empty()) continue;

        size_t originalSize = frame.borderCubes.size();

        std::set<std::shared_ptr<cube>, cubePtrComp> remainingCubes;
        std::set_difference(frame.borderCubes.begin(), frame.borderCubes.end(),
                            allCubes.begin(), allCubes.end(),
                            std::inserter(remainingCubes, remainingCubes.end()),
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
            m_log->L(0, "SAFE: Frame F_", i, " is an inductive invariant.");
            return true; // Proof found
        }
    }

    // 3. Simplify solvers to clean up internal states.
    for (int i = 1; i <= m_k + 1; ++i) {
        m_frames[i].solver->Simplify();
    }
    m_liftSolver->Simplify();

    // No proof was found in this iteration.
    return false;
}

void BasicIC3::GenerateCounterExample() {
    if (m_cexState) {
        m_log->L(0, "Counterexample found. Trace:");
        auto current = m_cexState;
        int step = 0;
        while (current) {
            m_log->L(0, "Step ", step++, ": inputs=", current->GetInputsString(), " -> latches=", current->GetLatchesString());
            if (!current->nextState) {
                // This is the last state in the trace.
                // The inputs stored here lead to the 'bad' state.
                m_log->L(0, "Step ", step++, ": primeInputs=", current->GetPrimeInputsString());
            }
            current = current->nextState;
        }
    } else {
        m_log->L(0, "Counterexample trace could not be generated.");
    }
}

double BasicIC3::GetIC3VariableActivity(int ic3Variable, 
                                       const std::unordered_map<int, double>& solverVarActivities) {
    // 根据GetLit函数的映射关系：int var = abs(id) - 1
    // 从IC3变量转换为SAT Solver变量：ic3Variable -> solverVar = abs(ic3Variable) - 1
    int solverVar = abs(ic3Variable) - 1;
    
    auto it = solverVarActivities.find(solverVar);
    if (it != solverVarActivities.end()) {
        return it->second;
    } else {
        return 0.0; // 如果没找到，返回0
    }
}

void BasicIC3::SortCubeByVariableActivity(std::shared_ptr<cube> c, 
                                         const std::unordered_map<int, double>& solverVarActivities) {
    std::sort(c->begin(), c->end(), 
              [&solverVarActivities, this](int literalA, int literalB) {
                  bool a_is_latch = m_model->IsLatch(literalA);
                  bool b_is_latch = m_model->IsLatch(literalB);
                  bool a_is_innard = m_model->IsInnard(literalA);
                  bool b_is_innard = m_model->IsInnard(literalB);
                  
                  // 策略1: innards在前面，latches在后面
                  if (a_is_innard && !b_is_innard) return true;   // A是innard，B不是 -> A在前
                  if (!a_is_innard && b_is_innard) return false;  // B是innard，A不是 -> B在前
                  
                  // 策略2: 如果都是innards，按variable activity排序（高activity优先）
                  if (a_is_innard && b_is_innard) {
                      int varA = abs(literalA);
                      int varB = abs(literalB);
                      
                      double actA = GetIC3VariableActivity(varA, solverVarActivities);
                      double actB = GetIC3VariableActivity(varB, solverVarActivities);
                      
                      return actA > actB; // 高activity优先
                  }
                  
                  // 策略3: 如果都是latches，保持原序
                  if (a_is_latch && b_is_latch) return false;
                  
                  // 其他情况保持原序
                  return false;
              });
}

} // namespace car