#include "ForwardChecker.h"
#include <stack>
#include <string>
namespace car {

ForwardChecker::ForwardChecker(Settings settings,
                               shared_ptr<AigerModel> model,
                               shared_ptr<Log> log) : m_settings(settings),
                                                      m_model(model),
                                                      m_log(log) {
    State::numInputs = model->GetNumInputs();
    State::numLatches = model->GetNumLatches();
    m_lastState = nullptr;
    GLOBAL_LOG = m_log;
}

bool ForwardChecker::Run() {
    signal(SIGINT, signalHandler);

    bool result = Check(m_model->GetBad());

    m_log->PrintStatistics();
    if (result) {
        m_log->L(0, "Safe");
        if (m_settings.witness)
            OutputWitness(m_model->GetBad());
    } else {
        m_log->L(0, "Unsafe");
        if (m_settings.witness)
            OutputCounterExample(m_model->GetBad());
    }

    return true;
}

bool ForwardChecker::Check(int badId) {
    if (m_model->GetTrueId() == badId)
        return false;
    else if (m_model->GetFalseId() == badId)
        return true;

    Init(badId);
    m_log->L(3, "Initialized");

    m_log->L(3, "Initial States Check");
    if (ImmediateSatisfiable(badId)) {
        m_log->L(3, "Result >>> SAT <<<");
        auto p = m_mainSolver->GetAssignment(false);
        m_log->L(3, "Get Assignment:", CubeToStr(p.second));
        m_initialState->inputs = p.first;
        m_lastState = m_initialState;
        return false;
    }

    m_log->L(3, "Result >>> UNSAT <<<");
    // initialize frame 0
    for (int l : *m_initialState->latches) {
        shared_ptr<cube> neg_init_l(new cube{-l});
        m_overSequence->Insert(neg_init_l, 0, false);
        m_mainSolver->AddUC(*neg_init_l, 0);
    }
    m_mainSolver->AddNegationBad();
    m_overSequence->effectiveLevel = 0;
    m_startSovler->UpdateStartSolverFlag();

    // main stage
    int frameStep = 0;
    stack<Task> workingStack;
    while (true) {
        m_log->L(1, m_overSequence->FramesInfo());
        m_minUpdateLevel = m_overSequence->GetLength();
        if (m_settings.end) { // from the deep and the end
            for (int i = m_underSequence.size() - 1; i >= 0; i--) {
                for (int j = m_underSequence[i].size() - 1; j >= 0; j--) {
                    workingStack.emplace(m_underSequence[i][j], frameStep, false);
                }
            }
        } else { // from the shallow and the start
            for (int i = 0; i < m_underSequence.size(); i++) {
                for (int j = 0; j < m_underSequence[i].size(); j++) {
                    workingStack.emplace(m_underSequence[i][j], frameStep, false);
                }
            }
        }
        m_log->Tick();
        shared_ptr<State> startState = EnumerateStartState();
        m_log->StatStartSolver();
        if (startState == nullptr) {
            m_overSequence->SetInvariantLevel(-1);
            return true;
        }
        m_log->L(3, "\nState from start solver");
        while (startState != nullptr) {
            workingStack.push(Task(startState, frameStep, true));

            while (!workingStack.empty()) {
                Task &task = workingStack.top();

                if (!task.isLocated) {
                    task.frameLevel = GetNewLevel(task.state, task.frameLevel + 1);
                    m_log->L(3, "State get new level ", task.frameLevel);
                    if (task.frameLevel > m_overSequence->effectiveLevel) {
                        workingStack.pop();
                        continue;
                    }
                }
                task.isLocated = false;

                if (task.frameLevel == -1) {
                    m_initialState->preState = task.state->preState;
                    m_initialState->inputs = task.state->inputs;
                    m_lastState = m_initialState;
                    return false;
                }
                m_log->L(3, "SAT CHECK on frame: ", task.frameLevel);
                m_log->L(3, "From state: ", CubeToStrShort(task.state->latches));
                m_log->L(3, "State Detail: ", CubeToStr(task.state->latches));
                shared_ptr<cube> assumption(new cube(*task.state->latches));
                OrderAssumption(assumption);
                GetPrimed(assumption);
                m_log->Tick();
                bool result = m_mainSolver->Solve(assumption, task.frameLevel);
                m_log->StatMainSolver();
                if (result) {
                    // Solver return SAT, get a new State, then continue
                    m_log->L(3, "Result >>> SAT <<<");
                    auto p = m_mainSolver->GetAssignment(false);
                    GeneralizePredecessor(p, task.state);
                    shared_ptr<State> newState(new State(task.state, p.first, p.second, task.state->depth + 1));
                    m_underSequence.push(newState);
                    m_log->L(3, "Get state: ", CubeToStrShort(newState->latches));
                    int newFrameLevel = GetNewLevel(newState);
                    workingStack.emplace(newState, newFrameLevel, true);
                    continue;
                } else {
                    // Solver return UNSAT, get uc, then continue
                    m_log->L(3, "Result >>> UNSAT <<<");
                    auto uc = m_mainSolver->GetUC(true);
                    assert(uc->size() > 0);
                    m_log->L(3, "Get UC: ", CubeToStr(uc));
                    if (Generalize(uc, task.frameLevel))
                        m_branching->Update(uc);
                    m_log->L(3, "Get Generalized UC: ", CubeToStr(uc));
                    AddUnsatisfiableCore(uc, task.frameLevel + 1);
                    PropagateUp(uc, task.frameLevel + 1);
                    m_log->L(3, "Frames: ", m_overSequence->FramesInfo());
                    task.frameLevel++;
                    continue;
                }
            } // end while (!workingStack.empty())
            m_log->Tick();
            startState = EnumerateStartState();
            m_log->StatStartSolver();
            m_log->L(3, "\nState from start solver");
        }

        frameStep++;
        m_log->L(3, "\nNew Frame Added");

        if (m_invSolver == nullptr) {
            m_invSolver.reset(new InvSolver(m_model));
        }
        IsInvariant(0);
        for (int i = 0; i < m_overSequence->GetLength() - 1; ++i) {
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
                m_log->L(1, "Proof at frame ", i + 1);
                m_log->L(1, m_overSequence->FramesInfo());
                m_overSequence->SetInvariantLevel(i);
                return true;
            }
        }
        m_invSolver = nullptr;

        m_log->L(3, m_overSequence->FramesDetail());
        m_mainSolver->simplify();
        m_overSequence->effectiveLevel++;
        m_startSovler->UpdateStartSolverFlag();
    }
}


void ForwardChecker::Init(int badId) {
    shared_ptr<cube> inputs(new cube(State::numInputs, 0));
    shared_ptr<cube> latches(new cube(m_model->GetInitialState()));
    m_initialState.reset(new State(nullptr, inputs, latches, 0));

    m_badId = badId;
    m_overSequence = make_shared<OverSequenceSet>(m_model);
    m_underSequence = UnderSequence();
    m_branching = make_shared<Branching>(m_settings.Branching);
    litOrder.branching = m_branching;
    blockerOrder.branching = m_branching;

    m_mainSolver = make_shared<MainSolver>(m_model, true);
    for (auto c : m_model->GetConstraints()) {
        m_mainSolver->AddClause(clause{c});
    }
    m_lifts = make_shared<MainSolver>(m_model, true);
    m_invSolver = make_shared<InvSolver>(m_model);
    m_startSovler = make_shared<StartSolver>(m_model);
}

bool ForwardChecker::AddUnsatisfiableCore(shared_ptr<vector<int>> uc, int frameLevel) {
    m_log->Tick();

    m_mainSolver->AddUC(*uc, frameLevel);
    if (frameLevel > m_overSequence->effectiveLevel) {
        clause cls;
        cls.reserve(uc->size() + 1);
        for (auto l : *uc) cls.push_back(-l);
        cls.push_back(-m_startSovler->GetFlag());
        m_startSovler->AddClause(cls);
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
    bool result = m_mainSolver->Solve(assumptions);
    return result;
}


shared_ptr<State> ForwardChecker::EnumerateStartState() {
    if (m_startSovler->Solve()) {
        auto p = m_startSovler->GetStartPair();
        GeneralizePredecessor(p);
        shared_ptr<State> newState(new State(nullptr, p.first, p.second, 0));
        return newState;
    } else {
        return nullptr;
    }
}


int ForwardChecker::GetNewLevel(shared_ptr<State> state, int start) {
    m_log->Tick();

    for (int i = start; i < m_overSequence->GetLength(); i++) {
        if (!m_overSequence->IsBlockedByFrame_lazy(state->latches, i)) {
            m_log->StatGetNewLevel();
            return i - 1;
        }
    }

    m_log->StatGetNewLevel();
    return m_overSequence->GetLength() - 1; // placeholder
}

bool ForwardChecker::IsInvariant(int frameLevel) {
    m_log->Tick();

    vector<shared_ptr<cube>> frame_i;
    m_overSequence->GetFrame(frameLevel, frame_i);

    if (frameLevel < m_minUpdateLevel) {
        m_invSolver->AddConstraintOr(frame_i);
        return false;
    }

    m_invSolver->AddConstraintAnd(frame_i);
    bool result = !m_invSolver->Solve();
    m_invSolver->FlipLastConstrain();
    m_invSolver->AddConstraintOr(frame_i);

    m_log->StatInvSolver();
    return result;
}


// ================================================================================
// @brief: t & input & T -> s'  =>  (t) & input & T & !s' is unsat, !bad & input & t & T is unsat
// @input: pair<input, latch>
// @output: pair<input, partial latch>
// ================================================================================
void ForwardChecker::GeneralizePredecessor(pair<shared_ptr<cube>, shared_ptr<cube>> &t, shared_ptr<State> s) {
    m_log->Tick();

    shared_ptr<cube> partial_latch = make_shared<cube>(*t.second);
    OrderAssumption(partial_latch);

    // necessary cube for constraints
    shared_ptr<cube> necessary(new cube());
    if (s != nullptr && m_model->GetConstraints().size() > 0) {
        shared_ptr<cube> assumption(new cube());
        copy(partial_latch->begin(), partial_latch->end(), back_inserter(*assumption));
        copy(t.first->begin(), t.first->end(), back_inserter(*assumption));
        int act = m_lifts->GetNewVar();
        clause cls;
        for (auto cons : m_model->GetConstraints()) cls.push_back(-cons);
        cls.push_back(-act);
        m_lifts->AddClause(cls);
        assumption->push_back(act);

        bool res = m_lifts->Solve(assumption);
        assert(!res);
        necessary = m_lifts->GetUC(false);
        m_lifts->FlipLastConstrain();
    }

    int act = m_lifts->GetNewVar();
    if (s == nullptr) {
        // add !bad ( | !cons) to assumption
        clause cls = {-m_badId};
        for (auto cons : m_model->GetConstraints()) cls.push_back(-cons);
        cls.push_back(-act);
        m_lifts->AddClause(cls);
    } else {
        // add !s' to clause
        clause cls;
        cls.reserve(s->latches->size() + 1);
        for (auto l : *s->latches) {
            cls.emplace_back(m_model->GetPrime(-l));
        }
        cls.push_back(-act);
        m_lifts->AddClause(cls);
    }

    while (true) {
        shared_ptr<cube> assumption(new cube());
        copy(partial_latch->begin(), partial_latch->end(), back_inserter(*assumption));
        copy(t.first->begin(), t.first->end(), back_inserter(*assumption));
        copy(necessary->begin(), necessary->end(), back_inserter(*assumption));
        assumption->push_back(act);

        bool res = m_lifts->Solve(assumption);
        assert(!res);
        shared_ptr<cube> temp_p = m_lifts->GetUC(false);
        if (temp_p->size() == partial_latch->size() &&
            equal(temp_p->begin(), temp_p->end(), partial_latch->begin()))
            break;
        else {
            partial_latch = temp_p;
        }
    }
    m_lifts->FlipLastConstrain();

    sort(partial_latch->begin(), partial_latch->end(), cmp);
    if (necessary->size() > 0) {
        sort(necessary->begin(), necessary->end(), cmp);
        shared_ptr<cube> merged = make_shared<cube>(partial_latch->size() + necessary->size());
        merge(necessary->begin(), necessary->end(), partial_latch->begin(), partial_latch->end(), merged->begin(), cmp);
        auto last = unique(merged->begin(), merged->end());
        merged->erase(last, merged->end());
        partial_latch = merged;
    }
    t.second = partial_latch;

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
        if (m_settings.Branching > 0)
            stable_sort(uc_blockers.begin(), uc_blockers.end(), blockerOrder);
        uc_blocker = uc_blockers[0];
    } else {
        uc_blocker = make_shared<cube>();
    }

    if (m_settings.skip_refer)
        for (auto b : *uc_blocker) required_lits.emplace(b);
    OrderAssumption(uc);
    for (int i = uc->size() - 1; i >= 0; i--) {
        if (uc->size() < 2) break;
        if (required_lits.find(uc->at(i)) != required_lits.end()) continue;
        shared_ptr<cube> temp_uc(new cube());
        temp_uc->reserve(uc->size());
        for (auto ll : *uc)
            if (ll != uc->at(i)) temp_uc->emplace_back(ll);
        if (Down(temp_uc, frame_lvl, rec_lvl, required_lits)) {
            uc->swap(*temp_uc);
            OrderAssumption(uc);
            i = uc->size();
        } else {
            required_lits.emplace(uc->at(i));
        }
    }
    sort(uc->begin(), uc->end(), cmp);
    if (uc->size() > uc_blocker->size() && frame_lvl != 0) {
        return false;
    } else {
        return true;
    }
}


bool ForwardChecker::Down(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl, unordered_set<int> required_lits) {
    int ctgs = 0;
    m_log->L(3, "Down:", CubeToStr(uc));
    shared_ptr<cube> assumption(new cube(*uc));
    GetPrimed(assumption);
    shared_ptr<State> p_ucs(new State(nullptr, nullptr, uc, 0));
    while (true) {
        // F_i & T & temp_uc'
        m_log->Tick();
        if (!m_mainSolver->Solve(assumption, frame_lvl)) {
            m_log->StatMainSolver();
            auto uc_ctg = m_mainSolver->GetUC(true);
            if (uc->size() < uc_ctg->size()) return false; // there are cases that uc_ctg longer than uc
            uc->swap(*uc_ctg);
            return true;
        } else if (rec_lvl > 2) {
            m_log->StatMainSolver();
            return false;
        } else {
            m_log->StatMainSolver();
            auto p = m_mainSolver->GetAssignment(false);
            GeneralizePredecessor(p, p_ucs);
            shared_ptr<State> cts(new State(nullptr, p.first, p.second, 0));
            int cts_lvl = GetNewLevel(cts);
            shared_ptr<cube> cts_ass(new cube(*cts->latches));
            OrderAssumption(cts_ass);
            GetPrimed(cts_ass);
            // F_i-1 & T & cts'
            m_log->L(3, "Try ctg:", CubeToStr(cts->latches));
            m_log->Tick();
            if (ctgs < 3 && cts_lvl >= 0 && !m_mainSolver->Solve(cts_ass, cts_lvl)) {
                m_log->StatMainSolver();
                ctgs++;
                auto uc_cts = m_mainSolver->GetUC(true);
                m_log->L(3, "CTG Get UC:", CubeToStr(uc_cts));
                if (Generalize(uc_cts, cts_lvl, rec_lvl + 1))
                    m_branching->Update(uc_cts);
                m_log->L(3, "CTG Get Generalized UC:", CubeToStr(uc_cts));
                AddUnsatisfiableCore(uc_cts, cts_lvl + 1);
                PropagateUp(uc_cts, cts_lvl + 1);
            } else {
                m_log->StatMainSolver();
                return false;
            }
        }
    }
}


bool ForwardChecker::Propagate(shared_ptr<cube> c, int lvl) {
    m_log->Tick();

    bool result;
    shared_ptr<cube> assumption(new cube(*c));
    GetPrimed(assumption);
    if (!m_mainSolver->Solve(assumption, lvl)) {
        AddUnsatisfiableCore(c, lvl + 1);
        result = true;
    } else {
        result = false;
    }

    m_log->StatPropagation();
    return result;
}


int ForwardChecker::PropagateUp(shared_ptr<cube> c, int lvl) {
    while (lvl + 1 < m_overSequence->GetLength()) {
        if (Propagate(c, lvl))
            m_branching->Update(c);
        else
            break;
        lvl++;
    }
    return lvl + 1;
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
    string outPath = m_settings.outputDir + aigName + ".w.aag";
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
        vector<shared_ptr<cube>> frame_i;
        m_overSequence->GetFrame(i, frame_i);
        vector<unsigned> frame_i_lits;
        for (unsigned j = 0; j < frame_i.size(); j++) {
            vector<unsigned> cube_j;
            for (int l : *frame_i[j]) cube_j.push_back(l > 0 ? (2 * l) : (2 * -l + 1));
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
    auto startIndex = m_settings.aigFilePath.find_last_of("/");
    if (startIndex == string::npos) {
        startIndex = 0;
    } else {
        startIndex++;
    }
    auto endIndex = m_settings.aigFilePath.find_last_of(".");
    assert(endIndex != string::npos);
    string aigName = m_settings.aigFilePath.substr(startIndex, endIndex - startIndex);
    string cexPath = m_settings.outputDir + aigName + ".cex";
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