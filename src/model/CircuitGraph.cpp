#include "CircuitGraph.h"

namespace car {

void AigerDeleter(aiger *aig) {
    aiger_reset(aig);
}

CircuitGraph::CircuitGraph(const shared_ptr<aiger> aig) {
    // number of variables
    numVar = aig->maxvar;
    numInputs = aig->num_inputs;
    numLatches = aig->num_latches;
    numOutputs = aig->num_outputs;
    numAnds = aig->num_ands;
    numBad = aig->num_bad;
    numConstraints = aig->num_constraints;
    numJustice = aig->num_justice;
    numFairness = aig->num_fairness;

    // I L O A B C J F
    Var i = 1;
    for (int j = 0; j < aig->num_inputs; j++, i++) {
        inputs.emplace_back(i);
        inputsSet.emplace(i);
    }
    for (int i = 0; i < aig->num_latches; i++) {
        Lit l_lit = FromAigerLit(aig->latches[i].lit);
        Var l = VarOf(l_lit);
        Lit next = FromAigerLit(aig->latches[i].next);
        Lit reset = FromAigerLit(aig->latches[i].reset);

        latches.emplace_back(l);
        latchesSet.emplace(l);
        latchNextMap[l] = next;
        latchResetMap[l] = reset;
    }
    for (int i = 0; i < aig->num_outputs; i++) {
        bad.emplace_back(FromAigerLit(aig->outputs[i].lit));
    }
    for (int i = 0; i < aig->num_ands; i++) {
        Var lhs = VarOf(FromAigerLit(aig->ands[i].lhs));
        ands.emplace_back(lhs);
        andsSet.emplace(lhs);
    }
    for (int i = 0; i < aig->num_bad; i++) {
        bad.emplace_back(FromAigerLit(aig->bad[i].lit));
    }
    for (int i = 0; i < aig->num_constraints; i++) {
        constraints.emplace_back(FromAigerLit(aig->constraints[i].lit));
    }
    for (int i = 0; i < aig->num_justice; i++) {
        Cube lits;
        for (unsigned j = 0; j < aig->justice[i].size; j++) {
            lits.emplace_back(FromAigerLit(aig->justice[i].lits[j]));
        }
        justice.emplace_back(lits);
    }
    for (int i = 0; i < aig->num_fairness; i++) {
        fairness.emplace_back(FromAigerLit(aig->fairness[i].lit));
    }

    // get gates
    unordered_set<unsigned> coi_lits;
    for (int i = 0; i < aig->num_latches; i++)
        coi_lits.emplace(aiger_strip(aig->latches[i].next));
    for (int i = 0; i < aig->num_constraints; i++)
        coi_lits.emplace(aiger_strip(aig->constraints[i].lit));
    for (int i = 0; i < aig->num_outputs; i++)
        coi_lits.emplace(aiger_strip(aig->outputs[i].lit));
    for (int i = 0; i < aig->num_bad; i++)
        coi_lits.emplace(aiger_strip(aig->bad[i].lit));
    for (int i = 0; i < aig->num_justice; i++) {
        for (unsigned j = 0; j < aig->justice[i].size; j++) {
            coi_lits.emplace(aiger_strip(aig->justice[i].lits[j]));
        }
    }
    for (int i = 0; i < aig->num_fairness; i++)
        coi_lits.emplace(aiger_strip(aig->fairness[i].lit));

    // tranverse and gates reversely
    for (int i = aig->num_ands - 1; i >= 0; i--) {
        aiger_and &a = aig->ands[i];
        if (coi_lits.find(a.lhs) != coi_lits.end()) {
            if (TryMakeXORGate(aig, a.lhs, coi_lits)) {
                continue;
            }
            if (TryMakeITEGate(aig, a.lhs, coi_lits)) {
                continue;
            }
            MakeAndGate(aig, a.lhs, coi_lits);
        }
    }

    // model inputs, latches, and gates, to be coi refined
    modelInputs = inputs;
    modelLatches = latches;
    modelGates = ands;
    COIRefine();

    // get property coi inputs
    unordered_set<Var> coi_ids;
    for (Lit id : constraints)
        coi_ids.emplace(VarOf(id));
    for (Lit id : bad)
        coi_ids.emplace(VarOf(id));
    for (int i = static_cast<int>(modelGates.size()) - 1; i >= 0; i--) {
        Var g = modelGates[i];
        if (coi_ids.find(g) != coi_ids.end()) {
            for (Lit fanin : gatesMap[g].fanins)
                coi_ids.emplace(VarOf(fanin));
        }
    }
    for (Var id : coi_ids) {
        if (inputsSet.find(id) != inputsSet.end()) {
            propertyCOIInputs.emplace_back(id);
        }
    }
    sort(propertyCOIInputs.begin(), propertyCOIInputs.end());
}


void CircuitGraph::COIRefine() {
    unordered_set<Var> coi_ids;
    vector<Var> todo_stack;

    for (Lit id : constraints) {
        coi_ids.emplace(VarOf(id));
        todo_stack.emplace_back(VarOf(id));
    }
    for (Lit id : bad) {
        coi_ids.emplace(VarOf(id));
        todo_stack.emplace_back(VarOf(id));
    }
    for (Lit id : fairness) {
        coi_ids.emplace(VarOf(id));
        todo_stack.emplace_back(VarOf(id));
    }
    for (const auto &j : justice) {
        for (Lit id : j) {
            coi_ids.emplace(VarOf(id));
            todo_stack.emplace_back(VarOf(id));
        }
    }

    while (!todo_stack.empty()) {
        Var id = todo_stack.back();
        todo_stack.pop_back();

        // is gate
        if (andsSet.find(id) != andsSet.end()) {
            assert(gatesMap.find(id) != gatesMap.end());
            auto &gate = gatesMap[id];
            for (Lit fanin : gate.fanins) {
                Var fanin_var = VarOf(fanin);
                if (coi_ids.find(fanin_var) == coi_ids.end()) {
                    coi_ids.emplace(fanin_var);
                    todo_stack.emplace_back(fanin_var);
                }
            }
        }
        // is latch
        else if (latchesSet.find(id) != latchesSet.end()) {
            Lit next = latchNextMap[id];
            Var next_var = VarOf(next);
            if (coi_ids.find(next_var) == coi_ids.end()) {
                coi_ids.emplace(next_var);
                todo_stack.emplace_back(next_var);
            }
        }
    }


    // refine model inputs, latches, and gates
    vector<Var> new_model_inputs;
    for (Var id : modelInputs) {
        if (coi_ids.find(id) != coi_ids.end()) {
            new_model_inputs.emplace_back(id);
        }
    }
    modelInputs = new_model_inputs;
    sort(modelInputs.begin(), modelInputs.end());

    vector<Var> new_model_latches;
    for (Var id : modelLatches) {
        if (coi_ids.find(id) != coi_ids.end()) {
            new_model_latches.emplace_back(id);
        }
    }
    modelLatches = new_model_latches;
    sort(modelLatches.begin(), modelLatches.end());

    vector<Var> new_model_gates;
    for (Var id : modelGates) {
        if (coi_ids.find(id) != coi_ids.end()) {
            new_model_gates.emplace_back(id);
        }
    }
    modelGates = new_model_gates;
    sort(modelGates.begin(), modelGates.end());
}


Var CircuitGraph::NewModelVar() {
    Var id = ++numVar;
    return id;
}

Var CircuitGraph::NewInputVar() {
    Var id = NewModelVar();
    inputs.emplace_back(id);
    inputsSet.emplace(id);
    modelInputs.emplace_back(id);
    numInputs++;
    return id;
}

Var CircuitGraph::NewLatchVar() {
    Var id = NewModelVar();
    latches.emplace_back(id);
    latchesSet.emplace(id);
    modelLatches.emplace_back(id);
    numLatches++;
    return id;
}

void CircuitGraph::SetLatchResetNext(Var latch, Lit reset, Lit next) {
    latchResetMap[latch] = reset;
    latchNextMap[latch] = next;
}

Var CircuitGraph::NewAndGate(Lit a, Lit b) {
    Var id = NewModelVar();
    ands.emplace_back(id);
    andsSet.emplace(id);
    modelGates.emplace_back(id);
    gatesMap[id] = CircuitGate(CircuitGate::GateType::AND, id, {a, b});
    numAnds++;
    return id;
}

bool CircuitGraph::TryMakeXORGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coiLits) {
    aiger_and *aa = aiger_is_and(aig.get(), a);
    assert(aa != nullptr);

    unsigned a0 = aa->rhs0;
    unsigned a1 = aa->rhs1;
    if (!aiger_sign(a0) || !aiger_sign(a1)) return false;

    aiger_and *aa0 = aiger_is_and(aig.get(), a0);
    aiger_and *aa1 = aiger_is_and(aig.get(), a1);
    if (aa0 == nullptr || aa1 == nullptr) return false;

    unsigned a00 = aa0->rhs0;
    unsigned a01 = aa0->rhs1;
    unsigned a10 = aa1->rhs0;
    unsigned a11 = aa1->rhs1;

    if (a00 == aiger_not(a10) && a01 == aiger_not(a11)) {
        if (a00 == a01) return false;

        Lit fanout = FromAigerLit(a);
        assert(!Sign(fanout));
        Lit fanin0 = FromAigerLit(a00);
        Lit fanin1 = FromAigerLit(a01);

        // is XOR
        gatesMap[VarOf(fanout)] = CircuitGate(CircuitGate::GateType::XOR, VarOf(fanout), {fanin0, fanin1});

        coiLits.emplace(aiger_strip(a00));
        coiLits.emplace(aiger_strip(a01));

        return true;
    }
    return false;
}

bool CircuitGraph::TryMakeITEGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coiLits) {
    aiger_and *aa = aiger_is_and(aig.get(), a);
    assert(aa != nullptr);

    unsigned a0 = aa->rhs0;
    unsigned a1 = aa->rhs1;
    if (!aiger_sign(a0) || !aiger_sign(a1)) return false;

    aiger_and *aa0 = aiger_is_and(aig.get(), a0);
    aiger_and *aa1 = aiger_is_and(aig.get(), a1);
    if (aa0 == nullptr || aa1 == nullptr) return false;

    unsigned a00 = aa0->rhs0;
    unsigned a01 = aa0->rhs1;
    unsigned a10 = aa1->rhs0;
    unsigned a11 = aa1->rhs1;

    std::vector<unsigned> ite;
    if (a00 == aiger_not(a10)) {
        ite = {a00, aiger_not(a01), aiger_not(a11)};
    } else if (a00 == aiger_not(a11)) {
        ite = {a00, aiger_not(a01), aiger_not(a10)};
    } else if (a01 == aiger_not(a10)) {
        ite = {a01, aiger_not(a00), aiger_not(a11)};
    } else if (a01 == aiger_not(a11)) {
        ite = {a01, aiger_not(a00), aiger_not(a10)};
    } else {
        return false;
    }

    // is ITE
    Lit fanout = FromAigerLit(a);
    assert(!Sign(fanout));
    Lit i = FromAigerLit(ite[0]);
    Lit t = FromAigerLit(ite[1]);
    Lit e = FromAigerLit(ite[2]);

    gatesMap[VarOf(fanout)] = CircuitGate(CircuitGate::GateType::ITE, VarOf(fanout), {i, t, e});

    coiLits.emplace(aiger_strip(ite[0]));
    coiLits.emplace(aiger_strip(ite[1]));
    coiLits.emplace(aiger_strip(ite[2]));

    return true;
}


bool CircuitGraph::MakeAndGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coiLits) {
    aiger_and *aa = aiger_is_and(aig.get(), a);
    assert(aa != nullptr);

    Lit fanout = FromAigerLit(a);
    assert(!Sign(fanout));
    Lit fanin0 = FromAigerLit(aa->rhs0);
    Lit fanin1 = FromAigerLit(aa->rhs1);

    gatesMap[VarOf(fanout)] = CircuitGate(CircuitGate::GateType::AND, VarOf(fanout), {fanin0, fanin1});

    coiLits.emplace(aiger_strip(aa->rhs0));
    coiLits.emplace(aiger_strip(aa->rhs1));

    return true;
}


} // namespace car
