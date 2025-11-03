#include "CircuitGraph.h"

namespace car {

void aigerDeleter(aiger *aig) {
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

    trueId = ++numVar;

    // I L O A B C J F
    int i = 1;
    for (int j = 0; j < aig->num_inputs; j++, i++) {
        inputs.emplace_back(i);
        inputsSet.emplace(i);
    }
    for (int i = 0; i < aig->num_latches; i++) {
        int l = GetCarId(aig->latches[i].lit);
        int next = GetCarId(aig->latches[i].next);
        int reset = GetCarId(aig->latches[i].reset);

        latches.emplace_back(l);
        latchesSet.emplace(l);
        latchNextMap[l] = next;
        latchResetMap[l] = reset;
    }
    for (int i = 0; i < aig->num_outputs; i++) {
        bad.emplace_back(GetCarId(aig->outputs[i].lit));
    }
    for (int i = 0; i < aig->num_ands; i++) {
        ands.emplace_back(GetCarId(aig->ands[i].lhs));
        andsSet.emplace(GetCarId(aig->ands[i].lhs));
    }
    for (int i = 0; i < aig->num_bad; i++) {
        bad.emplace_back(GetCarId(aig->bad[i].lit));
    }
    for (int i = 0; i < aig->num_constraints; i++) {
        constraints.emplace_back(GetCarId(aig->constraints[i].lit));
    }
    for (int i = 0; i < aig->num_justice; i++) {
        justice.emplace_back(GetCarId(aig->justice[i].lit));
    }
    for (int i = 0; i < aig->num_fairness; i++) {
        fairness.emplace_back(GetCarId(aig->fairness[i].lit));
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
    unordered_set<int> coi_ids;
    for (int id : constraints)
        coi_ids.emplace(abs(id));
    for (int id : bad)
        coi_ids.emplace(abs(id));
    for (int i = modelGates.size() - 1; i >= 0; i--) {
        int g = modelGates[i];
        if (coi_ids.find(g) != coi_ids.end()) {
            for (int fanin : gatesMap[g].fanins)
                coi_ids.emplace(abs(fanin));
        }
    }
    for (int id : coi_ids) {
        if (inputsSet.find(id) != inputsSet.end()) {
            propertyCOIInputs.emplace_back(id);
        }
    }
    sort(propertyCOIInputs.begin(), propertyCOIInputs.end(), cmp);
}


void CircuitGraph::COIRefine() {
    unordered_set<int> coi_ids;
    vector<int> todo_stack;

    for (int id : constraints) {
        coi_ids.emplace(abs(id));
        todo_stack.emplace_back(abs(id));
    }
    for (int id : bad) {
        coi_ids.emplace(abs(id));
        todo_stack.emplace_back(abs(id));
    }

    while (!todo_stack.empty()) {
        int id = todo_stack.back();
        todo_stack.pop_back();

        // is gate
        if (andsSet.find(id) != andsSet.end()) {
            assert(gatesMap.find(id) != gatesMap.end());
            auto &gate = gatesMap[id];
            for (int fanin : gate.fanins) {
                if (coi_ids.find(abs(fanin)) == coi_ids.end()) {
                    coi_ids.emplace(abs(fanin));
                    todo_stack.emplace_back(abs(fanin));
                }
            }
        }
        // is latch
        else if (latchesSet.find(id) != latchesSet.end()) {
            int next = latchNextMap[id];
            if (coi_ids.find(abs(next)) == coi_ids.end()) {
                coi_ids.emplace(abs(next));
                todo_stack.emplace_back(abs(next));
            }
        }
    }


    // refine model inputs, latches, and gates
    vector<int> new_modelInputs;
    for (int id : modelInputs) {
        if (coi_ids.find(id) != coi_ids.end()) {
            new_modelInputs.emplace_back(id);
        }
    }
    modelInputs = new_modelInputs;
    sort(modelInputs.begin(), modelInputs.end());

    vector<int> new_modelLatches;
    for (int id : modelLatches) {
        if (coi_ids.find(id) != coi_ids.end()) {
            new_modelLatches.emplace_back(id);
        }
    }
    modelLatches = new_modelLatches;
    sort(modelLatches.begin(), modelLatches.end());

    vector<int> new_modelGates;
    for (int id : modelGates) {
        if (coi_ids.find(id) != coi_ids.end()) {
            new_modelGates.emplace_back(id);
        }
    }
    modelGates = new_modelGates;
    sort(modelGates.begin(), modelGates.end());
}


bool CircuitGraph::TryMakeXORGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coi_lits) {
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

        int fanout = GetCarId(a);
        assert(fanout > 0);
        int fanin0 = GetCarId(a00);
        int fanin1 = GetCarId(a01);

        // is XOR
        gatesMap[fanout] = CircuitGate(CircuitGate::GateType::XOR, fanout, {fanin0, fanin1});

        coi_lits.emplace(aiger_strip(a00));
        coi_lits.emplace(aiger_strip(a01));

        return true;
    }
    return false;
}

bool CircuitGraph::TryMakeITEGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coi_lits) {
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
    int fanout = GetCarId(a);
    assert(fanout > 0);
    int i = GetCarId(ite[0]);
    int t = GetCarId(ite[1]);
    int e = GetCarId(ite[2]);

    gatesMap[fanout] = CircuitGate(CircuitGate::GateType::ITE, fanout, {i, t, e});

    coi_lits.emplace(aiger_strip(ite[0]));
    coi_lits.emplace(aiger_strip(ite[1]));
    coi_lits.emplace(aiger_strip(ite[2]));

    return true;
}


bool CircuitGraph::MakeAndGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coi_lits) {
    aiger_and *aa = aiger_is_and(aig.get(), a);
    assert(aa != nullptr);

    int fanout = GetCarId(a);
    assert(fanout > 0);
    int fanin0 = GetCarId(aa->rhs0);
    int fanin1 = GetCarId(aa->rhs1);

    gatesMap[fanout] = CircuitGate(CircuitGate::GateType::AND, fanout, {fanin0, fanin1});

    coi_lits.emplace(aiger_strip(aa->rhs0));
    coi_lits.emplace(aiger_strip(aa->rhs1));

    return true;
}


} // namespace car
