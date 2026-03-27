#ifndef CIRCUITGRAPH_H
#define CIRCUITGRAPH_H

extern "C" {
#include "aiger.h"
}

#include "CarTypes.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

namespace car {

void AigerDeleter(aiger *aig);

struct CircuitGate {
    enum GateType { AND,
                    XOR,
                    ITE };
    CircuitGate() {};

    CircuitGate(GateType gateType, Var fanout, const vector<Lit> &fanins) {
        this->gateType = gateType;
        this->fanout = fanout;
        this->fanins = fanins;
    }

    CircuitGate(const CircuitGate &other) {
        this->gateType = other.gateType;
        this->fanout = other.fanout;
        this->fanins = other.fanins;
    }

    GateType gateType;
    Var fanout;
    vector<Lit> fanins;
};


class CircuitGraph {
  public:
    CircuitGraph(const shared_ptr<aiger> aig);
    ~CircuitGraph() {};

    // variable numbers
    unsigned numVar;
    unsigned numInputs;
    unsigned numLatches;
    unsigned numOutputs;
    unsigned numAnds;
    unsigned numBad;
    unsigned numConstraints;
    unsigned numJustice;
    unsigned numFairness;

    // variables for tranverse
    vector<Var> inputs;
    vector<Var> latches;
    Cube outputs;
    vector<Var> ands;
    Cube bad;
    Cube constraints;
    vector<Cube> justice;
    Cube fairness;

    // variables for query
    unordered_set<Var> inputsSet;
    unordered_set<Var> latchesSet;
    unordered_set<Var> andsSet;

    // latch maps
    unordered_map<Var, Lit> latchNextMap;
    unordered_map<Var, Lit> latchResetMap;

    // refine the COI of property & constraints, get new model inputs, latches, and gates
    void COIRefine();

    Var NewModelVar();

    Var NewInputVar();

    Var NewLatchVar();

    void SetLatchResetNext(Var latch, Lit reset, Lit next);

    Var NewAndGate(Lit a, Lit b);

    // variables really matter
    vector<Var> modelInputs;
    vector<Var> modelLatches;
    vector<Var> modelGates;

    // inputs matter for property (but not for transition relation)
    vector<Var> propertyCOIInputs;

    unordered_map<Var, CircuitGate> gatesMap; // gates in the COI of property & constraints & transition relation

  private:
    bool TryMakeXORGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coiLits);

    bool TryMakeITEGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coiLits);

    bool MakeAndGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coiLits);
};

} // namespace car

#endif // CIRCUITGRAPH_H
