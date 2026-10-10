#pragma once

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

namespace car {

void AigerDeleter(aiger *aig);

struct CircuitGate {
    enum GateType { AND,
                    XOR,
                    ITE };
    CircuitGate() {};

    CircuitGate(GateType gateType, Var fanout, const std::vector<Lit> &fanins) {
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
    std::vector<Lit> fanins;
};


class CircuitGraph {
  public:
    CircuitGraph(const std::shared_ptr<aiger> aig);
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
    std::vector<Var> inputs;
    std::vector<Var> latches;
    Cube outputs;
    std::vector<Var> ands;
    Cube bad;
    Cube constraints;
    std::vector<Cube> justice;
    Cube fairness;

    // variables for query
    std::unordered_set<Var> inputsSet;
    std::unordered_set<Var> latchesSet;
    std::unordered_set<Var> andsSet;

    // latch maps
    std::unordered_map<Var, Lit> latchNextMap;
    std::unordered_map<Var, Lit> latchResetMap;

    // refine the COI of property & constraints, get new model inputs, latches, and gates
    void COIRefine();

    void CollectPropertyCOIInputs();

    Var NewModelVar();

    Var NewInputVar();

    Var NewLatchVar();

    void SetLatchResetNext(Var latch, Lit reset, Lit next);

    Var NewAndGate(Lit a, Lit b);

    // variables really matter
    std::vector<Var> modelInputs;
    std::vector<Var> modelLatches;
    std::vector<Var> modelGates;

    // inputs matter for property (but not for transition relation)
    std::vector<Var> propertyCOIInputs;

    std::unordered_map<Var, CircuitGate> gatesMap; // gates in the COI of property & constraints & transition relation

  private:
    bool TryMakeXORGate(const std::shared_ptr<aiger> aig, const unsigned a, std::unordered_set<unsigned> &coiLits);

    bool TryMakeITEGate(const std::shared_ptr<aiger> aig, const unsigned a, std::unordered_set<unsigned> &coiLits);

    bool MakeAndGate(const std::shared_ptr<aiger> aig, const unsigned a, std::unordered_set<unsigned> &coiLits);
};

} // namespace car
