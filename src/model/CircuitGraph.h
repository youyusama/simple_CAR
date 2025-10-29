#ifndef CIRCUITGRAPH_H
#define CIRCUITGRAPH_H

extern "C" {
#include "aiger.h"
}

using namespace std;

#include <algorithm>
#include <cassert>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace car {

inline bool cmp(int a, int b) {
    if (abs(a) != abs(b))
        return abs(a) < abs(b);
    else
        return a < b;
}

void aigerDeleter(aiger *aig);

struct CircuitGate {
    enum GateType { AND,
                    XOR,
                    ITE };
    CircuitGate() {};

    CircuitGate(GateType gateType, int fanout, const vector<int> &fanins) {
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
    int fanout;
    vector<int> fanins;
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
    vector<int> inputs;
    vector<int> latches;
    vector<int> outputs;
    vector<int> ands;
    vector<int> bad;
    vector<int> constraints;
    vector<int> justice;
    vector<int> fairness;

    // variables for query
    unordered_set<int> inputsSet;
    unordered_set<int> latchesSet;
    unordered_set<int> andsSet;

    // latch maps
    unordered_map<int, int> latchNextMap;
    unordered_map<int, int> latchResetMap;

    // ture id
    int trueId;

    // refine the COI of property & constraints, get new model inputs, latches, and gates
    void COIRefine();

    // variables really matter
    vector<int> modelInputs;
    vector<int> modelLatches;
    vector<int> modelGates;

    // inputs matter for property (but not for transition relation)
    vector<int> propertyCOIInputs;

    unordered_map<int, CircuitGate> gatesMap; // gates in the COI of property & constraints & transition relation

  private:
    bool TryMakeXORGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coi_lits);

    bool TryMakeITEGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coi_lits);

    bool MakeAndGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coi_lits);

    inline int GetCarId(const unsigned lit) {
        if (lit == 0)
            return -trueId;
        else if (lit == 1)
            return trueId;
        return (aiger_sign(lit) == 0) ? lit >> 1 : -(lit >> 1);
    }

    inline unsigned GetAigerLit(const int car_id) {
        if (car_id > 0)
            return car_id << 1;
        else
            return (-car_id << 1) + 1;
    }
};

} // namespace car

#endif // CIRCUITGRAPH_H