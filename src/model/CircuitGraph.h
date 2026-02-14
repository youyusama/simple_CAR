#ifndef CIRCUITGRAPH_H
#define CIRCUITGRAPH_H

extern "C" {
#include "aiger.h"
}

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

inline bool Cmp(int a, int b) {
    if (abs(a) != abs(b))
        return abs(a) < abs(b);
    else
        return a < b;
}

void AigerDeleter(aiger *aig);

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
    vector<vector<int>> justice;
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

    int NewModelVar();

    int NewInputVar();

    int NewLatchVar();

    void SetLatchResetNext(int latch, int reset, int next);

    int NewAndGate(int a, int b);

    // variables really matter
    vector<int> modelInputs;
    vector<int> modelLatches;
    vector<int> modelGates;

    // inputs matter for property (but not for transition relation)
    vector<int> propertyCOIInputs;

    unordered_map<int, CircuitGate> gatesMap; // gates in the COI of property & constraints & transition relation

  private:
    bool TryMakeXORGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coiLits);

    bool TryMakeITEGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coiLits);

    bool MakeAndGate(const shared_ptr<aiger> aig, const unsigned a, unordered_set<unsigned> &coiLits);

    inline int GetCarId(const unsigned lit) {
        if (lit == 0)
            return -trueId;
        else if (lit == 1)
            return trueId;
        return (aiger_sign(lit) == 0) ? lit >> 1 : -(lit >> 1);
    }

    inline unsigned GetAigerLit(const int carId) {
        if (carId > 0)
            return carId << 1;
        else
            return (-carId << 1) + 1;
    }
};

} // namespace car

#endif // CIRCUITGRAPH_H
