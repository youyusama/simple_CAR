#ifndef TERNARY_SIM_H
#define TERNARY_SIM_H

#include "CircuitGraph.h"
#include "Log.h"
#include "Settings.h"
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>

namespace car {

// ternary logic
class tbool {
    uint8_t value;

  public:
    tbool() : value(0) {}

    explicit tbool(uint8_t v) : value(v) {}

    explicit tbool(bool x) : value(!x) {}

    tbool operator!() const { return (value == 2) ? *this : tbool((uint8_t)(1 - value)); }

    bool operator==(tbool b) const { return (value == b.value) && (value != 2); }

    bool operator!=(tbool b) const { return !(*this == b); }

    tbool operator^(tbool b) const {
        uint8_t sel = (this->value << 1) | (b.value << 3);
        uint8_t v = (0x2A2421 >> sel) & 3;
        return tbool(v);
    }

    tbool operator&&(tbool b) const {
        uint8_t sel = (this->value << 1) | (b.value << 3);
        uint8_t v = (0x261524 >> sel) & 3;
        return tbool(v);
    }

    tbool operator||(tbool b) const {
        uint8_t sel = (this->value << 1) | (b.value << 3);
        uint8_t v = (0x282400 >> sel) & 3;
        return tbool(v);
    }

    friend const char *toStr(tbool l);
};

const tbool t_True((uint8_t)0);
const tbool t_False((uint8_t)1);
const tbool t_Undef((uint8_t)2);

inline tbool ite(tbool c, tbool a, tbool b) {
    if (c == t_True) return a;
    if (c == t_False) return b;
    return (a == b) ? a : t_Undef;
}

inline const char *toStr(tbool l) {
    switch (l.value) {
    case 0: return "1";
    case 1: return "0";
    case 2: return "X";
    default: return "E"; // E for Error/Invalid state
    }
}

void testTruthTables();

class TernarySimulator {
  public:
    TernarySimulator(shared_ptr<CircuitGraph> circuitGraph, shared_ptr<Log> log);
    ~TernarySimulator() {};

    bool setVal(int id, tbool v, int step);

    tbool getVal(int id, int step);

    tbool getVal(int id, unordered_map<int, tbool> &vmap);

    void simulate(int maxSteps);

    void simulateRandom(int maxSteps);

    bool isCycleReached() { return m_cycleStart != -1; };

    const vector<shared_ptr<unordered_map<int, tbool>>> &getValues() { return m_values; };

    const vector<shared_ptr<vector<int>>> &getStates() { return m_states; };

    const vector<shared_ptr<vector<int>>> &getGateStates() { return m_gateStates; };

  private:
    shared_ptr<Log> m_log;

    shared_ptr<CircuitGraph> m_circuitGraph;

    vector<shared_ptr<unordered_map<int, tbool>>> m_values;

    void pushState(int step, shared_ptr<vector<int>> &state);

    vector<shared_ptr<vector<int>>> m_states;

    void pushGateState(int step, shared_ptr<vector<int>> &gatestate);

    vector<shared_ptr<vector<int>>> m_gateStates;

    int m_step; // next step to simulate

    void reset();

    void simulateOneStep();

    bool reachCycle();

    int m_cycleStart;

    string stepValuesToString(int step);
};

} // namespace car

#endif // TERNARY_SIM_H