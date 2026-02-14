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
class Tbool {
    uint8_t m_value;

  public:
    Tbool() : m_value(0) {}

    explicit Tbool(uint8_t v) : m_value(v) {}

    explicit Tbool(bool x) : m_value(!x) {}

    Tbool operator!() const { return (m_value == 2) ? *this : Tbool((uint8_t)(1 - m_value)); }

    bool operator==(Tbool b) const { return (m_value == b.m_value) && (m_value != 2); }

    bool operator!=(Tbool b) const { return !(*this == b); }

    Tbool operator^(Tbool b) const {
        uint8_t sel = (this->m_value << 1) | (b.m_value << 3);
        uint8_t v = (0x2A2421 >> sel) & 3;
        return Tbool(v);
    }

    Tbool operator&&(Tbool b) const {
        uint8_t sel = (this->m_value << 1) | (b.m_value << 3);
        uint8_t v = (0x261524 >> sel) & 3;
        return Tbool(v);
    }

    Tbool operator||(Tbool b) const {
        uint8_t sel = (this->m_value << 1) | (b.m_value << 3);
        uint8_t v = (0x282400 >> sel) & 3;
        return Tbool(v);
    }

    uint8_t Raw() const { return m_value; }

    friend const char *ToStr(Tbool l);
};

const Tbool T_TRUE((uint8_t)0);
const Tbool T_FALSE((uint8_t)1);
const Tbool T_UNDEF((uint8_t)2);

inline Tbool Ite(Tbool c, Tbool a, Tbool b) {
    if (c == T_TRUE) return a;
    if (c == T_FALSE) return b;
    return (a == b) ? a : T_UNDEF;
}

inline const char *ToStr(Tbool l) {
    switch (l.m_value) {
    case 0: return "1";
    case 1: return "0";
    case 2: return "X";
    default: return "E"; // E for Error/Invalid state
    }
}

void TestTruthTables();

class TernarySimulator {
  public:
    TernarySimulator(shared_ptr<CircuitGraph> circuitGraph, Log &log);
    ~TernarySimulator() {};

    bool SetVal(int id, Tbool v, int step);

    Tbool GetVal(int id, int step);

    Tbool GetVal(int id, const vector<Tbool> &vmap);

    void Simulate(int maxPreciseDepth);

    void SimulateRandom(int maxSteps);

    bool IsCycleReached() { return m_cycleStart != -1; };

    const vector<vector<Tbool>> &GetValues() { return m_values; };

    const vector<vector<int>> &GetStates() { return m_states; };

    const vector<vector<int>> &GetGateStates() { return m_gateStates; };

  private:
    Log &m_log;

    shared_ptr<CircuitGraph> m_circuitGraph;

    vector<vector<Tbool>> m_values;

    void InitStepValues();

    void PushState(int step, vector<int> &state);

    vector<vector<int>> m_states;

    void PushGateState(int step, vector<int> &gatestate);

    vector<vector<int>> m_gateStates;

    int m_randomSeed;

    int m_step; // next step to simulate

    void Reset();

    void SimulateOneStep();

    bool ReachCycle();

    int m_cycleStart;

    string StepValuesToString(int step);

    int AbstractCurrentState(int step);
};

} // namespace car

#endif // TERNARY_SIM_H
