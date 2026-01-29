#include "TernarySim.h"

namespace car {

void testTruthTables() {
    std::cout << "--- lbool Truth Tables ---" << std::endl;
    std::cout << "T = l_True, F = l_False, U = l_Undef, E = Error/Invalid (value=3)" << std::endl;

    std::vector<tbool> states = {t_True, t_False, t_Undef};

    // unary operators
    std::cout << "\n--- Operator:!a ---" << std::endl;
    for (tbool a : states) {
        std::cout << "!" << toStr(a) << " = " << toStr(!a) << std::endl;
    }

    // binary operators
    const char *ops[] = {"&&", "||", "^", "=="};
    for (const char *op : ops) {
        std::cout << "\n--- Operator: a " << op << " b ---" << std::endl;
        std::cout << "a \\ b |  T    F    U" << std::endl;
        std::cout << "------+------------" << std::endl;
        for (tbool a : states) {
            std::cout << "  " << toStr(a) << "   | ";
            for (tbool b : states) {
                tbool result(false);
                if (std::string(op) == "&&")
                    result = a && b;
                else if (std::string(op) == "||")
                    result = a || b;
                else if (std::string(op) == "^")
                    result = a ^ b;
                else if (std::string(op) == "==") {
                    bool res_bool = (a == b);
                    std::cout << std::left << std::setw(5) << (res_bool ? "true" : "false");
                    continue;
                }
                std::cout << std::left << std::setw(5) << toStr(result);
            }
            std::cout << std::endl;
        }
    }

    // ternary operator
    std::cout << "\n--- Operator: ite(c, a, b) ---" << std::endl;
    for (tbool c : states) {
        std::cout << "\n  When c = " << toStr(c) << ":" << std::endl;
        std::cout << "  a \\ b |  T    F    U" << std::endl;
        std::cout << "  ------+------------" << std::endl;
        for (tbool a : states) {
            std::cout << "    " << toStr(a) << "   | ";
            for (tbool b : states) {
                tbool result = ite(c, a, b);
                std::cout << std::left << std::setw(5) << toStr(result);
            }
            std::cout << std::endl;
        }
    }
}


TernarySimulator::TernarySimulator(shared_ptr<CircuitGraph> circuitGraph, Log &log)
    : m_log(log),
      m_circuitGraph(circuitGraph),
      m_step(0),
      m_cycleStart(-1),
      m_randomSeed(42) {
    // initialize step 0
    initStepValues();
}

void TernarySimulator::initStepValues() {
    m_values.emplace_back(static_cast<size_t>(m_circuitGraph->numVar) + 1, t_Undef);
    if (m_circuitGraph->trueId > 0)
        m_values.back()[m_circuitGraph->trueId] = t_True;
    else
        m_values.back()[-m_circuitGraph->trueId] = t_False;
}


bool TernarySimulator::setVal(int id, tbool v, int step) {
    assert(id > 0);
    m_values[step][id] = v;
    return true;
}


tbool TernarySimulator::getVal(int id, int step) {
    int idx = abs(id);
    assert(idx <= m_circuitGraph->numVar);
    tbool v = m_values[step][idx];
    return (id > 0) ? v : !v;
}


tbool TernarySimulator::getVal(int id, const vector<tbool> &vmap) {
    int idx = abs(id);
    assert(idx < static_cast<int>(vmap.size()));
    tbool v = vmap[idx];
    return (id > 0) ? v : !v;
}


void TernarySimulator::simulateOneStep() {
    assert(m_step < m_values.size());

    vector<tbool> &vmap = m_values[m_step];

    // compute gates
    for (int i = 0; i < m_circuitGraph->modelGates.size(); i++) {
        int gid = m_circuitGraph->modelGates[i];
        CircuitGate &g = m_circuitGraph->gatesMap[gid];

        switch (g.gateType) {
        case CircuitGate::GateType::XOR:
            vmap[gid] = (getVal(g.fanins[0], vmap) ^ getVal(g.fanins[1], vmap));
            break;
        case CircuitGate::GateType::ITE:
            vmap[gid] = ite(getVal(g.fanins[0], vmap), getVal(g.fanins[1], vmap), getVal(g.fanins[2], vmap));
            break;
        case CircuitGate::GateType::AND:
            vmap[gid] = (getVal(g.fanins[0], vmap) && getVal(g.fanins[1], vmap));
            break;
        default:
            assert(false);
        }
    }
}


void TernarySimulator::reset() {
    m_values.clear();
    initStepValues();
    m_states.clear();
    m_gateStates.clear();
    m_step = 0;
    m_cycleStart = -1;
}


void TernarySimulator::simulate(int maxPreciseDepth) {
    LOG_L(m_log, 2, "Simulating circuit with max precise depth ", maxPreciseDepth);

    reset();
    // set initial values // TODO: gate reset not supported
    for (int latch_id : m_circuitGraph->modelLatches) {
        int reset = m_circuitGraph->latchResetMap[latch_id];
        if (reset == -m_circuitGraph->trueId)
            setVal(latch_id, t_False, 0);
        else if (reset == m_circuitGraph->trueId)
            setVal(latch_id, t_True, 0);
        else
            setVal(latch_id, t_Undef, 0);
    }

    while (true) {
        if (m_step > 0) {
            initStepValues();
            // set latches
            for (int latch_id : m_circuitGraph->modelLatches) {
                int next_id = m_circuitGraph->latchNextMap[latch_id];
                tbool next_val = getVal(next_id, m_step - 1);
                setVal(latch_id, next_val, m_step);
            }
        }

        if (maxPreciseDepth > 0 && m_step > maxPreciseDepth && m_step % 10 == 0) {
            abstractCurrentState(m_step);
        }

        simulateOneStep();
        LOG_L(m_log, 4, "Step ", m_step, ": ", stepValuesToString(m_step));
        m_states.emplace_back();
        pushState(m_step, m_states[m_step]);
        m_gateStates.emplace_back();
        pushGateState(m_step, m_gateStates[m_step]);

        if (m_states.back().size() == 1) {
            LOG_L(m_log, 2, "All X states, terminating simulation");
            break;
        }

        if (reachCycle()) {
            m_states.pop_back();
            m_gateStates.pop_back();
            LOG_L(m_log, 2, "Cycle detected at step: ", m_cycleStart);
            break;
        }
        m_step++;
    }
}


void TernarySimulator::simulateRandom(int maxSteps) {
    LOG_L(m_log, 2, "Simulating circuit for ", maxSteps, " steps (random inputs)");

    reset();

    std::mt19937 generator(m_randomSeed);
    m_randomSeed++;
    std::uniform_int_distribution<uint8_t> distribution(0, 1);

    // set initial values
    for (int latch_id : m_circuitGraph->modelLatches) {
        int reset = m_circuitGraph->latchResetMap[latch_id];
        if (reset == -m_circuitGraph->trueId)
            setVal(latch_id, t_False, 0);
        else if (reset == m_circuitGraph->trueId)
            setVal(latch_id, t_True, 0);
        else
            setVal(latch_id, tbool(distribution(generator)), 0);
    }

    while (m_step < maxSteps) {
        if (m_step > 0) {
            initStepValues();
            // set latches
            for (int latch_id : m_circuitGraph->modelLatches) {
                int next_id = m_circuitGraph->latchNextMap[latch_id];
                tbool next_val = getVal(next_id, m_step - 1);
                setVal(latch_id, next_val, m_step);
            }
        }

        // set random inputs
        for (int input_id : m_circuitGraph->modelInputs) {
            setVal(input_id, tbool(distribution(generator)), m_step);
        }

        simulateOneStep();
        LOG_L(m_log, 4, "Step ", m_step, ": ", stepValuesToString(m_step));
        m_step++;
    }
}


void TernarySimulator::pushState(int step, vector<int> &state) {
    vector<tbool> &vmap = m_values[step];

    for (int latch_id : m_circuitGraph->modelLatches) {
        if (vmap[latch_id] == t_True)
            state.emplace_back(latch_id);
        else if (vmap[latch_id] == t_False)
            state.emplace_back(-latch_id);
    }
    // append true to find constants
    state.emplace_back(m_circuitGraph->trueId);
}


void TernarySimulator::pushGateState(int step, vector<int> &gatestate) {
    vector<tbool> &vmap = m_values[step];

    for (int gate_id : m_circuitGraph->modelGates) {
        if (vmap[gate_id] == t_True)
            gatestate.emplace_back(gate_id);
        else if (vmap[gate_id] == t_False)
            gatestate.emplace_back(-gate_id);
    }
    // append true to find constants
    gatestate.emplace_back(m_circuitGraph->trueId);
}


bool TernarySimulator::reachCycle() {
    for (int i = 0; i < m_states.size() - 1; i++) {
        if (m_states[i] == m_states.back()) {
            m_cycleStart = i;
            return true;
        }
    }
    return false;
}


string TernarySimulator::stepValuesToString(int step) {
    vector<tbool> &vmap = m_values[step];
    stringstream ss;
    // for (int input_id : m_circuitGraph->modelInputs) {
    //     ss << toStr(vmap[input_id]);
    // }
    // ss << " | ";
    for (int latch_id : m_circuitGraph->modelLatches) {
        ss << toStr(vmap[latch_id]);
    }
    // for (int latch_id : m_circuitGraph->latches) {
    //     if (vmap[latch_id] == t_True)
    //         ss << latch_id << " ";
    //     else if (vmap[latch_id] == t_False)
    //         ss << -latch_id << " ";
    // }
    // ss << " | ";
    // for (int gate_id : m_circuitGraph->modelGates) {
    //     ss << toStr(vmap[gate_id]);
    // }
    return ss.str();
}

int TernarySimulator::abstractCurrentState(int step) {
    if (step <= 0) {
        return 0;
    }

    int count = 0;
    vector<tbool> &cur = m_values[step];
    vector<tbool> &prev = m_values[step - 1];

    for (int latch_id : m_circuitGraph->modelLatches) {
        if (cur[latch_id].raw() != prev[latch_id].raw()) {
            cur[latch_id] = t_Undef;
            ++count;
        }
    }
    return count;
}


} // namespace car
