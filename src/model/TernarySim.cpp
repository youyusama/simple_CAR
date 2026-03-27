#include "TernarySim.h"

namespace car {

void TestTruthTables() {
    std::cout << "--- lbool Truth Tables ---" << std::endl;
    std::cout << "T = l_True, F = l_False, U = l_Undef, E = Error/Invalid (value=3)" << std::endl;

    std::vector<Tbool> states = {T_TRUE, T_FALSE, T_UNDEF};

    // unary operators
    std::cout << "\n--- Operator:!a ---" << std::endl;
    for (Tbool a : states) {
        std::cout << "!" << ToStr(a) << " = " << ToStr(!a) << std::endl;
    }

    // binary operators
    const char *ops[] = {"&&", "||", "^", "=="};
    for (const char *op : ops) {
        std::cout << "\n--- Operator: a " << op << " b ---" << std::endl;
        std::cout << "a \\ b |  T    F    U" << std::endl;
        std::cout << "------+------------" << std::endl;
        for (Tbool a : states) {
            std::cout << "  " << ToStr(a) << "   | ";
            for (Tbool b : states) {
                Tbool result(false);
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
                std::cout << std::left << std::setw(5) << ToStr(result);
            }
            std::cout << std::endl;
        }
    }

    // ternary operator
    std::cout << "\n--- Operator: Ite(c, a, b) ---" << std::endl;
    for (Tbool c : states) {
        std::cout << "\n  When c = " << ToStr(c) << ":" << std::endl;
        std::cout << "  a \\ b |  T    F    U" << std::endl;
        std::cout << "  ------+------------" << std::endl;
        for (Tbool a : states) {
            std::cout << "    " << ToStr(a) << "   | ";
            for (Tbool b : states) {
                Tbool result = Ite(c, a, b);
                std::cout << std::left << std::setw(5) << ToStr(result);
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
    InitStepValues();
}

void TernarySimulator::InitStepValues() {
    m_values.emplace_back(static_cast<size_t>(m_circuitGraph->numVar) + 1, T_UNDEF);
    m_values.back()[0] = T_FALSE;
}


bool TernarySimulator::SetVal(Var id, Tbool v, int step) {
    m_values[step][id] = v;
    return true;
}


Tbool TernarySimulator::GetVal(Lit id, int step) {
    Tbool v = m_values[step][VarOf(id)];
    return Sign(id) ? !v : v;
}


Tbool TernarySimulator::GetVal(Lit id, const vector<Tbool> &vmap) {
    Tbool v = vmap[VarOf(id)];
    return Sign(id) ? !v : v;
}


void TernarySimulator::SimulateOneStep() {
    assert(m_step < m_values.size());

    vector<Tbool> &vmap = m_values[m_step];

    // compute gates
    for (size_t i = 0; i < m_circuitGraph->modelGates.size(); i++) {
        Var gid = m_circuitGraph->modelGates[i];
        CircuitGate &g = m_circuitGraph->gatesMap[gid];

        switch (g.gateType) {
        case CircuitGate::GateType::XOR:
            vmap[gid] = (GetVal(g.fanins[0], vmap) ^ GetVal(g.fanins[1], vmap));
            break;
        case CircuitGate::GateType::ITE:
            vmap[gid] = Ite(GetVal(g.fanins[0], vmap), GetVal(g.fanins[1], vmap), GetVal(g.fanins[2], vmap));
            break;
        case CircuitGate::GateType::AND:
            vmap[gid] = (GetVal(g.fanins[0], vmap) && GetVal(g.fanins[1], vmap));
            break;
        default:
            assert(false);
        }
    }
}


void TernarySimulator::Reset() {
    m_values.clear();
    InitStepValues();
    m_states.clear();
    m_gateStates.clear();
    m_step = 0;
    m_cycleStart = -1;
}


void TernarySimulator::Simulate(int maxPreciseDepth) {
    LOG_L(m_log, 2, "Simulating circuit with max precise depth ", maxPreciseDepth);

    Reset();
    // set initial values // TODO: gate reset not supported
    for (Var latch_id : m_circuitGraph->modelLatches) {
        Lit reset = m_circuitGraph->latchResetMap[latch_id];
        if (reset == LIT_FALSE)
            SetVal(latch_id, T_FALSE, 0);
        else if (reset == LIT_TRUE)
            SetVal(latch_id, T_TRUE, 0);
        else
            SetVal(latch_id, T_UNDEF, 0);
    }

    while (true) {
        if (m_step > 0) {
            InitStepValues();
            // set latches
            for (Var latch_id : m_circuitGraph->modelLatches) {
                Lit next_id = m_circuitGraph->latchNextMap[latch_id];
                Tbool next_val = GetVal(next_id, m_step - 1);
                SetVal(latch_id, next_val, m_step);
            }
        }

        if (maxPreciseDepth > 0 && m_step > maxPreciseDepth && m_step % 10 == 0) {
            AbstractCurrentState(m_step);
        }

        SimulateOneStep();
        LOG_L(m_log, 4, "Step ", m_step, ": ", StepValuesToString(m_step));
        m_states.emplace_back();
        PushState(m_step, m_states[m_step]);
        m_gateStates.emplace_back();
        PushGateState(m_step, m_gateStates[m_step]);

        if (m_states.back().size() == 1) {
            LOG_L(m_log, 2, "All X states, terminating simulation");
            break;
        }

        if (ReachCycle()) {
            m_states.pop_back();
            m_gateStates.pop_back();
            LOG_L(m_log, 2, "Cycle detected at step: ", m_cycleStart);
            break;
        }
        m_step++;
    }
}


void TernarySimulator::SimulateRandom(int maxSteps) {
    LOG_L(m_log, 2, "Simulating circuit for ", maxSteps, " steps (random inputs)");

    Reset();

    std::mt19937 generator(m_randomSeed);
    m_randomSeed++;
    std::uniform_int_distribution<uint8_t> distribution(0, 1);

    // set initial values
    for (Var latch_id : m_circuitGraph->modelLatches) {
        Lit reset = m_circuitGraph->latchResetMap[latch_id];
        if (reset == LIT_FALSE)
            SetVal(latch_id, T_FALSE, 0);
        else if (reset == LIT_TRUE)
            SetVal(latch_id, T_TRUE, 0);
        else
            SetVal(latch_id, Tbool(distribution(generator)), 0);
    }

    while (m_step < maxSteps) {
        if (m_step > 0) {
            InitStepValues();
            // set latches
            for (Var latch_id : m_circuitGraph->modelLatches) {
                Lit next_id = m_circuitGraph->latchNextMap[latch_id];
                Tbool next_val = GetVal(next_id, m_step - 1);
                SetVal(latch_id, next_val, m_step);
            }
        }

        // set random inputs
        for (Var input_id : m_circuitGraph->modelInputs) {
            SetVal(input_id, Tbool(distribution(generator)), m_step);
        }

        SimulateOneStep();
        LOG_L(m_log, 4, "Step ", m_step, ": ", StepValuesToString(m_step));
        m_step++;
    }
}


void TernarySimulator::PushState(int step, Cube &state) {
    vector<Tbool> &vmap = m_values[step];

    for (Var latch_id : m_circuitGraph->modelLatches) {
        if (vmap[latch_id] == T_TRUE)
            state.emplace_back(MkLit(latch_id));
        else if (vmap[latch_id] == T_FALSE)
            state.emplace_back(~MkLit(latch_id));
    }
    // append true to find constants
    state.emplace_back(LIT_TRUE);
}


void TernarySimulator::PushGateState(int step, Cube &gatestate) {
    vector<Tbool> &vmap = m_values[step];

    for (Var gate_id : m_circuitGraph->modelGates) {
        if (vmap[gate_id] == T_TRUE)
            gatestate.emplace_back(MkLit(gate_id));
        else if (vmap[gate_id] == T_FALSE)
            gatestate.emplace_back(~MkLit(gate_id));
    }
    // append true to find constants
    gatestate.emplace_back(LIT_TRUE);
}


bool TernarySimulator::ReachCycle() {
    for (int i = 0; i < m_states.size() - 1; i++) {
        if (m_states[i] == m_states.back()) {
            m_cycleStart = i;
            return true;
        }
    }
    return false;
}


string TernarySimulator::StepValuesToString(int step) {
    vector<Tbool> &vmap = m_values[step];
    stringstream ss;
    // for (int input_id : m_circuitGraph->modelInputs) {
    //     ss << ToStr(vmap[input_id]);
    // }
    // ss << " | ";
    for (Var latch_id : m_circuitGraph->modelLatches) {
        ss << ToStr(vmap[latch_id]);
    }
    // for (int latch_id : m_circuitGraph->latches) {
    //     if (vmap[latch_id] == T_TRUE)
    //         ss << latch_id << " ";
    //     else if (vmap[latch_id] == T_FALSE)
    //         ss << -latch_id << " ";
    // }
    // ss << " | ";
    // for (int gate_id : m_circuitGraph->modelGates) {
    //     ss << ToStr(vmap[gate_id]);
    // }
    return ss.str();
}

int TernarySimulator::AbstractCurrentState(int step) {
    if (step <= 0) {
        return 0;
    }

    int count = 0;
    vector<Tbool> &cur = m_values[step];
    vector<Tbool> &prev = m_values[step - 1];

    for (Var latch_id : m_circuitGraph->modelLatches) {
        if (cur[latch_id].Raw() != prev[latch_id].Raw()) {
            cur[latch_id] = T_UNDEF;
            ++count;
        }
    }
    return count;
}


} // namespace car
