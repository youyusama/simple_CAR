#include "SATSim.h"

#include "minicore/src/solver.h"
#include <algorithm>
#include <queue>
#include <utility>

namespace car {

SATSimulator::SATSimulator(std::shared_ptr<CircuitGraph> circuitGraph,
                           const std::vector<Clause> &cnfClauses,
                           const Cube &constraints,
                           Var trueVar)
    : m_circuitGraph(std::move(circuitGraph)),
      m_cnfClauses(cnfClauses),
      m_constraints(constraints),
      m_trueVar(trueVar),
      m_maxVar(std::max(trueVar, m_circuitGraph->numVar)) {
    m_latches = m_circuitGraph->modelLatches;
    m_latchLits.reserve(m_latches.size());
    m_negLatchLits.reserve(m_latches.size());
    m_nextCnfLits.reserve(m_latches.size());
    for (Var latch : m_latches) {
        Lit latch_lit = MkLit(latch);
        m_latchLits.emplace_back(latch_lit);
        m_negLatchLits.emplace_back(~latch_lit);
        m_nextCnfLits.emplace_back(ToCNFLit(m_circuitGraph->latchNextMap[latch]));
    }
}


Lit SATSimulator::ToCNFLit(Lit lit) const {
    if (!IsConst(lit)) return lit;
    return IsConstTrue(lit) ? MkLit(m_trueVar) : ~MkLit(m_trueVar);
}


Clause SATSimulator::ToCNFClause(const Clause &clause) const {
    Clause out;
    out.reserve(clause.size());
    for (Lit lit : clause) out.emplace_back(ToCNFLit(lit));
    return out;
}


void SATSimulator::AddBaseClauses(minicore::Solver &solver) const {
    solver.setRestartLimit(10);
    while (static_cast<int>(m_maxVar) >= solver.nVars()) solver.newVar();
    for (const Clause &c : m_cnfClauses) solver.addClause(c);
    for (Lit c : m_constraints) solver.addClause(Clause{c});
}


void SATSimulator::AddResetClauses(minicore::Solver &solver) const {
    for (size_t i = 0; i < m_latches.size(); ++i) {
        Var latch = m_latches[i];
        Lit latch_lit = m_latchLits[i];
        Lit reset = m_circuitGraph->latchResetMap[latch];
        if (reset == latch_lit) continue;
        if (reset == LIT_FALSE) {
            solver.addClause(Clause{~latch_lit});
        } else if (reset == LIT_TRUE) {
            solver.addClause(Clause{latch_lit});
        } else {
            solver.addClause(ToCNFClause(Clause{latch_lit, ~reset}));
            solver.addClause(ToCNFClause(Clause{~latch_lit, reset}));
        }
    }
}


Tbool SATSimulator::SolverLitValue(const minicore::Solver &solver, Lit lit) const {
    minicore::lbool value = solver.value(VarOf(lit));
    assert(value == minicore::l_True || value == minicore::l_False);

    bool bit = (value == minicore::l_True);
    if (Sign(lit)) bit = !bit;
    return Tbool(bit);
}


std::vector<std::vector<Tbool>> SATSimulator::InitSimulation(int maxSamples) {
    std::vector<std::vector<Tbool>> samples;
    if (maxSamples <= 0) return samples;

    minicore::Solver solver;
    AddBaseClauses(solver);
    AddResetClauses(solver);

    while (static_cast<int>(samples.size()) < maxSamples) {
        minicore::lbool res = solver.solve();
        if (res != minicore::l_True) break;

        std::vector<Tbool> state(m_maxVar + 1, T_UNDEF);
        state[0] = T_FALSE;
        if (m_trueVar < state.size()) state[m_trueVar] = T_TRUE;

        Clause block;
        block.reserve(m_latches.size());
        for (size_t i = 0; i < m_latches.size(); ++i) {
            Tbool value = SolverLitValue(solver, m_latchLits[i]);
            state[m_latches[i]] = value;
            if (value == T_TRUE)
                block.emplace_back(m_negLatchLits[i]);
            else if (value == T_FALSE)
                block.emplace_back(m_latchLits[i]);
        }

        samples.emplace_back(std::move(state));
        solver.addClause(block);
    }

    return samples;
}


std::vector<std::vector<Tbool>> SATSimulator::TransitionSimulation(const std::vector<std::vector<Tbool>> &initSamples,
                                                                   int maxSamples) {
    std::vector<std::vector<Tbool>> samples;
    if (maxSamples <= 0 || initSamples.empty()) return samples;

    minicore::Solver solver;
    AddBaseClauses(solver);

    std::queue<Cube> frontier;
    for (const auto &state : initSamples) {
        Cube assumptions;
        assumptions.reserve(m_latches.size());
        Clause init_next_block;
        init_next_block.reserve(m_latches.size());
        for (size_t i = 0; i < m_latches.size(); ++i) {
            Tbool value = state[m_latches[i]];
            assert(value == T_TRUE || value == T_FALSE);
            if (value == T_TRUE) {
                assumptions.emplace_back(m_latchLits[i]);
                init_next_block.emplace_back(~m_nextCnfLits[i]);
            } else {
                assumptions.emplace_back(m_negLatchLits[i]);
                init_next_block.emplace_back(m_nextCnfLits[i]);
            }
        }
        solver.addClause(init_next_block);
        frontier.emplace(std::move(assumptions));
    }

    constexpr int SUCCESSORS_PER_STATE = 4;
    while (!frontier.empty() && static_cast<int>(samples.size()) < maxSamples) {
        Cube assumptions = std::move(frontier.front());
        frontier.pop();

        for (int i = 0; i < SUCCESSORS_PER_STATE && static_cast<int>(samples.size()) < maxSamples; ++i) {
            minicore::lbool res = solver.solve(assumptions);
            if (res != minicore::l_True) break;

            std::vector<Tbool> next_state(m_maxVar + 1, T_UNDEF);
            next_state[0] = T_FALSE;
            if (m_trueVar < next_state.size()) next_state[m_trueVar] = T_TRUE;

            Clause block_next;
            block_next.reserve(m_latches.size());
            Cube next_assumptions;
            next_assumptions.reserve(m_latches.size());
            for (size_t j = 0; j < m_latches.size(); ++j) {
                Lit next_cnf_lit = m_nextCnfLits[j];
                Tbool value = SolverLitValue(solver, next_cnf_lit);
                assert(value == T_TRUE || value == T_FALSE);
                next_state[m_latches[j]] = value;
                if (value == T_TRUE) {
                    block_next.emplace_back(~next_cnf_lit);
                    next_assumptions.emplace_back(m_latchLits[j]);
                } else {
                    block_next.emplace_back(next_cnf_lit);
                    next_assumptions.emplace_back(m_negLatchLits[j]);
                }
            }

            samples.emplace_back(std::move(next_state));
            solver.addClause(block_next);
            frontier.emplace(std::move(next_assumptions));
        }
    }

    return samples;
}

} // namespace car
