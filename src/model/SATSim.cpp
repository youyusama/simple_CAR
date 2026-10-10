#include "SATSim.h"

#include "minicore/src/solver.h"
#include <utility>

namespace car {

SATSimulator::SATSimulator(std::shared_ptr<CircuitGraph> circuitGraph,
                           const std::vector<Clause> &cnfClauses,
                           const Cube &constraints,
                           const Cube &initialState,
                           Var trueVar)
    : m_circuitGraph(std::move(circuitGraph)),
      m_initSolver(std::make_unique<minicore::Solver>()),
      m_transitionSolver(std::make_unique<minicore::Solver>()),
      m_trueVar(trueVar) {
    m_latches = m_circuitGraph->modelLatches;
    m_modelGates = m_circuitGraph->modelGates;
    m_latchLits.reserve(m_latches.size());
    m_negLatchLits.reserve(m_latches.size());
    m_nextCnfLits.reserve(m_latches.size());
    for (Var latch : m_latches) {
        Lit latch_lit = MkLit(latch);
        m_latchLits.emplace_back(latch_lit);
        m_negLatchLits.emplace_back(~latch_lit);
        m_nextCnfLits.emplace_back(ToCNFLit(m_circuitGraph->latchNextMap[latch]));
    }

    AddTransClauses(*m_initSolver, cnfClauses, constraints);
    AddInitialClauses(*m_initSolver, initialState);
    AddTransClauses(*m_transitionSolver, cnfClauses, constraints);
}


SATSimulator::~SATSimulator() = default;


const std::vector<std::vector<Tbool>> &SATSimulator::GetGateSamples() const {
    return m_gateSamples;
}


Lit SATSimulator::ToCNFLit(Lit lit) const {
    if (!IsConst(lit)) return lit;
    return IsConstTrue(lit) ? MkLit(m_trueVar) : ~MkLit(m_trueVar);
}


void SATSimulator::AddTransClauses(minicore::Solver &solver,
                                   const std::vector<Clause> &cnfClauses,
                                   const Cube &constraints) const {
    solver.setRestartLimit(1);
    solver.newVarUntil(static_cast<minicore::Var>(m_trueVar));
    for (const Clause &c : cnfClauses) solver.addClause(c);
    for (Lit c : constraints) solver.addClause(Clause{c});
}


void SATSimulator::AddInitialClauses(minicore::Solver &solver,
                                     const Cube &initialState) const {
    for (Lit lit : initialState) solver.addClause(Clause{lit});
}


Tbool SATSimulator::SolverLitValue(const minicore::Solver &solver, Lit lit) const {
    minicore::lbool value = solver.value(VarOf(lit));
    assert(value == minicore::l_True || value == minicore::l_False);

    bool bit = (value == minicore::l_True);
    if (Sign(lit)) bit = !bit;
    return Tbool(bit);
}


void SATSimulator::RecordGateSample(const minicore::Solver &solver) {
    std::vector<Tbool> values(m_trueVar + 1, T_UNDEF);
    values[0] = T_FALSE;
    values[m_trueVar] = T_TRUE;
    for (Var gate : m_modelGates) {
        values[gate] = SolverLitValue(solver, MkLit(gate));
    }
    m_gateSamples.emplace_back(std::move(values));
}


std::vector<std::vector<Tbool>> SATSimulator::InitSimulation(int maxSamples) {
    std::vector<std::vector<Tbool>> samples;
    if (maxSamples <= 0) return samples;

    minicore::Solver &solver = *m_initSolver;
    constexpr int TRANSITION_ROOT_LIMIT = 10;

    while (static_cast<int>(samples.size()) < maxSamples) {
        minicore::lbool res = solver.solve();
        if (res != minicore::l_True) break;
        RecordGateSample(solver);
        bool enqueue_transition_root = static_cast<int>(samples.size()) < TRANSITION_ROOT_LIMIT;

        std::vector<Tbool> state(m_trueVar + 1, T_UNDEF);
        state[0] = T_FALSE;
        state[m_trueVar] = T_TRUE;

        Clause block;
        block.reserve(m_latches.size());
        Cube assumptions;
        assumptions.reserve(m_latches.size());
        Clause init_next_block;
        init_next_block.reserve(m_latches.size());
        for (size_t i = 0; i < m_latches.size(); ++i) {
            Tbool value = SolverLitValue(solver, m_latchLits[i]);
            state[m_latches[i]] = value;
            if (value == T_TRUE) {
                block.emplace_back(m_negLatchLits[i]);
                assumptions.emplace_back(m_latchLits[i]);
                init_next_block.emplace_back(~m_nextCnfLits[i]);
            } else if (value == T_FALSE) {
                block.emplace_back(m_latchLits[i]);
                assumptions.emplace_back(m_negLatchLits[i]);
                init_next_block.emplace_back(m_nextCnfLits[i]);
            }
        }

        samples.emplace_back(std::move(state));
        solver.addClause(block);
        m_transitionSolver->addClause(init_next_block);
        if (enqueue_transition_root) m_transitionFrontier.emplace(std::move(assumptions));
    }

    return samples;
}


std::vector<std::vector<Tbool>> SATSimulator::TransitionSimulation(const std::vector<std::vector<Tbool>> &initSamples,
                                                                   int maxSamples) {
    std::vector<std::vector<Tbool>> samples;
    if (maxSamples <= 0 || initSamples.empty() || m_transitionFrontier.empty()) return samples;

    minicore::Solver &solver = *m_transitionSolver;

    constexpr int SUCCESSORS_PER_STATE = 1;
    while (!m_transitionFrontier.empty() && static_cast<int>(samples.size()) < maxSamples) {
        Cube assumptions = std::move(m_transitionFrontier.front());
        m_transitionFrontier.pop();

        for (int i = 0; i < SUCCESSORS_PER_STATE && static_cast<int>(samples.size()) < maxSamples; ++i) {
            minicore::lbool res = solver.solve(assumptions);
            if (res != minicore::l_True) break;
            RecordGateSample(solver);

            std::vector<Tbool> next_state(m_trueVar + 1, T_UNDEF);
            next_state[0] = T_FALSE;
            next_state[m_trueVar] = T_TRUE;

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
            m_transitionFrontier.emplace(std::move(next_assumptions));
        }
    }

    return samples;
}

} // namespace car
