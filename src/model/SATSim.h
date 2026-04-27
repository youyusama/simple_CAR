#ifndef SAT_SIM_H
#define SAT_SIM_H

#include "CarTypes.h"
#include "CircuitGraph.h"
#include "TernarySim.h"
#include <memory>
#include <vector>

namespace minicore {
class Solver;
}

namespace car {

class SATSimulator {
  public:
    SATSimulator(std::shared_ptr<CircuitGraph> circuitGraph,
                 const std::vector<Clause> &cnfClauses,
                 const Cube &constraints,
                 Var trueVar);

    std::vector<std::vector<Tbool>> InitSimulation(int maxSamples);

    std::vector<std::vector<Tbool>> TransitionSimulation(const std::vector<std::vector<Tbool>> &initSamples,
                                                         int maxSamples);

  private:
    void AddBaseClauses(minicore::Solver &solver) const;

    void AddResetClauses(minicore::Solver &solver) const;

    Lit ToCNFLit(Lit lit) const;

    Clause ToCNFClause(const Clause &clause) const;

    Tbool SolverLitValue(const minicore::Solver &solver, Lit lit) const;

    std::shared_ptr<CircuitGraph> m_circuitGraph;
    const std::vector<Clause> &m_cnfClauses;
    const Cube &m_constraints;
    Var m_trueVar;
    Var m_maxVar;
    std::vector<Var> m_latches;
    Cube m_latchLits;
    Cube m_negLatchLits;
    Cube m_nextCnfLits;
};

} // namespace car

#endif // SAT_SIM_H
