#ifndef SAT_SIM_H
#define SAT_SIM_H

#include "CarTypes.h"
#include "CircuitGraph.h"
#include "TernarySim.h"
#include <memory>
#include <queue>
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
                 const Cube &initialState,
                 Var trueVar);
    ~SATSimulator();

    std::vector<std::vector<Tbool>> InitSimulation(int maxSamples);

    std::vector<std::vector<Tbool>> TransitionSimulation(const std::vector<std::vector<Tbool>> &initSamples,
                                                         int maxSamples);

    const std::vector<std::vector<Tbool>> &GetGateSamples() const;

  private:
    void AddTransClauses(minicore::Solver &solver,
                         const std::vector<Clause> &cnfClauses,
                         const Cube &constraints) const;

    void AddInitialClauses(minicore::Solver &solver,
                           const Cube &initialState) const;

    Lit ToCNFLit(Lit lit) const;

    Tbool SolverLitValue(const minicore::Solver &solver, Lit lit) const;

    void RecordGateSample(const minicore::Solver &solver);

    std::shared_ptr<CircuitGraph> m_circuitGraph;
    std::unique_ptr<minicore::Solver> m_initSolver;
    std::unique_ptr<minicore::Solver> m_transitionSolver;
    std::queue<Cube> m_transitionFrontier;
    Var m_trueVar;
    std::vector<Var> m_latches;
    std::vector<Var> m_modelGates;
    std::vector<std::vector<Tbool>> m_gateSamples;
    Cube m_latchLits;
    Cube m_negLatchLits;
    Cube m_nextCnfLits;
};

} // namespace car

#endif // SAT_SIM_H
