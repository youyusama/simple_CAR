#ifndef WITNESS_BUILDER_H
#define WITNESS_BUILDER_H

#include "CarTypes.h"
#include "Settings.h"

extern "C" {
#include "aiger.h"
}

#include <memory>
#include <string>
#include <vector>

namespace car {

class Log;
class Model;
class WLModel;
struct WLTraceMap;

struct EquivalenceWitness {
    std::vector<Clause> equivalence_clauses;
    std::vector<Cube> reached_state_cubes;
    bool has_reached_state_region{false};
};

class WitnessBuilder {
  public:
    WitnessBuilder(const Settings &settings,
                   Log &log,
                   const Model &model);
    WitnessBuilder(const Settings &settings,
                   Log &log,
                   const WLModel &model);

    void BeginWitness();

    bool WriteWitness();
    bool WriteCounterexample(const std::vector<std::pair<Cube, Cube>> &trace);

    unsigned GetPropertyLit() const { return m_propertyLit; }
    void SetPropertyLit(unsigned lit) { m_propertyLit = lit; }

    unsigned TrueLit() const { return ToAigerLit(LIT_TRUE); }
    unsigned FalseLit() const { return ToAigerLit(LIT_FALSE); }
    unsigned Negate(unsigned lit) const { return lit ^ 1U; }

    unsigned BuildCube(const Cube &cube);
    unsigned BuildClause(const Clause &clause);
    unsigned BuildAnd(const std::vector<unsigned> &lits);
    unsigned BuildOr(const std::vector<unsigned> &lits);

    void RegisterEquivalenceWitness(const EquivalenceWitness &witness);
    void BuildKInductionWitness(int safeK);

    static std::shared_ptr<aiger> CloneBaseAig(const aiger *src);

  private:
    std::string GetWitnessPath(const std::string &suffix) const;
    std::string GetBtor2CounterexamplePath() const;
    bool WriteAigWitness(const aiger *model_aig, unsigned invariant_lit);
    bool WriteAigerCounterexample(
        const std::vector<std::pair<Cube, Cube>> &trace);
    bool WriteBtor2Counterexample(
        const std::vector<std::pair<Cube, Cube>> &trace);
    std::string CubeToInputString(const Cube &cube) const;
    std::string CubeToLatchString(const Cube &cube) const;
    bool IsBtor2Input() const;

    const Settings &m_settings;
    Log &m_log;
    const Model *m_model{nullptr};
    const WLTraceMap *m_wlTraceMap{nullptr};
    const aiger *m_modelAig{nullptr};
    std::shared_ptr<aiger> m_witnessAigPtr;
    aiger *m_witnessAig{nullptr};
    unsigned m_propertyLit{0};
    int m_numInputs{0};
    int m_numLatches{0};
    EquivalenceWitness m_equivalenceWitness;
    bool m_hasEquivalenceWitness{false};
};

} // namespace car

#endif
