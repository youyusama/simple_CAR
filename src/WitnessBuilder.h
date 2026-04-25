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

class WitnessBuilder {
  public:
    WitnessBuilder(const Settings &settings, Log &log, const aiger *model_aig);

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

    static std::shared_ptr<aiger> CloneBaseAig(const aiger *src);

  private:
    std::string GetWitnessPath(const std::string &suffix) const;
    bool WriteAigWitness(const aiger *model_aig, unsigned invariant_lit);
    std::string CubeToInputString(const Cube &cube) const;
    std::string CubeToLatchString(const Cube &cube) const;

    const Settings &m_settings;
    Log &m_log;
    const aiger *m_modelAig{nullptr};
    std::shared_ptr<aiger> m_witnessAigPtr;
    aiger *m_witnessAig{nullptr};
    unsigned m_propertyLit{0};
    int m_numInputs{0};
    int m_numLatches{0};
};

} // namespace car

#endif
