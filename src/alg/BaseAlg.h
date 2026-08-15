#ifndef BASEALG_H
#define BASEALG_H

#include "Model.h"
#include "Settings.h"
#include <utility>
#include <vector>

namespace car {

class WitnessBuilder;

enum class CheckResult { Safe,
                         Unsafe,
                         Unknown };

class BaseAlg {
  public:
    virtual CheckResult Run() = 0;
    virtual std::vector<std::pair<Cube, Cube>> GetCexTrace() = 0;
    virtual bool SupportsWitness() const { return false; }
    virtual void RefineWitnessPropertyLit(WitnessBuilder &) const {}
    virtual ~BaseAlg() = default;
};

} // namespace car


#endif
