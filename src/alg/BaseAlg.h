#ifndef BASEALG_H
#define BASEALG_H

#include "Model.h"
#include "Settings.h"
#include <utility>
#include <vector>

namespace car {

enum class CheckResult { Safe,
                         Unsafe,
                         Unknown };

class BaseAlg {
  public:
    using Trace = std::vector<std::pair<std::vector<int>, std::vector<int>>>;
    virtual CheckResult Run() = 0;
    virtual void Witness() = 0;
    virtual Trace CounterexampleTrace() { return {}; }
    virtual ~BaseAlg() = default;
};

} // namespace car


#endif
