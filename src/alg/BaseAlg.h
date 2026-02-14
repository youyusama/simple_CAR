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
    virtual CheckResult Run() = 0;
    virtual void Witness() = 0;
    virtual std::vector<std::pair<Cube, Cube>> GetCexTrace() = 0;
    virtual ~BaseAlg() = default;
};

} // namespace car


#endif
