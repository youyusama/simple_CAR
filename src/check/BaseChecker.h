#ifndef BASECHECKER_H
#define BASECHECKER_H

#include "Model.h"
#include "Settings.h"

namespace car {

enum class CheckResult { Safe,
                         Unsafe,
                         Unknown };

class BaseChecker {
  public:
    virtual CheckResult Run() = 0;
    virtual void Witness() = 0;
};

} // namespace car


#endif
