#ifndef SIMPLECAR_H
#define SIMPLECAR_H

#include "BaseChecker.h"
#include "Settings.h"
#include <memory>

namespace car {

class Log;
class Model;

class SimpleCAR {
  public:
    explicit SimpleCAR(const Settings &settings);
    ~SimpleCAR();

    bool LoadModel();
    CheckResult Prove();

  private:
    Settings settings_;
    std::shared_ptr<Log> log_;
    std::shared_ptr<Model> aigerModel_;
    std::shared_ptr<BaseChecker> checker_;
};

} // namespace car

#endif
