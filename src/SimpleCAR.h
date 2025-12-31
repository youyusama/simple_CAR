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
    Settings m_settings;
    std::unique_ptr<Log> m_log;
    std::shared_ptr<Model> m_model;
    std::shared_ptr<BaseChecker> m_checker;
};

} // namespace car

#endif
