#ifndef SIMPLECAR_H
#define SIMPLECAR_H

#include "BaseAlg.h"
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
    std::unique_ptr<Model> m_model;
    std::unique_ptr<BaseAlg> m_checker;
};

} // namespace car

#endif
