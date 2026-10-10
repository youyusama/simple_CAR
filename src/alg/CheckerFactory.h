#pragma once

#include "BaseAlg.h"
#include <memory>

namespace car {
class Model;
class Log;

std::unique_ptr<BaseAlg> CreateBitLevelChecker(
    const Settings &settings, Model &model, Log &log);

} // namespace car
