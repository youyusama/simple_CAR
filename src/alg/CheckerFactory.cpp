#include "CheckerFactory.h"
#include "BCAR.h"
#include "BMC.h"
#include "FCAR.h"
#include "IC3.h"
#include "KFAIR.h"
#include "KIND.h"
#include "L2S.h"
#include "RLive.h"

namespace car {

std::unique_ptr<BaseAlg> CreateBitLevelChecker(
    const Settings &settings,
    Model &model,
    Log &log) {
    switch (settings.alg) {
    case MCAlgorithm::FCAR:
        return std::make_unique<FCAR>(settings, model, log);
    case MCAlgorithm::BCAR:
        return std::make_unique<BCAR>(settings, model, log);
    case MCAlgorithm::BMC:
        return std::make_unique<BMC>(settings, model, log);
    case MCAlgorithm::KIND:
        return std::make_unique<KIND>(settings, model, log);
    case MCAlgorithm::IC3:
        return std::make_unique<IC3>(settings, model, log);
    case MCAlgorithm::L2S:
        return std::make_unique<L2S>(settings, model, log);
    case MCAlgorithm::KLIVE:
    case MCAlgorithm::FAIR:
    case MCAlgorithm::KFAIR:
        return std::make_unique<KFAIR>(settings, model, log);
    case MCAlgorithm::RLIVE:
        return std::make_unique<RLive>(settings, model, log);
    default:
        return nullptr;
    }
}

} // namespace car
