#include "WLChecker.h"

#include "BCAR.h"
#include "BMC.h"
#include "FCAR.h"
#include "IC3.h"
#include "KFAIR.h"
#include "KIND.h"
#include "L2S.h"
#include "Log.h"
#include "Model.h"
#include "RLive.h"
#include "WLCegar.h"
#include "WLModel.h"

#include <stdexcept>

namespace car {

WLChecker::WLChecker(const Settings &settings,
                     WLModel &model,
                     Log &log)
    : m_settings(settings),
      m_log(log),
      m_model(model) {
    // Every BTOR2 model ultimately delegates proof search to a bit-level checker.
    auto checker = CreateBitLevelChecker(m_model.BitModel(), m_log);
    if (!checker) {
        throw std::runtime_error("word-level checker requires a bit-level checker.");
    }

    if (m_model.SourceHasArrays()) {
        // Array inputs wrap the selected checker in simulator-based CEGAR.
        WLCegar::CheckerFactory checkerFactory =
            [this](Model &nextModel, Log &nextLog) {
                return CreateBitLevelChecker(nextModel, nextLog);
            };
        m_cegar = std::make_unique<WLCegar>(
            m_settings,
            m_log,
            m_model,
            std::move(checker),
            std::move(checkerFactory));
    } else {
        // Array-free BTOR2 models need only the standard checker wrapper.
        m_checker = std::move(checker);
    }
}

WLChecker::~WLChecker() = default;

std::unique_ptr<BaseAlg> WLChecker::CreateBitLevelChecker(Model &model,
                                                          Log &log) {
    // Mirror the user-selected algorithm while hiding it behind WLChecker.
    switch (m_settings.alg) {
    case MCAlgorithm::FCAR:
        return std::make_unique<FCAR>(m_settings, model, log);
    case MCAlgorithm::BCAR:
        return std::make_unique<BCAR>(m_settings, model, log);
    case MCAlgorithm::BMC:
        return std::make_unique<BMC>(m_settings, model, log);
    case MCAlgorithm::KIND:
        return std::make_unique<KIND>(m_settings, model, log);
    case MCAlgorithm::IC3:
        return std::make_unique<IC3>(m_settings, model, log);
    case MCAlgorithm::L2S:
        return std::make_unique<L2S>(m_settings, model, log);
    case MCAlgorithm::KLIVE:
    case MCAlgorithm::FAIR:
    case MCAlgorithm::KFAIR:
        return std::make_unique<KFAIR>(m_settings, model, log);
    case MCAlgorithm::RLIVE:
        return std::make_unique<RLive>(m_settings, model, log);
    default:
        return nullptr;
    }
}

CheckResult WLChecker::Run() {
    if (m_cegar) return m_cegar->Run();
    return m_checker->Run();
}

std::vector<std::pair<Cube, Cube>> WLChecker::GetCexTrace() {
    if (m_cegar) return m_cegar->GetCexTrace();
    return m_checker->GetCexTrace();
}

int WLChecker::GetSafeDepth() const {
    if (m_cegar) return m_cegar->GetSafeDepth();
    return m_checker->GetSafeDepth();
}

} // namespace car
