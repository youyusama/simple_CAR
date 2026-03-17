#include "SimpleCAR.h"

#include "BCAR.h"
#include "BMC.h"
#include "BasicIC3.h"
#include "FCAR.h"
#include "KFAIR.h"
#include "L2S.h"
#include "Log.h"
#include "Model.h"
#include "RLive.h"
#include <iostream>
#include <memory>

namespace car {

static std::unique_ptr<BaseAlg> CreateChecker(
    const Settings &settings,
    Model &aigerModel,
    Log &log) {
    switch (settings.alg) {
    case MCAlgorithm::FCAR:
        return std::make_unique<FCAR>(settings, aigerModel, log);
    case MCAlgorithm::BCAR:
        return std::make_unique<BCAR>(settings, aigerModel, log);
    case MCAlgorithm::BMC:
        return std::make_unique<BMC>(settings, aigerModel, log);
    case MCAlgorithm::IC3:
        return std::make_unique<IC3>(settings, aigerModel, log);
    case MCAlgorithm::L2S:
        return std::make_unique<L2S>(settings, aigerModel, log);
    case MCAlgorithm::KLIVE:
    case MCAlgorithm::FAIR:
    case MCAlgorithm::KFAIR:
        return std::make_unique<KFAIR>(settings, aigerModel, log);
    case MCAlgorithm::RLIVE:
        return std::make_unique<RLive>(settings, aigerModel, log);
    default:
        return nullptr;
    }
}

SimpleCAR::SimpleCAR(const Settings &settings) : m_settings(settings) {}

SimpleCAR::~SimpleCAR() {
    global_log = nullptr;
}

bool SimpleCAR::LoadModel() {
    m_log = std::make_unique<Log>(m_settings.verbosity, m_settings.detailedTimers);
    [[maybe_unused]] auto init_scope = m_log->Section("Model_Init");
    m_model = std::make_unique<Model>(m_settings, *m_log);
    m_checker = CreateChecker(m_settings, *m_model, *m_log);
    return static_cast<bool>(m_checker);
}

CheckResult SimpleCAR::Prove() {
    if (!m_checker) return CheckResult::Unknown;

    CheckResult res = m_checker->Run();

    if (!m_settings.witnessOutputDir.empty())
        m_checker->Witness();

    m_log->PrintTotalTime();
    switch (res) {
    case CheckResult::Safe:
        std::cout << "Safe" << std::endl;
        break;
    case CheckResult::Unsafe:
        std::cout << "Unsafe" << std::endl;
        break;
    case CheckResult::Unknown:
        std::cout << "Unknown" << std::endl;
        break;
    default:
        break;
    }
    return res;
}

} // namespace car
