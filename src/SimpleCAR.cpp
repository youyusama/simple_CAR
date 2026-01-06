#include "SimpleCAR.h"

#include "BMC.h"
#include "BCAR.h"
#include "BasicIC3.h"
#include "FCAR.h"
#include "Log.h"
#include "Model.h"
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
        return std::make_unique<BasicIC3>(settings, aigerModel, log);
    default:
        return nullptr;
    }
}

SimpleCAR::SimpleCAR(const Settings &settings) : m_settings(settings) {}

SimpleCAR::~SimpleCAR() {
    GLOBAL_LOG = nullptr;
}

bool SimpleCAR::LoadModel() {
    m_log = std::make_unique<Log>(m_settings.verbosity);
    m_model = std::make_unique<Model>(m_settings, *m_log);
    m_log->StatInit();
    m_checker = CreateChecker(m_settings, *m_model, *m_log);
    return static_cast<bool>(m_checker);
}

CheckResult SimpleCAR::Prove() {
    if (!m_checker) return CheckResult::Unknown;

    CheckResult res = m_checker->Run();

    if (!m_settings.witnessOutputDir.empty())
        m_checker->Witness();

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
