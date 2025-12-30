#include "SimpleCAR.h"

#include "BMC.h"
#include "BackwardChecker.h"
#include "BasicIC3.h"
#include "ForwardChecker.h"
#include "Log.h"
#include "Model.h"
#include <iostream>
#include <memory>

namespace car {

static std::shared_ptr<BaseChecker> CreateChecker(
    const Settings &settings,
    const std::shared_ptr<Model> &aigerModel,
    const std::shared_ptr<Log> &log) {
    switch (settings.alg) {
    case MCAlgorithm::FCAR:
        return std::make_shared<ForwardChecker>(settings, aigerModel, log);
    case MCAlgorithm::BCAR:
        return std::make_shared<BackwardChecker>(settings, aigerModel, log);
    case MCAlgorithm::BMC:
        return std::make_shared<BMC>(settings, aigerModel, log);
    case MCAlgorithm::IC3:
        return std::make_shared<BasicIC3>(settings, aigerModel, log);
    default:
        return nullptr;
    }
}

SimpleCAR::SimpleCAR(const Settings &settings) : settings_(settings) {}

SimpleCAR::~SimpleCAR() = default;

bool SimpleCAR::LoadModel() {
    log_ = std::shared_ptr<Log>(new Log(settings_.verbosity));
    aigerModel_ = std::shared_ptr<Model>(new Model(settings_, log_));
    log_->StatInit();
    checker_ = CreateChecker(settings_, aigerModel_, log_);
    return static_cast<bool>(checker_);
}

CheckResult SimpleCAR::Prove() {
    if (!checker_) return CheckResult::Unknown;

    CheckResult res = checker_->Run();

    if (!settings_.witnessOutputDir.empty())
        checker_->Witness();

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
