#include "SimpleCAR.h"

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
#include "WLChecker.h"
#include "model/WLModel.h"
#include "WitnessBuilder.h"
#include <filesystem>
#include <iostream>
#include <memory>

namespace car {

static bool IsBtor2Input(const Settings &settings) {
    return std::filesystem::path(settings.aigFilePath).extension() == ".btor2";
}

static std::unique_ptr<BaseAlg> CreateChecker(
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

SimpleCAR::SimpleCAR(const Settings &settings) : m_settings(settings) {}

SimpleCAR::~SimpleCAR() {
    global_log = nullptr;
}

bool SimpleCAR::LoadModel() {
    if (m_log || m_model || m_wmodel || m_checker) {
        std::cerr << "LoadModel can only be called once." << std::endl;
        return false;
    }

    m_log =
        std::make_unique<Log>(m_settings.verbosity, m_settings.detailedTimers);
    [[maybe_unused]] auto init_scope = m_log->Section("Model_Init");

    // load model
    try {
        if (IsBtor2Input(m_settings)) {
            m_wmodel = std::make_unique<WLModel>(m_settings, *m_log);
        } else {
            m_model = std::make_unique<Model>(m_settings, *m_log);
        }
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return false;
    }

    // AIG export stops after word-level lowering and does not create a checker.
    if (!m_settings.wlBitblastOutputPath.empty()) {
        try {
            m_wmodel->Build({});
            m_wmodel->WriteBitblastAig();
        } catch (const std::exception &error) {
            std::cerr << error.what() << std::endl;
            return false;
        }
        return true;
    }

    // create checker
    try {
        if (m_wmodel) {
            m_checker = std::make_unique<WLChecker>(
                m_settings, *m_wmodel, *m_log);
        } else {
            m_checker = CreateChecker(m_settings, *m_model, *m_log);
        }
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return false;
    }
    return static_cast<bool>(m_checker);
}

CheckResult SimpleCAR::Prove() {
    if (!m_checker) return CheckResult::Unknown;

    CheckResult res = m_checker->Run();

    if (!m_settings.witnessOutputDir.empty()) {
        WitnessBuilder witness_builder =
            m_wmodel
                ? WitnessBuilder(m_settings, *m_log, *m_wmodel)
                : WitnessBuilder(m_settings, *m_log, *m_model);
        if (res == CheckResult::Safe && m_checker->SupportsWitness()) {
            witness_builder.BeginWitness();
            m_model->RefineWitnessPropertyLit(witness_builder);
            m_checker->RefineWitnessPropertyLit(witness_builder);
            if (!witness_builder.WriteWitness()) {
                LOG_L(*m_log, 1, "Failed to write safe witness.");
            }
        } else if (res == CheckResult::Unsafe) {
            bool written = m_wmodel
                               ? witness_builder.WriteCounterexample(
                                     static_cast<WLChecker &>(*m_checker)
                                         .GetWitnessTrace())
                               : witness_builder.WriteCounterexample(
                                     m_checker->GetCexTrace());
            if (!written) {
                LOG_L(*m_log, 1, "Failed to write counterexample witness.");
            }
        }
    }

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
