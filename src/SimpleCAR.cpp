#include "SimpleCAR.h"
#include "CheckerFactory.h"

#include "Log.h"
#include "Model.h"
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
            m_wmodel->WriteScalarAig(m_settings.wlBitblastOutputPath,
                                    !m_settings.wlDisablePackageResize);
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
            m_checker = CreateBitLevelChecker(m_settings, *m_model, *m_log);
        }
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return false;
    }
    return static_cast<bool>(m_checker);
}

CheckResult SimpleCAR::Prove() {
    if (!m_checker) return CheckResult::Unknown;

    // Cover word-level preprocessing, CEGAR replay, and native memory BMC.
    global_log = m_log.get();
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

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
                                         .GetTrace())
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
