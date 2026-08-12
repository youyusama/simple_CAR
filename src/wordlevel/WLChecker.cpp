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
#include "WLMemoryBMC.h"
#include "model/WLModel.h"

#include <stdexcept>

namespace car {
namespace {

WLWitnessTrace ToSparseWitnessTrace(WLReplayTrace replay) {
    WLWitnessTrace witness;
    witness.steps.resize(replay.steps.size());
    for (size_t time = 0; time < replay.steps.size(); ++time) {
        witness.steps[time].inputValues =
            std::move(replay.steps[time].inputValues);
        witness.steps[time].stateValues =
            std::move(replay.steps[time].stateValues);
    }
    return witness;
}

} // namespace

WLChecker::WLChecker(const Settings &settings,
                     WLModel &model,
                     Log &log)
    : m_settings(settings),
      m_log(log),
      m_model(model) {
    if (m_settings.alg == MCAlgorithm::WLBMC) {
        // Native memory BMC works directly on the source IR.
        m_memoryBmc =
            std::make_unique<WLMemoryBMC>(m_settings, m_model, m_log);
        return;
    }

    // Ordinary word-level checking needs the optimized bit-level model.
    m_model.Build({});
    if (m_model.SourceHasArrays()) {
        // Array CEGAR owns the checker lifecycle across abstraction rebuilds.
        m_cegar =
            std::make_unique<WLCegar>(m_settings, m_log, m_model);
    } else {
        // Array-free BTOR2 models need only the standard checker wrapper.
        m_checker = CreateBitLevelChecker(m_model.BitModel(), m_log);
        if (!m_checker) {
            throw std::runtime_error(
                "word-level checker requires a bit-level checker.");
        }
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
    m_witnessTrace = {};
    if (m_memoryBmc)
        return m_memoryBmc->Run(static_cast<unsigned>(m_settings.bmcK));
    if (m_cegar) return m_cegar->Run();
    return m_checker->Run();
}

std::vector<std::pair<Cube, Cube>> WLChecker::GetCexTrace() {
    if (m_memoryBmc) return {};
    if (m_cegar) return m_cegar->GetCexTrace();
    return m_checker->GetCexTrace();
}

const WLWitnessTrace &WLChecker::GetWitnessTrace() {
    if (m_memoryBmc) return m_memoryBmc->GetWitnessTrace();
    if (m_cegar) return m_cegar->GetWitnessTrace();
    if (m_witnessTrace.steps.empty()) {
        m_witnessTrace = ToSparseWitnessTrace(
            m_model.DecodeBitTrace(m_checker->GetCexTrace()));
    }
    return m_witnessTrace;
}

} // namespace car
