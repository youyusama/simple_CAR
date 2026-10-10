#include "WLChecker.h"
#include "CheckerFactory.h"

#include "Log.h"
#include "Model.h"
#include "WLCEGAR.h"
#include "MemoryBMC.h"
#include "model/WLModel.h"
#include "model/Bitblastor.h"
#include "model/PackageResize.h"

#include <chrono>
#include <stdexcept>

namespace car {
WLChecker::WLChecker(const Settings &settings,
                     WLModel &model,
                     Log &log)
    : m_settings(settings),
      m_log(log),
      m_model(model) {
    if (m_settings.alg == MCAlgorithm::WLBMC) {
        // Native memory BMC works directly on the source IR.
        m_memoryBmc =
            std::make_unique<MemoryBMC>(m_model, m_log);
        return;
    }

    if (m_model.SourceHasArrays()) {
        m_cegar = std::make_unique<WLCEGAR>(m_settings, m_log, m_model);
    } else {
        BuildScalarModel();
    }
}

WLChecker::~WLChecker() = default;

void WLChecker::BuildScalarModel() {
    const auto start = std::chrono::steady_clock::now();
    if (!m_settings.wlBitblastOutputPath.empty())
        throw std::runtime_error("checking build cannot be used for AIG export");
    if (!m_settings.wlDisablePackageResize)
        m_resize = std::make_unique<PackageResize>(m_model.PropertyIR());
    const auto &ir = m_resize ? m_resize->IR() : m_model.PropertyIR();
    m_aig = GenerateWLAig(ir, m_layout);
    m_bitModel = std::make_unique<Model>(m_settings, m_log, m_aig);
    LOG_L(m_log, 1, "word-level model build: ms=",
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count(),
          " selectors=0 slots=0");
    m_checker = CreateBitLevelChecker(m_settings, *m_bitModel, m_log);
    if (!m_checker) throw std::runtime_error("word-level checker requires a bit-level checker.");
}

CheckResult WLChecker::Run() {
    m_trace = {};
    if (m_memoryBmc) {
        const auto result = m_memoryBmc->CheckThrough(
            static_cast<unsigned>(m_settings.bmcK),
            static_cast<unsigned>(m_settings.bmcStep));
        if (result.status == MemoryBMC::Status::Counterexample)
            return CheckResult::Unsafe;
        if (result.status == MemoryBMC::Status::Unknown)
            LOG_L(m_log, 0, "WL memory BMC incomplete: ", result.reason);
        // A bounded proof does not establish unbounded safety.
        return CheckResult::Unknown;
    }
    if (m_cegar) return m_cegar->Run();
    return m_checker->Run();
}

std::vector<std::pair<Cube, Cube>> WLChecker::GetCexTrace() {
    if (m_memoryBmc) return {};
    if (m_cegar) return {};
    return m_checker->GetCexTrace();
}

const WLTrace &WLChecker::GetTrace() {
    if (m_memoryBmc) return m_memoryBmc->GetTrace();
    if (m_cegar) return m_cegar->GetTrace();
    if (m_trace.steps.empty()) {
        const auto &ir = m_resize ? m_resize->IR() : m_model.PropertyIR();
        auto choices = RecoverWLCheckerChoices(ir, *m_bitModel->GetAiger(),
            m_bitModel->GetEquivalenceMap(), m_bitModel->TrueId(),
            m_layout, m_checker->GetCexTrace());
        m_trace = m_resize ? m_resize->RestoreTrace(choices) : std::move(choices);
    }
    return m_trace;
}

} // namespace car
