#include "WLCegar.h"

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
#include "model/WLModel.h"
#include "WLMemoryBMC.h"
#include "WLSimulator.h"

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace car {

WLCegar::WLCegar(const Settings &settings,
                 Log &log,
                 WLModel &model)
    : m_settings(settings),
      m_log(log),
      m_model(model) {
    m_checker = CreateBitLevelChecker(m_model.BitModel(), m_log);
    if (!m_checker) {
        throw std::runtime_error(
            "word-level CEGAR requires a bit-level checker.");
    }
}

WLCegar::~WLCegar() = default;

bool WLCegar::AddPair(const WLMemoryPair &pair) {
    // Refinement pairs are unique by memory, address expression, and delay.
    if (pair.memoryStateId <= 0) return false;
    auto duplicate = std::find_if(
        m_memoryPairs.begin(),
        m_memoryPairs.end(),
        [&](const WLMemoryPair &existing) {
            return existing.memoryStateId == pair.memoryStateId &&
                   existing.addressNodeId == pair.addressNodeId &&
                   existing.delay == pair.delay;
        });
    if (duplicate == m_memoryPairs.end()) {
        m_memoryPairs.push_back(pair);
        return true;
    }
    return false;
}

unsigned WLCegar::MaxDelay() const {
    unsigned result = 0;
    for (const WLMemoryPair &pair : m_memoryPairs)
        result = std::max(result, pair.delay);
    return result;
}

std::unique_ptr<BaseAlg>
WLCegar::CreateBitLevelChecker(Model &model, Log &log) {
    // Each abstraction revision uses the same user-selected bit-level algorithm.
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

bool WLCegar::ReloadModel() {
    try {
        // Rebuild the abstraction and recreate the selected checker on its new AIG.
        m_model.Build(m_memoryPairs);
        m_checker = CreateBitLevelChecker(m_model.BitModel(), m_log);
        if (!m_checker) return false;
    } catch (const std::exception &error) {
        LOG_L(m_log, 0, "word-level refinement reload failed: ", error.what());
        return false;
    }
    return true;
}

CheckResult WLCegar::Run() {
    CheckResult res = CheckResult::Unknown;
    int refinements = 0;
    m_concreteCounterexample = false;
    m_cexTrace.clear();
    m_witnessTrace = {};

    while (true) {
        // Each iteration proves or refutes the current finite abstraction.
        if (!m_checker) return CheckResult::Unknown;
        res = m_checker->Run();

        if (res == CheckResult::Unsafe) {
            // Simulator replay distinguishes concrete and spurious abstract traces.
            auto trace = m_checker->GetCexTrace();
            WLReplayTrace replayTrace = m_model.DecodeBitTrace(trace);
            WLSimulator simulator(m_model.SourceIR());
            WLSimulator::Result replay =
                simulator.Replay(replayTrace);
            if (replay.kind ==
                WLSimulator::ReplayKind::ConcreteCounterexample) {
                m_concreteCounterexample = true;
                m_cexTrace = std::move(trace);
                m_witnessTrace = std::move(replay.witnessTrace);
                break;
            }

            // Every spurious trace must produce a new simulator-derived pair.
            bool refined = false;
            for (const WLReadMismatch &mismatch : replay.refinementReads) {
                LOG_L(m_log,
                      2,
                      "word-level erroneous read: read=",
                      mismatch.readNodeId,
                      " memory=",
                      mismatch.memoryStateId,
                      " address=",
                      mismatch.addressNodeId,
                      " time=",
                      mismatch.time,
                      " delay=",
                      mismatch.delay);
                refined |= AddPair({mismatch.memoryStateId,
                                    mismatch.addressNodeId,
                                    mismatch.delay});
            }
            if (!refined)
                throw std::runtime_error(
                    "spurious word-level counterexample produced no new "
                    "memory refinement pair");

            ++refinements;
            LOG_L(m_log,
                  1,
                  "word-level memory refinement ",
                  refinements,
                  ": ",
                  m_memoryPairs.size(),
                  " tracked address/delay pairs.");
            if (!ReloadModel()) {
                res = CheckResult::Unknown;
                break;
            }
            continue;
        }

        if (res == CheckResult::Safe) {
            // Close the finite prefix not covered by the delayed abstraction guards.
            WLMemoryBMC boundedChecker(m_settings, m_model, m_log);
            try {
                CheckResult bounded = boundedChecker.Run(MaxDelay());
                if (bounded == CheckResult::Unsafe) {
                    m_concreteCounterexample = true;
                    m_cexTrace.clear();
                    m_witnessTrace = boundedChecker.GetWitnessTrace();
                    res = CheckResult::Unsafe;
                } else {
                    // Normal Unknown means the complete finite prefix is safe.
                    res = CheckResult::Safe;
                }
            } catch (const std::exception &error) {
                LOG_L(m_log, 0, "WL memory BMC failed: ", error.what());
                res = CheckResult::Unknown;
            }
        }
        break;
    }

    return res;
}

std::vector<std::pair<Cube, Cube>> WLCegar::GetCexTrace() {
    if (m_concreteCounterexample) return m_cexTrace;
    if (!m_checker) return {};
    return m_checker->GetCexTrace();
}

} // namespace car
