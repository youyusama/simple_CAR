#include "WLCegar.h"

#include "BCAR.h"
#include "BMC.h"
#include "Btor2Frontend.h"
#include "FCAR.h"
#include "IC3.h"
#include "KFAIR.h"
#include "KIND.h"
#include "L2S.h"
#include "Log.h"
#include "Model.h"
#include "RLive.h"
#include "WLModel.h"
#include "WLSimulator.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace car {

WLCegar::WLCegar(const Settings &settings,
                 Log &log,
                 WLModel &model)
    : m_settings(settings),
      m_log(log),
      m_model(model) {
    try {
        // Replay always uses the original concrete BTOR2 model.
        m_ir =
            std::make_unique<Btor2IR>(
                Btor2Frontend::LoadIR(m_model.InputPath()));
    } catch (const std::exception &error) {
        throw std::runtime_error("word-level CEGAR parse error: " +
                                 std::string(error.what()));
    }
    m_checker = CreateBitLevelChecker(m_model.BitModel(), m_log);
    if (!m_checker) {
        throw std::runtime_error(
            "word-level CEGAR requires a bit-level checker.");
    }
}

WLCegar::~WLCegar() = default;

void WLCegar::AddPair(const WLMemoryPair &pair) {
    // Refinement pairs are unique by memory, address expression, and delay.
    if (pair.memoryStateId <= 0) return;
    auto duplicate = std::find_if(
        m_memoryPairs.begin(),
        m_memoryPairs.end(),
        [&](const WLMemoryPair &existing) {
            return existing.memoryStateId == pair.memoryStateId &&
                   existing.addressNodeId == pair.addressNodeId &&
                   existing.delay == pair.delay;
        });
    if (duplicate == m_memoryPairs.end())
        m_memoryPairs.push_back(pair);
}

unsigned WLCegar::MaxDelay() const {
    unsigned maxDelay = 0;
    for (const WLMemoryPair &pair : m_memoryPairs)
        maxDelay = std::max(maxDelay, pair.delay);
    return maxDelay;
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

    while (true) {
        // Each iteration proves or refutes the current finite abstraction.
        if (!m_checker) return CheckResult::Unknown;
        res = m_checker->Run();

        if (res == CheckResult::Unsafe) {
            // Simulator replay distinguishes concrete and spurious abstract traces.
            const auto &trace = m_checker->GetCexTrace();
            unsigned depth =
                trace.empty() ? 0 : static_cast<unsigned>(trace.size() - 1);
            WLSimulator simulator(*m_ir);
            WLSimulator::Result replay =
                simulator.Replay(trace, m_model.TraceMap());
            if (replay.concreteCounterexample) {
                m_concreteCounterexample = true;
                break;
            }

            // Prefer mismatch-directed refinement from the simulator.
            size_t oldSize = m_memoryPairs.size();
            for (const WLReadMismatch &mismatch : replay.mismatches) {
                AddPair({mismatch.memoryStateId,
                         mismatch.addressNodeId,
                         mismatch.delay});
            }
            // Fall back to tracking every read address through the trace depth.
            if (m_memoryPairs.size() == oldSize) {
                for (const WLArrayRead &read : m_model.ArrayReads()) {
                    if (read.memoryStateId <= 0) continue;
                    for (unsigned delay = 0; delay <= depth; ++delay) {
                        AddPair({read.memoryStateId, read.addressNodeId, delay});
                    }
                }
            }
            if (m_memoryPairs.size() == oldSize) {
                LOG_L(m_log,
                      0,
                      "word-level memory abstraction could not refine the counterexample.");
                res = CheckResult::Unknown;
                break;
            }

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
            // The proof must cover every delayed guard introduced by abstraction.
            unsigned maxDelay = MaxDelay();
            int safeDepth = m_checker->GetSafeDepth();
            if (safeDepth < 0 ||
                static_cast<unsigned>(safeDepth) < maxDelay) {
                LOG_L(m_log,
                      0,
                      "word-level memory abstraction safe proof depth ",
                      safeDepth,
                      " is smaller than MaxDelay ",
                      maxDelay,
                      "; returning Unknown.");
                res = CheckResult::Unknown;
            }
        }
        break;
    }

    if (res == CheckResult::Unsafe && !m_concreteCounterexample) {
        LOG_L(m_log,
              0,
              "word-level transformed counterexample could not be reproduced by simulator.");
        res = CheckResult::Unknown;
    }
    return res;
}

std::vector<std::pair<Cube, Cube>> WLCegar::GetCexTrace() {
    if (!m_checker) return {};
    return m_checker->GetCexTrace();
}

int WLCegar::GetSafeDepth() const {
    if (!m_checker) return -1;
    return m_checker->GetSafeDepth();
}

bool WLCegar::WriteCounterexample(const std::string &path) {
    if (!m_concreteCounterexample || !m_checker) return false;
    WLSimulator simulator(*m_ir);
    WLSimulator::Result replay = simulator.Replay(
        m_checker->GetCexTrace(), m_model.TraceMap());
    if (!replay.concreteCounterexample) return false;
    return simulator.WriteCounterexample(path);
}

} // namespace car
