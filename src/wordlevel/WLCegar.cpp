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
#include <stdexcept>
#include <vector>

namespace car {
namespace {

void CompleteReplaySeed(
    Model &model,
    std::vector<std::pair<Cube, Cube>> &trace) {
    if (trace.empty())
        throw std::runtime_error("checker returned an empty bit-level trace");

    auto complete = [](Cube &cube,
                       const std::vector<Var> &required,
                       size_t frame,
                       const char *kind) {
        Var maxVar = 0;
        for (Var var : required) maxVar = std::max(maxVar, var);
        std::vector<char> allowed(static_cast<size_t>(maxVar) + 1, 0);
        std::vector<int> values(static_cast<size_t>(maxVar) + 1, -1);
        for (Var var : required) allowed[var] = 1;
        for (Lit literal : cube) {
            Var var = VarOf(literal);
            if (var >= allowed.size() || !allowed[var]) continue;
            int value = Sign(literal) ? 0 : 1;
            if (values[var] != -1 && values[var] != value) {
                throw std::runtime_error(
                    "checker trace has contradictory " + std::string(kind) +
                    " variable " + std::to_string(var) + " at frame " +
                    std::to_string(frame));
            }
            values[var] = value;
        }
        for (Var var : required) {
            if (values[var] == -1) cube.push_back(~MkLit(var));
        }
    };

    for (size_t frame = 0; frame < trace.size(); ++frame) {
        complete(trace[frame].first,
                 model.GetModelInputs(),
                 frame,
                 "input");
    }
    complete(trace.front().second,
             model.GetModelLatches(),
             0,
             "initial latch");
}

} // namespace

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
    m_cexTrace.clear();

    while (true) {
        // Each iteration proves or refutes the current finite abstraction.
        if (!m_checker) return CheckResult::Unknown;
        res = m_checker->Run();

        if (res == CheckResult::Unsafe) {
            // Simulator replay distinguishes concrete and spurious abstract traces.
            auto trace = m_checker->GetCexTrace();
            CompleteReplaySeed(m_model.BitModel(), trace);
            WLSimulator simulator(*m_ir);
            WLSimulator::Result replay =
                simulator.Replay(trace, m_model.TraceMap());
            if (replay.kind ==
                WLSimulator::ReplayKind::ConcreteCounterexample) {
                m_concreteCounterexample = true;
                m_cexTrace = std::move(trace);
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

    return res;
}

std::vector<std::pair<Cube, Cube>> WLCegar::GetCexTrace() {
    if (m_concreteCounterexample) return m_cexTrace;
    if (!m_checker) return {};
    return m_checker->GetCexTrace();
}

int WLCegar::GetSafeDepth() const {
    if (!m_checker) return -1;
    return m_checker->GetSafeDepth();
}

bool WLCegar::WriteCounterexample(const std::string &path) {
    if (!m_concreteCounterexample) return false;
    WLSimulator simulator(*m_ir);
    WLSimulator::Result replay = simulator.Replay(
        m_cexTrace, m_model.TraceMap());
    if (replay.kind != WLSimulator::ReplayKind::ConcreteCounterexample)
        return false;
    return simulator.WriteCounterexample(path);
}

} // namespace car
