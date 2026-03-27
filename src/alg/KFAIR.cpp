#include "KFAIR.h"

#include "BCAR.h"
#include "IC3.h"
#include "FCAR.h"
#include <algorithm>
#include <unordered_set>

namespace car {

KFAIR::KFAIR(Settings settings,
             Model &model,
             Log &log) : m_settings(settings),
                         m_model(model),
                         m_log(log) {}

CheckResult KFAIR::Run() {
    signal(SIGINT, SignalHandler);

    if (m_model.GetPropKind() != Model::PropKind::Liveness) {
        LOG_L(m_log, 0, "KFAIR only supports liveness properties.");
        return CheckResult::Unknown;
    }

    bool enable_klive = (m_settings.alg == MCAlgorithm::KLIVE || m_settings.alg == MCAlgorithm::KFAIR);
    bool enable_fair = (m_settings.alg == MCAlgorithm::FAIR || m_settings.alg == MCAlgorithm::KFAIR);

    if (m_settings.alg == MCAlgorithm::KFAIR) {
        LOG_L(m_log, 1, "KFAIR running in KFAIR mode.");
    } else if (m_settings.alg == MCAlgorithm::KLIVE) {
        LOG_L(m_log, 1, "KFAIR running in k-Liveness mode.");
    } else if (m_settings.alg == MCAlgorithm::FAIR) {
        LOG_L(m_log, 1, "KFAIR running in FAIR mode.");
    }

    auto prefix = MakeSafeChecker();

    while (true) {
        // Search for a lasso prefix.
        LOG_L(m_log, 1, "===== Search for a prefix =====");
        prefix->SetWalls(m_globalWalls);
        CheckResult res = prefix->Run();
        if (res == CheckResult::Safe) {
            LOG_L(m_log, 1, "===== No unsafe prefix =====");
            return CheckResult::Safe;
        }

        if (enable_klive) {
            if (DetectKLiveCex(*prefix)) {
                LOG_L(m_log, 1, "===== CEX found in trace =====");
                return CheckResult::Unsafe;
            }
        }

        if (enable_fair) {
            LOG_L(m_log, 1, "===== Search for a loop by fair =====");
            auto trace = prefix->GetCexTrace();
            Cube t = trace.back().second;
            LOG_L(m_log, 2, "start from bad state ", CubeToStr(t));
            auto loop = MakeSafeChecker();
            loop->SetInit(t);
            loop->SetSearchFromInitSucc(true);
            loop->SetLoopRefuting(true);
            loop->SetWalls(m_globalWalls);
            CheckResult loop_res = loop->Run();
            if (loop_res == CheckResult::Unsafe) {
                return CheckResult::Unsafe;
            } else {
                FrameList new_wall = loop->GetInv();
                m_globalWalls.emplace_back(std::move(new_wall));
            }
        }

        if (enable_klive) {
            LOG_L(m_log, 1, "===== k-Liveness increase =====");
            prefix->KLiveIncr();
        }
    }
}

void KFAIR::Witness() {
    LOG_L(m_log, 1, "KFAIR witness generation is not implemented.");
}

std::vector<std::pair<Cube, Cube>> KFAIR::GetCexTrace() {
    return {};
}

std::unique_ptr<IncrAlg> KFAIR::MakeSafeChecker() {
    Settings sub_settings = m_settings;
    sub_settings.alg = m_settings.safetyBaseAlg;
    switch (m_settings.safetyBaseAlg) {
    case MCAlgorithm::FCAR:
        return std::make_unique<FCAR>(sub_settings, m_model, m_log);
    case MCAlgorithm::BCAR:
        return std::make_unique<BCAR>(sub_settings, m_model, m_log);
    case MCAlgorithm::IC3:
        return std::make_unique<IC3>(sub_settings, m_model, m_log);
    default:
        return std::make_unique<FCAR>(sub_settings, m_model, m_log);
    }
}

bool KFAIR::DetectKLiveCex(IncrAlg &checker) {
    std::vector<Cube> states;
    auto trace = checker.GetCexTrace();
    states.reserve(trace.size());
    for (auto &step : trace) {
        states.emplace_back(step.second);
    }

    // Need at least one k-liveness latch and a non-trivial trace.
    int klive_step = m_model.GetKLiveStep();
    if (states.size() <= 1 || klive_step < 1) return false;

    // Build a set of k-liveness latch variables for quick filtering.
    std::unordered_set<Var> live_set;
    for (int i = 1; i <= klive_step; i++)
        live_set.emplace(VarOf(m_model.GetKLiveSignal(i)));

    // Track indices where the k-counter value changes and store the trimmed trace
    // (state without k-liveness latch literals).
    std::vector<int> bad_indices;
    std::vector<Cube> trace_to_bad;
    trace_to_bad.reserve(states.size());

    int k_counter_last = -1;
    for (size_t i = 0; i < states.size(); i++) {
        int k_counter = 0;
        Cube s_trim;
        s_trim.reserve(states[i].size());
        for (Lit lit : states[i]) {
            if (live_set.count(VarOf(lit))) {
                if (!Sign(lit)) k_counter++;
            } else {
                s_trim.emplace_back(lit);
            }
        }

        // Record the end of a segment when the k-counter value changes.
        if (k_counter_last != -1 && k_counter != k_counter_last) {
            bad_indices.push_back(static_cast<int>(i) - 1);
        }
        k_counter_last = k_counter;
        trace_to_bad.emplace_back(std::move(s_trim));
    }
    bad_indices.push_back(static_cast<int>(states.size()) - 1);

    // Look for a loop in the trimmed trace that crosses a k-counter boundary.
    for (size_t i = 0; i < trace_to_bad.size(); i++) {
        for (size_t j = trace_to_bad.size(); j-- > i + 1;) {
            if (trace_to_bad[i] == trace_to_bad[j]) {
                for (int idx : bad_indices) {
                    if (idx >= static_cast<int>(i) && idx <= static_cast<int>(j)) {
                        return true;
                    }
                }
                break;
            }
        }
    }
    return false;
}

} // namespace car
