#include "rlive.h"

#include "BCAR.h"
#include "BasicIC3.h"
#include "FCAR.h"
#include <algorithm>

namespace car {

rlive::rlive(Settings settings,
             Model &model,
             Log &log) : m_settings(settings),
                         m_model(model),
                         m_log(log) {}

CheckResult rlive::Run() {
    if (m_model.GetPropKind() != Model::PropKind::Liveness) {
        m_log.L(0, "rlive only supports liveness properties.");
        return CheckResult::Unknown;
    }

    while (CheckReachable(cube{})) {
        auto trace = m_safeChecker->GetCexTrace();
        if (trace.empty()) {
            m_log.L(0, "Empty counterexample trace for unsafe reachability.");
            return CheckResult::Unknown;
        }
        cube t = trace.back().first;
        m_badStack.emplace_back(t);

        while (!m_badStack.empty()) {
            cube s = m_badStack.back();
            if (CheckReachable(s)) {
                auto new_trace = m_safeChecker->GetCexTrace();
                if (new_trace.empty()) {
                    m_log.L(0, "Empty counterexample trace for unsafe reachability.");
                    return CheckResult::Unknown;
                }
                cube new_t = new_trace.back().first;
                bool looped = false;
                for (const auto &b : m_badStack) {
                    if (Implies(b, new_t)) {
                        looped = true;
                        break;
                    }
                }
                if (looped) return CheckResult::Unsafe;
                m_badStack.emplace_back(new_t);
            } else {
                FrameList new_shoal = m_safeChecker->GetInv();
                if (!new_shoal.empty()) {
                    m_globalShoals.emplace_back(std::move(new_shoal));
                }

                m_badStack.pop_back();
            }
        }
    }

    return CheckResult::Safe;
}

void rlive::Witness() {
    m_log.L(1, "rlive witness generation is not implemented.");
}

std::vector<std::pair<cube, cube>> rlive::GetCexTrace() {
    return {};
}

std::unique_ptr<IncrAlg> rlive::MakeSafeChecker() {
    Settings sub_settings = m_settings;
    sub_settings.alg = m_settings.safetyBaseAlg;
    switch (m_settings.safetyBaseAlg) {
    case MCAlgorithm::FCAR:
        return std::make_unique<FCAR>(sub_settings, m_model, m_log);
    case MCAlgorithm::BCAR:
        return std::make_unique<BCAR>(sub_settings, m_model, m_log);
    case MCAlgorithm::IC3:
        return std::make_unique<BasicIC3>(sub_settings, m_model, m_log);
    default:
        return std::make_unique<FCAR>(sub_settings, m_model, m_log);
    }
}

bool rlive::CheckReachable(const cube &s) {
    m_safeChecker = MakeSafeChecker();
    m_log.L(1, "===== RLIVE SEARCH BAD =====", " at lvl: ", m_badStack.size());
    if (m_badStack.empty()) {
        m_log.L(2, "start from initial state");
    } else {
        m_log.L(1, "start from bad state ", CubeToStr(s));
        m_log.L(1, "reach bad set size: ", m_badStack.size());
        if (m_settings.verbosity > 2) {
            for (const auto &r : m_badStack) {
                m_log.L(3, CubeToStr(r));
            }
        }
    }

    m_safeChecker->SetInit(s);
    if (!s.empty()) {
        m_safeChecker->SetSearchFromInitSucc(true);
    }
    m_safeChecker->SetShoals(m_globalShoals);
    if (m_settings.rlivePruneDead) {
        m_safeChecker->SetDead(m_globalDead);
    }

    CheckResult res = m_safeChecker->Run();
    return res == CheckResult::Unsafe;
}

bool rlive::Implies(const cube &a, const cube &b) {
    if (a.size() > b.size()) return false;
    return std::includes(b.begin(), b.end(), a.begin(), a.end(), cmp);
}

} // namespace car
