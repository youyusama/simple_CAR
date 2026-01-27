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

    while (CheckReachable(cube())) {
        auto trace = m_safeChecker->GetCexTrace();
        cube t = trace.back().second;
        m_badStack.emplace_back(t);

        while (!m_badStack.empty()) {
            cube s = m_badStack.back();

            if (PruneDead(s)) {
                m_badStack.pop_back();
                continue;
            }

            if (CheckReachable(s)) {
                auto new_trace = m_safeChecker->GetCexTrace();
                cube new_t = new_trace.back().second;
                m_log.L(2, "get new bad state ", CubeToStr(new_t));
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
                    m_globalShoals.emplace_back(new_shoal);
                    m_pdSolver->AddShoalConstraints({new_shoal}, {}, 1);
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
        m_log.L(2, "start from bad state ", CubeToStr(s));
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
    m_safeChecker->SetDead(m_globalDead);

    // m_log.SetVerbosity(1);
    CheckResult res = m_safeChecker->Run();
    m_log.SetVerbosity(m_settings.verbosity);

    return res == CheckResult::Unsafe;
}


bool rlive::PruneDead(const cube &s) {
    m_log.L(1, "===== RLIVE PRUNE DEAD =====");
    m_log.L(3, "bad state ", CubeToStr(s));
    if (m_pdSolver == nullptr) {
        m_pdSolver = std::make_shared<SATSolver>(m_model, m_settings.solver);
        m_pdSolver->AddTrans();
        m_pdSolver->AddConstraints();
    }

    // s & T & !C'
    m_pdSolver->AddTempClause({m_model.GetBad()});
    bool sat = m_pdSolver->Solve(s);
    while (sat) {
        auto p = m_pdSolver->GetAssignment(true);
        m_pdSolver->ReleaseTempClause();

        auto assumption = p.second;
        bool is_not_dead = m_pdSolver->Solve(assumption);
        if (is_not_dead) {
            m_log.L(1, "not dead");
            return false;
        } else {
            auto new_dead = GetUnsatAssumption(m_pdSolver, assumption);
            m_pdSolver->AddShoalConstraints({}, {new_dead}, 1);
            m_globalDead.emplace_back(new_dead);
        }
        m_pdSolver->AddTempClause({m_model.GetBad()});
        sat = m_pdSolver->Solve(s);
    }
    auto new_dead = GetUnsatAssumption(m_pdSolver, s);
    m_pdSolver->ReleaseTempClause();
    m_pdSolver->AddShoalConstraints({}, {new_dead}, 1);
    m_globalDead.emplace_back(new_dead);

    m_log.L(1, "new dead size: ", m_globalDead.size());
    for (const auto &d : m_globalDead) {
        m_log.L(3, CubeToStr(d));
    }
    return true;
}


cube rlive::GetUnsatAssumption(shared_ptr<SATSolver> solver, const cube &assumptions) {
    const unordered_set<int> &conflict = solver->GetConflict();
    cube res;
    for (auto a : assumptions) {
        if (conflict.find(a) != conflict.end())
            res.emplace_back(a);
    }
    return res;
}


bool rlive::Implies(const cube &a, const cube &b) {
    if (a.size() > b.size()) return false;
    return std::includes(b.begin(), b.end(), a.begin(), a.end(), cmp);
}

} // namespace car
