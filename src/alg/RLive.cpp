#include "RLive.h"

#include "FCAR.h"
#include "IC3.h"
#include <algorithm>
#include <string>

namespace car {

RLive::RLive(Settings settings,
             Model &model,
             Log &log) : m_settings(settings),
                         m_model(model),
                         m_log(log) {

    m_pdSolver = std::make_shared<SATSolver>(m_model, m_settings.solver);
    m_pdSolver->AddTrans();
    m_pdSolver->AddConstraints();
}

CheckResult RLive::Run() {
    signal(SIGINT, SignalHandler);
    m_cexTrace.clear();
    m_traceStack.clear();
    m_badStack.clear();
    m_globalShoals.clear();
    m_globalDead.clear();

    if (m_model.GetPropKind() != Model::PropKind::Liveness) {
        LOG_L(m_log, 0, "rlive only supports liveness properties.");
        return CheckResult::Unknown;
    }
    while (CheckReachable(Cube())) {
        auto trace = m_safeChecker->GetCexTrace();
        Cube t = trace.back().second;
        m_badStack.emplace_back(t);
        m_traceStack.emplace_back(std::move(trace));

        while (!m_badStack.empty()) {
            Cube s = m_badStack.back();

            if (PruneDead(s)) {
                m_badStack.pop_back();
                if (!m_traceStack.empty()) m_traceStack.pop_back();
                continue;
            }

            if (CheckReachable(s)) {
                auto new_trace = m_safeChecker->GetCexTrace();
                Cube new_t = new_trace.back().second;
                LOG_L(m_log, 2, "get new bad state ", CubeToStr(new_t));
                bool looped = false;
                for (const auto &b : m_badStack) {
                    if (b == new_t) {
                        looped = true;
                        break;
                    }
                }
                if (looped) {
                    BuildCexTrace(new_trace);
                    return CheckResult::Unsafe;
                }

                m_badStack.emplace_back(new_t);
                m_traceStack.emplace_back(std::move(new_trace));
            } else {
                FrameList new_shoal = m_safeChecker->GetInv();
                if (!new_shoal.empty()) {
                    m_globalShoals.emplace_back(new_shoal);
                    m_pdSolver->AddInvAsClauseK(new_shoal, true, 1);
                }

                m_badStack.pop_back();
                if (!m_traceStack.empty()) m_traceStack.pop_back();
            }
        }
    }

    FrameList final_invariant = m_safeChecker->GetInv();
    if (!m_settings.rliveProofPath.empty() && !WriteProof(final_invariant)) {
        LOG_L(m_log, 1, "Failed to write RLive proof.");
    }
    return CheckResult::Safe;
}

bool RLive::WriteProof(const FrameList &finalInvariant) const {
    const std::string &path = m_settings.rliveProofPath;
    if (path.size() < 4 || path.substr(path.size() - 4) != ".aag") {
        LOG_L(m_log, 1, "RLive proof path must end in '.aag': ", path);
        return false;
    }
    if (finalInvariant.empty()) {
        LOG_L(m_log, 1, "RLive final invariant is empty.");
        return false;
    }

    const aiger *model_aig = m_model.GetAiger().get();
    std::shared_ptr<aiger> proof_ptr(aiger_init(), [](aiger *aig) {
        aiger_reset(aig);
    });
    aiger *proof = proof_ptr.get();

    std::unordered_set<Var> latch_vars;
    latch_vars.reserve(model_aig->num_latches);
    for (unsigned i = 0; i < model_aig->num_latches; ++i) {
        const aiger_symbol &latch = model_aig->latches[i];
        aiger_add_input(proof, latch.lit, latch.name);
        latch_vars.emplace(aiger_lit2var(latch.lit));
    }

    auto valid_cube = [&](const Cube &cube) {
        for (Lit lit : cube) {
            if (!IsConst(lit) && latch_vars.find(VarOf(lit)) == latch_vars.end()) {
                LOG_L(m_log, 1,
                      "RLive proof contains non-latch literal ",
                      ToSigned(lit), ".");
                return false;
            }
        }
        return true;
    };

    auto valid_invariant = [&](const FrameList &invariant) {
        for (const Frame &frame : invariant) {
            for (const Cube &cube : frame) {
                if (!valid_cube(cube)) return false;
            }
        }
        return true;
    };

    for (const FrameList &shoal : m_globalShoals) {
        if (!valid_invariant(shoal)) return false;
    }
    for (const Cube &state : m_globalDead) {
        if (!valid_cube(state)) return false;
    }
    if (!valid_invariant(finalInvariant)) return false;

    auto build_and = [&](const std::vector<unsigned> &lits) {
        if (lits.empty()) return ToAigerLit(LIT_TRUE);
        unsigned result = lits.front();
        for (size_t i = 1; i < lits.size(); ++i) {
            unsigned gate = (proof->maxvar + 1) * 2U;
            aiger_add_and(proof, gate, result, lits[i]);
            result = gate;
        }
        return result;
    };

    auto build_or = [&](const std::vector<unsigned> &lits) {
        if (lits.empty()) return ToAigerLit(LIT_FALSE);
        std::vector<unsigned> negated;
        negated.reserve(lits.size());
        for (unsigned lit : lits) negated.push_back(lit ^ 1U);
        return build_and(negated) ^ 1U;
    };

    auto build_cube = [&](const Cube &cube) {
        std::vector<unsigned> lits;
        lits.reserve(cube.size());
        for (Lit lit : cube) lits.push_back(ToAigerLit(lit));
        return build_and(lits);
    };

    auto build_frame = [&](const Frame &frame) {
        std::vector<unsigned> terms;
        terms.reserve(frame.size());
        for (const Cube &blocked_cube : frame) {
            terms.push_back(build_cube(blocked_cube) ^ 1U);
        }
        return build_and(terms);
    };

    auto build_invariant = [&](const FrameList &invariant) {
        std::vector<unsigned> frames;
        frames.reserve(invariant.size());
        for (const Frame &frame : invariant) frames.push_back(build_frame(frame));
        return build_or(frames);
    };

    size_t output_index = 0;
    for (const FrameList &shoal : m_globalShoals) {
        unsigned lit = build_invariant(shoal);
        std::string name = "shoal_" + std::to_string(output_index++);
        aiger_add_output(proof, lit, name.c_str());
    }
    for (const Cube &state : m_globalDead) {
        unsigned lit = build_cube(state);
        std::string name = "dead_" + std::to_string(output_index++);
        aiger_add_output(proof, lit, name.c_str());
    }
    aiger_add_output(proof, build_invariant(finalInvariant), "final_invariant");

    if (const char *err = aiger_check(proof)) {
        LOG_L(m_log, 1, "invalid RLive proof AAG: ", err);
        return false;
    }
    if (!aiger_open_and_write_to_file(proof, path.c_str())) {
        if (const char *err = aiger_error(proof)) {
            LOG_L(m_log, 1, "RLive proof write error: ", err);
        } else {
            LOG_L(m_log, 1, "failed to write RLive proof AAG: ", path);
        }
        return false;
    }

    LOG_L(m_log, 1, "RLive proof written to ", path);
    return true;
}

std::vector<std::pair<Cube, Cube>> RLive::GetCexTrace() {
    return m_cexTrace;
}

void RLive::BuildCexTrace(const std::vector<std::pair<Cube, Cube>> &closingTrace) {
    m_cexTrace.clear();
    for (auto segment : m_traceStack) {
        if (!segment.empty()) segment.pop_back();
        m_cexTrace.insert(m_cexTrace.end(), segment.begin(), segment.end());
    }
    auto closing_segment = closingTrace;
    if (!closing_segment.empty()) closing_segment.pop_back();
    m_cexTrace.insert(m_cexTrace.end(), closing_segment.begin(), closing_segment.end());
}

std::unique_ptr<IncrAlg> RLive::MakeSafeChecker() {
    Settings sub_settings = m_settings;
    sub_settings.alg = m_settings.safetyBaseAlg;
    switch (m_settings.safetyBaseAlg) {
    case MCAlgorithm::FCAR:
        return std::make_unique<FCAR>(sub_settings, m_model, m_log);
    case MCAlgorithm::IC3:
        return std::make_unique<IC3>(sub_settings, m_model, m_log);
    default:
        return std::make_unique<FCAR>(sub_settings, m_model, m_log);
    }
}

bool RLive::CheckReachable(const Cube &s) {
    m_safeChecker = MakeSafeChecker();
    LOG_L(m_log, 1, "===== RLIVE SEARCH BAD =====", " at lvl: ", m_badStack.size());
    if (m_badStack.empty()) {
        LOG_L(m_log, 2, "start from initial state");
    } else {
        LOG_L(m_log, 2, "start from bad state ", CubeToStr(s));
        if (m_settings.verbosity > 2) {
            for (const auto &r : m_badStack) {
                LOG_L(m_log, 3, CubeToStr(r));
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


bool RLive::PruneDead(const Cube &s) {
    LOG_L(m_log, 3, "bad state ", CubeToStr(s));

    // s & T & !C' & !q
    m_pdSolver->AddTempClause({m_model.GetBad()});
    bool sat = m_pdSolver->Solve(s);
    while (sat) {
        auto p = m_pdSolver->GetAssignment(true);
        m_pdSolver->ReleaseTempClause();

        // p & T & !C'
        auto assumption = p.second;

        LOG_L(m_log, 2, "get succ", CubeToStr(assumption));
        bool is_not_dead = m_pdSolver->Solve(assumption);
        if (is_not_dead) {
            LOG_L(m_log, 2, "not dead");
            return false;
        } else {
            auto new_dead = GetUnsatAssumption(m_pdSolver, assumption);

            // generalize dead
            if (new_dead.size() > 2) {
                std::unordered_set<int> required;

                for (int i = 0; i < static_cast<int>(new_dead.size()); ++i) {
                    if (required.find(i) != required.end()) continue;

                    Cube tmp;
                    tmp.reserve(new_dead.size() - 1);
                    for (int j = 0; j < static_cast<int>(new_dead.size()); ++j) {
                        if (j != i) tmp.emplace_back(new_dead[j]);
                    }

                    bool is_not_dead = m_pdSolver->Solve(tmp);
                    if (is_not_dead) {
                        required.emplace(i);
                    } else {
                        new_dead.swap(tmp);
                        required.clear();
                        i = -1;
                    }
                }
            }
            LOG_L(m_log, 2, "get new dead", CubeToStr(new_dead));
            m_pdSolver->AddCubeAsClauseK(new_dead, true, 1);
            m_globalDead.emplace_back(new_dead);
        }
        m_pdSolver->AddTempClause({m_model.GetBad()});
        sat = m_pdSolver->Solve(s);
    }
    m_pdSolver->ReleaseTempClause();

    LOG_L(m_log, 1, "===== RLIVE PRUNE DEAD =====");
    LOG_L(m_log, 2, "global dead size: ", m_globalDead.size());
    for (const auto &d : m_globalDead) {
        LOG_L(m_log, 3, CubeToStr(d));
    }
    return true;
}


Cube RLive::GetUnsatAssumption(shared_ptr<SATSolver> solver, const Cube &assumptions) {
    solver->GetConflict(m_conflictScratch);
    Cube res;
    for (auto a : assumptions) {
        if (m_conflictScratch.find(a) != m_conflictScratch.end())
            res.emplace_back(a);
    }
    return res;
}

} // namespace car
