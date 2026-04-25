#include "L2S.h"

#include "BCAR.h"
#include "BMC.h"
#include "FCAR.h"
#include "IC3.h"

namespace car {

L2S::L2S(Settings settings,
         Model &model,
         Log &log) : m_settings(settings),
                     m_model(model),
                     m_log(log),
                     m_checker(nullptr),
                     m_save(0) {
}

CheckResult L2S::Run() {
    if (m_model.GetPropKind() != Model::PropKind::Liveness) {
        LOG_L(m_log, 0, "L2S only supports liveness properties.");
        return CheckResult::Unknown;
    }

    Translate();

    m_checker = CreateSafetyChecker();
    if (!m_checker) {
        return CheckResult::Unknown;
    }

    return m_checker->Run();
}

std::vector<std::pair<Cube, Cube>> L2S::GetCexTrace() {

    return {};
}

std::unique_ptr<BaseAlg> L2S::CreateSafetyChecker() {
    Settings safety_settings = m_settings;
    safety_settings.alg = m_settings.safetyBaseAlg;

    switch (m_settings.safetyBaseAlg) {
    case MCAlgorithm::FCAR:
        return std::make_unique<FCAR>(safety_settings, m_model, m_log);
    case MCAlgorithm::BCAR:
        return std::make_unique<BCAR>(safety_settings, m_model, m_log);
    case MCAlgorithm::BMC:
        return std::make_unique<BMC>(safety_settings, m_model, m_log);
    case MCAlgorithm::IC3:
        return std::make_unique<IC3>(safety_settings, m_model, m_log);
    default:
        return std::make_unique<FCAR>(safety_settings, m_model, m_log);
    }
}

void L2S::Translate() {
    m_origInputs = m_model.GetModelInputs();
    m_origLatches = m_model.GetModelLatches();
    m_origInputSet.clear();
    m_origLatchSet.clear();
    for (Var v : m_origInputs) m_origInputSet.emplace(v);
    for (Var v : m_origLatches) m_origLatchSet.emplace(v);

    // save = input
    m_save = m_model.NewInputVar();

    std::vector<Var> latches = m_origLatches;
    m_latchCopy.clear();
    m_latchCopy.reserve(latches.size());

    for (Var v : latches) {
        Var nv = m_model.NewLatchVar();
        m_latchCopy.emplace_back(nv);

        // Init(nv) = Init(v)
        // Next(nv) = if (save) then v else nv
        Lit init = m_model.GetLatchResetLit(v);
        Lit next = m_model.MakeITE(MkLit(m_save), MkLit(v), MkLit(nv));
        m_model.SetLatchReset(nv, init);
        m_model.SetLatchNext(nv, next);
    }

    // Init(triggered) = false
    // Next(triggered) = justice | ( !save & triggered)
    Lit justice = m_model.GetBad();
    Var triggered = m_model.NewLatchVar();
    Lit next_triggered = m_model.MakeOR(justice, m_model.MakeAND(~MkLit(m_save), MkLit(triggered)));
    m_model.SetLatchReset(triggered, LIT_FALSE);
    m_model.SetLatchNext(triggered, next_triggered);

    // equality = Next(v_i) <-> Next(nv_i) & ...
    Lit equality = LIT_TRUE;
    for (size_t i = 0; i < latches.size(); i++) {
        Var v = latches[i];
        Var nv = m_latchCopy[i];
        Lit next_v = m_model.GetLatchNextLit(v);
        Lit next_nv = m_model.GetLatchNextLit(nv);
        Lit iff = m_model.MakeXNOR(next_v, next_nv);
        equality = m_model.MakeAND(equality, iff);
    }

    // bad = equality && triggered
    Lit bad = m_model.MakeAND(equality, next_triggered);

    m_model.SetBad(bad);
    m_model.Rebuild();
}

} // namespace car
