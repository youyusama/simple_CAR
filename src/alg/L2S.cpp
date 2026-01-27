#include "L2S.h"

#include "BCAR.h"
#include "BMC.h"
#include "BasicIC3.h"
#include "FCAR.h"

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
        m_log.L(0, "L2S only supports liveness properties.");
        return CheckResult::Unknown;
    }

    Translate();

    m_checker = CreateSafetyChecker();
    if (!m_checker) {
        return CheckResult::Unknown;
    }

    return m_checker->Run();
}


void L2S::Witness() {


    m_log.L(0, "L2S witness output is not implemented yet.");
}


std::vector<std::pair<cube, cube>> L2S::GetCexTrace() {

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
        return std::make_unique<BasicIC3>(safety_settings, m_model, m_log);
    default:
        return std::make_unique<FCAR>(safety_settings, m_model, m_log);
    }
}

void L2S::Translate() {
    m_origInputs = m_model.GetModelInputs();
    m_origLatches = m_model.GetModelLatches();
    m_origInputSet.clear();
    m_origLatchSet.clear();
    for (int v : m_origInputs) m_origInputSet.emplace(abs(v));
    for (int v : m_origLatches) m_origLatchSet.emplace(abs(v));

    // save = input
    m_save = m_model.NewInputVar();

    std::vector<int> latches = m_origLatches;
    m_statecopy.clear();
    m_statecopy.reserve(latches.size());

    for (int v : latches) {
        int nv = m_model.NewLatchVar();
        m_statecopy.emplace_back(nv);

        // Init(nv) = Init(v)
        // Next(nv) = if (save) then v else nv
        int init = m_model.GetLatchReset(v);
        int next = m_model.MakeIte(m_save, v, nv);
        m_model.SetLatchReset(nv, init);
        m_model.SetLatchNext(nv, next);
    }

    // Init(triggered) = false
    // Next(triggered) = justice | ( !save & triggered)
    int justice = m_model.GetBad();
    int triggered = m_model.NewLatchVar();
    int next_triggered = m_model.MakeOr(justice, m_model.MakeAnd(-m_save, triggered));
    m_model.SetLatchReset(triggered, -m_model.TrueId());
    m_model.SetLatchNext(triggered, next_triggered);

    // equality = Next(v_i) <-> Next(nv_i) & ...
    int equality = m_model.TrueId();
    for (size_t i = 0; i < latches.size(); i++) {
        int v = latches[i];
        int nv = m_statecopy[i];
        int next_v = m_model.GetLatchNext(v);
        int next_nv = m_model.GetLatchNext(nv);
        int iff = m_model.MakeXnor(next_v, next_nv);
        equality = m_model.MakeAnd(equality, iff);
    }

    // bad = equality && triggered
    int bad = m_model.MakeAnd(equality, next_triggered);

    m_model.SetBad(bad);
    m_model.Rebuild();
}

} // namespace car
