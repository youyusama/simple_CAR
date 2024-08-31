#include "MainSolver.h"
#include <fstream>

namespace car {
MainSolver::MainSolver(shared_ptr<AigerModel> model) {
    m_model = model;
    m_maxId = model->GetMaxId();
#ifdef CADICAL
    auto &clauses = m_model->GetClauses();
    for (int i = 0; i < clauses.size(); ++i) {
        AddClause(clauses[i]);
    }
#else
    shared_ptr<SimpSolver> sslv = m_model->GetSimpSolver();
    while (nVars() < sslv->nVars()) newVar();
    for (auto c = sslv->clausesBegin(); c != sslv->clausesEnd(); ++c) {
        const Clause &cls = *c;
        vec<Lit> cls_;
        for (int i = 0; i < cls.size(); ++i) {
            cls_.push(cls[i]);
        }
        addClause_(cls_);
    }
    for (auto c = sslv->trailBegin(); c != sslv->trailEnd(); ++c)
        addClause(*c);
#endif
}


bool MainSolver::SolveFrame(const shared_ptr<cube> assumption, int frameLevel) {
#ifdef CADICAL
    m_assumptions->clear();
    m_assumptions->push_back(GetFrameFlag(frameLevel));
    m_assumptions->resize(assumption->size() + 1);
    std::copy(assumption->begin(), assumption->end(), m_assumptions->begin() + 1);
#else
    m_assumptions.clear();
    m_assumptions.push(GetLit(GetFrameFlag(frameLevel)));
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
#endif
    return Solve();
}


void MainSolver::AddUC(const cube &uc, int frameLevel) {
    int flag = GetFrameFlag(frameLevel);
    cube cls;
    cls.push_back(-flag);
    for (int i = 0; i < uc.size(); ++i) {
        cls.push_back(-uc[i]);
    }
    AddClause(cls);
}


void MainSolver::AddNegationBad() {
    AddClause(cube{m_model->GetProperty()});
}


inline int MainSolver::GetFrameFlag(int frameLevel) {
    assert(frameLevel >= 0);
    m_maxId = m_model->GetMaxId();
    while (m_frameFlags.size() <= frameLevel) {
        m_frameFlags.emplace_back(GetNewVar());
    }
    m_model->SetMaxId(m_maxId);
    return m_frameFlags[frameLevel];
}


} // namespace car