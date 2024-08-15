#include "MainSolver.h"
#include <fstream>

namespace car {
MainSolver::MainSolver(shared_ptr<AigerModel> model, bool isForward, bool by_sslv) {
    m_isForward = isForward;
    m_model = model;
    m_maxFlag = model->GetMaxId() + 1;
    if (by_sslv) {
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
    } else {
        auto &clauses = m_model->GetClauses();
        for (int i = 0; i < clauses.size(); ++i) {
            AddClause(clauses[i]);
        }
    }
}

void MainSolver::AddNegationBad() {
    AddClause(clause{m_model->GetProperty()});
}

} // namespace car