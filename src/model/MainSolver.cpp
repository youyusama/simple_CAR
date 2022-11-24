#include "MainSolver.h"

namespace car {
MainSolver::MainSolver(std::shared_ptr<AigerModel> model, bool isForward, bool by_sslv) {
  m_isForward = isForward;
  m_model = model;
  m_maxFlag = model->GetMaxId() + 1;
  if (by_sslv) {
    sptr<SimpSolver> sslv = m_model->get_sslv();
    while (nVars() < sslv->nVars()) newVar();
    for (auto c = sslv->clausesBegin(); c != sslv->clausesEnd(); ++c) {
      const Clause &cls = *c;
      vec<Lit> cls_;
      for (int i = 0; i < cls.size(); ++i) {
        cls_.push(cls[i]);
        // std::cout << ((cls[i].x % 2) ? (-cls[i].x / 2) : (cls[i].x / 2)) << " | ";
      }
      // std::cout << std::endl;
      addClause_(cls_);
    }
    for (auto c = sslv->trailBegin(); c != sslv->trailEnd(); ++c)
      addClause(*c);
    std::cout << std::endl;
  } else {
    auto &clause = m_model->GetClause();
    for (int i = 0; i < clause.size(); ++i) {
      // for (auto j : clause[i]) {
      //   std::cout << j << " | ";
      // }
      // std::cout << std::endl;
      AddClause(clause[i]);
    }
  }
}

void MainSolver::add_negation_bad() {
  AddClause(m_model->GetNegBad());
}

} // namespace car