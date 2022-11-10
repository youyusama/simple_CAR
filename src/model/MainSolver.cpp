#include "MainSolver.h"

namespace car {
MainSolver::MainSolver(std::shared_ptr<AigerModel> model, bool isForward) {
  m_isForward = isForward;
  m_model = model;
  m_maxFlag = model->GetMaxId() + 1;
  auto &clause = m_model->GetClause();
  for (int i = 0; i < clause.size(); ++i) {
    // for (auto j : clause[i]) {
    //   std::cout << j << " | ";
    // }
    // std::cout << std::endl;
    AddClause(clause[i]);
  }
  AddClause(m_model->GetNegBad());
}

} // namespace car