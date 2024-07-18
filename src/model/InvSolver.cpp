#include "InvSolver.h"


namespace car {
InvSolver::InvSolver(std::shared_ptr<AigerModel> model) {
    m_model = model;
    m_maxId = model->GetMaxId();
    auto &clause = m_model->GetClause();
    for (int i = 0, end = model->GetOutputsStart(); i < end; i++) {
        AddClause(clause[i]);
    }
}

} // namespace car