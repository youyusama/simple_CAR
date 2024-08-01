#include "InvSolver.h"


namespace car {
InvSolver::InvSolver(std::shared_ptr<AigerModel> model) {
    m_model = model;
    m_maxFlag = model->GetMaxId() + 1;
    auto &clauses = m_model->GetClauses();
    for (int i = 0, end = model->GetOutputsStart(); i < end; i++) {
        AddClause(clauses[i]);
    }
}

} // namespace car