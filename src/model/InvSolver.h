#ifndef INVSOLVER_H
#define INVSOLVER_H

#ifdef CADICAL
#include "CarSolver_cadical.h"
#else
#include "CarSolver.h"
#endif

namespace car {

class InvSolver : public CarSolver {
  public:
    InvSolver(std::shared_ptr<AigerModel> model) {
        m_model = model;
        m_maxId = model->GetMaxId();
        auto &clauses = m_model->GetClauses();
        for (int i = 0, end = model->GetOutputsStart(); i < end; i++) {
            AddClause(clauses[i]);
        }
        for (auto c : m_model->GetConstraints()) {
            AddClause(clause{c});
        }
    }

  private:
};

} // namespace car

#endif