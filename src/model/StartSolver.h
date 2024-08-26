#ifndef STARTSOLVER_H
#define STARTSOLVER_H

#include "../sat/minisat/core/Solver.h"
#include "AigerModel.h"
#ifdef CADICAL
#include "CadicalSolver.h"
#else
#include "MinisatSolver.h"
#endif
#include <memory>
namespace car {

class StartSolver :
#ifdef CADICAL
    public CadicalSolver
#else
    public MinisatSolver
#endif
{
  public:
    StartSolver(shared_ptr<AigerModel> model) {
        m_model = model;
        m_maxId = model->GetMaxId();
        m_flag = -1;
        auto &clauses = m_model->GetClauses();
        for (int i = 0; i < model->GetLatchesStart(); ++i) {
            AddClause(clauses[i]);
        }
        shared_ptr<cube> assumption(new cube());
        for (auto c : m_model->GetConstraints()) {
            assumption->push_back(c);
        }
        assumption->push_back(m_model->GetBad());
        AddAssumption(assumption);
    }

    ~StartSolver() {}

    pair<shared_ptr<cube>, shared_ptr<cube>> GetStartPair() {
        return GetAssignment(false);
    }

    void UpdateStartSolverFlag() {
#ifdef CADICAL
        if (m_flag == -1) {
            m_flag = GetNewVar();
            m_assumptions->push_back(m_flag);
        } else {
            m_assumptions->pop_back();
            m_assumptions->push_back(-m_flag);
            m_flag = GetNewVar();
            m_assumptions->push_back(m_flag);
        }
#else
        if (m_flag == -1) {
            m_flag = GetNewVar();
            m_assumptions.push(GetLit(m_flag));
        } else {
            m_assumptions.pop();
            m_assumptions.push(GetLit(-m_flag));
            m_flag = GetNewVar();
            m_assumptions.push(GetLit(m_flag));
        }
#endif
    }

    inline int GetFlag() { return m_flag; }
    int m_flag;

}; // class StartSolver


} // namespace car


#endif