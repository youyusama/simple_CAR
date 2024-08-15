#ifndef STARTSOLVER_H
#define STARTSOLVER_H

#include "../sat/minisat/core/Solver.h"
#include "AigerModel.h"
#include "CarSolver.h"
#include <memory>
namespace car {

class StartSolver : public CarSolver {
  public:
    StartSolver(shared_ptr<AigerModel> model) {
        m_model = model;
        m_maxId = model->GetMaxId();
        auto &clauses = m_model->GetClauses();
        for (int i = 0; i < model->GetLatchesStart(); ++i) {
            CarSolver::AddClause(clauses[i]);
        }
        m_assumptions.push(GetLit(m_model->GetBad()));
    }

    ~StartSolver() {}

    pair<shared_ptr<cube>, shared_ptr<cube>> GetStartPair() {
        assert(m_model->GetNumInputs() < nVars());
        shared_ptr<cube> inputs(new cube());
        shared_ptr<cube> latches(new cube());
        inputs->reserve(m_model->GetNumInputs());
        latches->reserve(m_model->GetNumLatches());
        for (int i = 0; i < m_model->GetNumInputs(); ++i) {
            if (model[i] == l_True) {
                inputs->emplace_back(i + 1);
            } else {
                assert(model[i] == l_False);
                inputs->emplace_back(-i - 1);
            }
        }
        for (int i = m_model->GetNumInputs(), end = m_model->GetNumInputs() + m_model->GetNumLatches(); i < end; ++i) {
            int val;
            if (model[i] == l_True) {
                val = i + 1;
            } else {
                assert(model[i] == l_False);
                val = -i - 1;
            }
            if (abs(val) <= end) {
                latches->push_back(val);
            }
        }
        return pair<shared_ptr<cube>, shared_ptr<cube>>(inputs, latches);
    }

    void UpdateStartSolverFlag() {
        if (m_assumptions.size() <= 1) {
            m_flag = GetNewVar();
            m_assumptions.push(GetLit(m_flag));
        } else {
            m_assumptions.pop();
            m_assumptions.push(GetLit(-m_flag));
            m_flag = GetNewVar();
            m_assumptions.push(GetLit(m_flag));
        }
    }

    void AddClause(int flag, vector<int> &clause) {
        vec<Lit> literals;
        literals.push(GetLit(flag));
        for (int i = 0; i < clause.size(); ++i) {
            literals.push(GetLit(-clause[i]));
        }
        bool result = addClause(literals);
        assert(result != false);
    }

    inline int GetFlag() { return m_flag; }
    int m_flag;

}; // class StartSolver


} // namespace car


#endif