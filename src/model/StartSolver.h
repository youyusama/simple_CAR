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
        m_maxFlag = model->GetMaxId() + 1;
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
            m_assumptions.push(GetLit(m_maxFlag));
        } else {
            m_assumptions.pop();
            m_assumptions.push(GetLit(-m_maxFlag));
            m_assumptions.push(GetLit(++m_maxFlag));
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

    inline int GetFlag() { return m_maxFlag; }

}; // class StartSolver


} // namespace car


#endif