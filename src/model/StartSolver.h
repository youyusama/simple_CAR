#ifndef STARTSOLVER_H
#define STARTSOLVER_H

#include "CarSolver.h"
#include "Solver.h"
#include "AigerModel.h"
#include <memory>
namespace car
{

class StartSolver : public CarSolver
{
public:
    StartSolver(std::shared_ptr<AigerModel> model, int badId)
    {
        m_model = model;
        m_maxFlag = model->GetMaxId()+1;
		auto& clause = m_model->GetClause();
		for (int i = 0; i < model->GetLatchesStart(); ++i)
		{
			CarSolver::AddClause(clause[i]);
		}
        m_assumptions.push(GetLit(badId));
    }

    ~StartSolver(){;};

    std::shared_ptr<State> GetStartState()
    {
        assert(m_model->GetNumInputs() < nVars());
        std::shared_ptr<std::vector<int> > inputs(new std::vector<int>());
        std::shared_ptr<std::vector<int> > latches(new std::vector<int>());
        inputs->reserve(m_model->GetNumInputs());
        latches->reserve(m_model->GetNumLatches());
        for (int i = 0; i <m_model->GetNumInputs(); ++i)
        {
            if (model[i] == l_True)
            {
                inputs->emplace_back(i+1);
            }
            else if (model[i] == l_False)
            {
                inputs->emplace_back(-i-1);
            }
        }
        for (int i = m_model->GetNumInputs(), end = m_model->GetNumInputs() + m_model->GetNumLatches(); i < end; ++i)
        {
            int val;
            if (model[i] == l_True)
            {
                val = i+1;
            }
            else if (model[i] == l_False)
            {
                val = -i-1;
            }
            if (abs(val) <= end)
            {
                latches->push_back(val);
            }
        }

        std::shared_ptr<State> newState(new State(nullptr, inputs, latches, 0));
        return newState;
    }

    void UpdateStartSolverFlag()
    {
        if (m_assumptions.size() <= 1)
        {
            m_assumptions.push(GetLit(m_maxFlag));
        }
        else
        {
            m_assumptions.pop();
            m_assumptions.push(GetLit(-m_maxFlag));
            m_assumptions.push(GetLit(++m_maxFlag));
        }
    }

    void AddClause(int flag, std::vector<int>& clause)
    {
        vec<Lit> literals;
        literals.push(GetLit(flag));
        for (int i = 0; i < clause.size(); ++i)
        {
            literals.push(GetLit(-clause[i]));
        }
        bool result = addClause(literals);
        assert (result != false);
    }

    inline int GetFlag() {return m_maxFlag;}

};//class StartSolver


}//namespace car


#endif