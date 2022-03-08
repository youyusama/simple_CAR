#include "CarSolver_cadical.h"
#include <algorithm>

namespace  car
{
	CarSolver_cadical::CarSolver_cadical() {
    cadical_solver = new CaDiCaL::Solver;
  }

	CarSolver_cadical::~CarSolver_cadical()
	{
		delete cadical_solver;
	}

    // something about solve()
    // Try to solve the current formula.  Returns
    //
    //    0 = UNSOLVED     (limit reached or interrupted through 'terminate')
    //   10 = SATISFIABLE
    //   20 = UNSATISFIABLE

	bool CarSolver_cadical::SolveWithAssumption()
	{
    addAssumptionsToSolver();
    int res = cadical_solver->solve();
    if (res == 10){
      return true;
		}else if(res == 20){
      return false;
    }else{
      //placeholder
    }
	}

  bool CarSolver_cadical::SolveWithAssumption(std::vector<int>& assumption, int frameLevel)
  {
    assumptions.clear();
    AddAssumption(GetFrameFlag(frameLevel));
    for(int assu : assumption){
      assumptions.push_back(assu);
    }
    if (SolveWithAssumption()) return true;
		else return false;
  }


	void CarSolver_cadical::AddClause(const std::vector<int>& clause)
  {
    for (int lit : clause){
      cadical_solver->add(lit);
    }
    cadical_solver->add(0);
  }


	void CarSolver_cadical::AddUnsatisfiableCore(const std::vector<int>& clause, int frameLevel)
	{
    int flag = GetFrameFlag(frameLevel);
		cadical_solver->add(-flag);
    if (m_isForward){
      for (int lit : clause){
        cadical_solver->add(-lit);
      }
    }else{
      for (int lit : clause){
        cadical_solver->add(-m_model->GetPrime(lit));
      }
    }
    cadical_solver->add(0);
	}


  std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > > CarSolver_cadical::GetAssignment(std::ofstream& out)
	{
		out<<"GetAssignment:"<<std::endl;
    int model_inputs_num = m_model->GetNumInputs();
    int model_latches_num = m_model->GetNumLatches();
		assert(model_inputs_num < cadical_solver->vars());
		std::shared_ptr<std::vector<int>> inputs(new std::vector<int>());
		std::shared_ptr<std::vector<int>> latches(new std::vector<int>());
		inputs->reserve(model_inputs_num);
		latches->reserve(model_latches_num);

		for (int i = 0; i < model_inputs_num; ++i){
      if (cadical_solver->val(i+1)>0) inputs->emplace_back(i+1);
      else inputs->emplace_back(-i-1);
		}
		for (int i = model_inputs_num, end = model_inputs_num + model_latches_num; i < end; ++i)
		{
			if (m_isForward)
			{
        if (cadical_solver->val(i+1)>0) latches->emplace_back(i+1);
        else latches->emplace_back(-i-1);
			}
			else
			{
				int p = m_model->GetPrime(i+1);
        int val = cadical_solver->val(abs(p));
        if ((val>0 && p>0) || (val<0 && p<0)) latches->emplace_back(i+1);
        else latches->emplace_back(-i-1);
			}
		}
		//
		for (auto it = inputs->begin(); it != inputs->end(); ++it)
		{
			out<<*it<<" ";
		}
		for (auto it = latches->begin(); it != latches->end(); ++it)
		{
			out<<*it<<" ";
		}
		out<<std::endl;

		return std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > >(inputs, latches);
	}


	std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > > CarSolver_cadical::GetAssignment()
	{
		int model_inputs_num = m_model->GetNumInputs();
    int model_latches_num = m_model->GetNumLatches();
		assert(model_inputs_num < cadical_solver->vars());
		std::shared_ptr<std::vector<int>> inputs(new std::vector<int>());
		std::shared_ptr<std::vector<int>> latches(new std::vector<int>());
		inputs->reserve(model_inputs_num);
		latches->reserve(model_latches_num);

		for (int i = 0; i < model_inputs_num; ++i){
      if (cadical_solver->val(i+1)>0) inputs->emplace_back(i+1);
      else inputs->emplace_back(-i-1);
		}
		for (int i = model_inputs_num, end = model_inputs_num + model_latches_num; i < end; ++i)
		{
			if (m_isForward)
			{
        if (cadical_solver->val(i+1)>0) latches->emplace_back(i+1);
        else latches->emplace_back(-i-1);
			}
			else
			{
				int p = m_model->GetPrime(i+1);
        int val = cadical_solver->val(abs(p));
        if ((val>0 && p>0) || (val<0 && p<0)) latches->emplace_back(i+1);
        else latches->emplace_back(-i-1);
			}
		}
		return std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > >(inputs, latches);
	}


  std::shared_ptr<std::vector<int> > CarSolver_cadical:: GetUnsatisfiableCoreFromBad(int badId)
	{
		std::shared_ptr<std::vector<int> > uc(new std::vector<int>());
    getConflicts();
		uc->reserve(conflicts.size());
		int val;
		for (int i = 0; i < conflicts.size(); ++i)
		{
			val = conflicts[i];
			if (m_model->IsLatch(val) && val != badId)
			{
				uc->emplace_back(val);
			}
		}
		std::sort(uc->begin(), uc->end(), cmp);
		return uc;
	}

	std::shared_ptr<std::vector<int> > CarSolver_cadical::GetUnsatisfiableCore()
	{
		std::shared_ptr<std::vector<int>> uc(new std::vector<int>());
    getConflicts();
		uc->reserve(conflicts.size());
		int val;
		if (m_isForward)
		{
			for (int i = 0; i < conflicts.size(); ++i)
			{
				val = conflicts[i];
 				std::vector<int> ids = m_model->GetPrevious(val);
				if (val > 0)
				{
 					for(auto x: ids)
					{
						uc->push_back(x);
					}
				}
				else
				{
					for(auto x:ids)
					{
						uc->push_back(-x);
					}  
				}
			}
		}
		else
		{
			for (int i = 0; i < conflicts.size(); ++i)
			{
				val = conflicts[i];
				if (m_model->IsLatch(val))
				{
					uc->emplace_back(val);
				}
			}
		}
		
		std::sort(uc->begin(), uc->end(), cmp);
		return uc;
	}


	void CarSolver_cadical::AddNewFrame(const std::vector<std::shared_ptr<std::vector<int> > >& frame, int frameLevel)
	{
 		for (int i = 0; i < frame.size(); ++i)
		{
			AddUnsatisfiableCore(*frame[i], frameLevel);
		}
	}

  bool CarSolver_cadical::SolveWithAssumptionAndBad(std::vector<int>& assumption, int badId)
	{
    assumptions.clear();
    assumptions.push_back(badId);
    for (int assu : assumption){
      assumptions.push_back(assu);
    }
    if (SolveWithAssumption()) return true;
		else return false;
	}

	inline void CarSolver_cadical::AddConstraintOr(const std::vector<std::shared_ptr<std::vector<int> > > frame)
	{
		std::vector<int> clause;
		for (int i = 0; i < frame.size(); ++i)
		{
			int flag = GetNewVar();
			clause.push_back(flag);
			for (int j = 0; j < frame[i]->size(); ++j)
			{
				AddClause(std::vector<int> {-flag, (*frame[i])[j]});
			}
		}
		AddClause(clause);
	}

	inline void CarSolver_cadical::AddConstraintAnd(const std::vector<std::shared_ptr<std::vector<int> > > frame)
	{
		int flag = GetNewVar();
		for (int i = 0; i < frame.size(); ++i)
		{
			std::vector<int> clause;
			for (int j = 0; j < frame[i]->size(); ++j)
			{
				clause.push_back(-(*frame[i])[j]);
			}
			clause.push_back(-flag);
			AddClause(clause);
		}
		AddAssumption(flag);
	}


	inline void CarSolver_cadical::FlipLastConstrain()
	{
    int lit = assumptions.back();
    assumptions.pop_back();
    assumptions.push_back(-lit);
	}


#pragma region private

  void CarSolver_cadical::addAssumptionsToSolver(){
    for (int assu : assumptions){
      cadical_solver->assume(assu);
    }
  }

  
  void CarSolver_cadical::getConflicts(){
    conflicts.clear();
    assert(assumptions.size()>0);
    conflicts = cadical_solver->get_conflict(assumptions);
  }


	inline int CarSolver_cadical::GetFrameFlag(int frameLevel)
	{
		if (frameLevel < 0)
		{
			//placeholder
		}
		while (m_frameFlags.size() <= frameLevel)
		{
			m_frameFlags.emplace_back(m_maxFlag++);
		}
		return m_frameFlags[frameLevel];
	}

#pragma endregion


}//namespace car