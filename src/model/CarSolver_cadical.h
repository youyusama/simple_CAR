#ifndef CARSOLVER_CADICAL_H
#define CARSOLVER_CADICAL_H

#include "ISolver.h"
#include "cadical.hpp"
#include "AigerModel.h"
#include <memory>
#include <cassert>

namespace car
{

class CarSolver_cadical: public ISolver
{
public:
	CarSolver_cadical();
	~CarSolver_cadical();

  std::shared_ptr<std::vector<int> > GetUnsatisfiableCoreFromBad(int badId) override;
	void AddClause(const std::vector<int>& clause) override;
	void AddUnsatisfiableCore(const std::vector<int>& clause, int frameLevel) override;
	std::shared_ptr<std::vector<int> > GetUnsatisfiableCore() override;
	void AddNewFrame(const std::vector<std::shared_ptr<std::vector<int> > >& frame, int frameLevel) override;
	bool SolveWithAssumptionAndBad(std::vector<int>& assumption, int badId) override;
	bool SolveWithAssumption() override;
	inline void AddAssumption(int id) override {assumptions.push_back(id);}
	bool SolveWithAssumption(std::vector<int>& assumption, int frameLevel) override;

	std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > > GetAssignment(std::ofstream& out) override;

	std::pair<std::shared_ptr<std::vector<int> >, std::shared_ptr<std::vector<int> > > GetAssignment() override;

	inline void AddConstraintOr(const std::vector<std::shared_ptr<std::vector<int> > > frame);
	inline void AddConstraintAnd(const std::vector<std::shared_ptr<std::vector<int> > > frame);
	inline void FlipLastConstrain();

	std::shared_ptr<std::vector<int> > GetModel()
	{
		std::shared_ptr<std::vector<int>> res(new std::vector<int>());
		res->resize(cadical_solver->vars(), 0);
		for (int i=0; i<cadical_solver->vars(); i++){
			if (cadical_solver->val(i+1)>0)
				res->at(i) = i+1;
			else{
				res->at(i) = -(i+1);
			}
		}
   	return res;
	}
	
protected:
	CaDiCaL::Solver* cadical_solver;
	std::vector<int> assumptions;
	std::vector<int> conflicts;

	void getConflicts();
	void addAssumptionsToSolver();

	static bool cmp(int a, int b)
	{
		return abs(a) < abs(b);
	}
  
	inline int GetFrameFlag(int frameLevel);
	inline int GetNewVar() {return m_maxFlag++;}

	bool m_isForward = false;
	int m_maxFlag;
	std::shared_ptr<AigerModel> m_model;
	std::vector<int> m_frameFlags;
	
};

}//namespace car

#endif