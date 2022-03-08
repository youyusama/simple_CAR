#ifndef ISOLVER_H
#define ISOLVER_H

#include <vector>
#include "State.h"
#include <fstream>
#include <memory>

namespace car
{

class ISolver
{
public:
	virtual std::shared_ptr<std::vector<int> > GetUnsatisfiableCoreFromBad(int badId) = 0;
	virtual void AddClause(const std::vector<int>& clause) = 0;
	virtual void AddUnsatisfiableCore(const std::vector<int>& clause, int frameLevel) = 0;
	virtual std::shared_ptr<std::vector<int> > GetUnsatisfiableCore() = 0;
	inline virtual void AddAssumption(int id) = 0;
	virtual void AddNewFrame(const std::vector<std::shared_ptr<std::vector<int> > >& frame, int frameLevel) = 0;
	virtual bool SolveWithAssumptionAndBad(std::vector<int>& assumption, int badId) = 0;
	virtual bool SolveWithAssumption() = 0;
	virtual bool SolveWithAssumption(std::vector<int>& assumption, int frameLevel) = 0;
	virtual std::pair<std::shared_ptr<std::vector<int> > , std::shared_ptr<std::vector<int> > > GetAssignment(std::ofstream& out) = 0;

	virtual std::pair<std::shared_ptr<std::vector<int> > , std::shared_ptr<std::vector<int> > > GetAssignment() = 0;

	inline virtual void AddConstraintOr(const std::vector<std::shared_ptr<std::vector<int> > > frame) = 0;
	inline virtual void AddConstraintAnd(const std::vector<std::shared_ptr<std::vector<int> > > frame) = 0;
	inline virtual void FlipLastConstrain() = 0;
	virtual ~ISolver(){};
private:
};

} //namespace car


#endif
