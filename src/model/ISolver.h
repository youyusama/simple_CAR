#ifndef ISOLVER_H
#define ISOLVER_H

#include "State.h"
#include <fstream>
#include <memory>
#include <vector>

namespace car {

class ISolver {
  public:
    virtual void AddClause(const clause &cls) = 0;
    virtual void AddUnsatisfiableCore(const clause &cls, int frameLevel) = 0;
    virtual void GetUnsatisfiableCore(shared_ptr<cube> uc) = 0;
    virtual void GetUnsatisfiableCoreFromBad(shared_ptr<cube> uc, int badId) = 0;
    virtual void AddNewFrame(const vector<shared_ptr<cube>> &frame, int frameLevel) = 0;
    virtual void AddAssumption(int id) = 0;
    virtual bool SolveWithAssumption() = 0;
    virtual bool SolveWithAssumption(const shared_ptr<cube> assumption, int frameLevel) = 0;
    virtual bool SolveWithAssumptionAndBad(const shared_ptr<cube> assumption, int badId) = 0;
    virtual pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment() = 0;
    virtual void AddConstraintOr(const vector<shared_ptr<cube>> frame) = 0;
    virtual void AddConstraintAnd(const vector<shared_ptr<cube>> frame) = 0;
    virtual void FlipLastConstrain() = 0;

  private:
};

} // namespace car


#endif
