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
    virtual shared_ptr<cube> GetUnsatisfiableCore() = 0;
    virtual shared_ptr<cube> GetUnsatisfiableCoreFromBad(int badId) = 0;
    virtual void AddNewFrame(const vector<shared_ptr<cube>> &frame, int frameLevel) = 0;
    inline virtual void AddAssumption(int id) = 0;
    virtual bool SolveWithAssumption() = 0;
    virtual bool SolveWithAssumption(cube &assumption, int frameLevel) = 0;
    virtual bool SolveWithAssumptionAndBad(cube &assumption, int badId) = 0;
    virtual pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment() = 0;
    virtual void AddConstraintOr(const vector<shared_ptr<cube>> frame) = 0;
    virtual void AddConstraintAnd(const vector<shared_ptr<cube>> frame) = 0;
    virtual void FlipLastConstrain() = 0;

  private:
};

} // namespace car


#endif
