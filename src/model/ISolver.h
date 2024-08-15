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
    virtual void AddAssumption(int id) = 0;
    virtual void AddAssumption(const shared_ptr<cube> assumption) = 0;
    virtual bool Solve() = 0;
    virtual bool Solve(const shared_ptr<cube> assumption) = 0;
    virtual bool Solve(const shared_ptr<cube> assumption, int frameLevel) = 0;
    virtual pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment(bool prime) = 0;
    virtual shared_ptr<cube> GetUC(bool prime) = 0;
    virtual void AddUC(const clause &cls, int frameLevel) = 0;
    virtual void AddConstraintOr(const vector<shared_ptr<cube>> frame) = 0;
    virtual void AddConstraintAnd(const vector<shared_ptr<cube>> frame) = 0;
    virtual void FlipLastConstrain() = 0;

  private:
};

} // namespace car


#endif
