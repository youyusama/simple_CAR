#ifndef ISOLVER_H
#define ISOLVER_H

#include "CarTypes.h"
#include "TernarySim.h"
#include <memory>
#include <vector>

using namespace std;

namespace car {

class ISolver {
  public:
    virtual void AddClause(const Cube &cls) = 0;
    virtual bool Solve() = 0;
    virtual bool Solve(const Cube &assumption) = 0;
    virtual pair<Cube, Cube> GetAssignment(bool prime) = 0;
    virtual bool Failed(Lit assumption) = 0;
    virtual bool ShrinkConflict(int shrink) {
        (void)shrink;
        return false;
    }
    virtual Var GetNewVar() = 0;
    virtual void AddTempClause(const Cube &cls) = 0;
    virtual void ReleaseTempClause() = 0;
    virtual Tbool GetModel(Var id) = 0;

  private:
};

} // namespace car


#endif
