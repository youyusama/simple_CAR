#ifndef ISOLVER_H
#define ISOLVER_H

#include "CarTypes.h"
#include "TernarySim.h"
#include <memory>
#include <unordered_set>
#include <vector>

using namespace std;

namespace car {

class ISolver {
  public:
    virtual void AddClause(const Cube &cls) = 0;
    virtual bool Solve() = 0;
    virtual bool Solve(const Cube &assumption) = 0;
    virtual pair<Cube, Cube> GetAssignment(bool prime) = 0;
    virtual void GetConflict(unordered_set<Lit, LitHash> &out) = 0;
    virtual bool ShrinkConflict(unordered_set<Lit, LitHash> &out, int shrink) {
        out.clear();
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
