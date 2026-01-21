#ifndef ISOLVER_H
#define ISOLVER_H

#include "TernarySim.h"
#include <memory>
#include <unordered_set>
#include <vector>

using namespace std;
typedef vector<int> cube;

namespace car {

class ISolver {
  public:
    virtual void AddClause(const cube &cls) = 0;
    virtual bool Solve() = 0;
    virtual bool Solve(const cube &assumption) = 0;
    virtual pair<cube, cube> GetAssignment(bool prime) = 0;
    virtual unordered_set<int> GetConflict() = 0;
    virtual int GetNewVar() = 0;
    virtual void AddTempClause(const cube &cls) = 0;
    virtual void ReleaseTempClause() = 0;
    virtual tbool GetModel(int id) = 0;
    virtual void AddAssumption(const cube &assumption) = 0;
    virtual void ClearAssumption() = 0;
    virtual void PushAssumption(int a) = 0;
    virtual int PopAssumption() = 0;

  private:
};

} // namespace car


#endif
