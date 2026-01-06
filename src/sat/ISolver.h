#ifndef ISOLVER_H
#define ISOLVER_H

using namespace std;
#include <memory>
#include <unordered_set>
#include <vector>
typedef vector<int> cube;

namespace car {

class ISolver {
  public:
    virtual void AddClause(const cube &cls) = 0;
    virtual bool Solve() = 0;
    virtual bool Solve(const shared_ptr<cube> assumption) = 0;
    virtual pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment(bool prime) = 0;
    virtual unordered_set<int> GetConflict() = 0;
    virtual int GetNewVar() = 0;
    virtual void AddTempClause(const cube &cls) = 0;
    virtual void ReleaseTempClause() = 0;
    virtual bool GetModel(int id) = 0;
    virtual void AddAssumption(const shared_ptr<cube> assumption) = 0;
    virtual void ClearAssumption() = 0;
    virtual void PushAssumption(int a) = 0;
    virtual int PopAssumption() = 0;
    // special interface in minicore
    virtual void SetSolveInDomain() = 0;
    virtual void SetDomain(const shared_ptr<cube> domain) = 0;
    virtual void SetTempDomain(const shared_ptr<cube> domain) = 0;
    virtual void ResetTempDomain() = 0;

  private:
};

} // namespace car


#endif
