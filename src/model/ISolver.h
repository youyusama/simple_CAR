#ifndef ISOLVER_H
#define ISOLVER_H

using namespace std;
#include <memory>
#include <vector>
typedef vector<int> cube;

namespace car {

class ISolver {
  public:
    virtual void AddClause(const cube &cls) = 0;
    virtual void AddAssumption(const shared_ptr<cube> assumption) = 0;
    virtual bool Solve() = 0;
    virtual bool Solve(const shared_ptr<cube> assumption) = 0;
    virtual pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment(bool prime) = 0;
    virtual shared_ptr<cube> GetUC(bool prime) = 0;
    virtual int GetNewVar() = 0;
    virtual void AddTempClause(const cube &cls) = 0;
    virtual void ReleaseTempClause() = 0;
    virtual bool GetModel(int id) = 0;

  private:
};

} // namespace car


#endif
