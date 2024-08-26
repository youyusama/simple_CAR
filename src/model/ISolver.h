#ifndef ISOLVER_H
#define ISOLVER_H

using namespace std;
#include <memory>
#include <vector>

namespace car {

class ISolver {
  public:
    virtual void AddClause(const vector<int> &cls) = 0;
    virtual void AddAssumption(const shared_ptr<vector<int>> assumption) = 0;
    virtual bool Solve() = 0;
    virtual bool Solve(const shared_ptr<vector<int>> assumption) = 0;
    virtual pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> GetAssignment(bool prime) = 0;
    virtual shared_ptr<vector<int>> GetUC(bool prime) = 0;
    virtual int GetNewVar() = 0;
    virtual void AddTempClause(const vector<int> &cls) = 0;
    virtual void ReleaseTempClause() = 0;
    virtual bool GetModel(int id) = 0;

  private:
};

} // namespace car


#endif
