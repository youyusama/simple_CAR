#ifndef CARSOLVER_H
#define CARSOLVER_H

#include "../sat/minisat/core/Solver.h"
#include "AigerModel.h"
#include "ISolver.h"
#include <memory>

using namespace Minisat;

namespace car {

class CarSolver : public ISolver, public Minisat::Solver {
  public:
    CarSolver();
    ~CarSolver();

    void AddClause(const clause &cls) override;
    void AddUnsatisfiableCore(const cube &uc, int frameLevel) override;
    void GetUnsatisfiableCore(shared_ptr<cube> uc) override;
    void GetUnsatisfiableCoreFromBad(shared_ptr<cube> uc, int badId) override;
    void AddNewFrame(const vector<shared_ptr<cube>> &frame, int frameLevel) override;
    inline void AddAssumption(int id) override { m_assumptions.push(GetLit(id)); }
    bool SolveWithAssumption() override;
    bool SolveWithAssumption(const shared_ptr<cube> assumption, int frameLevel) override;
    bool SolveWithAssumptionAndBad(const shared_ptr<cube> assumption, int badId) override;
    pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment() override;
    void AddConstraintOr(const vector<shared_ptr<cube>> frame) override;
    void AddConstraintAnd(const vector<shared_ptr<cube>> frame) override;
    void FlipLastConstrain() override;

    int GetTempFlag();

    void AddTempClause(clause *cls, int temp_flag, bool is_primed);

    void ReleaseTempClause(int temp_flag);

    shared_ptr<cube> justGetUC();

    void CleanAssumptions();

    shared_ptr<cube> Getuc(bool minimal);

    void Getmuc(LSet &ass);

    shared_ptr<vector<int>> GetModel();

  protected:
    inline int GetLiteralId(const Lit &l);
    inline int GetFrameFlag(int frameLevel);
    inline Lit GetLit(int id) {
        int var = abs(id) - 1;
        while (var >= nVars()) newVar();
        return ((id > 0) ? mkLit(var) : ~mkLit(var));
    };
    inline int GetNewVar() { return (++*m_maxId); }

    bool m_isForward = false;
    shared_ptr<int> m_maxId;
    shared_ptr<AigerModel> m_model;
    vector<int> m_frameFlags;
    vec<Lit> m_assumptions;
};

} // namespace car

#endif