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
    std::shared_ptr<std::vector<int>> GetUnsatisfiableCoreFromBad(int badId) override;
    void AddClause(const std::vector<int> &clause) override;
    void AddUnsatisfiableCore(const std::vector<int> &clause, int frameLevel) override;
    std::shared_ptr<std::vector<int>> GetUnsatisfiableCore() override;
    std::shared_ptr<cube> Getuc(bool minimal);
    void Getmuc(LSet &ass);
    void AddNewFrame(const std::vector<std::shared_ptr<std::vector<int>>> &frame, int frameLevel) override;
    bool SolveWithAssumptionAndBad(std::vector<int> &assumption, int badId) override;
    bool SolveWithAssumption() override;
    inline void AddAssumption(int id) override { m_assumptions.push(GetLit(id)); }
    bool SolveWithAssumption(std::vector<int> &assumption, int frameLevel) override;
    int get_temp_flag();
    void add_temp_clause(std::vector<int> *cls, int temp_flag, bool is_primed);
    void release_temp_cls(int temp_flag);
    std::shared_ptr<std::vector<int>> justGetUC();
    void clean_assumptions();
    std::string ShowLatest5Clause();

    std::pair<std::shared_ptr<std::vector<int>>, std::shared_ptr<std::vector<int>>> GetAssignment(std::ofstream &out) override;

    std::pair<std::shared_ptr<std::vector<int>>, std::shared_ptr<std::vector<int>>> GetAssignment() override;

    void AddConstraintOr(const std::vector<std::shared_ptr<std::vector<int>>> frame);

    void AddConstraintAnd(const std::vector<std::shared_ptr<std::vector<int>>> frame);

    void FlipLastConstrain();

    std::shared_ptr<std::vector<int>> GetModel();

  protected:
    static bool cmp(int a, int b) {
        return abs(a) < abs(b);
    }
    inline int GetLiteralId(const Lit &l);
    inline int GetFrameFlag(int frameLevel);

    inline Lit GetLit(int id) {
        if (id == 0) {
            // placeholder
        }
        int var = abs(id) - 1;
        while (var >= nVars()) newVar();
        return ((id > 0) ? mkLit(var) : ~mkLit(var));
    };

    inline int GetNewVar() { return m_maxFlag++; }

    bool m_isForward = false;
    int m_maxFlag;
    std::shared_ptr<AigerModel> m_model;
    std::vector<int> m_frameFlags;
    vec<Lit> m_assumptions;
};

} // namespace car

#endif