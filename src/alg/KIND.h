#ifndef KIND_H
#define KIND_H

#include "BaseAlg.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace car {

class KIND : public BaseAlg {
  public:
    KIND(Settings settings,
         Model &model,
         Log &log);

    CheckResult Run() override;
    void Witness() override;
    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;

  private:
    struct DiffKey {
        int i;
        int j;
        Var latch;

        bool operator==(const DiffKey &other) const {
            return i == other.i && j == other.j && latch == other.latch;
        }
    };

    struct DiffKeyHash {
        size_t operator()(const DiffKey &key) const {
            size_t h1 = std::hash<int>{}(key.i);
            size_t h2 = std::hash<int>{}(key.j);
            size_t h3 = std::hash<Var>{}(key.latch);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };

    struct StatePairKey {
        int i;
        int j;

        bool operator==(const StatePairKey &other) const {
            return i == other.i && j == other.j;
        }
    };

    struct StatePairKeyHash {
        size_t operator()(const StatePairKey &key) const {
            size_t h1 = std::hash<int>{}(key.i);
            size_t h2 = std::hash<int>{}(key.j);
            return h1 ^ (h2 << 1);
        }
    };

    enum class AuxMode {
        Induction,
        Forward
    };

    void Init();
    void InitBaseSolver();
    void InitAuxSolver();

    CheckResult Check();
    CheckResult CheckNonIncremental();
    bool CheckBaseCase();
    bool CheckInductiveStep();
    bool CheckForwardCondition();
    bool CheckBaseCaseNonIncremental(int k);
    bool CheckInductiveStepNonIncremental(int k);
    bool CheckForwardConditionNonIncremental(int k);

    void AdvanceBaseToNextK();
    void AdvanceInductionToNextK();
    void AdvanceForwardToNextK();

    void AddClausesK(std::shared_ptr<SATSolver> solver, int k);
    void AddConstraintsK(std::shared_ptr<SATSolver> solver, int k);
    void AddInitial(std::shared_ptr<SATSolver> solver);
    void AddUniqueConstraintsK(std::shared_ptr<SATSolver> solver, int k);
    void AddStateDisequality(std::shared_ptr<SATSolver> solver, int i, int j);
    void AddStateDisequalities(std::shared_ptr<SATSolver> solver,
                               const std::vector<StatePairKey> &pairs);
    void ReplayBlockedPairsNonIncremental(std::shared_ptr<SATSolver> solver, int k);
    std::vector<StatePairKey> FindEqualStatePairs(const std::shared_ptr<SATSolver> &solver, int k) const;
    Var GetDiffVar(int i, int j, Var latch);

    const std::vector<Clause> &GetClausesKCached(int k);
    const Cube &GetConstraintsKCached(int k);
    void GetClausesK(int k, std::vector<Clause> &clauses);
    Lit GetBadK(int k);
    Cube GetConstraintsK(int k);

    void OutputCounterExample();

    Settings m_settings;
    Log &m_log;
    Model &m_model;
    int m_k;
    int m_maxK;
    int m_baseAddedTransitions;
    int m_auxAddedTransitions;
    int m_auxAddedUniqueLevel;
    CheckResult m_checkResult;
    AuxMode m_auxMode;
    bool m_useUnrollingCache;
    std::shared_ptr<SATSolver> m_baseSolver;
    std::shared_ptr<SATSolver> m_auxSolver;
    std::shared_ptr<SATSolver> m_cexSolver;
    std::unordered_map<DiffKey, Var, DiffKeyHash> m_diffVars;
    std::vector<std::vector<Clause>> m_transitionClausesCache;
    std::vector<char> m_transitionClausesReady;
    std::vector<Cube> m_constraintsCache;
    std::vector<char> m_constraintsReady;
    std::unordered_set<StatePairKey, StatePairKeyHash> m_nonIncIndBlockedPairs;
};

} // namespace car

#endif
