#ifndef BCAR_H
#define BCAR_H

#include "BaseAlg.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"
#include <algorithm>
#include <assert.h>
#include <memory>
#include <random>
#include <string>


namespace car {

class BCAR : public BaseAlg {
  public:
    BCAR(Settings settings,
         Model &model,
         Log &log);
    CheckResult Run() override;
    bool SupportsWitness() const override { return true; }
    void RefineWitnessPropertyLit(WitnessBuilder &builder) const override;

    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;

  private:
    bool Check();

    void Init();

    void InitializeStartSolver();

    bool AddUnsatisfiableCore(const Cube &uc, int frameLevel, bool fromCTR = false);

    enum class ALLProveStatus {
        Proved,
        Reachable,
        Bailout,
        Invalidated,
    };

    void ActiveLemmaLearning(OverSequenceSet::RefId newRef);

    std::vector<OverSequenceSet::RefId> FindHotSpots(const std::vector<OverSequenceSet::RefId> &ancestorChain);

    ALLProveStatus ActiveProve(OverSequenceSet::RefId targetRef);

    bool ImmediateSatisfiable();

    void InitializeBadSolver();

    int GetNewLevel(const Cube &states, int start = 0);

    bool IsInvariant(int frameLevel);

    struct LitOrder {
        shared_ptr<Branching> branching;

        LitOrder() {}

        bool operator()(Lit l1, Lit l2) const {
            return (branching->PriorityOf(l1) > branching->PriorityOf(l2));
        }
    } m_litOrder;

    struct InnOrder {
        Model &m;

        explicit InnOrder(Model &model) : m(model) {}

        bool operator()(Lit inn1, Lit inn2) const {
            return (m.GetInnardslvl(inn1) > m.GetInnardslvl(inn2));
        }
    } m_innOrder;

    void OrderAssumption(Cube &uc) {
        if (m_settings.randomSeed > 0) {
            shuffle(uc.begin(), uc.end(), default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        stable_sort(uc.begin(), uc.end(), m_litOrder);
        if (m_settings.internalSignals) {
            stable_sort(uc.begin(), uc.end(), m_innOrder);
        }
    }

    inline void GetPrimed(Cube &p) {
        for (auto &x : p) {
            x = m_model.LookupPrime(x);
        }
    }

    void Generalize(Cube &uc, int frameLvl, int recLvl = 0);

    bool Down(Cube &uc, int frameLvl, int recLvl, vector<Cube> &failedCtses);

    bool ExCTGBlock(shared_ptr<State> cts, int frameLvl, int recLvl, vector<Cube> &failedCtses, int blockLimit);

    bool DownHasFailed(const Cube &s, const vector<Cube> &failedCtses);

    bool Propagate(const Cube &c, int lvl);

    int PropagateUp(const Cube &c, int lvl);

    bool CheckBad(shared_ptr<State> s);

    void AddConstraintOr(const Frame &f);

    Lit AddConstraintAnd(const Frame &f);

    bool IsReachable(int lvl, const Cube &assumption, const string &label);

    Cube GetUnsatAssumption(shared_ptr<SATSolver> solver, const Cube &assumptions);

    shared_ptr<State> EnumerateStartState();

    void OverSequenceRefine(int lvl);

    void BuildCEXTrace();

    CheckResult m_checkResult;
    int m_minUpdateLevel;
    int m_k;
    shared_ptr<Branching> m_branching;
    shared_ptr<OverSequenceSet> m_overSequence;
    UnderSequence m_underSequence;
    Settings m_settings;
    Log &m_log;
    Model &m_model;
    vector<shared_ptr<SATSolver>> m_transSolvers;
    shared_ptr<SATSolver> m_startSolver;
    shared_ptr<SATSolver> m_badSolver;
    shared_ptr<SATSolver> m_invSolver;
    vector<shared_ptr<vector<int>>> m_rotation;
    shared_ptr<State> m_lastState;
    std::shared_ptr<Restart> m_restart;
    unordered_set<Lit, LitHash> m_conflictScratch;

    std::vector<std::pair<Cube, Cube>> m_cexTrace;
};

} // namespace car

#endif
