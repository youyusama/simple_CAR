#pragma once

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
        std::shared_ptr<Branching> branching;

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
            std::shuffle(uc.begin(), uc.end(), std::default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        std::stable_sort(uc.begin(), uc.end(), m_litOrder);
        if (m_settings.internalSignals) {
            std::stable_sort(uc.begin(), uc.end(), m_innOrder);
        }
    }

    inline void GetPrimed(Cube &p) {
        for (auto &x : p) {
            x = m_model.LookupPrime(x);
        }
    }

    void Generalize(Cube &uc, int frameLvl, int recLvl = 0);

    bool Down(Cube &uc, int frameLvl, int recLvl, std::vector<Cube> &failedCtses);

    bool ExCTGBlock(std::shared_ptr<State> cts, int frameLvl, int recLvl, std::vector<Cube> &failedCtses, int blockLimit);

    bool DownHasFailed(const Cube &s, const std::vector<Cube> &failedCtses);

    bool Propagate(const Cube &c, int lvl);

    int PropagateUp(const Cube &c, int lvl);

    bool CheckBad(std::shared_ptr<State> s);

    void AddConstraintOr(const Frame &f);

    Lit AddConstraintAnd(const Frame &f);

    bool IsReachable(int lvl, const Cube &assumption, const std::string &label);

    Cube GetUnsatAssumption(std::shared_ptr<SATSolver> solver, const Cube &assumptions);

    std::shared_ptr<State> EnumerateStartState();

    void OverSequenceRefine(int lvl);

    void BuildCEXTrace();

    CheckResult m_checkResult;
    int m_minUpdateLevel;
    int m_k;
    std::shared_ptr<Branching> m_branching;
    std::shared_ptr<OverSequenceSet> m_overSequence;
    UnderSequence m_underSequence;
    Settings m_settings;
    Log &m_log;
    Model &m_model;
    std::vector<std::shared_ptr<SATSolver>> m_transSolvers;
    std::shared_ptr<SATSolver> m_startSolver;
    std::shared_ptr<SATSolver> m_badSolver;
    std::shared_ptr<SATSolver> m_invSolver;
    std::vector<std::shared_ptr<std::vector<int>>> m_rotation;
    std::shared_ptr<State> m_lastState;
    std::shared_ptr<Restart> m_restart;

    std::vector<std::pair<Cube, Cube>> m_cexTrace;
};

} // namespace car
