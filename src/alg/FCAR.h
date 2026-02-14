#ifndef FCAR_H
#define FCAR_H

#include "IncrAlg.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"
#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>

namespace car {

class FCAR : public IncrAlg {
  public:
    FCAR(Settings settings,
         Model &model,
         Log &log);
    CheckResult Run() override;
    void Witness() override;

    void SetInit(const Cube &c) override { m_customInit = c; }
    void SetSearchFromInitSucc(bool b) override { m_searchFromInitSucc = b; }
    void SetLoopRefuting(bool b) override { m_loopRefuting = b; }
    void SetDead(const std::vector<Cube> &dead) override { m_dead = dead; }
    void SetShoals(const std::vector<FrameList> &shoals) override { m_shoals = shoals; }
    void SetWalls(const std::vector<FrameList> &walls) override { m_walls = walls; }

    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;
    FrameList GetInv() override;
    void KLiveIncr() override;

  private:
    bool Check();

    void Init();

    void Reset();

    bool ImmediateSatisfiable();

    void CreateTransSolver(int k);

    bool IsInitStateImplyBad();

    void InitializeStartSolver();

    bool AddUnsatisfiableCore(const Cube &uc, int frameLevel);

    bool IsInvariant(int frameLevel);

    struct LitOrder {
        shared_ptr<Branching> branching;

        LitOrder() {}

        bool operator()(const int &l1, const int &l2) const {
            return (branching->PriorityOf(l1) > branching->PriorityOf(l2));
        }
    } m_litOrder;

    struct InnOrder {
        Model &m;

        explicit InnOrder(Model &model) : m(model) {}

        bool operator()(const int &inn1, const int &inn2) const {
            return (m.GetInnardslvl(inn1) > m.GetInnardslvl(inn2));
        }
    } m_innOrder;

    struct BlockerOrder {
        shared_ptr<Branching> branching;

        BlockerOrder() {}

        bool operator()(const Cube &a, const Cube &b) const {
            float score_a = 0, score_b = 0;
            for (int i = 0; i < a.size(); i++) {
                score_a += branching->PriorityOf(a[i]);
            }
            score_a /= a.size();
            for (int i = 0; i < b.size(); i++) {
                score_b += branching->PriorityOf(b[i]);
            }
            score_b /= b.size();
            return score_a > score_b;
        }
    } m_blockerOrder;

    void OrderAssumption(Cube &c) {
        [[maybe_unused]] auto scoped = m_log.Section("DS_OrdAsm");
        if (m_settings.randomSeed > 0) {
            shuffle(c.begin(), c.end(), default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        stable_sort(c.begin(), c.end(), m_litOrder);
        if (m_settings.internalSignals) {
            stable_sort(c.begin(), c.end(), m_innOrder);
        }
    }

    inline void GetPrimed(Cube &p) {
        for (auto &x : p) {
            x = m_model.GetPrime(x);
        }
    }

    int GetNewLevel(const Cube &states, int start = 0);

    void GeneralizePredecessor(pair<Cube, Cube> &s, shared_ptr<State> t);

    bool Generalize(Cube &uc, int frameLvl, int recLvl = 0);

    bool Down(Cube &uc, int frameLvl, int recLvl, vector<Cube> &failedCtses);

    bool CTSBlock(shared_ptr<State> cts, int frameLvl, int recLvl, vector<Cube> &failedCtses, int ctsCount = 0);

    bool DownHasFailed(const Cube &s, const vector<Cube> &failedCtses);

    bool Propagate(const Cube &c, int lvl);

    int PropagateUp(const Cube &c, int lvl);

    bool IsReachable(int lvl, const Cube &assumption, const string &label);

    shared_ptr<State> EnumerateStartState();

    void OutputWitness();

    void OutputCounterExample();

    unsigned AddCubeToAndGates(aiger *circuit, vector<unsigned> cb);

    bool CheckInit(shared_ptr<State> s);

    void AddConstraintOr(const shared_ptr<OverSequenceSet::FrameSet> f);

    void AddConstraintAnd(const shared_ptr<OverSequenceSet::FrameSet> f);

    pair<Cube, Cube> GetInputAndState(int lvl);

    Cube GetUnsatCore(int lvl, const Cube &state);

    Cube GetUnsatAssumption(shared_ptr<SATSolver> solver, const Cube &assumptions);

    void BuildCEXTrace();

    CheckResult m_checkResult;
    int m_minUpdateLevel;
    int m_k;
    shared_ptr<OverSequenceSet> m_overSequence;
    UnderSequence m_underSequence;
    Settings m_settings;
    Log &m_log;
    Model &m_model;
    shared_ptr<State> m_initialState;
    vector<shared_ptr<SATSolver>> m_transSolvers;
    shared_ptr<SATSolver> m_liftSolver;
    shared_ptr<SATSolver> m_badLiftSolver;
    shared_ptr<SATSolver> m_invSolver;
    shared_ptr<SATSolver> m_startSolver;
    shared_ptr<Branching> m_branching;
    shared_ptr<State> m_lastState;
    shared_ptr<Restart> m_restart;

    // liveness
    bool m_initialized{false};
    Cube m_customInit;
    bool m_searchFromInitSucc{false};
    bool m_loopRefuting{false};
    std::vector<Cube> m_dead;
    std::vector<FrameList> m_shoals;
    std::vector<FrameList> m_walls;
    bool m_initStateImplyBad{false};
    std::vector<std::pair<Cube, Cube>> m_cexTrace;
    Cube m_shoalsLabels;
    Cube m_wallsLabels;
};


} // namespace car

#endif
