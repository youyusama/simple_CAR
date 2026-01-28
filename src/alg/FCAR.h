#ifndef FCAR_H
#define FCAR_H

#include "IncrAlg.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"
#include "random"
#include <memory>
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

    void SetInit(const cube &c) override { m_customInit = c; }
    void SetSearchFromInitSucc(bool b) override { m_searchFromInitSucc = b; }
    void SetLoopRefuting(bool b) override { m_loopRefuting = b; }
    void SetDead(const std::vector<cube> &dead) override { m_dead = dead; }
    void SetShoals(const std::vector<FrameList> &shoals) override { m_shoals = shoals; }
    void SetWalls(const std::vector<FrameList> &walls) override { m_walls = walls; }

    std::vector<std::pair<cube, cube>> GetCexTrace() override;
    FrameList GetInv() override;
    void KLiveIncr() override;

  private:
    bool Check();

    void Init();

    void Reset();

    bool IsInitStateImplyBad();

    void InitializeStartSolver();

    bool AddUnsatisfiableCore(const cube &uc, int frameLevel);

    bool IsInvariant(int frameLevel);

    struct LitOrder {
        shared_ptr<Branching> branching;

        LitOrder() {}

        bool operator()(const int &l1, const int &l2) const {
            return (branching->PriorityOf(l1) > branching->PriorityOf(l2));
        }
    } litOrder;

    struct InnOrder {
        Model &m;

        explicit InnOrder(Model &model) : m(model) {}

        bool operator()(const int &inn_1, const int &inn_2) const {
            return (m.GetInnardslvl(inn_1) > m.GetInnardslvl(inn_2));
        }
    } innOrder;

    struct BlockerOrder {
        shared_ptr<Branching> branching;

        BlockerOrder() {}

        bool operator()(const cube &a, const cube &b) const {
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
    } blockerOrder;

    void OrderAssumption(cube &c) {
        [[maybe_unused]] auto scoped = m_log.Section("DS_OrdAsm");
        if (m_settings.randomSeed > 0) {
            shuffle(c.begin(), c.end(), default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        stable_sort(c.begin(), c.end(), litOrder);
        if (m_settings.internalSignals) {
            stable_sort(c.begin(), c.end(), innOrder);
        }
    }

    inline void GetPrimed(cube &p) {
        for (auto &x : p) {
            x = m_model.GetPrime(x);
        }
    }

    int GetNewLevel(const cube &states, int start = 0);

    void GeneralizePredecessor(pair<cube, cube> &s, shared_ptr<State> t);

    bool Generalize(cube &uc, int frame_lvl, int rec_lvl = 0);

    bool Down(cube &uc, int frame_lvl, int rec_lvl, vector<cube> &failed_ctses);

    bool CTSBlock(shared_ptr<State> cts, int frame_lvl, int rec_lvl, vector<cube> &failed_ctses, int cts_count = 0);

    bool DownHasFailed(const cube &s, const vector<cube> &failed_ctses);

    bool Propagate(const cube &c, int lvl);

    int PropagateUp(const cube &c, int lvl);

    bool IsReachable(int lvl, const cube &assumption, const string &label);

    shared_ptr<State> EnumerateStartState();

    void OutputWitness();

    void OutputCounterExample();

    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube);

    bool CheckInit(shared_ptr<State> s);

    void AddConstraintOr(const shared_ptr<OverSequenceSet::FrameSet> f);

    void AddConstraintAnd(const shared_ptr<OverSequenceSet::FrameSet> f);

    pair<cube, cube> GetInputAndState(int lvl);

    cube GetUnsatCore(int lvl, const cube &state);

    cube GetUnsatAssumption(shared_ptr<SATSolver> solver, const cube &assumptions);

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
    cube m_customInit;
    bool m_searchFromInitSucc{false};
    bool m_loopRefuting{false};
    std::vector<cube> m_dead;
    std::vector<FrameList> m_shoals;
    std::vector<FrameList> m_walls;
    bool m_initStateImplyBad{false};
    std::vector<std::pair<cube, cube>> m_cexTrace;
    cube m_shoalsLabels;
    cube m_wallsLabels;
};


} // namespace car

#endif
