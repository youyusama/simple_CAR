#ifndef BCAR_H
#define BCAR_H

#include "BaseAlg.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"
#include "random"
#include <algorithm>
#include <assert.h>
#include <memory>
#include <string>


namespace car {

class BCAR : public BaseAlg {
  public:
    BCAR(Settings settings,
         Model &model,
         Log &log);
    CheckResult Run() override;
    void Witness() override;

  private:
    bool Check(int badId);

    void Init();

    bool AddUnsatisfiableCore(const cube &uc, int frameLevel);

    bool ImmediateSatisfiable(int badId);

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

    void OrderAssumption(cube &uc) {
        if (m_settings.randomSeed > 0) {
            shuffle(uc.begin(), uc.end(), default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        stable_sort(uc.begin(), uc.end(), litOrder);
        if (m_settings.internalSignals) {
            stable_sort(uc.begin(), uc.end(), innOrder);
        }
    }

    inline void GetPrimed(cube &p) {
        for (auto &x : p) {
            x = m_model.GetPrime(x);
        }
    }

    bool Generalize(cube &uc, int frame_lvl, int rec_lvl = 1);

    bool Down(cube &uc, int frame_lvl, int rec_lvl, vector<cube> &failed_ctses);

    bool DownHasFailed(const cube &s, const vector<cube> &failed_ctses);

    bool Propagate(const cube &c, int lvl);

    int PropagateUp(const cube &c, int lvl);

    void OutputWitness(int bad);

    void OutputCounterExample(int bad);

    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube);

    bool CheckBad(shared_ptr<State> s);

    void AddConstraintOr(const shared_ptr<frame> f);

    void AddConstraintAnd(const shared_ptr<frame> f);

    bool IsReachable(int lvl, const cube &assumption, const string &label);

    pair<cube, cube> GetInputAndState(int lvl);

    cube GetUnsatCore(int lvl, const cube &state);

    cube GetUnsatAssumption(shared_ptr<SATSolver> solver, const cube &assumptions);

    shared_ptr<State> EnumerateStartState();

    void OverSequenceRefine(int lvl);

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
};

} // namespace car

#endif
