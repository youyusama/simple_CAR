#ifndef FORWARDCHECKER_H
#define FORWARDCHECKER_H

#include "BaseChecker.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "Restart.h"
#include "SATSolver.h"
#include "random"
#include <memory>
#include <unordered_set>

namespace car {

class ForwardChecker : public BaseChecker {
  public:
    ForwardChecker(Settings settings,
                   shared_ptr<Model> model,
                   shared_ptr<Log> log);
    CheckResult Run();
    void Witness();

  private:
    bool Check(int badId);

    void Init(int badId);

    bool AddUnsatisfiableCore(shared_ptr<cube> uc, int frameLevel, bool implyCheck = false);

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
        shared_ptr<Model> m;

        InnOrder() {}

        bool operator()(const int &inn_1, const int &inn_2) const {
            return (m->GetInnardslvl(inn_1) > m->GetInnardslvl(inn_2));
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

    void OrderAssumption(shared_ptr<cube> c) {
        if (m_settings.randomSeed > 0) {
            shuffle(c->begin(), c->end(), default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        stable_sort(c->begin(), c->end(), litOrder);
        if (m_settings.internalSignals) {
            stable_sort(c->begin(), c->end(), innOrder);
        }
    }

    inline void GetPrimed(shared_ptr<cube> p) {
        for (auto &x : *p) {
            x = m_model->GetPrime(x);
        }
    }

    void GeneralizePredecessor(pair<shared_ptr<cube>, shared_ptr<cube>> &s, shared_ptr<State> t);

    bool Generalize(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl = 1);

    bool Down(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl, shared_ptr<vector<cube>> failed_ctses);

    bool DownHasFailed(const shared_ptr<cube> s, const shared_ptr<vector<cube>> failed_ctses);

    bool Propagate(shared_ptr<cube> c, int lvl);

    int PropagateUp(shared_ptr<cube> c, int lvl);

    shared_ptr<State> EnumerateStartState();

    void OutputWitness(int bad);

    void OutputCounterExample(int bad);

    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube);

    bool CheckInit(shared_ptr<State> s);

    void AddConstraintOr(const shared_ptr<frame> f);

    void AddConstraintAnd(const shared_ptr<frame> f);

    bool IsReachable(int lvl, const shared_ptr<cube> assumption);

    pair<shared_ptr<cube>, shared_ptr<cube>> GetInputAndState(int lvl);

    shared_ptr<cube> GetUnsatCore(int lvl, const shared_ptr<cube> state);

    shared_ptr<cube> GetUnsatAssumption(shared_ptr<SATSolver> solver, const shared_ptr<cube> assumptions);

    shared_ptr<cube> TopDomain();

    shared_ptr<cube> GetAndPushDomain(shared_ptr<cube> c);

    void PopDomain();

    CheckResult m_checkResult;
    int m_minUpdateLevel;
    int m_badId;
    int m_k;
    shared_ptr<OverSequenceSet> m_overSequence;
    UnderSequence m_underSequence;
    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<Model> m_model;
    shared_ptr<State> m_initialState;
    vector<shared_ptr<SATSolver>> m_transSolvers;
    shared_ptr<SATSolver> m_liftSolver;
    shared_ptr<SATSolver> m_badPredLiftSolver;
    shared_ptr<SATSolver> m_invSolver;
    shared_ptr<SATSolver> m_startSolver;
    shared_ptr<Branching> m_branching;
    shared_ptr<State> m_lastState;
    shared_ptr<Restart> m_restart;
    shared_ptr<vector<shared_ptr<cube>>> m_domainStack;
};


} // namespace car

#endif