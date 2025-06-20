#ifndef FORWARDCHECKER_H
#define FORWARDCHECKER_H

#include "BaseChecker.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
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
    bool Run();
    bool Check(int badId);

  private:
    void Init(int badId);

    bool AddUnsatisfiableCore(shared_ptr<vector<int>> uc, int frameLevel);

    bool ImmediateSatisfiable(int badId);

    bool IsInvariant(int frameLevel);

    int GetNewLevel(shared_ptr<State> state, int start = 0);

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

        bool operator()(const shared_ptr<cube> a, const shared_ptr<cube> b) const {
            float score_a = 0, score_b = 0;
            for (int i = 0; i < a->size(); i++) {
                score_a += branching->PriorityOf(a->at(i));
                score_b += branching->PriorityOf(b->at(i));
            }
            return score_a > score_b;
        }
    } blockerOrder;

    void OrderAssumption(shared_ptr<cube> c) {
        if (m_settings.seed > 0) {
            shuffle(c->begin(), c->end(), default_random_engine(m_settings.seed));
            return;
        }
        if (m_settings.internalSignals) {
            stable_sort(c->begin(), c->end(), innOrder);
            return;
        }
        if (m_settings.Branching == 0) return;
        stable_sort(c->begin(), c->end(), litOrder);
    }

    inline void GetPrimed(shared_ptr<cube> p) {
        for (auto &x : *p) {
            x = m_model->GetPrime(x);
        }
    }

    static bool cmp(int a, int b) {
        return abs(a) < abs(b);
    }

    void GeneralizePredecessor(pair<shared_ptr<cube>, shared_ptr<cube>> &s, shared_ptr<State> t);

    bool Generalize(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl = 1);

    bool Down(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl, unordered_set<int> required_lits);

    bool Propagate(shared_ptr<cube> c, int lvl);

    int PropagateUp(shared_ptr<cube> c, int lvl);

    shared_ptr<State> EnumerateStartState();

    void OutputWitness(int bad);

    void OutputCounterExample(int bad);

    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube);

    bool CheckInit(shared_ptr<State> s);

    void AddConstraintOr(const shared_ptr<frame> f);

    void AddConstraintAnd(const shared_ptr<frame> f);

    int m_minUpdateLevel;
    int m_badId;
    int m_k;
    shared_ptr<OverSequenceSet> m_overSequence;
    UnderSequence m_underSequence;
    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<Model> m_model;
    shared_ptr<State> m_initialState;
    shared_ptr<SATSolver> m_mainSolver;
    shared_ptr<SATSolver> m_liftSolver;
    shared_ptr<SATSolver> m_badPredLiftSolver;
    shared_ptr<SATSolver> m_invSolver;
    shared_ptr<SATSolver> m_startSolver;
    shared_ptr<Branching> m_branching;
    shared_ptr<State> m_lastState;
};


} // namespace car

#endif