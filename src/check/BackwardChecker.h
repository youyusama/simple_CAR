#ifndef BACKWARDCHECKER_H
#define BACKWARDCHECKER_H

#include "BaseChecker.h"
#include "ISolver.h"
#include "InvSolver.h"
#include "Log.h"
#include "MainSolver.h"
#include "OverSequenceSet.h"
#include "State.h"
#include "Task.h"
#include "UnderSequence.h"
#include "random"
#include <algorithm>
#include <assert.h>
#include <memory>


namespace car {

class BackwardChecker : public BaseChecker {
  public:
    BackwardChecker(Settings settings,
                    shared_ptr<AigerModel> model,
                    shared_ptr<Log> log);
    bool Run();
    bool Check(int badId);

  private:
    void Init();

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

    void OrderAssumption(shared_ptr<cube> uc) {
        if (m_settings.seed > 0) {
            shuffle(uc->begin(), uc->end(), default_random_engine(m_settings.seed));
            return;
        }
        if (m_settings.Branching == 0) return;
        stable_sort(uc->begin(), uc->end(), litOrder);
    }

    inline void GetPrimed(shared_ptr<cube> p) {
        for (auto &x : *p) {
            x = m_model->GetPrime(x);
        }
    }

    static bool cmp(int a, int b) {
        return abs(a) < abs(b);
    }

    bool Generalize(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl = 1);

    bool Down(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl, unordered_set<int> required_lits);

    bool Propagate(shared_ptr<cube> c, int lvl);

    int PropagateUp(shared_ptr<cube> c, int lvl);

    void OutputWitness(int bad);

    void OutputCounterExample(int bad);

    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube);

    int m_minUpdateLevel;
    shared_ptr<Branching> m_branching;
    shared_ptr<OverSequenceSet> m_overSequence;
    UnderSequence m_underSequence;
    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<AigerModel> m_model;
    shared_ptr<State> m_initialState;
    shared_ptr<MainSolver> m_mainSolver;
    shared_ptr<InvSolver> m_invSolver;
    vector<shared_ptr<vector<int>>> m_rotation;
    shared_ptr<State> m_lastState;
};

} // namespace car

#endif