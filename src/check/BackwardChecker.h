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

    struct LitOrder {
        shared_ptr<Branching> branching;

        LitOrder() {}

        bool operator()(const int &l1, const int &l2) const {
            return (branching->prior_of(l1) > branching->prior_of(l2));
        }
    } litOrder;


    struct BlockerOrder {
        shared_ptr<Branching> branching;

        BlockerOrder() {}

        bool operator()(const cube *a, const cube *b) const {
            float score_a = 0, score_b = 0;
            for (int i = 0; i < a->size(); i++) {
                score_a += branching->prior_of(a->at(i));
                score_b += branching->prior_of(b->at(i));
            }
            return score_a > score_b;
        }
    } blockerOrder;

    void updateLitOrder(cube uc) {
        m_branching->update(&uc);
    }

    void decayLitOrder(cube *uc, int gap = 1) {
        m_branching->decay(uc, gap);
    }

    // order according to preference
    void orderAssumption(vector<int> &uc, bool rev = false) {
        if (m_settings.Branching == 0) return;
        stable_sort(uc.begin(), uc.end(), litOrder);
        if (rev) reverse(uc.begin(), uc.end());
    }

  private:
    void Init();

    bool AddUnsatisfiableCore(shared_ptr<vector<int>> uc, int frameLevel);

    bool ImmediateSatisfiable(int badId);

    bool isInvExisted();

    bool IsInvariant(int frameLevel);

    int GetNewLevel(shared_ptr<State> state, int start = 0);

    void GetAssumption(shared_ptr<State> state, int frameLevel, vector<int> &ass, bool rev = false) {
        vector<int> l_ass;
        l_ass.reserve(state->latches->size());
        l_ass.insert(l_ass.end(), state->latches->begin(), state->latches->end());
        orderAssumption(l_ass, rev);
        // CAR_DEBUG_v("Assumption Detail: ", l_ass);

        ass.reserve(ass.size() + l_ass.size());
        ass.insert(ass.end(), l_ass.begin(), l_ass.end());
        // CAR_DEBUG_v("Assumption: ", ass);
        return;
    }


    static bool cmp(int a, int b) {
        return abs(a) < abs(b);
    }


    // ================================================================================
    // @brief: counter-example to generalization
    // @input:
    // @output:
    // ================================================================================
    bool generalize_ctg(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl = 1) {
        unordered_set<int> required_lits;
        vector<cube *> *uc_blockers = m_overSequence->GetBlockers(uc, frame_lvl);
        cube *uc_blocker;
        if (uc_blockers->size() > 0) {
            if (m_settings.Branching > 0)
                stable_sort(uc_blockers->begin(), uc_blockers->end(), blockerOrder);
            uc_blocker = uc_blockers->at(0);
        } else {
            uc_blocker = new cube();
        }
        if (m_settings.skip_refer)
            for (auto b : *uc_blocker) required_lits.emplace(b);
        orderAssumption(*uc);
        for (int i = uc->size() - 1; i > 0; i--) {
            if (required_lits.find(uc->at(i)) != required_lits.end()) continue;
            shared_ptr<cube> temp_uc(new cube());
            for (auto ll : *uc)
                if (ll != uc->at(i)) temp_uc->emplace_back(ll);
            if (down_ctg(temp_uc, frame_lvl, rec_lvl, required_lits)) {
                uc->swap(*temp_uc);
                orderAssumption(*uc);
                i = uc->size();
            } else {
                required_lits.emplace(uc->at(i));
            }
        }
        sort(uc->begin(), uc->end(), cmp);
        if (uc->size() > uc_blocker->size() && frame_lvl != 0) {
            return false;
        } else
            return true;
    }

    bool down_ctg(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl, unordered_set<int> required_lits) {
        int ctgs = 0;
        vector<int> ass;
        for (auto l : *uc) ass.emplace_back(l);
        shared_ptr<State> p_ucs(new State(nullptr, nullptr, uc, 0));
        while (true) {
            // F_i & T & temp_uc'
            if (!m_mainSolver->SolveWithAssumption(ass, frame_lvl)) {
                auto uc_ctg = m_mainSolver->Getuc(false);
                if (uc->size() < uc_ctg->size()) return false; // there are cases that uc_ctg longer than uc
                uc->swap(*uc_ctg);
                return true;
            } else if (rec_lvl > 2)
                return false;
            else {
                pair<shared_ptr<cube>, shared_ptr<cube>> pair = m_mainSolver->GetAssignment();
                shared_ptr<State> cts(new State(nullptr, pair.first, pair.second, 0));
                int cts_lvl = GetNewLevel(cts);
                vector<int> cts_ass;
                // int cts_lvl = frame_lvl - 1;
                GetAssumption(cts, cts_lvl, cts_ass);
                // F_i-1 & T & cts'
                if (ctgs < 3 && cts_lvl >= 0 && !m_mainSolver->SolveWithAssumption(cts_ass, cts_lvl)) {
                    ctgs++;
                    auto uc_cts = m_mainSolver->Getuc(false);
                    if (generalize_ctg(uc_cts, cts_lvl, rec_lvl + 1)) {
                        updateLitOrder(*uc);
                    }
                    m_log->L(3, "ctg Get gUC:", CubeToStr(*uc_cts));
                    if (AddUnsatisfiableCore(uc_cts, cts_lvl + 1))
                        m_overSequence->propagate_uc_from_lvl(uc_cts, cts_lvl + 1, m_branching);
                } else {
                    return false;
                }
            }
        }
    }


    void Propagation() {
        for (int i = m_minUpdateLevel; i < m_overSequence->GetLength() - 1; i++) {
            m_overSequence->propagate(i, m_branching);
        }
    }

    void OutputWitness(int bad);

    void OutputCounterExample(int bad);

    // ================================================================================
    // @brief: add the cube as and gates to the aiger model
    // @input:
    // @output:
    // ================================================================================
    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube) {
        assert(cube.size() > 0);
        unsigned res = cube[0];
        assert(res / 2 <= circuit->maxvar);
        for (unsigned i = 1; i < cube.size(); i++) {
            assert(cube[i] / 2 <= circuit->maxvar);
            unsigned new_gate = (circuit->maxvar + 1) * 2;
            aiger_add_and(circuit, new_gate, res, cube[i]);
            res = new_gate;
        }
        return res;
    }

    int m_minUpdateLevel;
    shared_ptr<Branching> m_branching;
    shared_ptr<OverSequenceSet> m_overSequence;
    UnderSequence m_underSequence;
    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<AigerModel> m_model;
    shared_ptr<State> m_initialState;
    shared_ptr<CarSolver> m_mainSolver;
    shared_ptr<ISolver> m_invSolver;
    vector<shared_ptr<vector<int>>> m_rotation;
    shared_ptr<State> m_lastState;
};

} // namespace car

#endif