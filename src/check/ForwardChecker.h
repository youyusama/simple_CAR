#ifndef FORWARDCHECKER_H
#define FORWARDCHECKER_H

#include "BaseChecker.h"
#include "Branching.h"
#include "ISolver.h"
#include "InvSolver.h"
#include "Log.h"
#include "MainSolver.h"
#include "OverSequenceSet.h"
#include "StartSolver.h"
#include "State.h"
#include "Task.h"
#include "UnderSequence.h"
#include "random"
#include <memory>
#include <unordered_set>

namespace car {

class ForwardChecker : public BaseChecker {
  public:
    ForwardChecker(Settings settings,
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
        if (m_settings.seed > 0) {
            shuffle(uc.begin(), uc.end(), default_random_engine(m_settings.seed));
            return;
        }
        if (m_settings.Branching == 0) return;
        stable_sort(uc.begin(), uc.end(), litOrder);
        if (rev) reverse(uc.begin(), uc.end());
    }

  private:
    void Init(int badId);

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
        for (auto &x : ass) {
            x = m_model->GetPrime(x);
        }
        return;
    }

    static bool cmp(int a, int b) {
        return abs(a) < abs(b);
    }

    // ================================================================================
    // @brief: t & input & T -> s'  =>  (t) & input & T & !s' is unsat, !bad & input & t & T is unsat
    // @input: pair<input, latch>
    // @output: pair<input, partial latch>
    // ================================================================================
    pair<shared_ptr<cube>, shared_ptr<cube>> get_predecessor(
        pair<shared_ptr<cube>, shared_ptr<cube>> t, shared_ptr<State> s = nullptr) {
        m_log->Tick();

        orderAssumption(*t.second);
        shared_ptr<cube> partial_latch(new cube(*t.second));

        int act = m_lifts->GetTempFlag();
        if (s == nullptr) {
            // add !bad ( | !cons) to assumption
            vector<int> constraints = m_model->GetConstraints();
            clause cls = {-m_badId};
            for (auto cons : constraints) cls.push_back(-cons);
            m_lifts->AddTempClause(&cls, act, false);
            m_lifts->CleanAssumptions();
        } else {
            // add !s' to clause
            vector<int> *neg_primed_s = new vector<int>();
            for (auto l : *s->latches) {
                neg_primed_s->emplace_back(-l);
            }
            m_lifts->AddTempClause(neg_primed_s, act, true);
            m_lifts->CleanAssumptions();
        }
        // add t
        for (auto l : *t.second) {
            m_lifts->AddAssumption(l);
        }
        // add input
        for (auto i : *t.first) {
            m_lifts->AddAssumption(i);
        }
        m_lifts->AddAssumption(act);
        while (true) {
            bool res = m_lifts->SolveWithAssumption();
            assert(!res);
            shared_ptr<cube> temp_p = m_lifts->justGetUC(); // not muc
            if (temp_p->size() == partial_latch->size() && equal(temp_p->begin(), temp_p->end(), partial_latch->begin()))
                break;
            else {
                partial_latch = temp_p;
                m_lifts->CleanAssumptions();
                // add t
                for (auto l : *partial_latch) {
                    m_lifts->AddAssumption(l);
                }
                // add input
                for (auto i : *t.first) {
                    m_lifts->AddAssumption(i);
                }
                m_lifts->AddAssumption(act);
            }
        }
        sort(partial_latch->begin(), partial_latch->end(), cmp);

        m_log->StatLiftSolver();
        return pair<shared_ptr<cube>, shared_ptr<cube>>(t.first, partial_latch);
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
        for (int i = uc->size() - 1; i >= 0; i--) {
            if (uc->size() < 2) break;
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
        } else {
            return true;
        }
    }

    bool down_ctg(shared_ptr<cube> &uc, int frame_lvl, int rec_lvl, unordered_set<int> required_lits) {
        int ctgs = 0;
        m_log->L(3, "down:", CubeToStr(*uc));
        vector<int> ass;
        for (auto l : *uc) ass.emplace_back(m_model->GetPrime(l));
        shared_ptr<State> p_ucs(new State(nullptr, nullptr, uc, 0));
        while (true) {
            // F_i & T & temp_uc'
            m_log->Tick();
            if (!m_mainSolver->SolveWithAssumption(ass, frame_lvl)) {
                m_log->StatMainSolver();
                auto uc_ctg = m_mainSolver->Getuc(false);
                if (uc->size() < uc_ctg->size()) return false; // there are cases that uc_ctg longer than uc
                uc->swap(*uc_ctg);
                return true;
            } else if (rec_lvl > 2) {
                m_log->StatMainSolver();
                return false;
            } else {
                m_log->StatMainSolver();
                pair<shared_ptr<cube>, shared_ptr<cube>> s_pair = m_mainSolver->GetAssignment();
                pair<shared_ptr<cube>, shared_ptr<cube>> partial_pair = get_predecessor(s_pair, p_ucs);
                shared_ptr<State> cts(new State(nullptr, partial_pair.first, partial_pair.second, 0));
                int cts_lvl = GetNewLevel(cts);
                vector<int> cts_ass;
                // int cts_lvl = frame_lvl - 1;
                GetAssumption(cts, cts_lvl, cts_ass);
                // F_i-1 & T & cts'
                m_log->L(3, "try ctg:", CubeToStr(*cts->latches));
                m_log->Tick();
                if (ctgs < 3 && cts_lvl >= 0 && !m_mainSolver->SolveWithAssumption(cts_ass, cts_lvl)) {
                    m_log->StatMainSolver();
                    ctgs++;
                    auto uc_cts = m_mainSolver->Getuc(false);
                    m_log->L(3, "ctg Get UC:", CubeToStr(*uc_cts));
                    if (generalize_ctg(uc_cts, cts_lvl, rec_lvl + 1))
                        updateLitOrder(*uc_cts);
                    m_log->L(3, "ctg Get gUC:", CubeToStr(*uc_cts));
                    if (AddUnsatisfiableCore(uc_cts, cts_lvl + 1))
                        m_overSequence->propagate_uc_from_lvl(uc_cts, cts_lvl + 1, m_branching);
                } else {
                    m_log->StatMainSolver();
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

    shared_ptr<State> EnumerateStartState() {
        if (m_startSovler->SolveWithAssumption()) {
            pair<shared_ptr<cube>, shared_ptr<cube>> pair = m_startSovler->GetStartPair();
            pair = get_predecessor(pair);
            shared_ptr<State> newState(new State(nullptr, pair.first, pair.second, 0));
            return newState;
        } else {
            return nullptr;
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
    int m_badId;
    shared_ptr<OverSequenceSet> m_overSequence;
    UnderSequence m_underSequence;
    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<AigerModel> m_model;
    shared_ptr<State> m_initialState;
    shared_ptr<MainSolver> m_mainSolver;
    shared_ptr<CarSolver> m_lifts;
    shared_ptr<ISolver> m_invSolver;
    shared_ptr<StartSolver> m_startSovler;
    shared_ptr<Branching> m_branching;
    shared_ptr<State> m_lastState;
};


} // namespace car

#endif