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
#include "Vis.h"
#include "random"
#include <memory>
#include <unordered_set>

namespace car {
#define CAR_DEBUG_v(s, v)          \
  do {                             \
    m_log->DebugPrintVector(v, s); \
  } while (0)


#define CAR_DEBUG(s)         \
  do {                       \
    m_log->DebugPrintSth(s); \
  } while (0)

#define CAR_DEBUG_od(s, o)                  \
  do {                                      \
    m_log->DebugPrintSth(s);                \
    m_overSequence->PrintOSequenceDetail(); \
  } while (0)

#define CAR_DEBUG_o(s, o)             \
  do {                                \
    m_log->DebugPrintSth(s);          \
    m_overSequence->PrintOSequence(); \
  } while (0)


#define CAR_DEBUG_s(t, s)      \
  do {                         \
    m_log->DebugPrintSth(t);   \
    m_log->PrintStateShort(s); \
  } while (0)


#define CAR_DEBUG_order(t, o) \
  do {                        \
    m_log->DebugPrintSth(t);  \
    m_log->PrintLitOrder(o);  \
  } while (0)

class ForwardChecker : public BaseChecker {
public:
  ForwardChecker(Settings settings, std::shared_ptr<AigerModel> model);
  bool Run();
  bool Check(int badId);

  struct LitOrder {
    sptr<Branching> branching;

    LitOrder() {}

    bool operator()(const int &l1, const int &l2) const {
      return (branching->prior_of(l1) > branching->prior_of(l2));
    }
  } litOrder;


  struct BlockerOrder {
    sptr<Branching> branching;

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
    CAR_DEBUG_order("\nLit Order:\n", *m_branching->get_counts());
  }

  void decayLitOrder(cube *uc, int gap = 1) {
    m_branching->decay(uc, gap);
  }

  // order according to preference
  void orderAssumption(std::vector<int> &uc, bool rev = false) {
    if (m_settings.seed > 0) {
      std::shuffle(uc.begin(), uc.end(), std::default_random_engine(m_settings.seed));
      return;
    }
    if (m_settings.Branching == 0) return;
    std::stable_sort(uc.begin(), uc.end(), litOrder);
    if (rev) std::reverse(uc.begin(), uc.end());
  }

private:
  void Init(int badId);

  bool AddUnsatisfiableCore(std::shared_ptr<std::vector<int>> uc, int frameLevel);

  bool ImmediateSatisfiable(int badId);

  bool isInvExisted();

  bool IsInvariant(int frameLevel);

  int GetNewLevel(std::shared_ptr<State> state, int start = 0);

  void GetAssumption(std::shared_ptr<State> state, int frameLevel, std::vector<int> &ass, bool rev = false) {
    if (m_settings.incr) {
      const std::vector<int> *uc_inc = m_overSequence->GetBlocker(state->latches, frameLevel);
      ass.reserve(ass.size() + uc_inc->size());
      ass.insert(ass.end(), uc_inc->begin(), uc_inc->end());
    }

    std::vector<int> l_ass;
    l_ass.reserve(state->latches->size());
    l_ass.insert(l_ass.end(), state->latches->begin(), state->latches->end());
    orderAssumption(l_ass, rev);
    CAR_DEBUG_v("Assumption Detail: ", l_ass);

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
  std::pair<sptr<cube>, sptr<cube>> get_predecessor(
    std::pair<sptr<cube>, sptr<cube>> t, sptr<State> s = nullptr) {

    orderAssumption(*t.second);
    std::shared_ptr<cube> partial_latch(new cube(*t.second));

    int act = m_lifts->get_temp_flag();
    if (s == nullptr) {
      // add !bad to assumption
      std::vector<int> *neg_bad = new std::vector<int>(1, -m_badId);
      m_lifts->add_temp_clause(neg_bad, act, false);
      m_lifts->clean_assumptions();
    } else {
      // add !s' to clause
      std::vector<int> *neg_primed_s = new std::vector<int>();
      for (auto l : *s->latches) {
        neg_primed_s->emplace_back(-l);
      }
      m_lifts->add_temp_clause(neg_primed_s, act, true);
      m_lifts->clean_assumptions();
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
      std::shared_ptr<cube> temp_p = m_lifts->justGetUC(); // not muc
      if (temp_p->size() == partial_latch->size() && std::equal(temp_p->begin(), temp_p->end(), partial_latch->begin()))
        break;
      else {
        partial_latch = temp_p;
        m_lifts->clean_assumptions();
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
    std::sort(partial_latch->begin(), partial_latch->end(), cmp);
    return std::pair<std::shared_ptr<cube>, std::shared_ptr<cube>>(t.first, partial_latch);
  }

  // ================================================================================
  // @brief: counter-example to generalization
  // @input:
  // @output:
  // ================================================================================
  bool generalize_ctg(sptr<cube> &uc, int frame_lvl, int rec_lvl = 1) {
    std::unordered_set<int> required_lits;
    std::vector<cube *> *uc_blockers = m_overSequence->GetBlockers(uc, frame_lvl);
    cube *uc_blocker;
    if (uc_blockers->size() > 0) {
      if (m_settings.Branching > 0)
        std::stable_sort(uc_blockers->begin(), uc_blockers->end(), blockerOrder);
      uc_blocker = uc_blockers->at(0);
    } else {
      uc_blocker = new cube();
    }
    if (m_settings.skip_refer)
      for (auto b : *uc_blocker) required_lits.emplace(b);
    orderAssumption(*uc);
    for (int i = uc->size() - 1; i > 0; i--) {
      if (required_lits.find(uc->at(i)) != required_lits.end()) continue;
      sptr<cube> temp_uc(new cube());
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
    std::sort(uc->begin(), uc->end(), cmp);
    if (uc->size() > uc_blocker->size() && frame_lvl != 0) {
      m_log->StatGenGood(false);
      return false;
    } else {
      m_log->StatGenGood(true);
      return true;
    }
  }

  bool down_ctg(sptr<cube> &uc, int frame_lvl, int rec_lvl, std::unordered_set<int> required_lits) {
    int ctgs = 0;
    std::vector<int> ass;
    for (auto l : *uc) ass.emplace_back(m_model->GetPrime(l));
    sptr<State> p_ucs(new State(nullptr, nullptr, uc, 0));
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
        std::pair<sptr<cube>, sptr<cube>> pair = m_mainSolver->GetAssignment();
        std::pair<sptr<cube>, sptr<cube>> partial_pair = get_predecessor(pair, p_ucs);
        sptr<State> cts(new State(nullptr, partial_pair.first, partial_pair.second, 0));
        int cts_lvl = GetNewLevel(cts);
        std::vector<int> cts_ass;
        // int cts_lvl = frame_lvl - 1;
        GetAssumption(cts, cts_lvl, cts_ass);
        // F_i-1 & T & cts'
        m_log->Tick();
        if (ctgs < 3 && cts_lvl >= 0 && !m_mainSolver->SolveWithAssumption(cts_ass, cts_lvl)) {
          m_log->StatMainSolver();
          ctgs++;
          auto uc_cts = m_mainSolver->Getuc(false);
          if (generalize_ctg(uc_cts, cts_lvl, rec_lvl + 1))
            updateLitOrder(*uc);
          CAR_DEBUG_v("ctg Get UC:", *uc_cts);
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

  std::shared_ptr<State> EnumerateStartState() {
    if (m_startSovler->SolveWithAssumption()) {
      std::pair<sptr<cube>, sptr<cube>> pair = m_startSovler->GetStartPair();
      // CAR_DEBUG_v("From state: ", *pair.second);
      pair = get_predecessor(pair);
      sptr<State> newState(new State(nullptr, pair.first, pair.second, 0));
      return newState;
    } else {
      return nullptr;
    }
  }

  int m_minUpdateLevel;
  int m_badId;
  std::shared_ptr<OverSequenceSet> m_overSequence;
  UnderSequence m_underSequence;
  Settings m_settings;
  std::shared_ptr<Vis> m_vis;
  std::shared_ptr<Log> m_log;
  std::shared_ptr<AigerModel> m_model;
  std::shared_ptr<State> m_initialState;
  std::shared_ptr<MainSolver> m_mainSolver;
  std::shared_ptr<CarSolver> m_lifts;
  std::shared_ptr<ISolver> m_invSolver;
  std::shared_ptr<StartSolver> m_startSovler;
  sptr<Branching> m_branching;
};


} // namespace car

#endif