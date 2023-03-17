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
#include "Vis.h"
#include "restart.h"
#include <algorithm>
#include <assert.h>
#include <memory>


namespace car {
#define CAR_DEBUG_v(s, v)          \
  do {                             \
    m_log->DebugPrintVector(v, s); \
  } while (0)


#define CAR_DEBUG(s)         \
  do {                       \
    m_log->DebugPrintSth(s); \
  } while (0)

#define CAR_DEBUG_o(s, o)             \
  do {                                \
    m_log->DebugPrintSth(s);          \
    m_overSequence->PrintOSequence(); \
  } while (0)

#define CAR_DEBUG_od(s, o)                  \
  do {                                      \
    m_log->DebugPrintSth(s);                \
    m_overSequence->PrintOSequenceDetail(); \
  } while (0)

#define CAR_DEBUG_s(t, s)      \
  do {                         \
    m_log->DebugPrintSth(t);   \
    m_log->PrintStateShort(s); \
  } while (0)


class BackwardChecker : public BaseChecker {
public:
  BackwardChecker(Settings settings, std::shared_ptr<AigerModel> model);
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
  }

  void decayLitOrder(cube *uc, int gap = 1) {
    m_branching->decay(uc, gap);
  }

  // order according to preference
  void orderAssumption(std::vector<int> &uc, bool rev = false) {
    if (m_settings.Branching == 0) return;
    std::stable_sort(uc.begin(), uc.end(), litOrder);
    if (rev) std::reverse(uc.begin(), uc.end());
  }

private:
  void Init();

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
      return false;
    } else
      return true;
  }

  bool down_ctg(sptr<cube> &uc, int frame_lvl, int rec_lvl, std::unordered_set<int> required_lits) {
    int ctgs = 0;
    std::vector<int> ass;
    for (auto l : *uc) ass.emplace_back(l);
    sptr<State> p_ucs(new State(nullptr, nullptr, uc, 0));
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
        std::pair<sptr<cube>, sptr<cube>> pair = m_mainSolver->GetAssignment();
        sptr<State> cts(new State(nullptr, pair.first, pair.second, 0));
        int cts_lvl = GetNewLevel(cts);
        std::vector<int> cts_ass;
        // int cts_lvl = frame_lvl - 1;
        GetAssumption(cts, cts_lvl, cts_ass);
        // F_i-1 & T & cts'
        if (ctgs < 3 && cts_lvl >= 0 && !m_mainSolver->SolveWithAssumption(cts_ass, cts_lvl)) {
          ctgs++;
          auto uc_cts = m_mainSolver->Getuc(false);
          if (generalize_ctg(uc_cts, cts_lvl, rec_lvl + 1)) {
            updateLitOrder(*uc);
          }
          CAR_DEBUG_v("ctg Get UC:", *uc_cts);
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


  int m_minUpdateLevel;
  sptr<Branching> m_branching;
  std::shared_ptr<OverSequenceSet> m_overSequence;
  UnderSequence m_underSequence;
  std::shared_ptr<Vis> m_vis;
  Settings m_settings;
  std::shared_ptr<Log> m_log;
  std::shared_ptr<AigerModel> m_model;
  std::shared_ptr<State> m_initialState;
  std::shared_ptr<CarSolver> m_mainSolver;
  std::shared_ptr<ISolver> m_invSolver;
  std::vector<std::shared_ptr<std::vector<int>>> m_rotation;
  std::shared_ptr<Restart> m_restart;
  int m_repeat_state_num = 0;
};

} // namespace car

#endif