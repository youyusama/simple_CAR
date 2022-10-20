#ifndef FORWARDCHECKER_H
#define FORWARDCHECKER_H

#include "BaseChecker.h"
#include "IOverSequence.h"
#include "ISolver.h"
#include "InvSolver.h"
#include "Log.h"
#include "MainSolver.h"
#include "OverSequence.h"
#include "OverSequenceNI.h"
#include "OverSequenceSet.h"
#include "StartSolver.h"
#include "State.h"
#include "Task.h"
#include "UnderSequence.h"
#include "Vis.h"
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

#define CAR_DEBUG_od(s, o)          \
  do {                              \
    m_log->DebugPrintSth(s);        \
    m_log->PrintOSequenceDetail(o); \
  } while (0)

#define CAR_DEBUG_o(s, o)     \
  do {                        \
    m_log->DebugPrintSth(s);  \
    m_log->PrintOSequence(o); \
  } while (0)


#define CAR_DEBUG_s(t, s)      \
  do {                         \
    m_log->DebugPrintSth(t);   \
    m_log->PrintStateShort(s); \
  } while (0)

class ForwardChecker : public BaseChecker {
public:
  ForwardChecker(Settings settings, std::shared_ptr<AigerModel> model);
  bool Run();
  bool Check(int badId);

  struct HeuristicLitOrder {
    HeuristicLitOrder() : _mini(1 << 20) {}
    std::vector<float> counts;
    int _mini;
    void count(const std::vector<int> &uc) {
      // assumes cube is ordered
      int sz = abs(uc.back());
      int a = counts.size();
      if (sz >= counts.size()) counts.resize(sz + 1);
      _mini = abs(uc[0]);
      for (std::vector<int>::const_iterator i = uc.begin(); i != uc.end(); ++i)
        counts[abs(*i)]++;
    }
    void decay() {
      for (int i = _mini; i < counts.size(); ++i)
        counts[i] *= 0.99;
    }
  } litOrder;

  struct SlimLitOrder {
    HeuristicLitOrder *heuristicLitOrder;

    SlimLitOrder() {}

    bool operator()(const int &l1, const int &l2) const {
      if (abs(l2) >= heuristicLitOrder->counts.size()) return true;
      if (abs(l1) >= heuristicLitOrder->counts.size()) return false;
      return (heuristicLitOrder->counts[abs(l1)] > heuristicLitOrder->counts[abs(l2)]);
    }
  } slimLitOrder;

  float numLits, numUpdates;
  void updateLitOrder(const std::vector<int> &uc) {
    litOrder.decay();
    litOrder.count(uc);
  }

  // order according to preference
  void orderAssumption(std::vector<int> &uc, bool rev = false) {
    std::stable_sort(uc.begin(), uc.end(), slimLitOrder);
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

    std::shared_ptr<cube> partial_latch(new cube());
    for (auto l : *t.second) partial_latch->emplace_back(l);

    int act = m_lifts->get_temp_flag();
    if (s == nullptr) {
      // add !bad to assumption
      m_lifts->clean_assumptions();
      m_lifts->AddAssumption(-m_badId);
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
    // while (true) {
    //   bool res = m_lifts->SolveWithAssumption();
    //   assert(!res);
    //   std::shared_ptr<cube> temp_p = m_lifts->justGetUC();//muc
    //   if (temp_p->size() == partial_latch->size() && std::equal(temp_p->begin(), temp_p->end(), partial_latch->begin()))
    //     break;
    //   else {
    //     partial_latch = temp_p;
    //     m_lifts->clean_assumptions();
    //     // add t
    //     for (auto l : *t.second) {
    //       m_lifts->AddAssumption(l);
    //     }
    //     // add input
    //     for (auto i : *t.first) {
    //       m_lifts->AddAssumption(i);
    //     }
    //     if (s == nullptr) {
    //       m_lifts->AddAssumption(-m_badId);
    //     }
    //     m_lifts->AddAssumption(act);
    //   }
    // }
    bool res = m_lifts->SolveWithAssumption();
    assert(!res);
    std::shared_ptr<cube> temp_p = m_lifts->justGetUC(); // muc
    partial_latch = temp_p;
    m_lifts->release_temp_cls(act);
    // delete !bad
    if (s == nullptr) {
      for (auto it = partial_latch->begin(); it != partial_latch->end(); it++) {
        if (*it == -m_badId) {
          partial_latch->erase(it);
          break;
        }
      }
    }
    return std::pair<std::shared_ptr<cube>, std::shared_ptr<cube>>(t.first, partial_latch);
  }

  // ================================================================================
  // @brief: counter-example to generalization
  // @input:
  // @output:
  // ================================================================================
  void generalize_ctg(sptr<cube> &uc, int frame_lvl, int rec_lvl = 1) {
    if (uc->size() == 1) return;
    std::unordered_set<int> required_lits;
    const std::vector<int> *uc_blocker = m_overSequence->GetBlocker(uc, frame_lvl);
    for (auto b : *uc_blocker) required_lits.emplace(b);
    for (int i = uc->size() - 1; i > 0; i--) {
      if (required_lits.find(uc->at(i)) != required_lits.end()) continue;
      sptr<cube> temp_uc(new cube());
      for (auto ll : *uc)
        if (ll != uc->at(i)) temp_uc->emplace_back(ll);
      if (down_ctg(temp_uc, frame_lvl, rec_lvl, required_lits)) {
        uc->swap(*temp_uc);
        i = uc->size();
      } else {
        required_lits.emplace(uc->at(i));
      }
    }
  }

  bool down_ctg(sptr<cube> &uc, int frame_lvl, int rec_lvl, std::unordered_set<int> required_lits) {
    int ctgs = 0;
    std::vector<int> ass;
    for (auto l : *uc) ass.emplace_back(l);
    orderAssumption(ass);
    for (auto &x : ass) {
      x = m_model->GetPrime(x);
    }
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
        std::pair<sptr<cube>, sptr<cube>> partial_pair = get_predecessor(pair, p_ucs);
        sptr<State> cts(new State(nullptr, partial_pair.first, partial_pair.second, 0));
        int cts_lvl = GetNewLevel(cts);
        std::vector<int> cts_ass;
        // int cts_lvl = frame_lvl - 1;
        GetAssumption(cts, cts_lvl, cts_ass);
        // F_i-1 & T & cts'
        if (ctgs < 3 && cts_lvl > 0 && !m_mainSolver->SolveWithAssumption(cts_ass, cts_lvl)) {
          ctgs++;
          auto uc_cts = m_mainSolver->Getuc(false);
          generalize_ctg(uc_cts, cts_lvl, rec_lvl + 1);
          if (AddUnsatisfiableCore(uc_cts, cts_lvl + 1))
            m_overSequence->propagate_uc_from_lvl(uc_cts, cts_lvl + 1);
        } else {
          sptr<cube> temp_uc(new cube());
          std::sort(cts->latches->begin(), cts->latches->end());
          for (auto lit : *uc) {
            if (std::binary_search(cts->latches->begin(), cts->latches->end(), lit)) {
              temp_uc->emplace_back(lit);
            } else {
              if (required_lits.find(lit) != required_lits.end()) return false;
            }
          }
          if (temp_uc->size() == uc->size()) return false;
          ctgs = 0;
          uc->swap(*temp_uc);
        }
      }
    }
  }


  void Propagation() {
    for (int i = m_minUpdateLevel; i < m_overSequence->GetLength() - 1; i++) {
      m_overSequence->propagate(i);
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
  std::shared_ptr<CarSolver> m_mainSolver;
  std::shared_ptr<CarSolver> m_lifts;
  std::shared_ptr<CarSolver> m_deads;
  std::shared_ptr<ISolver> m_invSolver;
  std::shared_ptr<StartSolver> m_startSovler;
};


} // namespace car

#endif