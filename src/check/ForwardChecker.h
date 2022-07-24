#ifndef FORWARDCHECKER_H
#define FORWARDCHECKER_H

#include "BaseChecker.h"
#include "IOverSequence.h"
#include "ISolver.h"
#include "InvSolver.h"
#include "Log.h"
#include "MainSolver.h"
#include "OverSequence.h"
#include "OverSequenceForProp.h"
#include "OverSequenceNI.h"
#include "StartSolver.h"
#include "State.h"
#include "Task.h"
#include "UnderSequence.h"
#include "Vis.h"
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
  void orderAssumption(std::vector<int> &uc) {
    std::stable_sort(uc.begin(), uc.end(), slimLitOrder);
  }

private:
  void Init(int badId);

  void AddUnsatisfiableCore(std::shared_ptr<std::vector<int>> uc, int frameLevel);

  bool ImmediateSatisfiable(int badId);

  bool isInvExisted();

  bool IsInvariant(int frameLevel);

  int GetNewLevel(std::shared_ptr<State> state, int start = 0);

  void GetAssumption(std::shared_ptr<State> state, int frameLevel, std::vector<int> &ass) {
    if (m_settings.empi) {
      ass.reserve(ass.size() + state->latches->size());
      ass.insert(ass.end(), state->latches->begin(), state->latches->end());
      orderAssumption(ass);
      CAR_DEBUG_v("Assumption: ", ass);
      for (auto &x : ass) {
        x = m_model->GetPrime(x);
      }
      return;
    }
  }

  void Propagation() {
    // OverSequenceForProp *sequence = dynamic_cast<OverSequenceForProp *>(m_overSequence.get());
    // for (int frameLevel = 0; frameLevel < sequence->GetLength() - 1; ++frameLevel) {
    //   std::vector<std::shared_ptr<std::vector<int>>> unpropFrame = sequence->GetUnProp(frameLevel);
    //   std::vector<std::shared_ptr<std::vector<int>>> propFrame = sequence->GetProp(frameLevel);
    //   std::vector<std::shared_ptr<std::vector<int>>> tmp;
    //   for (int j = 0; j < unpropFrame.size(); ++j) {
    //     if (sequence->IsBlockedByFrame(*unpropFrame[j], frameLevel + 1)) {
    //       propFrame.push_back(unpropFrame[j]);
    //       continue;
    //     }

    //     bool result = m_mainSolver->SolveWithAssumption(*unpropFrame[j], frameLevel);
    //     if (!result) {
    //       AddUnsatisfiableCore(unpropFrame[j], frameLevel + 1);
    //       sequence->InsertIntoProped(unpropFrame[j], frameLevel);
    //     } else {
    //       tmp.push_back(unpropFrame[j]);
    //     }
    //   }
    //   tmp.swap(unpropFrame);
    // }
  }

  std::shared_ptr<State> EnumerateStartState() {
    if (m_startSovler->SolveWithAssumption()) {
      return m_startSovler->GetStartState();
    } else {
      return nullptr;
    }
  }

  int m_minUpdateLevel;
  // std::shared_ptr<IOverSequence> m_overSequence;
  std::shared_ptr<OverSequenceNI> m_overSequence;
  // IOverSequence* m_overSequence;
  UnderSequence m_underSequence;
  Settings m_settings;
  std::shared_ptr<Vis> m_vis;
  std::shared_ptr<Log> m_log;
  std::shared_ptr<AigerModel> m_model;
  std::shared_ptr<State> m_initialState;
  std::shared_ptr<ISolver> m_mainSolver;
  std::shared_ptr<ISolver> m_invSolver;
  std::shared_ptr<StartSolver> m_startSovler;
};


} // namespace car

#endif