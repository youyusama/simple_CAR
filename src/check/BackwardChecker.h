#ifndef BACKWARDCHECKER_H
#define BACKWARDCHECKER_H

#include "BaseChecker.h"
#include "IOverSequence.h"
#include "ISolver.h"
#include "InvSolver.h"
#include "Log.h"
#include "MainSolver.h"
#include "OverSequence.h"
#include "OverSequenceForProp.h"
#include "OverSequenceNI.h"
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


class BackwardChecker : public BaseChecker {
public:
  BackwardChecker(Settings settings, std::shared_ptr<AigerModel> model);
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
  void Init();

  void AddUnsatisfiableCore(std::shared_ptr<std::vector<int>> uc, int frameLevel);

  bool ImmediateSatisfiable(int badId);

  bool isInvExisted();

  bool IsInvariant(int frameLevel);

  int GetNewLevel(std::shared_ptr<State> state, int start = 0);

  string GetFileName(string filePath) {
    auto startIndex = filePath.find_last_of("/");
    if (startIndex == string::npos) {
      startIndex = 0;
    } else {
      startIndex++;
    }
    auto endIndex = filePath.find_last_of(".");
    assert(endIndex != string::npos);
    return filePath.substr(startIndex, endIndex - startIndex);
  }

  void GetPriority(std::shared_ptr<std::vector<int>> latches, const int frameLevel, std::vector<int> &res) {
    if (frameLevel + 1 >= m_overSequence->GetLength()) {
      return;
    }
    std::vector<std::shared_ptr<std::vector<int>>> frame;
    m_overSequence->GetFrame(frameLevel + 1, frame);
    if (frame.size() == 0) {
      return;
    }

    std::shared_ptr<std::vector<int>> uc = frame[frame.size() - 1];
    res.reserve(uc->size());
    for (int i = 0; i < uc->size(); ++i) {
      if ((*latches)[abs((*uc)[i]) - m_model->GetNumInputs() - 1] == (*uc)[i]) {
        res.push_back((*uc)[i]);
      }
    }
  }


  void print_vector(std::vector<int> &v, std::string s = "") {
    std::cout << std::endl
              << s << std::endl;
    for (auto l : v) {
      std::cout << l << " ";
    }
  }


  bool static id_comp(int a, int b) {
    if (abs(a) != abs(b))
      return abs(a) < abs(b);
    else
      return a < b;
  }


  void GetAssumptionByPine(std::shared_ptr<State> state, int frameLevel, std::vector<int> &ass) {
    if (state->pine_state_type == 0) {
      std::shared_ptr<std::vector<int>> nextl = m_model->Get_next_latches_for_pine(*state->latches);
      if (nextl->size() / (float)m_model->GetNumLatches() > 0.1) {
        state->pine_state_type = 1;
        build_ass_by_l_list(state, ass);
        return;
      } else
        state->pine_state_type = 2;
    } else if (state->pine_state_type == 1) {
      ass.resize(state->latches->size());
      std::copy(state->pine_assumptions->begin(), state->pine_assumptions->end(), ass.begin());
      return;
    } else if (state->pine_state_type == 2) {
    }
  }


  void build_ass_by_l_list(std::shared_ptr<State> state, std::vector<int> &ass) {
    std::shared_ptr<std::vector<int>> l_0 = state->latches;
    state->pine_l_index.reset(new std::vector<int>());
    state->pine_assumptions.reset(new std::vector<int>());
    state->pine_assumptions->resize(l_0->size());
    ass.resize(l_0->size());
    std::vector<int>::iterator ass_iter = ass.end();
    std::vector<int> l_im1(*l_0); // l_(i-1)
    bool end_flag = true;
    do {
      std::shared_ptr<std::vector<int>> next_l_im1 = m_model->Get_next_latches_for_pine(l_im1);
      std::vector<int> l_i(l_0->size());
      auto iter = std::set_intersection(l_im1.begin(), l_im1.end(), next_l_im1->begin(), next_l_im1->end(), l_i.begin(), id_comp);
      l_i.resize(iter - l_i.begin());
      std::vector<int> l_im1_m_l_i(l_0->size());
      iter = std::set_difference(l_im1.begin(), l_im1.end(), l_i.begin(), l_i.end(), l_im1_m_l_i.begin(), id_comp);
      l_im1_m_l_i.resize(iter - l_im1_m_l_i.begin());
      int temp_length = l_im1_m_l_i.size();
      if (temp_length == 0) {
        state->pine_l_index->emplace_back(l_i.size());
        std::copy(l_i.begin(), l_i.end(), ass.begin());
        end_flag = false;
        state->pine_l_list_type = 1;
        break;
      }
      state->pine_l_index->emplace_back(temp_length);
      ass_iter -= temp_length;
      std::copy(l_im1_m_l_i.begin(), l_im1_m_l_i.end(), ass_iter);
      // print_vector(ass, "ass:===========");
      if (l_i.empty()) {
        end_flag = false;
        state->pine_l_list_type = 0;
      }
      l_i.swap(l_im1);
    } while (end_flag);
    std::reverse(state->pine_l_index->begin(), state->pine_l_index->end());
    for (int i = 1; i < state->pine_l_index->size(); i++) {
      state->pine_l_index->at(i) += state->pine_l_index->at(i - 1);
    }
    std::copy(ass.begin(), ass.end(), state->pine_assumptions->begin());
  }


  void GetAssumption(std::shared_ptr<State> state, int frameLevel, std::vector<int> &ass) {
    if (m_settings.empi) {
      ass.reserve(ass.size() + state->latches->size());
      ass.insert(ass.end(), state->latches->begin(), state->latches->end());
      orderAssumption(ass);
      return;
    }

    if (m_settings.inter) {
      GetPriority(state->latches, frameLevel, ass);
    }

    ass.reserve(ass.size() + state->latches->size());

    if (m_settings.rotate) {
      std::vector<int> tmp;
      tmp.reserve(state->latches->size());
      int aa = m_rotation.size();
      if (frameLevel + 2 > m_rotation.size()) {
        ass.insert(ass.end(), state->latches->begin(), state->latches->end());
        return;
      } else if (m_rotation[frameLevel + 1] == nullptr) {
        ass.insert(ass.end(), state->latches->begin(), state->latches->end());
        return;
      }
      std::vector<int> &cube = *m_rotation[frameLevel + 1];
      for (int i = 0; i < cube.size(); ++i) {
        if ((*state->latches)[abs(cube[i]) - m_model->GetNumInputs() - 1] == cube[i])
          ass.push_back(cube[i]);
        else
          tmp.push_back(-cube[i]);
      }
      ass.insert(ass.end(), tmp.begin(), tmp.end());
    } else {
      ass.insert(ass.end(), state->latches->begin(), state->latches->end());
    }
  }

  void PushToRotation(std::shared_ptr<State> state, int frameLevel) {
    while (frameLevel + 2 > m_rotation.size()) {
      m_rotation.push_back(nullptr);
    }
    m_rotation[frameLevel + 1] = state->latches;
  }

  // void Propagation()
  // {
  // 	OverSequenceForProp* sequence = dynamic_cast<OverSequenceForProp*>(m_overSequence.get());
  // 	for (int frameLevel = 0; frameLevel < sequence->GetLength()-1; ++frameLevel)
  // 	{
  // 		std::vector<std::shared_ptr<std::vector<int> > > unpropFrame = sequence->GetUnProp(frameLevel);
  // 		std::vector<std::shared_ptr<std::vector<int> > > propFrame = sequence->GetProp(frameLevel);
  // 		std::vector<std::shared_ptr<std::vector<int> > > tmp;
  // 		for (int j = 0; j < unpropFrame.size(); ++j)
  // 		{
  // 			if (sequence->IsBlockedByFrame(*unpropFrame[j], frameLevel+1))
  // 			{
  // 				propFrame.push_back(unpropFrame[j]);
  // 				continue;
  // 			}

  // 			bool result = m_mainSolver->SolveWithAssumption(*unpropFrame[j], frameLevel);
  // 			if (!result)
  // 			{
  // 				AddUnsatisfiableCore(unpropFrame[j], frameLevel+1);
  // 				sequence->InsertIntoProped(unpropFrame[j], frameLevel);
  // 			}
  // 			else
  // 			{
  // 				tmp.push_back(unpropFrame[j]);
  // 			}
  // 		}
  // 		tmp.swap(unpropFrame);
  // 	}
  // }

  void Propagation() {
    // for (int i = 0; i < m_overSequence->GetLength()-1; i++){
    // 	m_overSequence->propagate(i);
    // }
  }


  int m_minUpdateLevel;
  std::shared_ptr<OverSequenceNI> m_overSequence;
  // std::shared_ptr<OverSequenceSet> m_overSequence;
  // std::shared_ptr<IOverSequence> m_overSequence;
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