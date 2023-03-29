// oversequence implemented in set

#ifndef OVERSEQUENCESET_H
#define OVERSEQUENCESET_H

#include <algorithm>
#include <fstream>
#include <memory>
#include <set>
#include <time.h>
#include <vector>

#include "Branching.h"
#include "CarSolver.h"
#include "Log.h"
#include "MainSolver.h"
#include "InvSolver.h"

namespace car {

class OverSequenceSet {
public:
  typedef std::vector<int> cube;

  OverSequenceSet(std::shared_ptr<AigerModel> model) {
    m_model = model;
    m_blockSolver = new BlockSolver(model);
    m_block_counter.clear();
    rep_counter = 0;
  }

  void set_log(sptr<Log> log) {
    m_log = log;
  }

  bool Insert(std::shared_ptr<std::vector<int>> uc, int index);

  void Init_Frame_0(sptr<cube> latches);

  void GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int>>> &out);

  bool IsBlockedByFrame_lazy(std::vector<int> &latches, int frameLevel);

  bool IsBlockedByFrame_sat(std::vector<int> &latches, int frameLevel);

  bool IsBlockedByFrame(std::vector<int> &latches, int frameLevel);

  int GetLength();

  void propagate(int level, sptr<Branching> b);

  int propagate_uc_from_lvl(sptr<cube> uc, int lvl, sptr<Branching> b);

  void set_solver(std::shared_ptr<CarSolver> slv);

  std::vector<int> *GetBlocker(std::shared_ptr<std::vector<int>> latches, int framelevel);

  std::vector<cube *> *GetBlockers(std::shared_ptr<std::vector<int>> latches, int framelevel);

  void PrintFramesInfo();

  void PrintOSequence();

  void PrintOSequenceDetail();

  void compute_same_stat();

  void compute_cls_in_fixpoint_ratio(int lvl);

  void compute_monotone_degree();

  void compute_monotone_degree_frame();

  int effectiveLevel;
  bool isForward = false;
  int rep_counter;

private:
  static bool cmp(int a, int b) {
    return abs(a) < abs(b);
  }

  bool is_imply(cube a, cube b);

  static bool _cubeComp(const cube &v1, const cube &v2) {
    if (v1.size() != v2.size()) return v1.size() < v2.size();
    for (size_t i = 0; i < v1.size(); ++i) {
      if (abs(v1[i]) != abs(v2[i]))
        return abs(v1[i]) < abs(v2[i]);
      else if (v1[i] != v2[i])
        return v1[i] > v2[i];
    }
    return false;
  }

  struct cubeComp {
  public:
    bool operator()(const cube &uc1, const cube &uc2) {
      return _cubeComp(uc1, uc2);
    }
  };

  std::set<cube, cubeComp> Ucs;

  class BlockSolver : public CarSolver {
  public:
    BlockSolver(std::shared_ptr<AigerModel> model) {
      m_isForward = true;
      m_model = model;
      m_maxFlag = model->GetMaxId() + 1;
    };
  };

  struct cubepComp {
  public:
    bool operator()(const cube *v1, const cube *v2) {
      return _cubeComp(*v1, *v2);
    }
  };

  typedef std::set<cube *, cubepComp> frame;
  void add_uc_to_frame(const cube *uc, frame &f);

  std::shared_ptr<AigerModel> m_model;

  std::shared_ptr<CarSolver> m_mainSolver;
  std::vector<frame> m_sequence;
  CarSolver *m_blockSolver;
  std::vector<int> m_block_counter;
  sptr<Log> m_log;
};

} // namespace car
#endif