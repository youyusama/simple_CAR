// oversequence implemented in set

#ifndef OVERSEQUENCESET_H
#define OVERSEQUENCESET_H

#include <algorithm>
#include <fstream>
#include <memory>
#include <set>
#include <time.h>
#include <vector>

#include "CarSolver.h"
#include "MainSolver.h"

namespace car {

class OverSequenceSet {
public:
  OverSequenceSet(std::shared_ptr<AigerModel> model) {
    m_model = model;
    m_blockSolver = new BlockSolver(model);
    m_block_counter.clear();
    rep_counter = 0;
  }

  void Insert(std::shared_ptr<std::vector<int>> uc, int index);

  void GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int>>> &out);

  bool IsBlockedByFrame_lazy(std::vector<int> &latches, int frameLevel);

  bool IsBlockedByFrame(std::vector<int> &latches, int frameLevel);

  int GetLength();

  void propagate(int level);

  void set_solver(std::shared_ptr<ISolver> slv);

  std::vector<int> *GetBlocker(std::shared_ptr<std::vector<int>> latches, int framelevel);

  int effectiveLevel;
  bool isForward = false;
  int rep_counter;

private:
  typedef std::vector<int> cube;

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

  std::shared_ptr<AigerModel> m_model;

  std::shared_ptr<ISolver> m_mainSolver;
  std::vector<frame> m_sequence;
  CarSolver *m_blockSolver;
  std::vector<int> m_block_counter;
};

} // namespace car
#endif