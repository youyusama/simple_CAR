// oversequence with out/less imply

#ifndef OVERSEQUENCENI_H
#define OVERSEQUENCENI_H

#include <vector>
#include <set>
#include <memory>
#include <algorithm>
#include <fstream>

#include "CarSolver.h"
#include "MainSolver.h"

namespace car
{

class OverSequenceNI
{
public:
	OverSequenceNI() {
    implyQuerryTimes = 0;
	  implyTimes = 0;
    blockQuerryTimes = 0;
    blockedTimes = 0;
    m_blockSolvers.clear();
  }

	void Insert(std::shared_ptr<std::vector<int> > uc, int index, std::shared_ptr<AigerModel> model);
	
	void GetFrame(int frameLevel, std::vector<std::shared_ptr<std::vector<int> > >& out);
	
	bool IsBlockedByFrame(std::vector<int>& latches, int frameLevel);
	
	int GetLength();

  void propagate(int level);

  int effectiveLevel;
	bool isForward = false;

	std::vector<int> GetStat();

	int blockQuerryTimes;
	int blockedTimes;
	int implyQuerryTimes;
	int implyTimes;

private:
  typedef std::vector<int> cube;

  class BlockSolver : public CarSolver
  {
  public:
    BlockSolver(std::shared_ptr<AigerModel> model){
      m_isForward = true;
      m_model = model;
      m_maxFlag = model->GetMaxId()+1;
    };
  private:
  };

  static bool _cubeComp(const cube & v1, const cube & v2) {
    if (v1.size() < v2.size()) return true;
    if (v1.size() > v2.size()) return false;
    for (size_t i = 0; i < v1.size(); ++i) {
      if (v1[i] < v2[i]) return true;
      if (v2[i] < v1[i]) return false;
    }
    return false;
  }

  struct cubeComp {
  public:
    bool operator()(const cube & v1, const cube & v2) {
      return _cubeComp(v1, v2);
    }
  };

  typedef std::set<cube, cubeComp> frame;

  std::vector<frame> m_sequence;
  std::vector<CarSolver*> m_blockSolvers;
};

}//namespace car
#endif