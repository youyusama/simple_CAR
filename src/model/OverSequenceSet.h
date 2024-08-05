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
#include "InvSolver.h"
#include "MainSolver.h"

namespace car {

class OverSequenceSet {
  public:
    typedef vector<int> cube;

    OverSequenceSet(shared_ptr<AigerModel> model) {
        m_model = model;
        m_blockSolver = new BlockSolver(model);
        m_block_counter.clear();
        m_invariantLevel = 0;
    }

    void SetInvariantLevel(int lvl) { m_invariantLevel = lvl; }

    int GetInvariantLevel() { return m_invariantLevel; }

    bool Insert(shared_ptr<vector<int>> uc, int index);

    void Init_Frame_0(shared_ptr<cube> latches);

    void GetFrame(int frameLevel, vector<shared_ptr<vector<int>>> &out);

    bool IsBlockedByFrame_lazy(vector<int> &latches, int frameLevel);

    bool IsBlockedByFrame_sat(vector<int> &latches, int frameLevel);

    bool IsBlockedByFrame(vector<int> &latches, int frameLevel);

    int GetLength();

    void propagate(int level, shared_ptr<Branching> b);

    int propagate_uc_from_lvl(shared_ptr<cube> uc, int lvl, shared_ptr<Branching> b);

    void set_solver(shared_ptr<CarSolver> slv);

    vector<int> *GetBlocker(shared_ptr<vector<int>> latches, int framelevel);

    vector<cube *> *GetBlockers(shared_ptr<vector<int>> latches, int framelevel);

    string FramesInfo();

    string FramesDetail();

    int effectiveLevel;

    bool isForward = false;

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

    set<cube, cubeComp> Ucs;

    class BlockSolver : public CarSolver {
      public:
        BlockSolver(shared_ptr<AigerModel> model) {
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

    typedef set<cube *, cubepComp> frame;
    void add_uc_to_frame(const cube *uc, frame &f);

    shared_ptr<AigerModel> m_model;

    shared_ptr<CarSolver> m_mainSolver;
    vector<frame> m_sequence;
    CarSolver *m_blockSolver;
    vector<int> m_block_counter;
    int m_invariantLevel;
};

} // namespace car
#endif