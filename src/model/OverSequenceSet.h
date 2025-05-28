// oversequence implemented in set

#ifndef OVERSEQUENCESET_H
#define OVERSEQUENCESET_H

#include <algorithm>
#include <memory>
#include <set>
#include <vector>

#include "Branching.h"
#ifdef CADICAL
#include "CadicalSolver.h"
#else
#include "MinisatSolver.h"
#endif

using namespace std;

namespace car {

static bool _cubePtrComp(const shared_ptr<cube> &c1, const shared_ptr<cube> &c2) {
    if (c1->size() != c2->size()) return c1->size() < c2->size();
    for (size_t i = 0; i < c1->size(); ++i) {
        int v1 = c1->at(i), v2 = c2->at(i);
        if (abs(v1) != abs(v2))
            return abs(v1) < abs(v2);
        else if (v1 != v2)
            return v1 > v2;
    }
    return false;
}

struct cubePtrComp {
  public:
    bool operator()(const shared_ptr<cube> &c1, const shared_ptr<cube> &c2) {
        return _cubePtrComp(c1, c2);
    }
};

typedef set<shared_ptr<cube>, cubePtrComp> frame;


class BlockSolver :
#ifdef CADICAL
    public CadicalSolver
#else
    public MinisatSolver
#endif
{
  public:
    BlockSolver(shared_ptr<Model> model) {
        m_maxId = model->GetMaxId();
    };
    bool SolveFrame(const shared_ptr<cube> assumption, int frameLevel);
    void AddUC(const cube &uc, int frameLevel);

  private:
    inline int GetFrameFlag(int frameLevel);
    vector<int> m_frameFlags;
};


class OverSequenceSet {
  public:
    typedef vector<int> cube;

    OverSequenceSet(shared_ptr<Model> model) {
        m_model = model;
        m_blockSolver = make_shared<BlockSolver>(m_model);
        m_blockCounter.clear();
        m_invariantLevel = 0;
    }

    void SetInvariantLevel(int lvl) { m_invariantLevel = lvl; }

    int GetInvariantLevel() { return m_invariantLevel; }

    bool Insert(shared_ptr<cube> uc, int index, bool need_imply = true);

    void GetFrame(int frameLevel, vector<shared_ptr<cube>> &f);

    shared_ptr<frame> GetFrame(int lvl) { return m_sequence[lvl]; };

    bool IsBlockedByFrame_lazy(shared_ptr<cube> latches, int frameLevel);

    bool IsBlockedByFrame_sat(shared_ptr<cube> latches, int frameLevel);

    bool IsBlockedByFrame(shared_ptr<cube> latches, int frameLevel);

    int GetLength();

    void GetBlockers(shared_ptr<cube> latches, int framelevel, vector<shared_ptr<cube>> &b);

    string FramesInfo();

    string FramesDetail();

    int effectiveLevel;

  private:
    bool is_imply(shared_ptr<cube> a, shared_ptr<cube> b);

    void add_uc_to_frame(const shared_ptr<cube> uc, shared_ptr<frame> f);

    shared_ptr<Model> m_model;
    set<shared_ptr<cube>, cubePtrComp> m_UCSet;
    vector<shared_ptr<frame>> m_sequence;
    shared_ptr<BlockSolver> m_blockSolver;
    vector<int> m_blockCounter;
    int m_invariantLevel;
};

} // namespace car
#endif