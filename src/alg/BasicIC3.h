#ifndef BASICIC3_H
#define BASICIC3_H

#include "IncrAlg.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"
#include <memory>
#include <random>
#include <set>
#include <vector>

namespace car {

struct Obligation {
    Obligation(shared_ptr<State> s, int l, int d) : state(s), level(l), depth(d) {}
    shared_ptr<State> state;
    int level;
    int depth;

    bool operator<(const Obligation &other) const {
        if (level != other.level) {
            return level < other.level;
        }
        return depth < other.depth;
    }
};

struct IC3Frame {
    int k;
    shared_ptr<SATSolver> solver;
    struct CubeComp {
        bool operator()(const cube &a, const cube &b) const {
            if (a.size() != b.size()) return a.size() < b.size();
            for (size_t i = 0; i < a.size(); ++i) {
                int v1 = a[i], v2 = b[i];
                if (abs(v1) != abs(v2))
                    return abs(v1) < abs(v2);
                else if (v1 != v2)
                    return v1 > v2;
            }
            return false;
        }
    };
    set<cube, CubeComp> borderCubes;
};

class BasicIC3 : public IncrAlg {
  public:
    BasicIC3(Settings settings,
             Model &model,
             Log &log);
    ~BasicIC3();

    CheckResult Run() override;
    void Witness() override;

    void SetInit(const cube &c) override;
    void SetSearchFromInitSucc(bool b) override;
    void SetLoopRefuting(bool b) override;
    void SetDead(const std::vector<cube> &dead) override;
    void SetShoals(const std::vector<FrameList> &shoals) override;
    void SetWalls(const std::vector<FrameList> &walls) override;

    cube GetReachedTarget() override;
    std::vector<std::pair<cube, cube>> GetCexTrace() override;
    FrameList GetInv() override;
    void KLiveIncr() override;
    int GetDepth() override;

  private:
    bool Check(int badId);
    void ApplyExternalCubes(const shared_ptr<SATSolver> &solver);
    bool IsStateImplyBad();
    bool IsLivenessWallDuplicated();
    void AddWallConstraints(SATSolver *solver);
    void AddShoalConstraints(SATSolver *solver);
    bool GetInit(cube &out);
    bool PruneDead();
    bool IsDeadState(const cube &c);

    bool BaseCases();
    void AddNewFrame();
    void AddNewFrames();
    void AddBlockingCube(const cube &blockingCube, int frameLevel, bool toAll);

    bool Strengthen();
    bool HandleObligations(set<Obligation> &obligations);
    size_t Generalize(cube &cb, int frameLvl);
    bool MIC(cube &cb, int frameLvl, int recLvl);
    bool Down(cube &c, int frameLvl, int recLvl, const set<int> &triedLits);
    void GeneralizePredecessor(const shared_ptr<State> &predecessorState, const shared_ptr<State> &successorState);


    inline void GetPrimed(cube &p) {
        for (auto &x : p) {
            x = m_model.GetPrimeK(x, 1);
        }
    }
    string FramesInfo() const;
    int PushLemmaForward(const cube &cb, int startLevel);

    struct LitOrder {
        shared_ptr<Branching> branching;

        LitOrder() {}

        bool operator()(const int &l1, const int &l2) const {
            return (branching->PriorityOf(l1) > branching->PriorityOf(l2));
        }
    } litOrder;

    struct BlockerOrder {
        shared_ptr<Branching> branching;

        BlockerOrder() {}

        bool operator()(const cube &a, const cube &b) const {
            float score_a = 0, score_b = 0;
            for (int i = 0; i < a.size(); i++) {
                score_a += branching->PriorityOf(a[i]);
                score_b += branching->PriorityOf(b[i]);
            }
            return score_a > score_b;
        }
    } blockerOrder;

    void OrderAssumption(cube &c) {
        if (m_settings.randomSeed > 0) {
            shuffle(c.begin(), c.end(), default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        sort(c.begin(), c.end(), litOrder);
    }

    void Extend();
    bool Propagate();


    shared_ptr<State> EnumerateStartState();

    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube);
    void OutputWitness(int bad);
    void OutputCounterExample();


    cube GetCore(const shared_ptr<SATSolver> &solver, const cube &fallbackCube, bool prime);
    bool UnreachabilityCheck(const cube &cb, const shared_ptr<SATSolver> &slv);
    bool InductionCheck(const cube &cb, const shared_ptr<SATSolver> &slv);
    cube GetAndValidateCore(const shared_ptr<SATSolver> &solver, const cube &fallbackCube);
    void InitiationAugmentation(const cube &failureCube, const cube &fallbackCube);
    bool InitiationCheck(const cube &cb);

    void GetBlockers(const cube &c, int framelevel, vector<cube> &blockers);

    CheckResult m_checkResult;

    int m_badId;
    int m_k;

    Settings m_settings;
    Log &m_log;
    Model &m_model;
    vector<IC3Frame> m_frames;
    shared_ptr<SATSolver> m_liftSolver;
    shared_ptr<SATSolver> m_startSolver;
    shared_ptr<SATSolver> m_badPredLiftSolver;
    set<int> m_initialStateSet;
    shared_ptr<State> m_cexStart;
    bool m_trivial;
    int m_earliest;
    int lemmaCount;
    int m_invariantLevel;
    shared_ptr<Branching> m_branching;

    vector<int> m_defaultInit;
    cube m_customInit;
    cube m_reachedTarget;
    bool m_searchFromInitSucc = false;
    bool m_loopRefuting = false;
    const std::vector<cube> *m_dead = nullptr;
    const std::vector<FrameList> *m_shoals = nullptr;
    const std::vector<FrameList> *m_walls = nullptr;
    bool m_stateImplyBad = false;
    bool m_hasDuplicatedWall = false;
    int m_shoalUnroll = 1;
    std::vector<cube> m_newDead;
};

} // namespace car
#endif // BASICIC3_H
