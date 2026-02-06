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
    Obligation(shared_ptr<State> s, int l, int d, double a = 0.0)
        : state(s), level(l), depth(d), act(a) {}
    shared_ptr<State> state;
    int level;
    int depth;
    double act;

    bool operator<(const Obligation &other) const {
        if (level != other.level)
            return level < other.level;
        if (depth != other.depth)
            return depth > other.depth;
        return cubeComp(state->latches, other.state->latches);
    }
};

class BasicIC3 : public IncrAlg {
  public:
    BasicIC3(Settings settings,
             Model &model,
             Log &log);
    ~BasicIC3();

    CheckResult Run() override;
    void Witness() override;

    void SetInit(const cube &c) override { m_customInit = c; }
    void SetSearchFromInitSucc(bool b) override { m_searchFromInitSucc = b; }
    void SetLoopRefuting(bool b) override { m_loopRefuting = b; }
    void SetDead(const std::vector<cube> &dead) override { m_dead = dead; }
    void SetShoals(const std::vector<FrameList> &shoals) override { m_shoals = shoals; }
    void SetWalls(const std::vector<FrameList> &walls) override { m_walls = walls; }

    std::vector<std::pair<cube, cube>> GetCexTrace() override;
    FrameList GetInv() override;
    void KLiveIncr() override;

  private:
    bool Check();

    void Init();

    void Reset();

    bool ImmediateSatisfiable();

    bool IsInitStateImplyBad();

    void InitializeStartSolver();

    void AddNewFrame();

    void AddLemma(const cube &blockingCube, int frameLevel);

    void AddLemmaToSolvers(const cube &blockingCube, int beginLevel, int endLevel);

    bool Strengthen();

    bool HandleObligations(set<Obligation> &obligations);

    bool PopObligation(set<Obligation> &obligations, Obligation &ob);

    void PushObligation(set<Obligation> &obligations, Obligation ob, int newLevel);

    int LazyCheck(const cube &cb, int startLvl);

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
    int PropagateUp(const cube &cb, int startLevel);

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

    bool PropagateFrame();

    bool Propagate();

    shared_ptr<State> EnumerateStartState();

    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube);
    void OutputWitness();
    void OutputCounterExample();


    cube GetUnsatCore(const shared_ptr<SATSolver> &solver, const cube &fallbackCube, bool prime);
    bool UnreachabilityCheck(const cube &cb, const shared_ptr<SATSolver> &slv);
    bool InductionCheck(const cube &cb, const shared_ptr<SATSolver> &slv);
    cube GetAndValidateCore(const shared_ptr<SATSolver> &solver, const cube &fallbackCube);
    bool InitiationCheck(const cube &cb);

    CheckResult m_checkResult;

    int m_k;

    Settings m_settings;
    Log &m_log;
    Model &m_model;
    vector<shared_ptr<SATSolver>> m_transSolvers;
    shared_ptr<SATSolver> m_liftSolver;
    shared_ptr<SATSolver> m_startSolver;
    shared_ptr<SATSolver> m_badLiftSolver;
    unordered_set<int> m_initialStateSet;
    LemmaForestManager m_lfm;
    shared_ptr<State> m_initialState;
    shared_ptr<State> m_cexStart;
    int m_minUpdateLevel;
    int m_invariantLevel;
    shared_ptr<Branching> m_branching;

    // liveness
    bool m_initialized{false};
    cube m_customInit;
    bool m_searchFromInitSucc{false};
    bool m_loopRefuting{false};
    std::vector<cube> m_dead;
    std::vector<FrameList> m_shoals;
    std::vector<FrameList> m_walls;
    bool m_initStateImplyBad{false};
    std::vector<std::pair<cube, cube>> m_cexTrace;
    cube m_shoalsLabels;
    cube m_wallsLabels;
};

} // namespace car
#endif // BASICIC3_H
