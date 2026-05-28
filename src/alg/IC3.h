#ifndef IC3_H
#define IC3_H

#include "IncrAlg.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"
#include <cstdint>
#include <memory>
#include <random>
#include <set>
#include <vector>

namespace car {

struct Obligation {
    Obligation() = default;
    Obligation(int inId, uint64_t inVersion, shared_ptr<State> s, int l, int d, double a = 0.0)
        : id(inId),
          version(inVersion),
          state(s),
          level(l),
          depth(d),
          act(a) {}

    int id{-1};
    uint64_t version{0};
    shared_ptr<State> state;
    int level{0};
    int depth{0};
    double act{0.0};

    bool operator<(const Obligation &other) const {
        if (level != other.level)
            return level < other.level;
        if (depth != other.depth)
            return depth > other.depth;
        const Cube &lhs = state->latches;
        const Cube &rhs = other.state->latches;
        if (lhs.size() != rhs.size()) return lhs.size() < rhs.size();
        for (size_t i = 0; i < lhs.size(); ++i) {
            if (lhs[i] < rhs[i]) return true;
            if (rhs[i] < lhs[i]) return false;
        }
        if (id != other.id) return id < other.id;
        return version < other.version;
    }
};

struct ObligationRecord {
    shared_ptr<State> state;
    int level{0};
    int depth{0};
    double act{0.0};
    uint64_t version{0};
    bool alive{true};
    bool queued{false};
};

class IC3 : public IncrAlg {
  public:
    IC3(Settings settings,
        Model &model,
        Log &log);
    ~IC3();

    CheckResult Run() override;
    bool SupportsWitness() const override { return true; }
    void RefineWitnessPropertyLit(WitnessBuilder &builder) const override;

    void SetInit(const Cube &c) override { m_customInit = c; }
    void SetSearchFromInitSucc(bool b) override { m_searchFromInitSucc = b; }
    void SetLoopRefuting(bool b) override { m_loopRefuting = b; }
    void SetDead(const std::vector<Cube> &dead) override { m_dead = dead; }
    void SetShoals(const std::vector<FrameList> &shoals) override { m_shoals = shoals; }
    void SetWalls(const std::vector<FrameList> &walls) override { m_walls = walls; }

    std::vector<std::pair<Cube, Cube>> GetCexTrace() override;
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

    int AddLemma(const Cube &blockingCube, int frameLevel, bool fromCTI = false, int obligationId = -1);

    void AddLemmaToSolvers(const Cube &blockingCube, int beginLevel, int endLevel);

    void ActiveLemmaLearning(int newLemmaId);

    std::vector<int> FindHotSpots(const std::vector<int> &ancestorChain);

    void PrintALLStats() const;

    enum class ALLProveStatus {
        Proved,
        Reachable,
        Bailout,
        Invalidated,
    };

    ALLProveStatus ActiveProve(int targetLemmaId);

    bool Strengthen();

    bool HandleObligations();

    int AddObligation(shared_ptr<State> state, int level, int depth, double act = 0.0);

    bool PopObligation(Obligation &ob);

    void BumpObligationAct(int obligationId);

    void PushObligation(int obligationId, int newLevel, bool onlyIfQueued = false);

    void DropObligation(int obligationId);

    int GetSubsumeLevel(const Cube &cb, int startLvl);

    void Generalize(Cube &cb, int frameLvl, int recLvl = 0);

    bool Down(Cube &c, int frameLvl, int recLvl, const LitSet &triedLits, const Cube &fullCube, vector<pair<LitSet, LitSet>> &cexCache);

    bool ExCTGBlock(const Cube &cb, int frameLvl, int recLvl, int blockLimit);

    void GeneralizePredecessor(const shared_ptr<State> &predecessorState, const shared_ptr<State> &successorState);

    inline void GetPrimed(Cube &p) {
        for (auto &x : p) {
            x = m_model.EnsurePrimeK(x, 1);
        }
    }
    string FramesInfo() const;

    string FramesDetail() const;

    struct LitOrder {
        shared_ptr<Branching> branching;

        LitOrder() {}

        bool operator()(Lit l1, Lit l2) const {
            return (branching->PriorityOf(l1) > branching->PriorityOf(l2));
        }
    } m_litOrder;

    void OrderAssumption(Cube &c) {
        if (m_settings.randomSeed > 0) {
            shuffle(c.begin(), c.end(), default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        sort(c.begin(), c.end(), m_litOrder);
    }

    void Extend();

    bool PropagateFrame();

    bool Propagate(int lemmaId, int lvl);

    int PropagateUp(int lemmaId, int startLevel);

    shared_ptr<State> EnumerateStartState();


    Cube GetUnsatCore(const shared_ptr<SATSolver> &solver, const Cube &fallbackCube, bool prime);
    bool IsReachable(const Cube &cb, const shared_ptr<SATSolver> &slv);
    bool IsInductive(const Cube &cb, const shared_ptr<SATSolver> &slv);
    Cube GetAndValidateCore(const shared_ptr<SATSolver> &solver, const Cube &fallbackCube);
    bool InitiationCheck(const Cube &cb);

    bool GetShrunkUnsatCore(const shared_ptr<SATSolver> &solver, Cube &core, const Cube &fallbackCube, bool prime);

    CheckResult m_checkResult;

    int m_k;

    Settings m_settings;
    Log &m_log;
    Model &m_model;
    vector<shared_ptr<SATSolver>> m_transSolvers;
    shared_ptr<SATSolver> m_liftSolver;
    shared_ptr<SATSolver> m_startSolver;
    shared_ptr<SATSolver> m_badLiftSolver;
    unordered_set<Lit, LitHash> m_initialStateSet;
    unordered_set<Lit, LitHash> m_conflictScratch;
    LemmaForestManager m_lfm;
    shared_ptr<State> m_initialState;
    shared_ptr<State> m_cexStart;
    int m_minUpdateLevel;
    int m_invariantLevel;
    shared_ptr<Branching> m_branching;
    std::set<Obligation> m_obligations;
    std::vector<ObligationRecord> m_obligationRecords;

    // all stats
    uint64_t m_allPushAttempted{0};
    uint64_t m_allStatusProved{0};
    uint64_t m_allStatusReachable{0};
    uint64_t m_allStatusBailout{0};
    uint64_t m_allStatusInvalidated{0};

    // liveness
    bool m_initialized{false};
    Cube m_customInit;
    bool m_searchFromInitSucc{false};
    bool m_loopRefuting{false};
    std::vector<Cube> m_dead;
    std::vector<FrameList> m_shoals;
    std::vector<FrameList> m_walls;
    bool m_initStateImplyBad{false};
    std::vector<std::pair<Cube, Cube>> m_cexTrace;
    Cube m_shoalsLabels;
    Cube m_wallsLabels;
};

} // namespace car
#endif // IC3_H
