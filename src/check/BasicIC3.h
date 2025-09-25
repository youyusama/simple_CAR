#ifndef BASICIC3_H
#define BASICIC3_H

#include "BaseChecker.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"
#include <memory>
#include <random>
#include <set>
#include <vector>

namespace car {

struct Obligation {
    Obligation(shared_ptr<State> s, int l, int d) : state(s), level(l), depth(d) {
        act = 0.0;
    }
    shared_ptr<State> state;
    int level;
    int depth;
    double act;
};

static bool _obligationPtrComp(const shared_ptr<Obligation> &o1, const shared_ptr<Obligation> &o2) {
    if (o1->level != o2->level) {
        return o1->level < o2->level;
    }
    if (o1->depth != o2->depth) {
        return o1->depth > o2->depth;
    }
    return cubePtrComp()(o1->state->latches, o2->state->latches);
}

struct obligationPtrComp {
  public:
    bool operator()(const shared_ptr<Obligation> &o1, const shared_ptr<Obligation> &o2) const {
        return _obligationPtrComp(o1, o2);
    }
};

struct IC3Frame {
    int k;
    shared_ptr<SATSolver> solver;
    vector<shared_ptr<cube>> borderCubes;
};


class LitSet {
  public:
    LitSet() = default;

    // Ensure bitmap has at least `size` slots.
    void reserve(int size) {
        assert(size >= 0);
        std::size_t need = static_cast<std::size_t>(size);
        if (has_.size() < need) has_.resize(need, 0);
    }

    // Insert a literal; lit must be non-zero.
    void insert(int lit) {
        assert(lit != 0);
        std::size_t i = idx(lit);
        if (i >= has_.size()) has_.resize(i + 1, 0);
        if (!has_[i]) {
            set_.push_back(lit);
            has_[i] = 1;
        }
    }

    // Query membership; lit must be non-zero.
    bool has(int lit) const {
        assert(lit != 0);
        std::size_t i = idx(lit);
        return i < has_.size() && has_[i];
    }

    // O(k) clear over current elements.
    void clear() {
        for (int l : set_) has_[idx(l)] = 0;
        set_.clear();
    }

    // Return number of elements.
    int size() const { return set_.size(); }

  private:
    // Map non-zero literal to bitmap index: +v -> 2*v, -v -> 2*v+1.
    static std::size_t idx(int lit) {
        assert(lit != 0);
        unsigned v = static_cast<unsigned>(lit > 0 ? lit : -lit);
        return (static_cast<std::size_t>(v) << 1) | (lit < 0 ? 1u : 0u);
    }

    std::vector<int> set_;
    std::vector<uint8_t> has_;
};

class BasicIC3 : public BaseChecker {
  public:
    BasicIC3(Settings settings,
             shared_ptr<Model> model,
             shared_ptr<Log> log);
    ~BasicIC3();

    CheckResult Run();
    void Witness();

  private:
    bool Check(int badId);

    bool BaseCases();
    void AddNewFrame();
    void AddBlockingCube(const shared_ptr<cube> &blockingCube, int frameLevel, bool lazyCheck);

    bool Strengthen();
    bool HandleObligations();
    size_t Generalize(const shared_ptr<cube> &cb, int frameLvl);
    void MIC(const shared_ptr<cube> &cb, int frameLvl, int recLvl);
    bool Down(const shared_ptr<cube> &c, int frameLvl, int recLvl, const set<int> &triedLits);
    void GeneralizePredecessor(const shared_ptr<State> &predecessorState, const shared_ptr<cube> &succ);


    inline void GetPrimed(shared_ptr<cube> p) {
        for (auto &x : *p) {
            x = m_model->GetPrimeK(x, 1);
        }
    }
    string FramesInfo() const;
    int PushLemmaForward(const shared_ptr<cube> &cb, int startLevel);

    struct LitOrder {
        shared_ptr<Branching> branching;

        LitOrder() {}

        bool operator()(const int &l1, const int &l2) const {
            return (branching->PriorityOf(l1) > branching->PriorityOf(l2));
        }
    } litOrder;

    void OrderAssumption(const shared_ptr<cube> &c) {
        if (m_settings.randomSeed > 0) {
            shuffle(c->begin(), c->end(), default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        sort(c->begin(), c->end(), litOrder);
    }

    bool Propagate();


    bool EnumerateStartState();

    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube);
    void OutputWitness(int bad);
    void OutputCounterExample();
    void AddSamePrimeConstraints(shared_ptr<SATSolver> slv);


    std::shared_ptr<cube> GetCore(const shared_ptr<SATSolver> &solver, const shared_ptr<cube> &fallbackCube, bool prime);
    bool UnreachabilityCheck(const shared_ptr<cube> &cb, const shared_ptr<SATSolver> &slv);
    bool InductionCheck(const shared_ptr<cube> &cb, const shared_ptr<SATSolver> &slv);
    shared_ptr<cube> GetAndValidateCore(const shared_ptr<SATSolver> &solver, const shared_ptr<cube> &fallbackCube);
    // void InitiationAugmentation(const shared_ptr<cube> &failureCube, const shared_ptr<cube> &fallbackCube);
    bool InitiationCheck(const shared_ptr<cube> &cb);

    shared_ptr<cube> GetBlocker(const shared_ptr<cube> &c, int framelevel);
    int LazyCheck(const shared_ptr<cube> &cb, int startLvl);
    bool SubsumeSet(const shared_ptr<cube> &a, const LitSet &b);
    shared_ptr<Obligation> PopObligation();
    void PushObligation(const shared_ptr<Obligation> &ob, int newLevel);
    int MaxLevel() const {
        return m_frames.size() - 1;
    }
    bool BaseCheck();
    void NewStartSolver();

    CheckResult m_checkResult;

    int m_badId;
    shared_ptr<cube> badCube;

    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<Model> m_model;
    vector<IC3Frame> m_frames;
    shared_ptr<SATSolver> m_liftSolver;
    set<int> m_initialStateSet;
    shared_ptr<State> m_cexStart;
    int m_earliest;
    int lemmaCount;
    int m_invariantLevel;
    shared_ptr<Branching> m_branching;
    set<shared_ptr<Obligation>, obligationPtrComp> m_obligations;

    shared_ptr<SATSolver> m_startSolver;
    shared_ptr<SATSolver> m_badPredLiftSolver;
    LitSet m_tmpLitSet; // used in LazyCheck
};

} // namespace car
#endif // BASICIC3_H