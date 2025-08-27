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
    set<shared_ptr<cube>, cubePtrComp> borderCubes;
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
    void AddNewFrames();
    void AddBlockingCube(shared_ptr<cube> blockingCube, int frameLevel);

    bool Strengthen();
    bool HandleObligations(set<Obligation> &obligations);
    void Generalize(shared_ptr<cube> c, int level);
    void GeneralizePredecessor(shared_ptr<State> predecessorState, shared_ptr<State> successorState = nullptr);

    bool InitiationCheck(const shared_ptr<cube> &c);
    shared_ptr<cube> GetAndValidateUC(const shared_ptr<SATSolver> solver, const shared_ptr<cube> fallbackCube);
    shared_ptr<cube> GetPrimeCube(const shared_ptr<cube> &c);
    string FramesInfo() const;
    int PushLemmaForward(shared_ptr<cube> c, int startLevel);

    void OrderAssumption(shared_ptr<cube> c) {
        if (m_settings.randomSeed > 0) {
            shuffle(c->begin(), c->end(), default_random_engine(m_settings.randomSeed));
            return;
        }
        if (m_settings.branching == 0) return;
        // branching
    }

    bool Propagate();

    unsigned addCubeToANDGates(aiger *circuit, vector<unsigned> cube);
    void OutputWitness(int bad);
    void OutputCounterExample();
    void AddSamePrimeConstraints(shared_ptr<SATSolver> slv);

    CheckResult m_checkResult;

    int m_badId;
    int m_k;

    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<Model> m_model;
    vector<IC3Frame> m_frames;
    shared_ptr<SATSolver> m_liftSolver;
    set<int> m_initialStateSet;
    shared_ptr<State> m_cexStart;
    bool m_trivial;
    int m_earliest;

    int m_invariantLevel;
};

} // namespace car
#endif // BASICIC3_H