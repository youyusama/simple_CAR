#ifndef BASICIC3_H
#define BASICIC3_H

#include "BaseChecker.h"
#include "IncrCheckerHelpers.h"
#include "Log.h"
#include "SATSolver.h"
#include <memory>
#include <set>
#include <vector>
#include <random>

namespace car {

// A proof obligation: a state to be generalized relative to a frame level.
struct Obligation {
    Obligation(std::shared_ptr<State> s, int l, int d) : state(s), level(l), depth(d) {}
    std::shared_ptr<State> state;
    int level;
    int depth; // Length of CTI trace

    // For priority queue ordering.
    bool operator<(const Obligation &other) const {
        if (level != other.level) {
            return level < other.level;
        }
        return depth < other.depth;
    }
};

// Represents a single frame in the IC3 algorithm.
// Each frame has its own set of clauses and a dedicated SAT solver.
struct IC3Frame {
    int k; // Frame level
    std::shared_ptr<SATSolver> solver;
    std::set<std::shared_ptr<cube>, cubePtrComp> borderCubes; // Renamed to reflect its role
};

class BasicIC3 : public BaseChecker {
  public:
    BasicIC3(Settings settings, std::shared_ptr<Model> model, std::shared_ptr<Log> log);
    ~BasicIC3();

    bool Run() override;

  private:
    // --- Core IC3 Algorithm ---
    bool Check();

    // Phase 1: Foundational Scaffolding
    bool BaseCases();
    void AddNewFrame();
    void AddNewFrames();
    void AddBlockingCube(std::shared_ptr<cube> blockingCube, int frameLevel);

    // --- Phase 2: State Strengthening ---
    bool Strengthen();
    bool HandleObligations(std::set<Obligation>& obligations);
    void Generalize(std::shared_ptr<cube> c, int level);
    void GeneralizePredecessor(std::shared_ptr<State> predecessorState, std::shared_ptr<State> successorState = nullptr);

    // --- Helpers ---
    bool InitiationCheck(const std::shared_ptr<cube>& c);
    std::shared_ptr<cube> GetAndValidateUC(const std::shared_ptr<SATSolver> solver, const std::shared_ptr<cube> fallbackCube);
    std::shared_ptr<cube> GetPrimeCube(const std::shared_ptr<cube>& c);
    std::string FramesInfo() const;
    int PushLemmaForward(std::shared_ptr<cube> c, int startLevel);

    // Phase 2: 直接获取IC3变量的activity
    double GetIC3VariableActivity(int ic3Variable, 
                                  const std::unordered_map<int, double>& solverVarActivities);

    // Phase 3: 基于Variable Activity的Cube排序
    void SortCubeByVariableActivity(std::shared_ptr<cube> c, 
                                   const std::unordered_map<int, double>& solverVarActivities);

    // --- Heuristic Ordering for Generalization ---
    struct InnOrder {
        std::shared_ptr<Model> m;
        InnOrder() {}
        bool operator()(const int &inn_1, const int &inn_2) const {
            return (m->GetInnardsLevel(inn_1) > m->GetInnardsLevel(inn_2));
        }
    } innOrder;

    void OrderAssumption(std::shared_ptr<cube> c) {
        if (m_settings.seed > 0) {
            shuffle(c->begin(), c->end(), std::default_random_engine(m_settings.seed));
            return;
        }
        if (m_settings.internalSignals) {
            if (m_settings.activityDriven && !m_solverVarActivities.empty()) {
                // Activity-driven排序：innards在前按高activity优先，latches在后
                SortCubeByVariableActivity(c, m_solverVarActivities);
            } else {
                // 传统排序：按innards level排序
                std::stable_sort(c->begin(), c->end(), innOrder);
            }
            return;
        }
    }

    // --- Phase 3: Clause Propagation ---
    bool Propagate();

    // --- Phase 4: Counterexample Generation ---
    void GenerateCounterExample();

    // --- Data Members ---
    Settings m_settings;
    std::shared_ptr<Log> m_log;
    std::shared_ptr<Model> m_model;
    std::vector<IC3Frame> m_frames;
    int m_k;
    std::shared_ptr<State> m_cexState;
    std::shared_ptr<SATSolver> m_liftSolver;
    std::set<int> m_initialStateSet;

    // Optimization flags
    bool m_trivial;
    int m_earliest;
    
    // Phase 4: Activity-driven优化
    std::unordered_map<int, double> m_solverVarActivities;
};

} // namespace car
#endif // BASICIC3_H