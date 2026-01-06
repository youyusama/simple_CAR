#ifndef INCRCHECKERHELPERS_H
#define INCRCHECKERHELPERS_H

#include "Log.h"
#include "Model.h"
#include "SATSolver.h"
#include "Settings.h"
#include <algorithm>
#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace car {

class Branching {
  public:
    Branching(int type = 1);
    ~Branching();
    void Update(const shared_ptr<cube> uc);
    void Decay();
    void Decay(const shared_ptr<cube> uc, int gap);

    inline float PriorityOf(int lit) {
        if (abs(lit) >= counts.size()) return 0;
        return counts[abs(lit)];
    }

  private:
    int branching_type; // 1: sum 2: vsids 3: acids 4: MAB (to do) 0: static
    int conflict_index;
    int mini;
    std::vector<float> counts;
};


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
    bool operator()(const shared_ptr<cube> &c1, const shared_ptr<cube> &c2) const {
        return _cubePtrComp(c1, c2);
    }
};

struct CubeHash {
    size_t operator()(const vector<int> &cube) const noexcept {
        size_t seed = cube.size();
        for (int i : cube) {
            seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

using frame = unordered_set<cube, CubeHash>;

class OverSequenceSet {
  public:
    OverSequenceSet(Model &model) : m_model(model) {
        m_blockSolver = make_shared<SATSolver>(model, MCSATSolver::minisat);
        m_invariantLevel = 0;
        m_blockCounter.emplace_back(0);
    }

    void SetInvariantLevel(int lvl) { m_invariantLevel = lvl; }

    int GetInvariantLevel() { return m_invariantLevel; }

    bool Insert(const cube &uc, int index, bool implyCheck = false);

    shared_ptr<frame> GetFrame(int lvl);

    bool IsBlockedByFrame(const cube &latches, int frameLevel);

    bool IsBlockedByFrameLazy(const cube &latches, int frameLevel);

    void GetBlockers(const cube &latches, int framelevel, vector<cube> &b);

    bool IsEmpty(int frameLevel) {
        if (frameLevel < 0 || frameLevel >= m_sequence.size()) return true;
        return m_sequence[frameLevel]->empty();
    }

    string FramesInfo();

    string FramesDetail();

  private:
    bool Imply(const cube &a, const cube &b);

    Model &m_model;
    vector<shared_ptr<frame>> m_sequence;
    shared_ptr<SATSolver> m_blockSolver;
    vector<int> m_blockCounter;
    int m_invariantLevel;
};


class State {
  public:
    State(shared_ptr<State> inPreState,
          shared_ptr<cube> inInputs,
          shared_ptr<cube> inLatches,
          int inDepth) : preState(inPreState),
                         inputs(inInputs),
                         latches(inLatches),
                         depth(inDepth),
                         dtScore(0) {}
    static int numInputs;
    static int numLatches;

    string GetLatchesString();
    string GetInputsString();

    int depth;
    shared_ptr<State> preState = nullptr;
    shared_ptr<cube> inputs;
    shared_ptr<cube> latches;
    double dtScore;

    void HasUC() {
        dtScore += 0.7;
    }

    void HasSucc() {
        dtScore += 0.3;
    }
};


struct Task {
  public:
    Task(shared_ptr<State> inState, int inFrameLevel) : state(inState),
                                                        frameLevel(inFrameLevel) {};
    int frameLevel;
    shared_ptr<State> state;
};


class UnderSequence {
  public:
    UnderSequence() {}
    ~UnderSequence() {
        for (int i = 0; i < m_sequence.size(); ++i) {
            m_sequence.clear();
        }
    }

    void push(shared_ptr<State> state) {
        while (m_sequence.size() <= state->depth) {
            m_sequence.emplace_back(vector<shared_ptr<State>>());
        }
        m_sequence[state->depth].emplace_back(state);
    }

    int size() { return m_sequence.size(); }

    static bool state_ptr_cmp(shared_ptr<State> s1, shared_ptr<State> s2) {
        return s1->dtScore > s2->dtScore;
    }

    shared_ptr<vector<shared_ptr<State>>> GetSeqDT() {
        shared_ptr<vector<shared_ptr<State>>> res(new vector<shared_ptr<State>>());
        for (int i = 0; i < m_sequence.size(); ++i) {
            for (int j = 0; j < m_sequence[i].size(); ++j) {
                res->emplace_back(m_sequence[i][j]);
            }
        }
        sort(res->begin(), res->end(), state_ptr_cmp);
        res->resize(res->size() / 5);
        return res;
    }

    vector<shared_ptr<State>> &operator[](int i) { return m_sequence[i]; }

  private:
    vector<vector<shared_ptr<State>>> m_sequence;
};

struct Luby {
  public:
    Luby() {
        m_index = 0;
    }

    int GetLuby(int index) {
        int distance = index + 1 - m_luby.size();
        if (distance > 0) {
            PushLuby(distance);
        }
        return m_luby[index];
    }

    int GetNextLuby() {
        int distance = m_index + 1 - m_luby.size();
        if (distance > 0) {
            PushLuby(distance);
        }
        return m_luby[m_index++];
    }

    void PushLuby(int numbers = 1) {
        int size = m_luby.size();
        int k;
        for (int i = 0; i < numbers; ++i) {
            k = GetLog2(size + 2);
            if (size + 2 == GetPower2(k)) {
                m_luby.push_back(GetPower2(k - 1));
            } else {
                m_luby.push_back(m_luby[size - GetPower2(k) + 1]);
            }
            size++;
        }
    }

  private:
    int GetLog2(int num) {
        if (num < 1) {
            return -1;
        }
        int power = 0;
        while (num > 1) {
            num = num / 2;
            power++;
        }
        return power;
    }

    int GetPower2(int power) {
        int res = 1;
        while (power > 0) {
            res = res * 2;
            power--;
        }
        return res;
    }

    std::vector<int> m_luby;
    int m_index;
};


class Restart {
  public:
    Restart(Settings settings) {
        if (settings.restartLuby) {
            isLubyActived = true;
            m_luby.PushLuby(15);
        }
        m_baseThreshold = settings.restartThreshold;
        m_threshold = settings.restartThreshold;
        m_growthRate = settings.restartGrowthRate;
    }

    bool RestartCheck() {
        GLOBAL_LOG->L(3, "Restart Check: ", m_ucCounts, " > ", m_threshold);
        return m_ucCounts > m_threshold;
    }

    void UpdateThreshold() {
        if (isLubyActived) {
            m_threshold = m_luby.GetNextLuby() * m_baseThreshold;
        } else {
            m_threshold = m_threshold * m_growthRate;
        }
        GLOBAL_LOG->L(2, "Updated Restart Threshold: ", m_threshold);
    }

    void UcCountsPlus1() { m_ucCounts++; }

    void ResetUcCounts() { m_ucCounts = 0; }

  private:
    bool isLubyActived = false;
    int m_threshold;
    int m_baseThreshold;
    int m_ucCounts = 0;
    float m_growthRate = 1.5;
    Luby m_luby;
};

} // namespace car

#endif
