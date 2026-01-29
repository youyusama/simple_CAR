#ifndef INCRCHECKERHELPERS_H
#define INCRCHECKERHELPERS_H

#include "Log.h"
#include "Model.h"
#include "Settings.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace car {

class Branching {
  public:
    Branching(int type = 1);
    ~Branching();
    void Update(const cube &uc);
    void Decay();
    void Decay(const cube &uc, int gap);

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


struct CubeHash {
    size_t operator()(const vector<int> &cube) const noexcept {
        size_t seed = cube.size();
        for (int i : cube) {
            seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

using frame = std::vector<cube>;
using FrameList = std::vector<frame>;

class OverSequenceSet {
  public:
    using FrameSet = unordered_set<cube, CubeHash>;

    OverSequenceSet(Model &model) : m_model(model) {
        m_invariantLevel = 0;
        m_blockCounter.emplace_back(0);
        m_insertCounter.emplace_back(0);
        m_tmpLitOffset = m_model.NumVar();
        m_tmpLitFlags.assign(static_cast<size_t>(m_tmpLitOffset * 2 + 1), 0);
    }

    void SetInvariantLevel(int lvl) { m_invariantLevel = lvl; }

    int GetInvariantLevel() { return m_invariantLevel; }

    bool Insert(const cube &uc, int index);

    shared_ptr<FrameSet> GetFrame(int lvl);
    frame FrameSetToFrame(const FrameSet &fset) const;

    bool IsBlockedByFrame(const cube &latches, int frameLevel);

    void GetBlockers(const cube &latches, int framelevel, vector<cube> &b);

    bool IsEmpty(int frameLevel) {
        if (frameLevel < 0 || frameLevel >= m_sequence.size()) return true;
        return m_sequence[frameLevel]->empty();
    }

    string FramesInfo();

    string FramesDetail();

  private:
    void CleanupImplied(int frameLevel);

    bool Imply(const cube &a, const cube &b);

    void EnsureTmpLitCapacity(const cube &latches);
    void TmpLitSetInsert(int lit);
    bool TmpLitSetHas(int lit) const;
    void ClearTmpLitSet();

    Model &m_model;
    vector<shared_ptr<FrameSet>> m_sequence;
    vector<int> m_blockCounter;
    vector<int> m_insertCounter;
    int m_invariantLevel;
    vector<uint8_t> m_tmpLitFlags;
    vector<size_t> m_tmpLitList;
    int m_tmpLitOffset = 0;
    static constexpr int kCleanupThreshold = 128;
};


struct State {
    State(shared_ptr<State> inPreState,
          const cube &inInputs,
          const cube &inLatches,
          int inDepth) : depth(inDepth),
                         preState(inPreState),
                         inputs(inInputs),
                         latches(inLatches),
                         dtScore(0) {}
    static int numInputs;
    static int numLatches;

    string GetLatchesString();
    string GetInputsString();

    int depth;
    shared_ptr<State> preState = nullptr;
    cube inputs;
    cube latches;
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
    Task(shared_ptr<State> inState, int inFrameLevel, bool isLocated)
        : state(inState),
          frameLevel(inFrameLevel),
          isLocated(isLocated) {};
    int frameLevel;
    shared_ptr<State> state;
    bool isLocated;
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

    void clear() { m_sequence.clear(); }

    static bool state_ptr_cmp(shared_ptr<State> s1, shared_ptr<State> s2) {
        return s1->dtScore > s2->dtScore;
    }

    vector<shared_ptr<State>> GetSeqDT() {
        vector<shared_ptr<State>> res;
        for (int i = 0; i < m_sequence.size(); ++i) {
            for (int j = 0; j < m_sequence[i].size(); ++j) {
                res.emplace_back(m_sequence[i][j]);
            }
        }
        sort(res.begin(), res.end(), state_ptr_cmp);
        res.resize(res.size() / 5);
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

bool IsStateInInv(const cube &s, const FrameList &inv);


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
        LOG_LP(GLOBAL_LOG, 3, "Restart Check: ", m_ucCounts, " > ", m_threshold);
        return m_ucCounts > m_threshold;
    }

    void UpdateThreshold() {
        if (isLubyActived) {
            m_threshold = m_luby.GetNextLuby() * m_baseThreshold;
        } else {
            m_threshold = m_threshold * m_growthRate;
        }
        LOG_LP(GLOBAL_LOG, 2, "Updated Restart Threshold: ", m_threshold);
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
