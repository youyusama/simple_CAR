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
#include <utility>
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


static bool cubeComp(const cube &c1, const cube &c2) {
    if (c1.size() != c2.size()) return c1.size() < c2.size();
    for (size_t i = 0; i < c1.size(); i++) {
        int v1 = c1[i], v2 = c2[i];
        if (abs(v1) != abs(v2))
            return abs(v1) < abs(v2);
        else if (v1 != v2)
            return v1 > v2;
    }
    return false;
}


struct CubeHash {
    size_t operator()(const vector<int> &cube) const noexcept {
        size_t seed = cube.size();
        for (int i : cube) {
            seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class LitSet {
  public:
    LitSet() = default;

    void newSet(const cube &c) {
        clear();
        for (int lit : c) {
            insert(lit);
        }
    }

    void insert(int lit) {
        assert(lit != 0);
        std::size_t i = idx(lit);
        if (i >= has_.size()) has_.resize(i + 1, 0);
        if (!has_[i]) {
            set_.push_back(lit);
            has_[i] = 1;
        }
    }

    bool has(int lit) const {
        assert(lit != 0);
        std::size_t i = idx(lit);
        return i < has_.size() && has_[i];
    }

    void clear() {
        for (int l : set_) has_[idx(l)] = 0;
        set_.clear();
    }

    int size() const { return static_cast<int>(set_.size()); }

  private:
    static std::size_t idx(int lit) {
        assert(lit != 0);
        unsigned v = static_cast<unsigned>(lit > 0 ? lit : -lit);
        return (static_cast<std::size_t>(v) << 1) | (lit < 0 ? 1u : 0u);
    }

    std::vector<int> set_;
    std::vector<uint8_t> has_;
};


inline bool SubsumeSet(const cube &a, const LitSet &b) {
    if (a.size() > static_cast<size_t>(b.size())) return false;
    for (int lit : a) {
        if (!b.has(lit)) return false;
    }
    return true;
}

struct ForestNode {
    ForestNode() = default;

    ForestNode(int inParentId, int inFrameLvl, int inDepth)
        : parentId(inParentId),
          frameLvl(inFrameLvl),
          depth(inDepth) {}

    int parentId{-1};
    std::vector<int> childrenIds;
    int frameLvl{0};
    int depth{0};

    int refineCount{0};
    int refineCountSinceALL{0};
    bool reachable{false};

    std::vector<std::pair<cube, int>> ctpPreds;
    cube ctpSucc;
    bool hasCTPSucc{false};
};

struct AddLemmaResult {
    int lemmaId{-1};
    int beginLevel{0};
    int endLevel{0};
};

class LemmaForestManager {
  public:
    LemmaForestManager() = default;

    void Reset();
    void EnsureLevel(int level);

    AddLemmaResult AddLemma(const cube &cb, int frameLevel);
    int PropagateLemma(int lemmaId, int newFrameLevel);

    void GetBlockers(const cube &blockingCube, int level, std::vector<cube> &blockers) const;
    bool IsBlockedAtLevel(const cube &cb, int level) const;
    std::vector<int> GetAncestorChain(int lemmaId) const;
    int FrameLevelOf(int lemmaId) const;
    int ParentOf(int lemmaId) const;
    int RefineCountSinceALL(int lemmaId) const;
    void ResetRefineCountSinceALL(int lemmaId);
    bool Reachable(int lemmaId) const;
    void SetReachable(int lemmaId, bool value = true);
    bool PopCTPPred(int lemmaId, cube &ctpCube, int &ctpLevel);
    void PushCTPPred(int lemmaId, const cube &ctpCube, int ctpLevel);
    bool HasCTPPreds(int lemmaId) const;
    void ClearCTPState(int lemmaId);

    const std::vector<int> &BorderIds(int level) const;
    bool BorderEmpty(int level) const;
    size_t BorderSize(int level) const;
    void CleanDeadBorders(int level);
    void SortBorderByCubeSize(int level);

    const cube &CubeOf(int id) const;
    bool Alive(int id) const;

    class BorderCubesRange {
      public:
        struct Iterator {
            const LemmaForestManager *lfm;
            int level;
            size_t idx;

            void SkipDead();
            const cube &operator*() const;
            Iterator &operator++();
            bool operator!=(const Iterator &other) const;
        };

        BorderCubesRange(const LemmaForestManager *inLfm, int inLevel)
            : lfm(inLfm), level(inLevel) {}

        Iterator begin() const;
        Iterator end() const;

      private:
        const LemmaForestManager *lfm;
        int level;
    };

    BorderCubesRange BorderCubes(int level) const;

  private:
    std::pair<int, int> FindParentLemma(int startLevel, const cube &cb);
    int CreateLemma(const cube &cb, int parentId, int frameLevel);
    int SwapCreateLemma(const cube &cb, int existingLemmaId, int frameLevel);
    void AddLemmaToBorder(int frameLevel, int lemmaId);
    void RemoveFromBorder(int level, int lemmaId);
    void UnregisterLemma(int lemmaId);
    void AdoptRelations(int newLemmaId, int oldLemmaId);
    uint64_t RemoveRedundantLemmas(int startLevel, int endLevel, int newLemmaId);
    void UpdateRefineCountersOnInsert(int newLemmaId);

    std::vector<cube> m_lemmas;
    std::vector<ForestNode> m_forest;
    std::vector<uint8_t> m_alive;
    std::vector<std::vector<int>> m_borders;
    mutable LitSet m_tmpLitSet;
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
