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
    void Update(const Cube &uc);
    void Decay();
    void Decay(const Cube &uc, int gap);

    inline float PriorityOf(Lit lit) {
        Var lit_var = VarOf(lit);
        if (lit_var >= m_counts.size()) return 0;
        return m_counts[lit_var];
    }

  private:
    int m_branchingType; // 1: sum 2: vsids 3: acids 4: MAB (to do) 0: static
    int m_conflictIndex;
    int m_mini;
    std::vector<float> m_counts;
};

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

    std::vector<std::pair<Cube, int>> ctpPreds;
    Cube ctpSucc;
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

    AddLemmaResult AddLemma(const Cube &cb, int frameLevel);
    int PropagateLemma(int lemmaId, int newFrameLevel);

    void GetBlockers(const Cube &blockingCube, int level, std::vector<Cube> &blockers) const;
    bool IsBlockedAtLevel(const Cube &cb, int level) const;
    std::vector<int> GetAncestorChain(int lemmaId) const;
    int FrameLevelOf(int lemmaId) const;
    int ParentOf(int lemmaId) const;
    int RefineCountSinceALL(int lemmaId) const;
    void ResetRefineCountSinceALL(int lemmaId);
    bool Reachable(int lemmaId) const;
    void SetReachable(int lemmaId, bool value = true);
    bool PopCTPPred(int lemmaId, Cube &ctpCube, int &ctpLevel);
    void PushCTPPred(int lemmaId, const Cube &ctpCube, int ctpLevel);
    bool HasCTPPreds(int lemmaId) const;
    void ClearCTPState(int lemmaId);

    const std::vector<int> &BorderIds(int level) const;
    bool BorderEmpty(int level) const;
    size_t BorderSize(int level) const;
    void CleanDeadBorders(int level);
    void SortBorderByCubeSize(int level);

    const Cube &CubeOf(int id) const;
    bool Alive(int id) const;

    class BorderCubesRange {
      public:
        struct Iterator {
            const LemmaForestManager *lfm;
            int level;
            size_t idx;

            void SkipDead();
            const Cube &operator*() const;
            Iterator &operator++();
            bool operator!=(const Iterator &other) const;
        };

        BorderCubesRange(const LemmaForestManager *inLfm, int inLevel)
            : m_lfm(inLfm), m_level(inLevel) {}

        Iterator begin() const;
        Iterator end() const;

      private:
        const LemmaForestManager *m_lfm;
        int m_level;
    };

    BorderCubesRange BorderCubes(int level) const;

  private:
    std::pair<int, int> FindParentLemma(int startLevel, const Cube &cb);
    int CreateLemma(const Cube &cb, int parentId, int frameLevel);
    int SwapCreateLemma(const Cube &cb, int existingLemmaId, int frameLevel);
    void AddLemmaToBorder(int frameLevel, int lemmaId);
    void RemoveFromBorder(int level, int lemmaId);
    void UnregisterLemma(int lemmaId);
    void AdoptRelations(int newLemmaId, int oldLemmaId);
    uint64_t RemoveRedundantLemmas(int startLevel, int endLevel, int newLemmaId);
    void UpdateRefineCountersOnInsert(int newLemmaId);

    std::vector<Cube> m_lemmas;
    std::vector<ForestNode> m_forest;
    std::vector<uint8_t> m_alive;
    std::vector<std::vector<int>> m_borders;
    mutable LitSet m_tmpLitSet;
};


using Frame = std::vector<Cube>;
using FrameList = std::vector<Frame>;

class OverSequenceSet {
  public:
    using FrameSet = unordered_set<Cube, CubeHash>;

    OverSequenceSet(Model &model) : m_model(model) {
        m_invariantLevel = 0;
        m_blockCounter.emplace_back(0);
        m_insertCounter.emplace_back(0);
    }

    void SetInvariantLevel(int lvl) { m_invariantLevel = lvl; }

    int GetInvariantLevel() { return m_invariantLevel; }

    bool Insert(const Cube &uc, int index);

    shared_ptr<FrameSet> GetFrame(int lvl);
    Frame FrameSetToFrame(const FrameSet &fset) const;

    bool IsBlockedByFrame(const Cube &latches, int frameLevel);

    void GetBlockers(const Cube &latches, int frameLevel, vector<Cube> &b);

    bool IsEmpty(int frameLevel) {
        if (frameLevel < 0 || frameLevel >= m_sequence.size()) return true;
        return m_sequence[frameLevel]->empty();
    }

    string FramesInfo();

    string FramesDetail();

  private:
    void CleanupImplied(int frameLevel);

    bool Imply(const Cube &a, const Cube &b);

    Model &m_model;
    vector<shared_ptr<FrameSet>> m_sequence;
    vector<int> m_blockCounter;
    vector<int> m_insertCounter;
    int m_invariantLevel;
    LitSet m_tmpLitSet;
    static constexpr int K_CLEANUP_THRESHOLD = 128;
};


struct State {
    State(shared_ptr<State> inPreState,
          const Cube &inInputs,
          const Cube &inLatches,
          int inDepth) : depth(inDepth),
                         preState(inPreState),
                         inputs(inInputs),
                         latches(inLatches),
                         dtScore(0) {}
    static int num_inputs;
    static int num_latches;

    string GetLatchesString();
    string GetInputsString();

    int depth;
    shared_ptr<State> preState = nullptr;
    Cube inputs;
    Cube latches;
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

    void Push(shared_ptr<State> state) {
        while (m_sequence.size() <= state->depth) {
            m_sequence.emplace_back(vector<shared_ptr<State>>());
        }
        m_sequence[state->depth].emplace_back(state);
    }

    int Size() { return m_sequence.size(); }

    void Clear() { m_sequence.clear(); }

    static bool StatePtrCmp(shared_ptr<State> s1, shared_ptr<State> s2) {
        return s1->dtScore > s2->dtScore;
    }

    vector<shared_ptr<State>> GetSeqDT() {
        vector<shared_ptr<State>> res;
        for (int i = 0; i < m_sequence.size(); ++i) {
            for (int j = 0; j < m_sequence[i].size(); ++j) {
                res.emplace_back(m_sequence[i][j]);
            }
        }
        sort(res.begin(), res.end(), StatePtrCmp);
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

bool IsStateInInv(const Cube &s, const FrameList &inv);


class Restart {
  public:
    Restart(Settings settings) {
        if (settings.restartLuby) {
            m_isLubyActive = true;
            m_luby.PushLuby(15);
        }
        m_baseThreshold = settings.restartThreshold;
        m_threshold = settings.restartThreshold;
        m_growthRate = settings.restartGrowthRate;
    }

    bool RestartCheck() {
        LOG_LP(global_log, 3, "Restart Check: ", m_ucCounts, " > ", m_threshold);
        return m_ucCounts > m_threshold;
    }

    void UpdateThreshold() {
        if (m_isLubyActive) {
            m_threshold = m_luby.GetNextLuby() * m_baseThreshold;
        } else {
            m_threshold = m_threshold * m_growthRate;
        }
        LOG_LP(global_log, 2, "Updated Restart Threshold: ", m_threshold);
    }

    void UcCountsPlus1() { m_ucCounts++; }

    void ResetUcCounts() { m_ucCounts = 0; }

  private:
    bool m_isLubyActive = false;
    int m_threshold;
    int m_baseThreshold;
    int m_ucCounts = 0;
    float m_growthRate = 1.5;
    Luby m_luby;
};

} // namespace car

#endif
