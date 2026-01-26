#include "IncrCheckerHelpers.h"
#include <unordered_map>

namespace car {

Branching::Branching(int type) {
    branching_type = type;
    conflict_index = 1;
    mini = 1 << 20;
    counts.clear();
}


Branching::~Branching() {}


void Branching::Update(const cube &uc) {
    if (uc.empty()) return;
    conflict_index++;
    switch (branching_type) {
    case 1: {
        Decay();
        break;
    }
    case 2: {
        if (conflict_index == 256) {
            for (int i = mini; i < counts.size(); i++)
                counts[i] *= 0.5;
            conflict_index = 0;
        }
        break;
    }
    }
    // assumes cube is ordered
    int sz = abs(uc.back());
    if (sz >= counts.size()) counts.resize(sz + 1);
    if (mini > abs(uc.at(0))) mini = abs(uc.at(0));
    for (auto l : uc) {
        switch (branching_type) {
        case 1:
        case 2: {
            assert(abs(l) < counts.size());
            counts[abs(l)]++;
            break;
        }
        case 3:
            counts[abs(l)] = (counts[abs(l)] + conflict_index) / 2.0;
            break;
        }
    }
}


void Branching::Decay() {
    for (int i = mini; i < counts.size(); i++)
        counts[i] *= 0.99;
}


void Branching::Decay(const cube &uc, int gap) {
    if (uc.empty()) return;
    conflict_index++;
    // assumes cube is ordered
    int sz = abs(uc.back());
    if (sz >= counts.size()) counts.resize(sz + 1);
    if (mini > abs(uc.at(0))) mini = abs(uc.at(0));
    for (auto l : uc) {
        counts[abs(l)] *= 1 - 0.01 * (gap - 1);
    }
}


static bool _cmp(int a, int b) {
    if (abs(a) != abs(b))
        return abs(a) < abs(b);
    else
        return a < b;
}


static bool CubeImplies(const cube &a, const cube &b) {
    if (a.size() > b.size()) return false;
    unordered_set<int> bset;
    bset.reserve(b.size() * 2);
    for (int lit : b) bset.emplace(lit);
    for (int lit : a) {
        if (bset.find(lit) == bset.end()) return false;
    }
    return true;
}


// ================================================================================
// @brief: if a implies b
// @input:
// @output:
// ================================================================================
bool OverSequenceSet::Imply(const cube &a, const cube &b) {
    if (a.size() > b.size())
        return false;
    return (includes(b.begin(), b.end(), a.begin(), a.end(), _cmp));
}

void OverSequenceSet::EnsureTmpLitCapacity(const cube &latches) {
    int max_abs = 0;
    for (int lit : latches) {
        int a = abs(lit);
        if (a > max_abs) max_abs = a;
    }
    if (max_abs <= m_tmpLitOffset) {
        return;
    }
    m_tmpLitOffset = max_abs;
    m_tmpLitFlags.assign(static_cast<size_t>(m_tmpLitOffset * 2 + 1), 0);
    m_tmpLitList.clear();
}

void OverSequenceSet::TmpLitSetInsert(int lit) {
    size_t idx = static_cast<size_t>(lit + m_tmpLitOffset);
    if (!m_tmpLitFlags[idx]) {
        m_tmpLitFlags[idx] = 1;
        m_tmpLitList.emplace_back(idx);
    }
}

bool OverSequenceSet::TmpLitSetHas(int lit) const {
    size_t idx = static_cast<size_t>(lit + m_tmpLitOffset);
    return m_tmpLitFlags[idx] != 0;
}

void OverSequenceSet::ClearTmpLitSet() {
    for (size_t idx : m_tmpLitList) {
        m_tmpLitFlags[idx] = 0;
    }
    m_tmpLitList.clear();
}


bool OverSequenceSet::Insert(const cube &uc, int index) {
    auto f = GetFrame(index);
    if (f->find(uc) != f->end()) return false;

    f->emplace(uc);
    int &counter = m_insertCounter[index];
    counter++;
    if (counter >= kCleanupThreshold) {
        counter = 0;
        CleanupImplied(index);
    }

    return true;
}


bool IsStateInInv(const cube &s, const FrameList &inv) {
    bool flag = false;
    for (const auto &f : inv) {
        flag = false;
        for (const auto &fc : f) {
            if (CubeImplies(fc, s)) {
                flag = true;
                break;
            }
        }
        if (!flag) break;
    }
    return flag;
}


shared_ptr<OverSequenceSet::FrameSet> OverSequenceSet::GetFrame(int lvl) {
    while (lvl >= m_sequence.size()) {
        m_sequence.emplace_back(make_shared<FrameSet>());
        m_blockCounter.emplace_back(0);
        m_insertCounter.emplace_back(0);
    }
    return m_sequence[lvl];
}

frame OverSequenceSet::FrameSetToFrame(const FrameSet &fset) const {
    frame out;
    out.reserve(fset.size());
    for (const auto &fc : fset) {
        out.emplace_back(fc);
    }
    return out;
}


void OverSequenceSet::CleanupImplied(int frameLevel) {
    auto f = GetFrame(frameLevel);

    vector<cube> cubes;
    cubes.reserve(f->size());
    for (const auto &uc : *f) {
        cubes.emplace_back(uc);
    }

    sort(cubes.begin(), cubes.end(), [](const cube &a, const cube &b) {
        if (a.size() != b.size()) return a.size() < b.size();
        return lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(), _cmp);
    });

    unordered_map<int, vector<size_t>> occurs;
    occurs.reserve(cubes.size() * 4);
    for (size_t i = 0; i < cubes.size(); ++i) {
        for (int lit : cubes[i]) {
            occurs[lit].emplace_back(i);
        }
    }

    vector<bool> remove(cubes.size(), false);
    for (size_t i = 0; i < cubes.size(); ++i) {
        if (remove[i]) continue;

        int best_lit = cubes[i][0];
        size_t best_occ = occurs[best_lit].size();
        for (int lit : cubes[i]) {
            auto it = occurs.find(lit);
            if (it == occurs.end()) continue;
            size_t occ_size = it->second.size();
            if (occ_size < best_occ) {
                best_occ = occ_size;
                best_lit = lit;
            }
        }
        for (size_t idx : occurs[best_lit]) {
            if (idx == i || remove[idx]) continue;
            if (Imply(cubes[i], cubes[idx])) {
                remove[idx] = true;
            }
        }
    }

    for (size_t i = 0; i < cubes.size(); ++i) {
        if (remove[i]) {
            f->erase(cubes[i]);
        }
    }
}


bool OverSequenceSet::IsBlockedByFrame(const cube &latches, int frameLevel) {
    auto f = GetFrame(frameLevel);
    if (f->empty()) return false;

    EnsureTmpLitCapacity(latches);
    for (int lit : latches) {
        TmpLitSetInsert(lit);
    }

    size_t latches_size = latches.size();
    for (const auto &uc : *f) {
        if (uc.size() > latches_size) continue;
        bool subsumed = true;
        for (int lit : uc) {
            if (!TmpLitSetHas(lit)) {
                subsumed = false;
                break;
            }
        }
        if (subsumed) {
            ClearTmpLitSet();
            return true;
        }
    }
    ClearTmpLitSet();
    return false;
}


void OverSequenceSet::GetBlockers(const cube &latches, int framelevel, vector<cube> &b) {
    auto f = GetFrame(framelevel);
    if (f->empty()) return;

    EnsureTmpLitCapacity(latches);
    for (int lit : latches) {
        TmpLitSetInsert(lit);
    }

    size_t latches_size = latches.size();
    for (const auto &uc : *f) {
        if (uc.size() > latches_size) continue;
        bool subsumed = true;
        for (int lit : uc) {
            if (!TmpLitSetHas(lit)) {
                subsumed = false;
                break;
            }
        }
        if (subsumed) {
            b.emplace_back(uc);
        }
    }
    ClearTmpLitSet();
}


string OverSequenceSet::FramesInfo() {
    string res;
    res += "Frames " + to_string(m_sequence.size() - 1) + "\n";
    for (int i = 0; i < m_sequence.size(); ++i) {
        res += to_string(m_sequence[i]->size()) + " ";
    }
    return res;
}


string OverSequenceSet::FramesDetail() {
    string res;
    for (int i = 0; i < m_sequence.size(); ++i) {
        res += "Frame " + to_string(i) + "\n";
        if (i != 0) {
            for (auto uc : *m_sequence[i]) {
                for (auto j : uc) {
                    res += to_string(j) + " ";
                }
                res += "\n";
            }
        }
        res += "size: " + to_string(m_sequence[i]->size()) + "\n";
    }
    return res;
}


int State::numInputs = -1;
int State::numLatches = -1;


string State::GetLatchesString() {
    string result = "";
    result.reserve(numLatches);
    int j = 0;
    for (int i = 0; i < numLatches; ++i) {
        if (j >= latches.size() || numInputs + i + 1 < abs(latches.at(j))) {
            result += "x";
        } else {
            result += (latches.at(j) > 0) ? "1" : "0";
            ++j;
        }
    }
    return result;
}


string State::GetInputsString() {
    string result = "";
    result.reserve(numInputs);
    int j = 0;
    for (int i = 1; i <= numInputs; ++i) {
        if (j >= inputs.size() || i < abs(inputs.at(j))) {
            result += "x";
        } else {
            result += (inputs.at(j) > 0) ? "1" : "0";
            ++j;
        }
    }
    return result;
}


} // namespace car
