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


void LemmaForestManager::Reset() {
    m_lemmas.clear();
    m_forest.clear();
    m_alive.clear();
    m_borders.clear();
    m_tmpLitSet.clear();
}

void LemmaForestManager::EnsureLevel(int level) {
    if (level < 0) return;
    if (static_cast<int>(m_borders.size()) <= level) {
        m_borders.resize(level + 1);
    }
}

const std::vector<int> &LemmaForestManager::BorderIds(int level) const {
    static const std::vector<int> empty;
    if (level < 0 || level >= static_cast<int>(m_borders.size())) return empty;
    return m_borders[level];
}

bool LemmaForestManager::BorderEmpty(int level) const {
    return BorderIds(level).empty();
}

size_t LemmaForestManager::BorderSize(int level) const {
    return BorderIds(level).size();
}

void LemmaForestManager::CleanDeadBorders(int level) {
    if (level < 0 || level >= static_cast<int>(m_borders.size())) return;
    auto &border = m_borders[level];
    size_t w = 0;
    for (int lemmaId : border) {
        if (Alive(lemmaId)) {
            border[w++] = lemmaId;
        }
    }
    border.resize(w);
}

void LemmaForestManager::SortBorderByCubeSize(int level) {
    if (level < 0 || level >= static_cast<int>(m_borders.size())) return;
    auto &border = m_borders[level];
    std::sort(border.begin(), border.end(), [&](int a, int b) {
        size_t sa = m_lemmas[a].size();
        size_t sb = m_lemmas[b].size();
        if (sa != sb) return sa < sb;
        return a < b;
    });
}

const cube &LemmaForestManager::CubeOf(int id) const {
    assert(id >= 0 && id < static_cast<int>(m_lemmas.size()));
    return m_lemmas[id];
}

bool LemmaForestManager::Alive(int id) const {
    return id >= 0 && id < static_cast<int>(m_alive.size()) && m_alive[id];
}

void LemmaForestManager::BorderCubesRange::Iterator::SkipDead() {
    if (!lfm) return;
    if (level < 0 || level >= static_cast<int>(lfm->m_borders.size())) return;
    const auto &border = lfm->m_borders[level];
    while (idx < border.size()) {
        int lemmaId = border[idx];
        if (lfm->Alive(lemmaId)) {
            return;
        }
        idx++;
    }
}

const cube &LemmaForestManager::BorderCubesRange::Iterator::operator*() const {
    const auto &border = lfm->m_borders[level];
    int lemmaId = border[idx];
    return lfm->CubeOf(lemmaId);
}

LemmaForestManager::BorderCubesRange::Iterator &LemmaForestManager::BorderCubesRange::Iterator::operator++() {
    idx++;
    SkipDead();
    return *this;
}

bool LemmaForestManager::BorderCubesRange::Iterator::operator!=(const Iterator &other) const {
    return lfm != other.lfm || level != other.level || idx != other.idx;
}

LemmaForestManager::BorderCubesRange::Iterator LemmaForestManager::BorderCubesRange::begin() const {
    Iterator it{lfm, level, 0};
    it.SkipDead();
    return it;
}

LemmaForestManager::BorderCubesRange::Iterator LemmaForestManager::BorderCubesRange::end() const {
    if (!lfm || level < 0 || level >= static_cast<int>(lfm->m_borders.size())) {
        return Iterator{lfm, level, 0};
    }
    return Iterator{lfm, level, lfm->m_borders[level].size()};
}

LemmaForestManager::BorderCubesRange LemmaForestManager::BorderCubes(int level) const {
    return BorderCubesRange(this, level);
}

std::pair<int, int> LemmaForestManager::FindParentLemma(int startLevel, const cube &cb) {
    m_tmpLitSet.newSet(cb);
    for (int lvl = startLevel; lvl >= 1; --lvl) {
        const auto &borders = m_borders[lvl];
        for (int j = 0; j < static_cast<int>(borders.size()); ++j) {
            int lemmaId = borders[j];
            if (!Alive(lemmaId)) continue;
            if (SubsumeSet(m_lemmas[lemmaId], m_tmpLitSet)) {
                return {lemmaId, j};
            }
        }
    }
    return {-1, -1};
}

int LemmaForestManager::CreateLemma(const cube &cb, int parentId, int frameLevel) {
    int id = static_cast<int>(m_lemmas.size());
    m_lemmas.push_back(cb);
    ForestNode node;
    node.parentId = parentId;
    node.frameLvl = frameLevel;
    node.depth = (parentId == -1) ? 0 : (m_forest[parentId].depth + 1);
    m_forest.push_back(std::move(node));
    m_alive.push_back(1);
    if (parentId != -1) {
        m_forest[parentId].childrenIds.push_back(id);
    }
    return id;
}

void LemmaForestManager::UnregisterLemma(int lemmaId) {
    if (!Alive(lemmaId)) return;
    m_alive[lemmaId] = 0;
}

void LemmaForestManager::RemoveFromBorder(int level, int lemmaId) {
    if (level < 0 || level >= static_cast<int>(m_borders.size())) return;
    auto &border = m_borders[level];
    for (int i = 0; i < static_cast<int>(border.size()); ++i) {
        if (border[i] == lemmaId) {
            border[i] = border.back();
            border.pop_back();
            return;
        }
    }
}

int LemmaForestManager::SwapCreateLemma(const cube &cb, int existingLemmaId, int frameLevel) {
    assert(Alive(existingLemmaId));
    int parentId = m_forest[existingLemmaId].parentId;
    int depth = m_forest[existingLemmaId].depth;

    int newId = static_cast<int>(m_lemmas.size());
    m_lemmas.push_back(cb);
    ForestNode node;
    node.parentId = parentId;
    node.frameLvl = frameLevel;
    node.depth = depth;
    m_forest.push_back(std::move(node));
    m_alive.push_back(1);

    if (parentId != -1) {
        auto &siblings = m_forest[parentId].childrenIds;
        for (int &sid : siblings) {
            if (sid == existingLemmaId) {
                sid = newId;
                break;
            }
        }
    }

    auto children = std::move(m_forest[existingLemmaId].childrenIds);
    for (int childId : children) {
        m_forest[childId].parentId = newId;
        m_forest[newId].childrenIds.push_back(childId);
    }
    UnregisterLemma(existingLemmaId);
    return newId;
}

void LemmaForestManager::AddLemmaToBorder(int frameLevel, int lemmaId) {
    m_borders[frameLevel].push_back(lemmaId);
}

void LemmaForestManager::AdoptRelations(int newLemmaId, int oldLemmaId) {
    assert(Alive(newLemmaId));
    assert(Alive(oldLemmaId));
    ForestNode &newMeta = m_forest[newLemmaId];
    ForestNode &oldMeta = m_forest[oldLemmaId];

    if (oldMeta.parentId != -1) {
        auto &siblings = m_forest[oldMeta.parentId].childrenIds;
        for (int i = 0; i < static_cast<int>(siblings.size()); ++i) {
            if (siblings[i] == oldLemmaId) {
                siblings[i] = siblings.back();
                siblings.pop_back();
                break;
            }
        }
    }

    for (int childId : oldMeta.childrenIds) {
        m_forest[childId].parentId = newLemmaId;
        newMeta.childrenIds.push_back(childId);
    }
    oldMeta.childrenIds.clear();
    UnregisterLemma(oldLemmaId);
}

uint64_t LemmaForestManager::RemoveRedundantLemmas(int startLevel, int endLevel, int newLemmaId) {
    if (startLevel < 1) startLevel = 1;
    if (endLevel >= static_cast<int>(m_borders.size())) endLevel = static_cast<int>(m_borders.size()) - 1;
    if (startLevel > endLevel) return 0;

    const cube &newCube = m_lemmas[newLemmaId];
    uint64_t removed = 0;

    for (int lvl = startLevel; lvl <= endLevel; ++lvl) {
        auto &border = m_borders[lvl];
        int i = 0;
        while (i < static_cast<int>(border.size())) {
            int existingId = border[i];
            if (!Alive(existingId)) {
                border[i] = border.back();
                border.pop_back();
                continue;
            }
            if (existingId == newLemmaId) {
                i++;
                continue;
            }
            m_tmpLitSet.newSet(m_lemmas[existingId]);
            if (SubsumeSet(newCube, m_tmpLitSet)) {
                AdoptRelations(newLemmaId, existingId);
                border[i] = border.back();
                border.pop_back();
                removed++;
                continue;
            }
            i++;
        }
    }
    return removed;
}

void LemmaForestManager::UpdateRefineCountersOnInsert(int newLemmaId) {
    int parentId = m_forest[newLemmaId].parentId;
    while (parentId != -1) {
        m_forest[parentId].refineCount++;
        m_forest[parentId].refineCountSinceALL++;
        parentId = m_forest[parentId].parentId;
    }
}

AddLemmaResult LemmaForestManager::AddLemma(const cube &cb, int frameLevel) {
    assert(frameLevel >= 1);
    EnsureLevel(frameLevel);

    auto parent = FindParentLemma(frameLevel - 1, cb);
    int parentId = parent.first;
    int parentIndex = parent.second;
    int parentLevel = (parentId == -1) ? 0 : m_forest[parentId].frameLvl;

    int newLemmaId;
    if (parentId != -1 && cb.size() == m_lemmas[parentId].size()) {
        auto &parentBorder = m_borders[parentLevel];
        assert(parentIndex >= 0 && parentIndex < static_cast<int>(parentBorder.size()));
        assert(parentBorder[parentIndex] == parentId);
        parentBorder[parentIndex] = parentBorder.back();
        parentBorder.pop_back();
        newLemmaId = SwapCreateLemma(cb, parentId, frameLevel);
    } else {
        newLemmaId = CreateLemma(cb, parentId, frameLevel);
    }

    int beginLevel = parentLevel + 1;
    RemoveRedundantLemmas(beginLevel, frameLevel, newLemmaId);
    AddLemmaToBorder(frameLevel, newLemmaId);
    UpdateRefineCountersOnInsert(newLemmaId);
    return AddLemmaResult{newLemmaId, beginLevel, frameLevel};
}

int LemmaForestManager::PropagateLemma(int lemmaId, int newFrameLevel) {
    assert(Alive(lemmaId));
    EnsureLevel(newFrameLevel);
    ForestNode &meta = m_forest[lemmaId];
    RemoveFromBorder(meta.frameLvl, lemmaId);
    meta.frameLvl = newFrameLevel;
    ClearCTPState(lemmaId);

    RemoveRedundantLemmas(newFrameLevel, newFrameLevel, lemmaId);
    AddLemmaToBorder(newFrameLevel, lemmaId);
    return newFrameLevel;
}

void LemmaForestManager::GetBlockers(const cube &blockingCube, int level, std::vector<cube> &blockers) const {
    if (level < 0 || level >= static_cast<int>(m_borders.size())) return;
    m_tmpLitSet.newSet(blockingCube);
    for (int lemmaId : m_borders[level]) {
        if (!Alive(lemmaId)) continue;
        if (SubsumeSet(m_lemmas[lemmaId], m_tmpLitSet)) {
            blockers.push_back(m_lemmas[lemmaId]);
        }
    }
}

bool LemmaForestManager::IsBlockedAtLevel(const cube &cb, int level) const {
    if (level < 0 || level >= static_cast<int>(m_borders.size())) return false;
    m_tmpLitSet.newSet(cb);
    for (int lemmaId : m_borders[level]) {
        if (!Alive(lemmaId)) continue;
        if (SubsumeSet(m_lemmas[lemmaId], m_tmpLitSet)) {
            return true;
        }
    }
    return false;
}

std::vector<int> LemmaForestManager::GetAncestorChain(int lemmaId) const {
    std::vector<int> chain;
    if (!Alive(lemmaId)) return chain;
    int parentId = m_forest[lemmaId].parentId;
    while (parentId != -1) {
        if (!Alive(parentId)) break;
        chain.push_back(parentId);
        parentId = m_forest[parentId].parentId;
    }
    return chain;
}

int LemmaForestManager::FrameLevelOf(int lemmaId) const {
    assert(Alive(lemmaId));
    return m_forest[lemmaId].frameLvl;
}

int LemmaForestManager::ParentOf(int lemmaId) const {
    assert(Alive(lemmaId));
    return m_forest[lemmaId].parentId;
}

int LemmaForestManager::RefineCountSinceALL(int lemmaId) const {
    assert(Alive(lemmaId));
    return m_forest[lemmaId].refineCountSinceALL;
}

void LemmaForestManager::ResetRefineCountSinceALL(int lemmaId) {
    assert(Alive(lemmaId));
    m_forest[lemmaId].refineCountSinceALL = 0;
}

bool LemmaForestManager::Reachable(int lemmaId) const {
    return Alive(lemmaId) && m_forest[lemmaId].reachable;
}

void LemmaForestManager::SetReachable(int lemmaId, bool value) {
    assert(Alive(lemmaId));
    m_forest[lemmaId].reachable = value;
}

bool LemmaForestManager::PopCTPPred(int lemmaId, cube &ctpCube, int &ctpLevel) {
    assert(Alive(lemmaId));
    auto &preds = m_forest[lemmaId].ctpPreds;
    if (preds.empty()) return false;
    auto item = std::move(preds.back());
    preds.pop_back();
    ctpCube = std::move(item.first);
    ctpLevel = item.second;
    return true;
}

void LemmaForestManager::PushCTPPred(int lemmaId, const cube &ctpCube, int ctpLevel) {
    assert(Alive(lemmaId));
    m_forest[lemmaId].ctpPreds.emplace_back(ctpCube, ctpLevel);
}

bool LemmaForestManager::HasCTPPreds(int lemmaId) const {
    assert(Alive(lemmaId));
    return !m_forest[lemmaId].ctpPreds.empty();
}

void LemmaForestManager::ClearCTPState(int lemmaId) {
    assert(Alive(lemmaId));
    auto &meta = m_forest[lemmaId];
    meta.ctpPreds.clear();
    meta.ctpSucc.clear();
    meta.hasCTPSucc = false;
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
