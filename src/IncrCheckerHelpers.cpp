#include "IncrCheckerHelpers.h"
#include <unordered_map>

namespace car {

Branching::Branching(int type) {
    m_branchingType = type;
    m_conflictIndex = 1;
    m_mini = 1 << 20;
    m_counts.clear();
}


Branching::~Branching() {}


void Branching::Update(const Cube &uc) {
    if (uc.empty()) return;
    m_conflictIndex++;
    switch (m_branchingType) {
    case 1: {
        Decay();
        break;
    }
    case 2: {
        if (m_conflictIndex == 256) {
            for (int i = m_mini; i < m_counts.size(); i++)
                m_counts[i] *= 0.5;
            m_conflictIndex = 0;
        }
        break;
    }
    }
    // assumes Cube is ordered
    Var sz = VarOf(uc.back());
    if (sz >= m_counts.size()) m_counts.resize(sz + 1);
    if (m_mini > static_cast<int>(VarOf(uc.at(0)))) m_mini = static_cast<int>(VarOf(uc.at(0)));
    for (auto l : uc) {
        Var lit_var = VarOf(l);
        switch (m_branchingType) {
        case 1:
        case 2: {
            assert(lit_var < m_counts.size());
            m_counts[lit_var]++;
            break;
        }
        case 3:
            m_counts[lit_var] = (m_counts[lit_var] + m_conflictIndex) / 2.0;
            break;
        }
    }
}


void Branching::Decay() {
    for (int i = m_mini; i < m_counts.size(); i++)
        m_counts[i] *= 0.95;
}


void Branching::Decay(const Cube &uc, int gap) {
    if (uc.empty()) return;
    m_conflictIndex++;
    // assumes Cube is ordered
    Var sz = VarOf(uc.back());
    if (sz >= m_counts.size()) m_counts.resize(sz + 1);
    if (m_mini > static_cast<int>(VarOf(uc.at(0)))) m_mini = static_cast<int>(VarOf(uc.at(0)));
    for (auto l : uc) {
        m_counts[VarOf(l)] *= 1 - 0.01 * (gap - 1);
    }
}

bool IsStateInInv(const Cube &s, const FrameList &inv) {
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


void OverSequenceSet::EnsureLevel(int lvl) {
    if (lvl < 0) return;
    while (lvl >= static_cast<int>(m_frames.size())) {
        m_frames.emplace_back();
    }
}


size_t OverSequenceSet::FrameSize(int lvl) const {
    if (lvl < 0 || lvl >= static_cast<int>(m_frames.size())) return 0;
    return m_frames[lvl].refOfLemma.size();
}


bool OverSequenceSet::RefAlive(RefId ref) const {
    return ref >= 0 && ref < static_cast<int>(m_refs.size()) && m_refs[ref].alive;
}


bool OverSequenceSet::RefAliveInFrame(RefId ref, int frameLevel) const {
    return RefAlive(ref) && m_refs[ref].level == frameLevel;
}


const Cube &OverSequenceSet::CubeOf(LemmaId id) const {
    assert(id >= 0 && id < static_cast<int>(m_lemmas.size()));
    return m_lemmas[id];
}


const Cube &OverSequenceSet::CubeOfRef(RefId ref) const {
    assert(RefAlive(ref));
    return m_lemmas[m_refs[ref].lemmaId];
}


OverSequenceSet::LemmaId OverSequenceSet::LemmaOfRef(RefId ref) const {
    assert(RefAlive(ref));
    return m_refs[ref].lemmaId;
}


int OverSequenceSet::LevelOfRef(RefId ref) const {
    assert(RefAlive(ref));
    return m_refs[ref].level;
}


OverSequenceSet::RefId OverSequenceSet::RefOf(LemmaId id, int frameLevel) const {
    if (frameLevel < 0 || frameLevel >= static_cast<int>(m_frames.size())) return -1;
    auto it = m_frames[frameLevel].refOfLemma.find(id);
    if (it == m_frames[frameLevel].refOfLemma.end()) return -1;
    return RefAliveInFrame(it->second, frameLevel) ? it->second : -1;
}


bool OverSequenceSet::Contains(LemmaId id, int frameLevel) const {
    return RefOf(id, frameLevel) != -1;
}


bool OverSequenceSet::Alive(RefId ref) const {
    return RefAlive(ref);
}


std::vector<OverSequenceSet::LemmaId> OverSequenceSet::FrameIds(int lvl) const {
    std::vector<LemmaId> out;
    if (lvl < 0 || lvl >= static_cast<int>(m_frames.size())) return out;
    const auto &frame = m_frames[lvl];
    out.reserve(frame.refOfLemma.size());
    for (RefId ref : frame.refs) {
        if (RefAliveInFrame(ref, lvl)) out.emplace_back(m_refs[ref].lemmaId);
    }
    return out;
}


std::vector<OverSequenceSet::RefId> OverSequenceSet::FrameRefs(int lvl) const {
    std::vector<RefId> out;
    if (lvl < 0 || lvl >= static_cast<int>(m_frames.size())) return out;
    const auto &frame = m_frames[lvl];
    out.reserve(frame.refOfLemma.size());
    for (RefId ref : frame.refs) {
        if (RefAliveInFrame(ref, lvl)) out.emplace_back(ref);
    }
    return out;
}


Frame OverSequenceSet::FrameToFrame(int lvl) const {
    Frame out;
    if (lvl < 0 || lvl >= static_cast<int>(m_frames.size())) return out;
    const auto &frame = m_frames[lvl];
    out.reserve(frame.refOfLemma.size());
    for (RefId ref : frame.refs) {
        if (RefAliveInFrame(ref, lvl)) out.emplace_back(CubeOfRef(ref));
    }
    return out;
}


OverSequenceSet::LemmaId OverSequenceSet::InternLemma(const Cube &uc) {
    auto it = m_idOfCube.find(uc);
    if (it != m_idOfCube.end()) return it->second;

    LemmaId id = static_cast<LemmaId>(m_lemmas.size());
    m_lemmas.emplace_back(uc);
    m_idOfCube.emplace(m_lemmas[id], id);
    return id;
}


std::vector<OverSequenceSet::RefId> OverSequenceSet::CandidateRefsForSubsuming(const Cube &uc, const FrameData &frame) const {
    if (uc.empty() || frame.refOfLemma.size() <= 64) {
        return frame.refs;
    }

    std::vector<RefId> candidates;
    std::unordered_set<RefId> seen;
    for (Lit lit : uc) {
        auto it = frame.occurs.find(lit);
        if (it == frame.occurs.end()) continue;
        for (RefId ref : it->second) {
            if (seen.emplace(ref).second) candidates.emplace_back(ref);
        }
    }
    return candidates;
}


std::vector<OverSequenceSet::RefId> OverSequenceSet::FindSubsumedInFrame(const Cube &uc, int frameLevel) {
    std::vector<RefId> out;
    if (frameLevel < 0 || frameLevel >= static_cast<int>(m_frames.size())) return out;
    const auto &frame = m_frames[frameLevel];
    if (frame.refOfLemma.empty()) return out;

    const std::vector<RefId> *candidate_list = nullptr;
    std::vector<RefId> all_candidates;

    size_t best_size = static_cast<size_t>(-1);
    for (Lit lit : uc) {
        auto it = frame.occurs.find(lit);
        if (it == frame.occurs.end()) return out;
        if (it->second.size() < best_size) {
            best_size = it->second.size();
            candidate_list = &it->second;
        }
    }

    if (!candidate_list) return out;
    std::unordered_set<RefId> seen;
    for (RefId ref : *candidate_list) {
        if (!seen.emplace(ref).second) continue;
        if (!RefAliveInFrame(ref, frameLevel)) continue;
        const Cube &old = CubeOfRef(ref);
        if (uc.size() > old.size()) continue;
        m_tmpLitSet.NewSet(old);
        if (SubsumeSet(uc, m_tmpLitSet)) {
            out.emplace_back(ref);
        }
    }
    return out;
}


void OverSequenceSet::AttachParent(RefId childRef, RefId parentRef) {
    if (!RefAlive(childRef) || !RefAlive(parentRef)) return;
    DetachFromParent(childRef);
    m_refs[childRef].parentRef = parentRef;
    auto &children = m_refs[parentRef].childRefs;
    if (find(children.begin(), children.end(), childRef) == children.end()) {
        children.emplace_back(childRef);
    }
}


void OverSequenceSet::DetachFromParent(RefId ref) {
    if (!RefAlive(ref)) return;
    RefId parent = m_refs[ref].parentRef;
    if (parent == -1) return;
    if (RefAlive(parent)) {
        auto &siblings = m_refs[parent].childRefs;
        for (int i = 0; i < static_cast<int>(siblings.size()); ++i) {
            if (siblings[i] == ref) {
                siblings[i] = siblings.back();
                siblings.pop_back();
                break;
            }
        }
    }
    m_refs[ref].parentRef = -1;
}


OverSequenceSet::RefId OverSequenceSet::FindBestParentInPrevFrame(RefId ref) const {
    if (!RefAlive(ref)) return -1;
    int parent_level = m_refs[ref].level - 1;
    if (parent_level < 0 || parent_level >= static_cast<int>(m_frames.size())) return -1;

    const Cube &child_cube = CubeOfRef(ref);
    LitSet child_set;
    child_set.NewSet(child_cube);

    RefId best = -1;
    size_t best_size = 0;
    for (RefId candidate : m_frames[parent_level].refs) {
        if (!RefAliveInFrame(candidate, parent_level)) continue;
        const Cube &parent_cube = CubeOfRef(candidate);
        if (parent_cube.size() > child_cube.size()) continue;
        if (parent_cube.size() < best_size) continue;
        if (SubsumeSet(parent_cube, child_set)) {
            best = candidate;
            best_size = parent_cube.size();
        }
    }
    return best;
}


void OverSequenceSet::RepairParent(RefId ref) {
    if (!RefAlive(ref)) return;
    DetachFromParent(ref);
    RefId parent = FindBestParentInPrevFrame(ref);
    if (parent != -1) AttachParent(ref, parent);
}


void OverSequenceSet::AttachNeighborRefs(RefId ref) {
    if (!RefAlive(ref)) return;

    RefId parent = FindBestParentInPrevFrame(ref);
    if (parent != -1) AttachParent(ref, parent);

    int child_level = m_refs[ref].level + 1;
    if (child_level >= static_cast<int>(m_frames.size())) return;

    const Cube &parent_cube = CubeOfRef(ref);
    for (RefId child : m_frames[child_level].refs) {
        if (!RefAliveInFrame(child, child_level)) continue;
        const Cube &child_cube = CubeOfRef(child);
        if (parent_cube.size() > child_cube.size()) continue;
        m_tmpLitSet.NewSet(child_cube);
        if (!SubsumeSet(parent_cube, m_tmpLitSet)) continue;

        RefId old_parent = m_refs[child].parentRef;
        if (old_parent == -1 || CubeOfRef(old_parent).size() < parent_cube.size()) {
            AttachParent(child, ref);
        }
    }
}


OverSequenceSet::RefId OverSequenceSet::AddMembership(LemmaId id, int frameLevel) {
    EnsureLevel(frameLevel);
    auto &frame = m_frames[frameLevel];
    auto existing = frame.refOfLemma.find(id);
    if (existing != frame.refOfLemma.end()) return existing->second;

    RefId ref = static_cast<RefId>(m_refs.size());
    RefNode node;
    node.lemmaId = id;
    node.level = frameLevel;
    m_refs.emplace_back(std::move(node));

    frame.refs.emplace_back(ref);
    frame.refOfLemma.emplace(id, ref);
    for (Lit lit : m_lemmas[id]) {
        frame.occurs[lit].emplace_back(ref);
    }
    AttachNeighborRefs(ref);
    return ref;
}


void OverSequenceSet::RemoveMembership(RefId ref) {
    if (!RefAlive(ref)) return;
    int frameLevel = m_refs[ref].level;
    auto &frame = m_frames[frameLevel];

    std::vector<RefId> children = m_refs[ref].childRefs;
    for (RefId child : children) {
        if (RefAlive(child)) {
            m_refs[child].parentRef = -1;
        }
    }
    m_refs[ref].childRefs.clear();
    DetachFromParent(ref);

    frame.refOfLemma.erase(m_refs[ref].lemmaId);
    m_refs[ref].alive = 0;
    frame.deadCount++;

    for (RefId child : children) {
        RepairParent(child);
    }

    if (frame.deadCount * K_REBUILD_DEAD_RATIO > frame.refs.size()) {
        RebuildFrame(frameLevel);
    }
}


void OverSequenceSet::RebuildFrame(int frameLevel) {
    if (frameLevel < 0 || frameLevel >= static_cast<int>(m_frames.size())) return;
    auto &frame = m_frames[frameLevel];
    std::vector<RefId> refs;
    refs.reserve(frame.refOfLemma.size());
    for (RefId ref : frame.refs) {
        if (RefAliveInFrame(ref, frameLevel)) refs.emplace_back(ref);
    }
    frame.refs.swap(refs);
    frame.occurs.clear();
    for (RefId ref : frame.refs) {
        for (Lit lit : CubeOfRef(ref)) {
            frame.occurs[lit].emplace_back(ref);
        }
    }
    frame.deadCount = 0;
}


OverSequenceSet::MembershipState *OverSequenceSet::MutableState(RefId ref) {
    if (!RefAlive(ref)) return nullptr;
    return &m_refs[ref].state;
}


const OverSequenceSet::MembershipState *OverSequenceSet::StateOf(RefId ref) const {
    if (!RefAlive(ref)) return nullptr;
    return &m_refs[ref].state;
}


void OverSequenceSet::IncrementAncestorCounters(RefId ref) {
    if (!RefAlive(ref)) return;
    RefId parent = m_refs[ref].parentRef;
    while (parent != -1) {
        if (auto *state = MutableState(parent)) {
            state->refineCountSinceALL++;
        }
        parent = m_refs[parent].parentRef;
    }
}


OverSequenceSet::InsertResult OverSequenceSet::Insert(const Cube &uc, int index) {
    EnsureLevel(index);

    auto it = m_idOfCube.find(uc);
    if (it != m_idOfCube.end() && Contains(it->second, index)) {
        return InsertResult{false, it->second, RefOf(it->second, index), index, uc, {}};
    }

    LemmaId id = InternLemma(uc);
    std::vector<RefId> removed = FindSubsumedInFrame(uc, index);
    for (RefId old_ref : removed) {
        RemoveMembership(old_ref);
    }

    RefId ref = AddMembership(id, index);
    IncrementAncestorCounters(ref);
    return InsertResult{true, id, ref, index, uc, removed};
}


bool OverSequenceSet::IsBlockedByFrame(const Cube &latches, int frameLevel) {
    if (frameLevel < 0 || frameLevel >= static_cast<int>(m_frames.size())) return false;
    const auto &frame = m_frames[frameLevel];
    if (frame.refOfLemma.empty()) return false;

    m_tmpLitSet.NewSet(latches);

    size_t latches_size = latches.size();
    for (RefId ref : frame.refs) {
        if (!RefAliveInFrame(ref, frameLevel)) continue;
        if (CubeOfRef(ref).empty()) return true;
    }
    auto candidates = CandidateRefsForSubsuming(latches, frame);
    for (RefId ref : candidates) {
        if (!RefAliveInFrame(ref, frameLevel)) continue;
        const Cube &uc = CubeOfRef(ref);
        if (uc.size() <= latches_size && SubsumeSet(uc, m_tmpLitSet)) {
            return true;
        }
    }
    return false;
}


bool OverSequenceSet::GetParentCube(const Cube &cube, int parentLevel, Cube &parent) const {
    parent.clear();
    if (parentLevel < 0 || parentLevel >= static_cast<int>(m_frames.size())) return false;
    const auto &frame = m_frames[parentLevel];
    if (frame.refOfLemma.empty()) return false;

    LitSet cube_set;
    cube_set.NewSet(cube);

    RefId best = -1;
    size_t best_size = 0;
    for (RefId ref : frame.refs) {
        if (!RefAliveInFrame(ref, parentLevel)) continue;
        const Cube &candidate = CubeOfRef(ref);
        if (candidate.size() > cube.size()) continue;
        if (candidate.size() < best_size) continue;
        if (SubsumeSet(candidate, cube_set)) {
            best = ref;
            best_size = candidate.size();
        }
    }
    if (best == -1) return false;
    parent = CubeOfRef(best);
    return true;
}


string OverSequenceSet::FramesInfo() {
    string res;
    res += "Frames " + to_string(m_frames.size() - 1) + "\n";
    for (int i = 0; i < static_cast<int>(m_frames.size()); ++i) {
        res += to_string(FrameSize(i)) + " ";
    }
    return res;
}


string OverSequenceSet::FramesDetail() {
    string res;
    for (int i = 0; i < static_cast<int>(m_frames.size()); ++i) {
        res += "Frame " + to_string(i) + "\n";
        if (i != 0) {
            for (RefId ref : m_frames[i].refs) {
                if (!RefAliveInFrame(ref, i)) continue;
                const Cube &uc = CubeOfRef(ref);
                for (auto j : uc) {
                    res += to_string(ToSigned(j)) + " ";
                }
                res += "\n";
            }
        }
        res += "size: " + to_string(FrameSize(i)) + "\n";
    }
    return res;
}


std::vector<OverSequenceSet::RefId> OverSequenceSet::GetAncestorRefs(RefId ref) const {
    std::vector<RefId> refs;
    if (!RefAlive(ref)) return refs;
    RefId parent = m_refs[ref].parentRef;
    while (parent != -1) {
        if (!RefAlive(parent)) break;
        refs.push_back(parent);
        parent = m_refs[parent].parentRef;
    }
    return refs;
}


int OverSequenceSet::RefineCountSinceALL(RefId ref) const {
    const auto *state = StateOf(ref);
    return state ? state->refineCountSinceALL : 0;
}


void OverSequenceSet::ResetRefineCountSinceALL(RefId ref) {
    if (auto *state = MutableState(ref)) state->refineCountSinceALL = 0;
}


bool OverSequenceSet::Reachable(RefId ref) const {
    const auto *state = StateOf(ref);
    return state && state->reachable;
}


void OverSequenceSet::MarkReachableChain(RefId ref) {
    RefId cur = ref;
    while (cur != -1) {
        if (auto *state = MutableState(cur)) {
            if (state->reachable) break;
            state->reachable = true;
        }
        cur = RefAlive(cur) ? m_refs[cur].parentRef : -1;
    }
}


bool OverSequenceSet::PopCTPPred(RefId ref, Cube &ctpCube, int &ctpLevel) {
    auto *state = MutableState(ref);
    if (!state || state->ctpPreds.empty()) return false;
    auto item = std::move(state->ctpPreds.back());
    state->ctpPreds.pop_back();
    ctpCube = std::move(item.first);
    ctpLevel = item.second;
    return true;
}


void OverSequenceSet::PushCTPPred(RefId ref, const Cube &ctpCube, int ctpLevel) {
    if (auto *state = MutableState(ref)) {
        state->ctpPreds.emplace_back(ctpCube, ctpLevel);
    }
}


bool OverSequenceSet::HasCTPPreds(RefId ref) const {
    const auto *state = StateOf(ref);
    return state && !state->ctpPreds.empty();
}


void OverSequenceSet::ClearCTPState(RefId ref) {
    if (auto *state = MutableState(ref)) state->ctpPreds.clear();
}


void LemmaForestManager::Reset() {
    m_lemmas.clear();
    m_forest.clear();
    m_lemmaStates.clear();
    m_alive.clear();
    m_borders.clear();
    m_tmpLitSet.Clear();
}

void LemmaForestManager::EnsureLevel(int level) {
    assert(level >= 1);
    if (static_cast<int>(m_borders.size()) <= level) {
        m_borders.resize(level + 1);
    }
}

const std::vector<int> &LemmaForestManager::BorderIds(int level) const {
    static const std::vector<int> EMPTY;
    if (level < 0 || level >= static_cast<int>(m_borders.size())) return EMPTY;
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
    for (int lemma_id : border) {
        if (Alive(lemma_id)) {
            border[w++] = lemma_id;
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

const Cube &LemmaForestManager::CubeOf(int id) const {
    assert(id >= 0 && id < static_cast<int>(m_lemmas.size()));
    return m_lemmas[id];
}

bool LemmaForestManager::Alive(int id) const {
    return id >= 0 && id < static_cast<int>(m_alive.size()) && m_alive[id];
}

const std::vector<int> &LemmaForestManager::ObligationsOf(int lemmaId) const {
    static const std::vector<int> EMPTY;
    if (lemmaId < 0 || lemmaId >= static_cast<int>(m_lemmaStates.size())) return EMPTY;
    return m_lemmaStates[lemmaId].obligationIds;
}

void LemmaForestManager::AddObligationBinding(int lemmaId, int obligationId) {
    if (!Alive(lemmaId) || obligationId < 0) return;

    auto &ids = m_lemmaStates[lemmaId].obligationIds;
    if (std::find(ids.begin(), ids.end(), obligationId) == ids.end()) {
        ids.push_back(obligationId);
    }
}

void LemmaForestManager::CopyObligationBindings(int newLemmaId, int oldLemmaId) {
    if (!Alive(newLemmaId) || oldLemmaId < 0 || oldLemmaId >= static_cast<int>(m_lemmaStates.size())) return;

    for (int obligation_id : m_lemmaStates[oldLemmaId].obligationIds) {
        AddObligationBinding(newLemmaId, obligation_id);
    }
}

void LemmaForestManager::BorderCubesRange::Iterator::SkipDead() {
    if (!lfm) return;
    if (level < 0 || level >= static_cast<int>(lfm->m_borders.size())) return;
    const auto &border = lfm->m_borders[level];
    while (idx < border.size()) {
        int lemma_id = border[idx];
        if (lfm->Alive(lemma_id)) {
            return;
        }
        idx++;
    }
}

const Cube &LemmaForestManager::BorderCubesRange::Iterator::operator*() const {
    const auto &border = lfm->m_borders[level];
    int lemma_id = border[idx];
    return lfm->CubeOf(lemma_id);
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
    Iterator it{m_lfm, m_level, 0};
    it.SkipDead();
    return it;
}

LemmaForestManager::BorderCubesRange::Iterator LemmaForestManager::BorderCubesRange::end() const {
    if (!m_lfm || m_level < 0 || m_level >= static_cast<int>(m_lfm->m_borders.size())) {
        return Iterator{m_lfm, m_level, 0};
    }
    return Iterator{m_lfm, m_level, m_lfm->m_borders[m_level].size()};
}

LemmaForestManager::BorderCubesRange LemmaForestManager::BorderCubes(int level) const {
    return BorderCubesRange(this, level);
}

std::pair<int, int> LemmaForestManager::FindParentLemma(int startLevel, const Cube &cb) const {
    m_tmpLitSet.NewSet(cb);
    for (int lvl = startLevel; lvl >= 1; --lvl) {
        const auto &borders = m_borders[lvl];
        for (int j = 0; j < static_cast<int>(borders.size()); ++j) {
            int lemma_id = borders[j];
            if (!Alive(lemma_id)) continue;
            if (SubsumeSet(m_lemmas[lemma_id], m_tmpLitSet)) {
                return {lemma_id, j};
            }
        }
    }
    return {-1, -1};
}


bool LemmaForestManager::GetParentCube(const Cube &blockingCube, int startLevel, Cube &parent) const {
    parent.clear();
    auto parent_info = FindParentLemma(startLevel, blockingCube);
    if (parent_info.first == -1) return false;
    parent = m_lemmas[parent_info.first];
    return true;
}

int LemmaForestManager::CreateLemma(const Cube &cb, int parentId, int frameLevel) {
    int id = static_cast<int>(m_lemmas.size());

    ForestNode node;
    node.parentId = parentId;
    node.frameLvl = frameLevel;

    m_lemmas.push_back(cb);
    m_forest.push_back(std::move(node));
    m_lemmaStates.emplace_back();
    m_alive.push_back(1);

    if (parentId != -1) {
        m_forest[parentId].childrenIds.push_back(id);
    }
    return id;
}

void LemmaForestManager::UnregisterLemma(int lemmaId) {
    assert(Alive(lemmaId));
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

void LemmaForestManager::AddLemmaToBorder(int frameLevel, int lemmaId) {
    m_borders[frameLevel].push_back(lemmaId);
}

void LemmaForestManager::MergeObligationBindings(int newLemmaId, int oldLemmaId) {
    CopyObligationBindings(newLemmaId, oldLemmaId);
}

void LemmaForestManager::AdoptRelations(int newLemmaId, int oldLemmaId) {
    assert(Alive(newLemmaId));
    assert(Alive(oldLemmaId));
    ForestNode &new_meta = m_forest[newLemmaId];
    ForestNode &old_meta = m_forest[oldLemmaId];

    if (old_meta.parentId != -1) {
        auto &siblings = m_forest[old_meta.parentId].childrenIds;
        for (int i = 0; i < static_cast<int>(siblings.size()); ++i) {
            if (siblings[i] == oldLemmaId) {
                siblings[i] = siblings.back();
                siblings.pop_back();
                break;
            }
        }
    }

    for (int child_id : old_meta.childrenIds) {
        m_forest[child_id].parentId = newLemmaId;
        new_meta.childrenIds.push_back(child_id);
    }
    MergeObligationBindings(newLemmaId, oldLemmaId);
    old_meta.childrenIds.clear();
    UnregisterLemma(oldLemmaId);
}

uint64_t LemmaForestManager::RemoveRedundantLemmas(int startLevel, int endLevel, int newLemmaId) {
    if (startLevel < 1) startLevel = 1;
    if (endLevel >= static_cast<int>(m_borders.size())) endLevel = static_cast<int>(m_borders.size()) - 1;
    if (startLevel > endLevel) return 0;

    const Cube &new_cube = m_lemmas[newLemmaId];
    uint64_t removed = 0;

    for (int lvl = startLevel; lvl <= endLevel; ++lvl) {
        auto &border = m_borders[lvl];
        int i = 0;
        while (i < static_cast<int>(border.size())) {
            int existing_id = border[i];
            if (!Alive(existing_id)) {
                border[i] = border.back();
                border.pop_back();
                continue;
            }
            if (existing_id == newLemmaId) {
                i++;
                continue;
            }
            m_tmpLitSet.NewSet(m_lemmas[existing_id]);
            if (SubsumeSet(new_cube, m_tmpLitSet)) {
                AdoptRelations(newLemmaId, existing_id);
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
    int parent_id = m_forest[newLemmaId].parentId;
    while (parent_id != -1) {
        m_lemmaStates[parent_id].refineCountSinceALL++;
        parent_id = m_forest[parent_id].parentId;
    }
}

AddLemmaResult LemmaForestManager::AddLemma(const Cube &cb, int frameLevel, int obligationId) {
    assert(frameLevel >= 1);
    EnsureLevel(frameLevel);

    auto parent = FindParentLemma(frameLevel - 1, cb);
    int parent_id = parent.first;
    int parent_index = parent.second;
    int parent_level = (parent_id == -1) ? 0 : m_forest[parent_id].frameLvl;

    int new_lemma_id;
    if (parent_id != -1 && cb.size() == m_lemmas[parent_id].size()) {
        auto &parent_border = m_borders[parent_level];
        assert(parent_index >= 0 && parent_index < static_cast<int>(parent_border.size()));
        assert(parent_border[parent_index] == parent_id);
        parent_border[parent_index] = parent_border.back();
        parent_border.pop_back();

        int old_parent_id = m_forest[parent_id].parentId;
        new_lemma_id = CreateLemma(cb, old_parent_id, frameLevel);
        AdoptRelations(new_lemma_id, parent_id);
    } else {
        new_lemma_id = CreateLemma(cb, parent_id, frameLevel);
    }

    int begin_level = parent_level + 1;
    RemoveRedundantLemmas(begin_level, frameLevel, new_lemma_id);
    AddObligationBinding(new_lemma_id, obligationId);
    AddLemmaToBorder(frameLevel, new_lemma_id);
    UpdateRefineCountersOnInsert(new_lemma_id);
    return AddLemmaResult{new_lemma_id, begin_level, frameLevel};
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

bool LemmaForestManager::IsBlockedAtLevel(const Cube &cb, int level) const {
    if (level < 0 || level >= static_cast<int>(m_borders.size())) return false;
    m_tmpLitSet.NewSet(cb);
    for (int lemma_id : m_borders[level]) {
        if (!Alive(lemma_id)) continue;
        if (SubsumeSet(m_lemmas[lemma_id], m_tmpLitSet)) {
            return true;
        }
    }
    return false;
}

std::vector<int> LemmaForestManager::GetAncestorChain(int lemmaId) const {
    std::vector<int> chain;
    if (!Alive(lemmaId)) return chain;
    int parent_id = m_forest[lemmaId].parentId;
    while (parent_id != -1) {
        if (!Alive(parent_id)) break;
        chain.push_back(parent_id);
        parent_id = m_forest[parent_id].parentId;
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
    return m_lemmaStates[lemmaId].refineCountSinceALL;
}

void LemmaForestManager::ResetRefineCountSinceALL(int lemmaId) {
    assert(Alive(lemmaId));
    m_lemmaStates[lemmaId].refineCountSinceALL = 0;
}

bool LemmaForestManager::Reachable(int lemmaId) const {
    return Alive(lemmaId) && m_lemmaStates[lemmaId].reachable;
}

void LemmaForestManager::MarkReachableChain(int lemmaId) {
    int cur = lemmaId;
    while (cur != -1) {
        if (!Alive(cur)) break;
        if (m_lemmaStates[cur].reachable) break;
        m_lemmaStates[cur].reachable = true;
        cur = m_forest[cur].parentId;
    }
}

bool LemmaForestManager::PopCTPPred(int lemmaId, Cube &ctpCube, int &ctpLevel) {
    assert(Alive(lemmaId));
    auto &preds = m_lemmaStates[lemmaId].ctpPreds;
    if (preds.empty()) return false;
    auto item = std::move(preds.back());
    preds.pop_back();
    ctpCube = std::move(item.first);
    ctpLevel = item.second;
    return true;
}

void LemmaForestManager::PushCTPPred(int lemmaId, const Cube &ctpCube, int ctpLevel) {
    assert(Alive(lemmaId));
    m_lemmaStates[lemmaId].ctpPreds.emplace_back(ctpCube, ctpLevel);
}

bool LemmaForestManager::HasCTPPreds(int lemmaId) const {
    assert(Alive(lemmaId));
    return !m_lemmaStates[lemmaId].ctpPreds.empty();
}

void LemmaForestManager::ClearCTPState(int lemmaId) {
    assert(Alive(lemmaId));
    m_lemmaStates[lemmaId].ctpPreds.clear();
}


int State::num_inputs = -1;
int State::num_latches = -1;


string State::GetLatchesString() {
    string result = "";
    result.reserve(num_latches);
    int j = 0;
    for (int i = 0; i < num_latches; ++i) {
        if (j >= latches.size() || num_inputs + i + 1 < static_cast<int>(VarOf(latches.at(j)))) {
            result += "x";
        } else {
            result += Sign(latches.at(j)) ? "0" : "1";
            ++j;
        }
    }
    return result;
}


string State::GetInputsString() {
    string result = "";
    result.reserve(num_inputs);
    int j = 0;
    for (int i = 1; i <= num_inputs; ++i) {
        if (j >= inputs.size() || i < static_cast<int>(VarOf(inputs.at(j)))) {
            result += "x";
        } else {
            result += Sign(inputs.at(j)) ? "0" : "1";
            ++j;
        }
    }
    return result;
}


} // namespace car
