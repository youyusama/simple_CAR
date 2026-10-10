#include "DAGCNFSimplifier.h"

#include <algorithm>
#include <cassert>
#include <queue>
#include <unordered_set>

namespace car {

void DAGCNFSimplifier::FreezeVar(Var var) {
    if (var != 0) {
        m_frozen.insert(var);
    }
}

std::vector<Clause> DAGCNFSimplifier::Simplify(const std::vector<Clause> &dagClauses) {
    Reset(dagClauses);
    ConstSimplify();
    BveSimplify();
    SubsumeSimplify();
    return Finalize();
}

void DAGCNFSimplifier::Reset(const std::vector<Clause> &dagClauses) {
    m_occurEnabled = false;
    m_maxVar = 0;
    m_clauseDb.clear();
    m_headClauses.clear();
    m_occurs.clear();

    for (const auto &cls : dagClauses) {
        for (Lit lit : cls) {
            Var lit_var = VarOf(lit);
            if (lit_var > m_maxVar) {
                m_maxVar = lit_var;
            }
        }
    }

    m_values.assign(static_cast<size_t>(m_maxVar) + 1, 0);
    m_headClauses.assign((static_cast<size_t>(m_maxVar) + 1) * 2, std::vector<int>());

    for (const auto &cls : dagClauses) {
        AddRel(cls);
    }
}

void DAGCNFSimplifier::EnableOccur() {
    if (m_occurEnabled) {
        return;
    }
    m_occurEnabled = true;
    m_occurs.assign((static_cast<size_t>(m_maxVar) + 1) * 2, Occur{});

    for (Var v = 1; v <= m_maxVar; ++v) {
        Lit lit_pos = MkLit(v);
        Lit lit_neg = MkLit(v, true);
        const auto &pos = m_headClauses[lit_pos.x];
        const auto &neg = m_headClauses[lit_neg.x];
        for (int cls_id : pos) {
            if (m_clauseDb[cls_id].removed) {
                continue;
            }
            Var head_var = VarOf(m_clauseDb[cls_id].lits.back());
            for (Lit lit : m_clauseDb[cls_id].lits) {
                if (VarOf(lit) != head_var) {
                    OccurAdd(lit, cls_id);
                }
            }
        }
        for (int cls_id : neg) {
            if (m_clauseDb[cls_id].removed) {
                continue;
            }
            Var head_var = VarOf(m_clauseDb[cls_id].lits.back());
            for (Lit lit : m_clauseDb[cls_id].lits) {
                if (VarOf(lit) != head_var) {
                    OccurAdd(lit, cls_id);
                }
            }
        }
    }
}

void DAGCNFSimplifier::DisableOccur() {
    m_occurEnabled = false;
    m_occurs.clear();
}

void DAGCNFSimplifier::AddRel(Clause rel) {
    assert(!rel.empty());

    std::sort(rel.begin(), rel.end());

    if (!TryOrderedSimplify(rel)) {
        return;
    }
    if (rel.empty()) {
        return;
    }

    Lit head_lit = rel.back();
    if (rel.size() == 1) {
        SetLitValue(head_lit);
    }

    int rel_id = static_cast<int>(m_clauseDb.size());
    m_clauseDb.push_back(ClauseEntry{rel, false});
    m_headClauses[head_lit.x].push_back(rel_id);

    if (m_occurEnabled) {
        Var head_var = VarOf(head_lit);
        for (Lit lit : rel) {
            if (VarOf(lit) != head_var) {
                OccurAdd(lit, rel_id);
            }
        }
    }
}

void DAGCNFSimplifier::RemoveRel(int clauseId) {
    if (clauseId < 0 || clauseId >= static_cast<int>(m_clauseDb.size()) || m_clauseDb[clauseId].removed) {
        return;
    }

    Lit head_lit = m_clauseDb[clauseId].lits.back();
    auto &list = m_headClauses[head_lit.x];
    list.erase(std::remove(list.begin(), list.end(), clauseId), list.end());

    if (m_occurEnabled) {
        Var head_var = VarOf(head_lit);
        for (Lit lit : m_clauseDb[clauseId].lits) {
            if (VarOf(lit) != head_var) {
                OccurDel(lit, clauseId);
            }
        }
    }
    m_clauseDb[clauseId].removed = true;
}

void DAGCNFSimplifier::RemoveRels(const std::vector<int> &clauseIds) {
    std::unordered_set<int> unique(clauseIds.begin(), clauseIds.end());
    for (int id : unique) {
        RemoveRel(id);
    }
}

void DAGCNFSimplifier::RemoveNode(Var var_id) {
    Lit lit_pos = MkLit(var_id);
    Lit lit_neg = MkLit(var_id, true);
    auto &pos = m_headClauses[lit_pos.x];
    auto &neg = m_headClauses[lit_neg.x];

    for (int cls_id : pos) {
        if (m_clauseDb[cls_id].removed) {
            continue;
        }
        if (m_occurEnabled) {
            Var head_var = VarOf(m_clauseDb[cls_id].lits.back());
            for (Lit lit : m_clauseDb[cls_id].lits) {
                if (VarOf(lit) != head_var) {
                    OccurDel(lit, cls_id);
                }
            }
        }
        m_clauseDb[cls_id].removed = true;
    }
    for (int cls_id : neg) {
        if (m_clauseDb[cls_id].removed) {
            continue;
        }
        if (m_occurEnabled) {
            Var head_var = VarOf(m_clauseDb[cls_id].lits.back());
            for (Lit lit : m_clauseDb[cls_id].lits) {
                if (VarOf(lit) != head_var) {
                    OccurDel(lit, cls_id);
                }
            }
        }
        m_clauseDb[cls_id].removed = true;
    }
    pos.clear();
    neg.clear();
}

std::vector<int> DAGCNFSimplifier::VarRels(Var var_id) const {
    std::vector<int> res;
    Lit lit_pos = MkLit(var_id);
    Lit lit_neg = MkLit(var_id, true);
    const auto &pos = m_headClauses[lit_pos.x];
    const auto &neg = m_headClauses[lit_neg.x];
    res.reserve(pos.size() + neg.size());
    res.insert(res.end(), pos.begin(), pos.end());
    res.insert(res.end(), neg.begin(), neg.end());
    return res;
}

int8_t DAGCNFSimplifier::LitValue(Lit lit) const {
    if (IsConstTrue(lit)) {
        return static_cast<int8_t>(1);
    }
    if (IsConstFalse(lit)) {
        return static_cast<int8_t>(-1);
    }

    Var lit_var = VarOf(lit);
    if (lit_var >= m_values.size()) {
        return 0;
    }
    int8_t val = m_values[lit_var];
    if (val == 0) {
        return 0;
    }
    return Sign(lit) ? static_cast<int8_t>(-val) : val;
}

void DAGCNFSimplifier::SetLitValue(Lit lit) {
    if (IsConst(lit)) {
        return;
    }
    Var lit_var = VarOf(lit);
    int8_t next = Sign(lit) ? static_cast<int8_t>(-1) : static_cast<int8_t>(1);
    if (m_values[lit_var] == -next) {
        return;
    }
    m_values[lit_var] = next;
}

bool DAGCNFSimplifier::IsVarAssigned(Var var_id) const {
    if (var_id >= m_values.size()) {
        return false;
    }
    return m_values[var_id] != 0;
}

Lit DAGCNFSimplifier::AssignedLit(Var var_id) const {
    return (m_values[var_id] > 0) ? MkLit(var_id) : MkLit(var_id, true);
}

bool DAGCNFSimplifier::TryOrderedSimplify(Clause &cls) const {
    Clause res;
    res.reserve(cls.size());
    for (Lit lit : cls) {
        int8_t lv = LitValue(lit);
        if (lv > 0) {
            return false;
        }
        if (lv < 0) {
            continue;
        }
        if (!res.empty()) {
            if (lit == res.back()) {
                continue;
            }
            if (lit == ~res.back()) {
                return false;
            }
        }
        res.push_back(lit);
    }
    assert(!res.empty());
    cls.swap(res);
    return true;
}

bool DAGCNFSimplifier::TryOrderedResolvent(const Clause &a,
                                           const Clause &b,
                                           Var pivotVar,
                                           Clause &out) const {
    const Clause *x = &a;
    const Clause *y = &b;
    if (a.size() > b.size()) {
        x = &b;
        y = &a;
    }
    out.clear();
    out.reserve(a.size() + b.size());

    size_t i = 0;
    size_t j = 0;
    while (i < x->size()) {
        Lit lit = (*x)[i];
        if (VarOf(lit) == pivotVar) {
            ++i;
            continue;
        }
        while (j < y->size() && VarOf((*y)[j]) < VarOf(lit)) {
            ++j;
        }
        if (j < y->size() && VarOf((*y)[j]) == VarOf(lit)) {
            if (lit == ~(*y)[j]) {
                return false;
            }
        } else {
            out.push_back(lit);
        }
        ++i;
    }

    for (Lit lit : *y) {
        if (VarOf(lit) != pivotVar) {
            out.push_back(lit);
        }
    }
    return true;
}

bool DAGCNFSimplifier::TryOrderedSubsumeExceptOne(const Clause &a,
                                                  const Clause &b,
                                                  bool &subsume,
                                                  bool &diffFound,
                                                  Lit &diff) {
    subsume = false;
    diffFound = false;
    diff = Lit{};
    if (a.size() > b.size()) {
        return false;
    }
    size_t j = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        Var av = VarOf(a[i]);
        while (j < b.size() && VarOf(b[j]) < av) {
            ++j;
        }
        if (j == b.size()) {
            return false;
        }
        if (a[i] != b[j]) {
            if (!diffFound && VarOf(b[j]) == av) {
                diffFound = true;
                diff = a[i];
            } else {
                return false;
            }
        }
    }
    subsume = !diffFound;
    return subsume || diffFound;
}

void DAGCNFSimplifier::OccurAdd(Lit lit, int clauseId) {
    if (!m_occurEnabled) {
        return;
    }
    Occur &o = m_occurs[lit.x];
    o.occur.push_back(clauseId);
    o.size++;
}

void DAGCNFSimplifier::OccurDel(Lit lit, int clauseId) {
    (void)clauseId;
    if (!m_occurEnabled) {
        return;
    }
    Occur &o = m_occurs[lit.x];
    if (o.size > 0) {
        o.size--;
    }
    o.dirty = true;
}

size_t DAGCNFSimplifier::OccurNum(Lit lit) const {
    return m_occurEnabled ? m_occurs[lit.x].size : 0;
}

const std::vector<int> &DAGCNFSimplifier::OccurGet(Lit lit) {
    OccurClean(lit);
    return m_occurs[lit.x].occur;
}

void DAGCNFSimplifier::OccurClean(Lit lit) {
    if (!m_occurEnabled) {
        return;
    }
    Occur &o = m_occurs[lit.x];
    if (!o.dirty) {
        return;
    }
    o.occur.erase(std::remove_if(o.occur.begin(), o.occur.end(),
                                 [&](int id) {
                                     return id < 0 || id >= static_cast<int>(m_clauseDb.size()) ||
                                            m_clauseDb[id].removed;
                                 }),
                  o.occur.end());
    o.dirty = false;
}

bool DAGCNFSimplifier::TryResolvent(const std::vector<int> &pcnf,
                                    const std::vector<int> &ncnf,
                                    Var pivotVar,
                                    size_t limit,
                                    std::vector<Clause> &out) {
    out.clear();
    for (int pcls : pcnf) {
        if (pcls < 0 || pcls >= static_cast<int>(m_clauseDb.size()) || m_clauseDb[pcls].removed) {
            continue;
        }
        for (int ncls : ncnf) {
            if (ncls < 0 || ncls >= static_cast<int>(m_clauseDb.size()) || m_clauseDb[ncls].removed) {
                continue;
            }
            Clause resolvent;
            if (TryOrderedResolvent(m_clauseDb[pcls].lits, m_clauseDb[ncls].lits, pivotVar, resolvent)) {
                out.push_back(resolvent);
            }
            if (out.size() > limit) {
                return false;
            }
        }
    }
    return true;
}

void DAGCNFSimplifier::Eliminate(Var var_id) {
    if (m_frozen.count(var_id) != 0) {
        return;
    }
    Lit lv = MkLit(var_id);
    Lit lnv = MkLit(var_id, true);
    int ocost = static_cast<int>(OccurNum(lv) + OccurNum(lnv) +
                                 m_headClauses[lv.x].size() + m_headClauses[lnv.x].size());
    if (ocost == 0 || ocost > 2000) {
        return;
    }

    std::vector<int> pos = m_headClauses[lv.x];
    std::vector<int> neg = m_headClauses[lnv.x];

    int ncost = 0;
    std::vector<int> opos = OccurGet(lv);
    std::vector<int> oneg = OccurGet(lnv);

    std::vector<Clause> respn;
    if (!TryResolvent(pos, oneg, var_id, static_cast<size_t>(ocost - ncost), respn)) {
        return;
    }
    ncost += static_cast<int>(respn.size());
    if (ncost > ocost) {
        return;
    }

    std::vector<Clause> resnp;
    if (!TryResolvent(neg, opos, var_id, static_cast<size_t>(ocost - ncost), resnp)) {
        return;
    }
    ncost += static_cast<int>(resnp.size());
    if (ncost > ocost) {
        return;
    }

    std::vector<Clause> res = respn;
    res.insert(res.end(), resnp.begin(), resnp.end());
    res = ClauseSubsumeSimplify(res);

    opos.insert(opos.end(), oneg.begin(), oneg.end());
    RemoveRels(opos);
    RemoveNode(var_id);

    for (auto &r : res) {
        AddRel(r);
    }
}

void DAGCNFSimplifier::BveSimplify() {
    EnableOccur();

    struct Entry {
        int cost;
        Var var;
    };
    struct EntryCmp {
        bool operator()(const Entry &a, const Entry &b) const {
            return a.cost > b.cost;
        }
    };

    std::priority_queue<Entry, std::vector<Entry>, EntryCmp> heap;
    std::vector<bool> popped(static_cast<size_t>(m_maxVar) + 1, false);
    for (Var v = 1; v <= m_maxVar; ++v) {
        int cost = static_cast<int>(OccurNum(MkLit(v)) + OccurNum(MkLit(v, true)));
        heap.push(Entry{cost, v});
    }

    while (!heap.empty()) {
        Entry entry = heap.top();
        heap.pop();
        Var v = entry.var;
        if (v > m_maxVar || popped[v]) {
            continue;
        }
        int cost = static_cast<int>(OccurNum(MkLit(v)) + OccurNum(MkLit(v, true)));
        if (cost != entry.cost) {
            heap.push(Entry{cost, v});
            continue;
        }
        popped[v] = true;
        Eliminate(v);
    }
}

void DAGCNFSimplifier::ConstSimplifyVar(Var var_id) {
    std::vector<int> cls_ids = VarRels(var_id);
    std::vector<int> removed;

    for (int id : cls_ids) {
        if (id < 0 || id >= static_cast<int>(m_clauseDb.size()) || m_clauseDb[id].removed) {
            continue;
        }
        Clause cls = m_clauseDb[id].lits;
        if (!TryOrderedSimplify(cls)) {
            removed.push_back(id);
            continue;
        }
        assert(!cls.empty());
        Var head_var = VarOf(cls.back());
        if (head_var != var_id) {
            removed.push_back(id);
        } else if (m_clauseDb[id].lits.size() != cls.size()) {
            AddRel(cls);
        }
    }
    RemoveRels(removed);
}

void DAGCNFSimplifier::ConstSimplify() {
    DisableOccur();
    for (Var v = 1; v <= m_maxVar; ++v) {
        ConstSimplifyVar(v);
    }

    for (Var v = 1; v <= m_maxVar; ++v) {
        if (!IsVarAssigned(v)) {
            continue;
        }
        RemoveNode(v);
        if (m_frozen.count(v) != 0) {
            AddRel(Clause{AssignedLit(v)});
        }
    }
}

void DAGCNFSimplifier::ClauseSubsumeCheck(int clauseId) {
    if (clauseId < 0 || clauseId >= static_cast<int>(m_clauseDb.size()) || m_clauseDb[clauseId].removed) {
        return;
    }

    Clause &ci = m_clauseDb[clauseId].lits;
    assert(!ci.empty());
    Lit best_lit = ci.front();
    size_t best_cost = static_cast<size_t>(-1);

    for (Lit lit : ci) {
        size_t cost = OccurNum(lit) + OccurNum(~lit) +
                      m_headClauses[lit.x].size() + m_headClauses[(~lit).x].size();
        if (cost < best_cost) {
            best_cost = cost;
            best_lit = lit;
        }
    }

    std::vector<int> occurs = OccurGet(best_lit);
    const auto &occ_neg = OccurGet(~best_lit);
    occurs.insert(occurs.end(), occ_neg.begin(), occ_neg.end());
    const auto &head_pos = m_headClauses[best_lit.x];
    const auto &head_neg = m_headClauses[(~best_lit).x];
    occurs.insert(occurs.end(), head_pos.begin(), head_pos.end());
    occurs.insert(occurs.end(), head_neg.begin(), head_neg.end());

    for (int cj_id : occurs) {
        if (cj_id < 0 || cj_id >= static_cast<int>(m_clauseDb.size()) || m_clauseDb[cj_id].removed || cj_id == clauseId) {
            continue;
        }
        Clause &cj = m_clauseDb[cj_id].lits;
        bool subsume = false;
        bool diff_found = false;
        Lit diff{};
        if (!TryOrderedSubsumeExceptOne(ci, cj, subsume, diff_found, diff)) {
            continue;
        }
        if (subsume) {
            Lit head_lit = m_clauseDb[cj_id].lits.back();
            auto &list = m_headClauses[head_lit.x];
            list.erase(std::remove(list.begin(), list.end(), cj_id), list.end());
            m_clauseDb[cj_id].removed = true;
            continue;
        }
        if (!diff_found) {
            continue;
        }

        Lit ci_head = m_clauseDb[clauseId].lits.back();
        Lit cj_head = m_clauseDb[cj_id].lits.back();
        if (ci.size() == cj.size()) {
            if (VarOf(diff) == VarOf(ci_head)) {
                auto &list_ci = m_headClauses[ci_head.x];
                list_ci.erase(std::remove(list_ci.begin(), list_ci.end(), clauseId), list_ci.end());
                auto &list_cj = m_headClauses[cj_head.x];
                list_cj.erase(std::remove(list_cj.begin(), list_cj.end(), cj_id), list_cj.end());
                m_clauseDb[clauseId].removed = true;
                m_clauseDb[cj_id].removed = true;
                return;
            }
            ci.erase(std::remove(ci.begin(), ci.end(), diff), ci.end());
            auto &list_cj = m_headClauses[cj_head.x];
            list_cj.erase(std::remove(list_cj.begin(), list_cj.end(), cj_id), list_cj.end());
            m_clauseDb[cj_id].removed = true;
        } else if (VarOf(diff) == VarOf(cj_head)) {
            auto &list_cj = m_headClauses[cj_head.x];
            list_cj.erase(std::remove(list_cj.begin(), list_cj.end(), cj_id), list_cj.end());
            m_clauseDb[cj_id].removed = true;
        } else {
            Lit nd = ~diff;
            cj.erase(std::remove(cj.begin(), cj.end(), nd), cj.end());
        }
    }
}

void DAGCNFSimplifier::SubsumeSimplify() {
    EnableOccur();
    for (Var v = 1; v <= m_maxVar; ++v) {
        std::vector<int> pos = m_headClauses[MkLit(v).x];
        for (int cls : pos) {
            ClauseSubsumeCheck(cls);
        }
        std::vector<int> neg = m_headClauses[MkLit(v, true).x];
        for (int cls : neg) {
            ClauseSubsumeCheck(cls);
        }
    }
    DisableOccur();

    for (auto &list : m_headClauses) {
        list.erase(std::remove_if(list.begin(), list.end(),
                                  [&](int id) {
                                      return id < 0 || id >= static_cast<int>(m_clauseDb.size()) ||
                                             m_clauseDb[id].removed;
                                  }),
                   list.end());
    }
}

std::vector<Clause> DAGCNFSimplifier::ClauseSubsumeSimplify(std::vector<Clause> clauses) {
    for (auto &cls : clauses) {
        std::sort(cls.begin(), cls.end());
        cls.erase(std::unique(cls.begin(), cls.end()), cls.end());
    }
    std::sort(clauses.begin(), clauses.end(),
              [](const Clause &a, const Clause &b) { return a.size() < b.size(); });

    std::vector<bool> removed(clauses.size(), false);
    size_t i = 0;
    while (i < clauses.size()) {
        if (removed[i]) {
            ++i;
            continue;
        }
        assert(!clauses[i].empty());
        bool update = false;
        for (size_t j = i + 1; j < clauses.size(); ++j) {
            if (removed[j]) {
                continue;
            }
            assert(!clauses[j].empty());
            bool subsume = false;
            bool diff_found = false;
            Lit diff{};
            if (!TryOrderedSubsumeExceptOne(clauses[i], clauses[j], subsume, diff_found, diff)) {
                continue;
            }
            if (subsume) {
                removed[j] = true;
                continue;
            }
            if (!diff_found) {
                continue;
            }
            if (clauses[i].size() == clauses[j].size()) {
                update = true;
                clauses[i].erase(std::remove(clauses[i].begin(), clauses[i].end(), diff),
                                 clauses[i].end());
                removed[j] = true;
            } else {
                Lit nd = ~diff;
                clauses[j].erase(std::remove(clauses[j].begin(), clauses[j].end(), nd),
                                 clauses[j].end());
            }
        }
        if (!update) {
            ++i;
        }
    }

    std::vector<Clause> res;
    res.reserve(clauses.size());
    for (size_t k = 0; k < clauses.size(); ++k) {
        if (!removed[k]) {
            assert(!clauses[k].empty());
            res.push_back(clauses[k]);
        }
    }
    return res;
}

std::vector<Clause> DAGCNFSimplifier::Finalize() const {
    std::vector<Clause> res;
    for (Var v = 1; v <= m_maxVar; ++v) {
        if (m_frozen.count(v) != 0 && IsVarAssigned(v)) {
            res.push_back(Clause{AssignedLit(v)});
            continue;
        }
        Lit lit_pos = MkLit(v);
        Lit lit_neg = MkLit(v, true);
        const auto &pos = m_headClauses[lit_pos.x];
        const auto &neg = m_headClauses[lit_neg.x];
        for (int cls_id : pos) {
            if (!m_clauseDb[cls_id].removed) {
                res.push_back(m_clauseDb[cls_id].lits);
            }
        }
        for (int cls_id : neg) {
            if (!m_clauseDb[cls_id].removed) {
                res.push_back(m_clauseDb[cls_id].lits);
            }
        }
    }
    return res;
}

} // namespace car
