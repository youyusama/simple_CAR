#include "DAGCNFSimplifier.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <queue>
#include <unordered_set>

namespace car {
using Clause = DAGCNFSimplifier::Clause;

// Mark a variable as frozen so it won't be eliminated.
void DAGCNFSimplifier::FreezeVar(int var) {
    if (var < 0) {
        var = -var;
    }
    m_frozen.insert(var);
}

// Run the full DAG CNF simplification pipeline and return simplified clauses.
std::vector<Clause> DAGCNFSimplifier::Simplify(const std::vector<Clause> &dagClauses, int trueId) {
    const int true_id_abs = std::abs(trueId);
    std::vector<Clause> internal;
    internal.reserve(dagClauses.size());
    for (const auto &cls : dagClauses) {
        Clause conv;
        conv.reserve(cls.size());
        for (int lit : cls) {
            int v = std::abs(lit);
            if (v == true_id_abs) {
                conv.push_back(lit == trueId ? 0 : 1);
                continue;
            }
            int ilit = (v << 1) | (lit < 0);
            conv.push_back(ilit);
        }
        internal.emplace_back(std::move(conv));
    }
    Reset(internal);
    // Pipeline: const-prop, bounded variable elimination, then subsumption.
    ConstSimplify();
    BveSimplify();
    SubsumeSimplify();
    std::vector<Clause> out = Finalize();
    for (auto &cls : out) {
        for (int &lit : cls) {
            if (lit == 0) {
                lit = trueId;
            } else if (lit == 1) {
                lit = -trueId;
            } else {
                int v = lit >> 1;
                bool neg = (lit & 1) != 0;
                lit = neg ? -v : v;
            }
        }
    }

    out.insert(out.begin(), Clause{trueId});

    return out;
}

// Initialize internal state from input clauses and seed the Clause database.
void DAGCNFSimplifier::Reset(const std::vector<Clause> &dagClauses) {
    m_occurEnabled = false;
    m_maxVar = 0;
    m_clauseDb.clear();
    m_headClauses.clear();
    m_occurs.clear();

    for (const auto &cls : dagClauses) {
        for (int lit : cls) {
            int v = LitVar(lit);
            if (v > m_maxVar) {
                m_maxVar = v;
            }
        }
    }

    m_values.assign(m_maxVar + 1, 0);
    m_headClauses.assign((m_maxVar + 1) * 2, std::vector<int>());

    for (const auto &cls : dagClauses) {
        AddRel(cls);
    }
}

// Build occurrence lists for non-head literals to speed up elimination/subsumption.
void DAGCNFSimplifier::EnableOccur() {
    if (m_occurEnabled) {
        return;
    }
    m_occurEnabled = true;
    m_occurs.assign((m_maxVar + 1) * 2, Occur{});

    for (int v = 1; v <= m_maxVar; ++v) {
        int lit_pos = VarPosLit(v);
        int lit_neg = VarNegLit(v);
        const auto &pos = m_headClauses[lit_pos];
        const auto &neg = m_headClauses[lit_neg];
        // Only index non-head literals; head literal is the DAG node itself.
        for (int cls_id : pos) {
            if (m_clauseDb[cls_id].removed) {
                continue;
            }
            int head_var = LitVar(m_clauseDb[cls_id].lits.back());
            for (int lit : m_clauseDb[cls_id].lits) {
                if (LitVar(lit) != head_var) {
                    OccurAdd(lit, cls_id);
                }
            }
        }
        for (int cls_id : neg) {
            if (m_clauseDb[cls_id].removed) {
                continue;
            }
            int head_var = LitVar(m_clauseDb[cls_id].lits.back());
            for (int lit : m_clauseDb[cls_id].lits) {
                if (LitVar(lit) != head_var) {
                    OccurAdd(lit, cls_id);
                }
            }
        }
    }
}

// Drop occurrence lists to save memory and avoid stale indices.
void DAGCNFSimplifier::DisableOccur() {
    m_occurEnabled = false;
    m_occurs.clear();
}

// Insert a Clause into the database, applying ordering and unit propagation.
void DAGCNFSimplifier::AddRel(Clause rel) {
    assert(!rel.empty());

    std::sort(rel.begin(), rel.end(), LitLess);

    if (!TryOrderedSimplify(rel)) {
        return;
    }

    if (rel.empty()) {
        return;
    }

    int head_lit = rel.back();
    if (rel.size() == 1) {
        // Unit Clause fixes the variable value early.
        SetLitValue(head_lit);
    }

    int rel_id = static_cast<int>(m_clauseDb.size());
    m_clauseDb.push_back(ClauseEntry{rel, false});
    m_headClauses[head_lit].push_back(rel_id);

    if (m_occurEnabled) {
        int head_var = LitVar(head_lit);
        for (int lit : rel) {
            if (LitVar(lit) != head_var) {
                OccurAdd(lit, rel_id);
            }
        }
    }
}

// Remove one Clause and update indices/occurrence lists.
void DAGCNFSimplifier::RemoveRel(int clauseId) {
    if (clauseId < 0 || clauseId >= static_cast<int>(m_clauseDb.size())) {
        return;
    }
    if (m_clauseDb[clauseId].removed) {
        return;
    }

    int head_lit = m_clauseDb[clauseId].lits.back();
    auto &list = m_headClauses[head_lit];
    list.erase(std::remove(list.begin(), list.end(), clauseId), list.end());

    if (m_occurEnabled) {
        int head_var = LitVar(head_lit);
        for (int lit : m_clauseDb[clauseId].lits) {
            if (LitVar(lit) != head_var) {
                OccurDel(lit, clauseId);
            }
        }
    }
    m_clauseDb[clauseId].removed = true;
}

// Remove a batch of clauses, deduplicating ids first.
void DAGCNFSimplifier::RemoveRels(const std::vector<int> &clauseIds) {
    std::unordered_set<int> unique(clauseIds.begin(), clauseIds.end());
    for (int id : unique) {
        RemoveRel(id);
    }
}

// Remove all clauses whose head is the given variable.
void DAGCNFSimplifier::RemoveNode(int var) {
    int lit_pos = VarPosLit(var);
    int lit_neg = VarNegLit(var);
    auto &pos = m_headClauses[lit_pos];
    auto &neg = m_headClauses[lit_neg];

    for (int cls_id : pos) {
        if (m_clauseDb[cls_id].removed) {
            continue;
        }
        if (m_occurEnabled) {
            int head_var = LitVar(m_clauseDb[cls_id].lits.back());
            for (int lit : m_clauseDb[cls_id].lits) {
                if (LitVar(lit) != head_var) {
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
            int head_var = LitVar(m_clauseDb[cls_id].lits.back());
            for (int lit : m_clauseDb[cls_id].lits) {
                if (LitVar(lit) != head_var) {
                    OccurDel(lit, cls_id);
                }
            }
        }
        m_clauseDb[cls_id].removed = true;
    }
    pos.clear();
    neg.clear();
}

// Collect all Clause ids whose head literal is var or -var.
std::vector<int> DAGCNFSimplifier::VarRels(int var) const {
    std::vector<int> res;
    int lit_pos = VarPosLit(var);
    int lit_neg = VarNegLit(var);
    const auto &pos = m_headClauses[lit_pos];
    const auto &neg = m_headClauses[lit_neg];
    res.reserve(pos.size() + neg.size());
    res.insert(res.end(), pos.begin(), pos.end());
    res.insert(res.end(), neg.begin(), neg.end());
    return res;
}

// Read the current assignment value of a literal.
int8_t DAGCNFSimplifier::LitValue(int lit) const {
    int var = LitVar(lit);
    if (var == 0) {
        return (lit == 0) ? 1 : static_cast<int8_t>(-1);
    }
    if (var < 0 || var >= static_cast<int>(m_values.size())) {
        return 0;
    }
    int8_t val = m_values[var];
    if (val == 0) {
        return 0;
    }
    return ((lit & 1) == 0) ? val : static_cast<int8_t>(-val);
}

// Fix a literal's variable to a value, detecting conflicts.
void DAGCNFSimplifier::SetLitValue(int lit) {
    int var = LitVar(lit);
    if (var == 0) {
        return;
    }
    int8_t next = ((lit & 1) == 0) ? 1 : -1;
    if (m_values[var] == -next) {
        // Clause already unsat; ignore.
    }
    m_values[var] = next;
}

// Check whether a variable has a fixed value.
bool DAGCNFSimplifier::IsVarAssigned(int var) const {
    if (var < 0 || var >= static_cast<int>(m_values.size())) {
        return false;
    }
    return m_values[var] != 0;
}

// Return the literal implied by a fixed variable assignment.
int DAGCNFSimplifier::AssignedLit(int var) const {
    int8_t val = m_values[var];
    return val > 0 ? VarPosLit(var) : VarNegLit(var);
}

// Sort literals by variable id, then polarity.
bool DAGCNFSimplifier::LitLess(int a, int b) {
    int av = LitVar(a);
    int bv = LitVar(b);
    if (av != bv) {
        return av < bv;
    }
    return a < b;
}

// Extract variable id from a literal.
int DAGCNFSimplifier::LitVar(int lit) {
    return lit >> 1;
}

// Flip literal polarity.
int DAGCNFSimplifier::LitNeg(int lit) {
    return lit ^ 1;
}

int DAGCNFSimplifier::VarPosLit(int var) {
    return var << 1;
}

int DAGCNFSimplifier::VarNegLit(int var) {
    return (var << 1) | 1;
}

// Simplify a sorted Clause with current assignments and tautology checks.
bool DAGCNFSimplifier::TryOrderedSimplify(Clause &cls) const {
    Clause res;
    res.reserve(cls.size());
    for (int lit : cls) {
        int8_t lv = LitValue(lit);
        // Clause is satisfied or becomes tautology -> drop.
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
            if (lit == LitNeg(res.back())) {
                return false;
            }
        }
        res.push_back(lit);
    }
    assert(!res.empty());
    cls.swap(res);
    return true;
}

// Build a resolvent of two sorted clauses on the given pivot.
bool DAGCNFSimplifier::TryOrderedResolvent(const Clause &a,
                                           const Clause &b,
                                           int pivotVar,
                                           Clause &out) const {
    // Both clauses are sorted; merge while skipping the pivot.
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
        int lit = (*x)[i];
        if (LitVar(lit) == pivotVar) {
            ++i;
            continue;
        }
        while (j < y->size() && LitVar((*y)[j]) < LitVar(lit)) {
            ++j;
        }
        if (j < y->size() && LitVar((*y)[j]) == LitVar(lit)) {
            if (lit == LitNeg((*y)[j])) {
                return false;
            }
        } else {
            out.push_back(lit);
        }
        ++i;
    }

    for (int lit : *y) {
        if (LitVar(lit) != pivotVar) {
            out.push_back(lit);
        }
    }
    return true;
}

// Check subsumption allowing at most one differing literal by variable.
bool DAGCNFSimplifier::TryOrderedSubsumeExceptOne(const Clause &a,
                                                  const Clause &b,
                                                  bool &subsume,
                                                  bool &diffFound,
                                                  int &diff) {
    subsume = false;
    diffFound = false;
    diff = 0;
    if (a.size() > b.size()) {
        return false;
    }
    size_t j = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        int av = LitVar(a[i]);
        while (j < b.size() && LitVar(b[j]) < av) {
            ++j;
        }
        if (j == b.size()) {
            return false;
        }
        if (a[i] != b[j]) {
            if (!diffFound && LitVar(b[j]) == av) {
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

// Add a Clause id to a literal's occurrence list.
void DAGCNFSimplifier::OccurAdd(int lit, int clauseId) {
    if (!m_occurEnabled) {
        return;
    }
    Occur &o = m_occurs[lit];
    o.occur.push_back(clauseId);
    o.size++;
}

// Lazily remove a Clause id from a literal's occurrence list.
void DAGCNFSimplifier::OccurDel(int lit, int clauseId) {
    (void)clauseId;
    if (!m_occurEnabled) {
        return;
    }
    Occur &o = m_occurs[lit];
    if (o.size > 0) {
        o.size--;
    }
    o.dirty = true;
}

// Return the current occurrence count for a literal.
size_t DAGCNFSimplifier::OccurNum(int lit) const {
    if (!m_occurEnabled) {
        return 0;
    }
    return m_occurs[lit].size;
}

// Get the cleaned occurrence list for a literal.
const std::vector<int> &DAGCNFSimplifier::OccurGet(int lit) {
    OccurClean(lit);
    return m_occurs[lit].occur;
}

// Drop removed Clause ids from a literal's occurrence list.
void DAGCNFSimplifier::OccurClean(int lit) {
    if (!m_occurEnabled) {
        return;
    }
    Occur &o = m_occurs[lit];
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

// Generate resolvents between two Clause-id sets within a size limit.
bool DAGCNFSimplifier::TryResolvent(const std::vector<int> &pcnf,
                                    const std::vector<int> &ncnf,
                                    int pivotVar,
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

// Try bounded variable elimination for a single variable.
void DAGCNFSimplifier::Eliminate(int var) {
    if (m_frozen.count(var) != 0) {
        return;
    }
    // Cost guard prevents blow-up of resolvents.
    int lv = VarPosLit(var);
    int lnv = VarNegLit(var);
    int ocost = static_cast<int>(OccurNum(lv) + OccurNum(lnv) +
                                 m_headClauses[lv].size() + m_headClauses[lnv].size());
    if (ocost == 0 || ocost > 2000) {
        return;
    }

    std::vector<int> pos = m_headClauses[lv];
    std::vector<int> neg = m_headClauses[lnv];

    int ncost = 0;
    std::vector<int> opos = OccurGet(lv);
    std::vector<int> oneg = OccurGet(lnv);

    std::vector<Clause> respn;
    if (!TryResolvent(pos, oneg, var, static_cast<size_t>(ocost - ncost), respn)) {
        return;
    }
    ncost += static_cast<int>(respn.size());
    if (ncost > ocost) {
        return;
    }

    std::vector<Clause> resnp;
    if (!TryResolvent(neg, opos, var, static_cast<size_t>(ocost - ncost), resnp)) {
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
    RemoveNode(var);

    for (auto &r : res) {
        AddRel(r);
    }
}

// Run bounded variable elimination in increasing occurrence order.
void DAGCNFSimplifier::BveSimplify() {
    EnableOccur();

    struct Entry {
        int cost;
        int var;
    };
    struct EntryCmp {
        bool operator()(const Entry &a, const Entry &b) const {
            return a.cost > b.cost;
        }
    };

    std::priority_queue<Entry, std::vector<Entry>, EntryCmp> heap;
    std::vector<bool> popped(m_maxVar + 1, false);
    // Process vars from low occurrence to high occurrence.
    for (int v = 1; v <= m_maxVar; ++v) {
        int cost = static_cast<int>(OccurNum(VarPosLit(v)) + OccurNum(VarNegLit(v)));
        heap.push(Entry{cost, v});
    }

    while (!heap.empty()) {
        Entry entry = heap.top();
        heap.pop();
        int v = entry.var;
        if (v < 0 || v > m_maxVar) {
            continue;
        }
        if (popped[v]) {
            continue;
        }
        int cost = static_cast<int>(OccurNum(VarPosLit(v)) + OccurNum(VarNegLit(v)));
        if (cost != entry.cost) {
            heap.push(Entry{cost, v});
            continue;
        }
        popped[v] = true;
        Eliminate(v);
    }
}

// Simplify all clauses headed by a variable under current assignments.
void DAGCNFSimplifier::ConstSimplifyVar(int var) {
    std::vector<int> cls_ids = VarRels(var);
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
        int head_var = LitVar(cls.back());
        if (head_var != var) {
            removed.push_back(id);
        } else if (m_clauseDb[id].lits.size() != cls.size()) {
            AddRel(cls);
        }
    }
    RemoveRels(removed);
}

// Apply constant propagation and remove fixed nodes.
void DAGCNFSimplifier::ConstSimplify() {
    DisableOccur();
    for (int v = 1; v <= m_maxVar; ++v) {
        ConstSimplifyVar(v);
    }

    for (int v = 1; v <= m_maxVar; ++v) {
        if (!IsVarAssigned(v)) {
            continue;
        }
        RemoveNode(v);
        if (m_frozen.count(v) != 0) {
            AddRel(Clause{AssignedLit(v)});
        }
    }
}

// Check one Clause for subsumption or self-subsumption against nearby candidates.
void DAGCNFSimplifier::ClauseSubsumeCheck(int clauseId) {
    if (clauseId < 0 || clauseId >= static_cast<int>(m_clauseDb.size())) {
        return;
    }
    if (m_clauseDb[clauseId].removed) {
        return;
    }

    Clause &ci = m_clauseDb[clauseId].lits;
    assert(!ci.empty());
    // Use a low-cost literal to limit candidate comparisons.
    int best_lit = ci.front();
    size_t best_cost = static_cast<size_t>(-1);

    for (int lit : ci) {
        size_t cost = OccurNum(lit) + OccurNum(LitNeg(lit)) +
                      m_headClauses[lit].size() + m_headClauses[LitNeg(lit)].size();
        if (cost < best_cost) {
            best_cost = cost;
            best_lit = lit;
        }
    }

    std::vector<int> occurs = OccurGet(best_lit);
    const auto &occ_neg = OccurGet(LitNeg(best_lit));
    occurs.insert(occurs.end(), occ_neg.begin(), occ_neg.end());
    const auto &head_pos = m_headClauses[best_lit];
    const auto &head_neg = m_headClauses[LitNeg(best_lit)];
    occurs.insert(occurs.end(), head_pos.begin(), head_pos.end());
    occurs.insert(occurs.end(), head_neg.begin(), head_neg.end());

    for (int cj_id : occurs) {
        if (cj_id < 0 || cj_id >= static_cast<int>(m_clauseDb.size())) {
            continue;
        }
        if (m_clauseDb[cj_id].removed || cj_id == clauseId) {
            continue;
        }
        Clause &cj = m_clauseDb[cj_id].lits;
        bool subsume = false;
        bool diff_found = false;
        int diff = 0;
        if (!TryOrderedSubsumeExceptOne(ci, cj, subsume, diff_found, diff)) {
            continue;
        }
        if (subsume) {
            int head_lit = m_clauseDb[cj_id].lits.back();
            auto &list = m_headClauses[head_lit];
            list.erase(std::remove(list.begin(), list.end(), cj_id), list.end());
            m_clauseDb[cj_id].removed = true;
            continue;
        }
        if (!diff_found) {
            continue;
        }
        int d = diff;
        int ci_head = m_clauseDb[clauseId].lits.back();
        int cj_head = m_clauseDb[cj_id].lits.back();
        if (ci.size() == cj.size()) {
            if (LitVar(d) == LitVar(ci_head)) {
                int head_ci = m_clauseDb[clauseId].lits.back();
                auto &list_ci = m_headClauses[head_ci];
                list_ci.erase(std::remove(list_ci.begin(), list_ci.end(), clauseId), list_ci.end());
                int head_cj = m_clauseDb[cj_id].lits.back();
                auto &list_cj = m_headClauses[head_cj];
                list_cj.erase(std::remove(list_cj.begin(), list_cj.end(), cj_id), list_cj.end());
                m_clauseDb[clauseId].removed = true;
                m_clauseDb[cj_id].removed = true;
                return;
            }
            ci.erase(std::remove(ci.begin(), ci.end(), d), ci.end());
            int head_cj = m_clauseDb[cj_id].lits.back();
            auto &list_cj = m_headClauses[head_cj];
            list_cj.erase(std::remove(list_cj.begin(), list_cj.end(), cj_id), list_cj.end());
            m_clauseDb[cj_id].removed = true;
        } else if (LitVar(d) == LitVar(cj_head)) {
            auto &list_cj = m_headClauses[cj_head];
            list_cj.erase(std::remove(list_cj.begin(), list_cj.end(), cj_id), list_cj.end());
            m_clauseDb[cj_id].removed = true;
        } else {
            int nd = LitNeg(d);
            cj.erase(std::remove(cj.begin(), cj.end(), nd), cj.end());
        }
    }
}

// Run subsumption-based simplification over all clauses.
void DAGCNFSimplifier::SubsumeSimplify() {
    EnableOccur();
    for (int v = 1; v <= m_maxVar; ++v) {
        std::vector<int> pos = m_headClauses[VarPosLit(v)];
        for (int cls : pos) {
            ClauseSubsumeCheck(cls);
        }
        std::vector<int> neg = m_headClauses[VarNegLit(v)];
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

// Simplify a set of clauses by pairwise subsumption rules.
std::vector<Clause> DAGCNFSimplifier::ClauseSubsumeSimplify(std::vector<Clause> clauses) {
    for (auto &cls : clauses) {
        std::sort(cls.begin(), cls.end(), LitLess);
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
            int diff = 0;
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
            int d = diff;
            if (clauses[i].size() == clauses[j].size()) {
                update = true;
                clauses[i].erase(std::remove(clauses[i].begin(), clauses[i].end(), d),
                                 clauses[i].end());
                removed[j] = true;
            } else {
                int nd = LitNeg(d);
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

// Reconstruct the simplified Clause list from the database.
std::vector<Clause> DAGCNFSimplifier::Finalize() const {
    std::vector<Clause> res;
    for (int v = 1; v <= m_maxVar; ++v) {
        if (m_frozen.count(v) != 0 && IsVarAssigned(v)) {
            // Preserve fixed values for frozen vars.
            res.push_back(Clause{AssignedLit(v)});
            continue;
        }
        int lit_pos = VarPosLit(v);
        int lit_neg = VarNegLit(v);
        const auto &pos = m_headClauses[lit_pos];
        const auto &neg = m_headClauses[lit_neg];
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
