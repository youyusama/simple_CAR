#include "DAGCNFSimplifier.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <queue>
#include <unordered_set>

namespace car {
using clause = DAGCNFSimplifier::clause;

// Mark a variable as frozen so it won't be eliminated.
void DAGCNFSimplifier::FreezeVar(int var) {
    if (var < 0) {
        var = -var;
    }
    m_frozen.insert(var);
}

// Run the full DAG CNF simplification pipeline and return simplified clauses.
std::vector<clause> DAGCNFSimplifier::Simplify(const std::vector<clause> &dag_clauses) {
    Reset(dag_clauses);
    // Pipeline: const-prop, bounded variable elimination, then subsumption.
    ConstSimplify();
    BveSimplify();
    SubsumeSimplify();
    return Finalize();
}

// Initialize internal state from input clauses and seed the clause database.
void DAGCNFSimplifier::Reset(const std::vector<clause> &dag_clauses) {
    m_occurEnabled = false;
    m_maxVar = 0;
    m_clauseDb.clear();
    m_headClauses.clear();
    m_occurs.clear();

    for (const auto &cls : dag_clauses) {
        for (int lit : cls) {
            int v = LitVar(lit);
            if (v > m_maxVar) {
                m_maxVar = v;
            }
        }
    }

    m_values.assign(m_maxVar + 1, 0);
    m_headClauses.assign((m_maxVar + 1) * 2, std::vector<int>());

    for (const auto &cls : dag_clauses) {
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

    for (int v = 0; v <= m_maxVar; ++v) {
        int lit_pos = v;
        int lit_neg = -v;
        const auto &pos = m_headClauses[LitIndex(lit_pos)];
        const auto &neg = m_headClauses[LitIndex(lit_neg)];
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

// Insert a clause into the database, applying ordering and unit propagation.
void DAGCNFSimplifier::AddRel(clause rel) {
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
        // Unit clause fixes the variable value early.
        SetLitValue(head_lit);
    }

    int rel_id = static_cast<int>(m_clauseDb.size());
    m_clauseDb.push_back(ClauseEntry{rel, false});
    m_headClauses[LitIndex(head_lit)].push_back(rel_id);

    if (m_occurEnabled) {
        int head_var = LitVar(head_lit);
        for (int lit : rel) {
            if (LitVar(lit) != head_var) {
                OccurAdd(lit, rel_id);
            }
        }
    }
}

// Remove one clause and update indices/occurrence lists.
void DAGCNFSimplifier::RemoveRel(int clause_id) {
    if (clause_id < 0 || clause_id >= static_cast<int>(m_clauseDb.size())) {
        return;
    }
    if (m_clauseDb[clause_id].removed) {
        return;
    }

    int head_lit = m_clauseDb[clause_id].lits.back();
    auto &list = m_headClauses[LitIndex(head_lit)];
    list.erase(std::remove(list.begin(), list.end(), clause_id), list.end());

    if (m_occurEnabled) {
        int head_var = LitVar(head_lit);
        for (int lit : m_clauseDb[clause_id].lits) {
            if (LitVar(lit) != head_var) {
                OccurDel(lit, clause_id);
            }
        }
    }
    m_clauseDb[clause_id].removed = true;
}

// Remove a batch of clauses, deduplicating ids first.
void DAGCNFSimplifier::RemoveRels(const std::vector<int> &clause_ids) {
    std::unordered_set<int> unique(clause_ids.begin(), clause_ids.end());
    for (int id : unique) {
        RemoveRel(id);
    }
}

// Remove all clauses whose head is the given variable.
void DAGCNFSimplifier::RemoveNode(int var) {
    int lit_pos = var;
    int lit_neg = -var;
    auto &pos = m_headClauses[LitIndex(lit_pos)];
    auto &neg = m_headClauses[LitIndex(lit_neg)];

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

// Collect all clause ids whose head literal is var or -var.
std::vector<int> DAGCNFSimplifier::VarRels(int var) const {
    std::vector<int> res;
    int lit_pos = var;
    int lit_neg = -var;
    const auto &pos = m_headClauses[LitIndex(lit_pos)];
    const auto &neg = m_headClauses[LitIndex(lit_neg)];
    res.reserve(pos.size() + neg.size());
    res.insert(res.end(), pos.begin(), pos.end());
    res.insert(res.end(), neg.begin(), neg.end());
    return res;
}

// Read the current assignment value of a literal.
int8_t DAGCNFSimplifier::LitValue(int lit) const {
    int var = LitVar(lit);
    if (var < 0 || var >= static_cast<int>(m_values.size())) {
        return 0;
    }
    int8_t val = m_values[var];
    if (val == 0) {
        return 0;
    }
    return (lit > 0) ? val : static_cast<int8_t>(-val);
}

// Fix a literal's variable to a value, detecting conflicts.
void DAGCNFSimplifier::SetLitValue(int lit) {
    int var = LitVar(lit);
    int8_t next = (lit > 0) ? 1 : -1;
    if (m_values[var] == -next) {
        // clause already unsat; ignore.
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
    return val > 0 ? var : -var;
}

// Sort literals by variable id, then polarity.
bool DAGCNFSimplifier::LitLess(int a, int b) {
    int av = std::abs(a);
    int bv = std::abs(b);
    if (av != bv) {
        return av < bv;
    }
    return a < b;
}

// Extract variable id from a literal.
int DAGCNFSimplifier::LitVar(int lit) {
    return std::abs(lit);
}

// Map a literal to the internal index space [0, 2*maxVar].
int DAGCNFSimplifier::LitIndex(int lit) {
    int v = LitVar(lit);
    return (v << 1) | (lit < 0);
}

// Flip literal polarity.
int DAGCNFSimplifier::LitNeg(int lit) {
    return -lit;
}

// Simplify a sorted clause with current assignments and tautology checks.
bool DAGCNFSimplifier::TryOrderedSimplify(clause &cls) const {
    clause res;
    res.reserve(cls.size());
    for (int lit : cls) {
        int8_t lv = LitValue(lit);
        // clause is satisfied or becomes tautology -> drop.
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
bool DAGCNFSimplifier::TryOrderedResolvent(const clause &a,
                                           const clause &b,
                                           int pivot_var,
                                           clause &out) const {
    // Both clauses are sorted; merge while skipping the pivot.
    const clause *x = &a;
    const clause *y = &b;
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
        if (LitVar(lit) == pivot_var) {
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
        if (LitVar(lit) != pivot_var) {
            out.push_back(lit);
        }
    }
    return true;
}

// Check subsumption allowing at most one differing literal by variable.
bool DAGCNFSimplifier::TryOrderedSubsumeExceptOne(const clause &a,
                                                  const clause &b,
                                                  bool &subsume,
                                                  bool &diff_found,
                                                  int &diff) {
    subsume = false;
    diff_found = false;
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
            if (!diff_found && LitVar(b[j]) == av) {
                diff_found = true;
                diff = a[i];
            } else {
                return false;
            }
        }
    }
    subsume = !diff_found;
    return subsume || diff_found;
}

// Add a clause id to a literal's occurrence list.
void DAGCNFSimplifier::OccurAdd(int lit, int clause_id) {
    if (!m_occurEnabled) {
        return;
    }
    Occur &o = m_occurs[LitIndex(lit)];
    o.occur.push_back(clause_id);
    o.size++;
}

// Lazily remove a clause id from a literal's occurrence list.
void DAGCNFSimplifier::OccurDel(int lit, int clause_id) {
    (void)clause_id;
    if (!m_occurEnabled) {
        return;
    }
    Occur &o = m_occurs[LitIndex(lit)];
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
    return m_occurs[LitIndex(lit)].size;
}

// Get the cleaned occurrence list for a literal.
const std::vector<int> &DAGCNFSimplifier::OccurGet(int lit) {
    OccurClean(lit);
    return m_occurs[LitIndex(lit)].occur;
}

// Drop removed clause ids from a literal's occurrence list.
void DAGCNFSimplifier::OccurClean(int lit) {
    if (!m_occurEnabled) {
        return;
    }
    Occur &o = m_occurs[LitIndex(lit)];
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

// Generate resolvents between two clause-id sets within a size limit.
bool DAGCNFSimplifier::TryResolvent(const std::vector<int> &pcnf,
                                    const std::vector<int> &ncnf,
                                    int pivot_var,
                                    size_t limit,
                                    std::vector<clause> &out) {
    out.clear();
    for (int pcls : pcnf) {
        if (pcls < 0 || pcls >= static_cast<int>(m_clauseDb.size()) || m_clauseDb[pcls].removed) {
            continue;
        }
        for (int ncls : ncnf) {
            if (ncls < 0 || ncls >= static_cast<int>(m_clauseDb.size()) || m_clauseDb[ncls].removed) {
                continue;
            }
            clause resolvent;
            if (TryOrderedResolvent(m_clauseDb[pcls].lits, m_clauseDb[ncls].lits, pivot_var, resolvent)) {
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
    int lv = var;
    int ocost = static_cast<int>(OccurNum(lv) + OccurNum(-lv) +
                                 m_headClauses[LitIndex(lv)].size() + m_headClauses[LitIndex(-lv)].size());
    if (ocost == 0 || ocost > 2000) {
        return;
    }

    std::vector<int> pos = m_headClauses[LitIndex(lv)];
    std::vector<int> neg = m_headClauses[LitIndex(-lv)];

    int ncost = 0;
    std::vector<int> opos = OccurGet(lv);
    std::vector<int> oneg = OccurGet(-lv);

    std::vector<clause> respn;
    if (!TryResolvent(pos, oneg, var, static_cast<size_t>(ocost - ncost), respn)) {
        return;
    }
    ncost += static_cast<int>(respn.size());
    if (ncost > ocost) {
        return;
    }

    std::vector<clause> resnp;
    if (!TryResolvent(neg, opos, var, static_cast<size_t>(ocost - ncost), resnp)) {
        return;
    }
    ncost += static_cast<int>(resnp.size());
    if (ncost > ocost) {
        return;
    }

    std::vector<clause> res = respn;
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
    for (int v = 0; v <= m_maxVar; ++v) {
        int cost = static_cast<int>(OccurNum(v) + OccurNum(-v));
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
        int cost = static_cast<int>(OccurNum(v) + OccurNum(-v));
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
        clause cls = m_clauseDb[id].lits;
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
            AddRel(clause{AssignedLit(v)});
        }
    }
}

// Check one clause for subsumption or self-subsumption against nearby candidates.
void DAGCNFSimplifier::ClauseSubsumeCheck(int clause_id) {
    if (clause_id < 0 || clause_id >= static_cast<int>(m_clauseDb.size())) {
        return;
    }
    if (m_clauseDb[clause_id].removed) {
        return;
    }

    clause &ci = m_clauseDb[clause_id].lits;
    assert(!ci.empty());
    // Use a low-cost literal to limit candidate comparisons.
    int best_lit = ci.front();
    size_t best_cost = static_cast<size_t>(-1);

    for (int lit : ci) {
        size_t cost = OccurNum(lit) + OccurNum(-lit) +
                      m_headClauses[LitIndex(lit)].size() + m_headClauses[LitIndex(-lit)].size();
        if (cost < best_cost) {
            best_cost = cost;
            best_lit = lit;
        }
    }

    std::vector<int> occurs = OccurGet(best_lit);
    const auto &occ_neg = OccurGet(-best_lit);
    occurs.insert(occurs.end(), occ_neg.begin(), occ_neg.end());
    const auto &head_pos = m_headClauses[LitIndex(best_lit)];
    const auto &head_neg = m_headClauses[LitIndex(-best_lit)];
    occurs.insert(occurs.end(), head_pos.begin(), head_pos.end());
    occurs.insert(occurs.end(), head_neg.begin(), head_neg.end());

    for (int cj_id : occurs) {
        if (cj_id < 0 || cj_id >= static_cast<int>(m_clauseDb.size())) {
            continue;
        }
        if (m_clauseDb[cj_id].removed || cj_id == clause_id) {
            continue;
        }
        clause &cj = m_clauseDb[cj_id].lits;
        bool subsume = false;
        bool diff_found = false;
        int diff = 0;
        if (!TryOrderedSubsumeExceptOne(ci, cj, subsume, diff_found, diff)) {
            continue;
        }
        if (subsume) {
            int head_lit = m_clauseDb[cj_id].lits.back();
            auto &list = m_headClauses[LitIndex(head_lit)];
            list.erase(std::remove(list.begin(), list.end(), cj_id), list.end());
            m_clauseDb[cj_id].removed = true;
            continue;
        }
        if (!diff_found) {
            continue;
        }
        int d = diff;
        int ci_head = m_clauseDb[clause_id].lits.back();
        int cj_head = m_clauseDb[cj_id].lits.back();
        if (ci.size() == cj.size()) {
            if (LitVar(d) == LitVar(ci_head)) {
                int head_ci = m_clauseDb[clause_id].lits.back();
                auto &list_ci = m_headClauses[LitIndex(head_ci)];
                list_ci.erase(std::remove(list_ci.begin(), list_ci.end(), clause_id), list_ci.end());
                int head_cj = m_clauseDb[cj_id].lits.back();
                auto &list_cj = m_headClauses[LitIndex(head_cj)];
                list_cj.erase(std::remove(list_cj.begin(), list_cj.end(), cj_id), list_cj.end());
                m_clauseDb[clause_id].removed = true;
                m_clauseDb[cj_id].removed = true;
                return;
            }
            ci.erase(std::remove(ci.begin(), ci.end(), d), ci.end());
            int head_cj = m_clauseDb[cj_id].lits.back();
            auto &list_cj = m_headClauses[LitIndex(head_cj)];
            list_cj.erase(std::remove(list_cj.begin(), list_cj.end(), cj_id), list_cj.end());
            m_clauseDb[cj_id].removed = true;
        } else if (LitVar(d) == LitVar(cj_head)) {
            auto &list_cj = m_headClauses[LitIndex(cj_head)];
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
    for (int v = 0; v <= m_maxVar; ++v) {
        std::vector<int> pos = m_headClauses[LitIndex(v)];
        for (int cls : pos) {
            ClauseSubsumeCheck(cls);
        }
        std::vector<int> neg = m_headClauses[LitIndex(-v)];
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
std::vector<clause> DAGCNFSimplifier::ClauseSubsumeSimplify(std::vector<clause> clauses) {
    for (auto &cls : clauses) {
        std::sort(cls.begin(), cls.end(), LitLess);
        cls.erase(std::unique(cls.begin(), cls.end()), cls.end());
    }
    std::sort(clauses.begin(), clauses.end(),
              [](const clause &a, const clause &b) { return a.size() < b.size(); });

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

    std::vector<clause> res;
    res.reserve(clauses.size());
    for (size_t k = 0; k < clauses.size(); ++k) {
        if (!removed[k]) {
            assert(!clauses[k].empty());
            res.push_back(clauses[k]);
        }
    }
    return res;
}

// Reconstruct the simplified clause list from the database.
std::vector<clause> DAGCNFSimplifier::Finalize() const {
    std::vector<clause> res;
    for (int v = 1; v <= m_maxVar; ++v) {
        if (m_frozen.count(v) != 0 && IsVarAssigned(v)) {
            // Preserve fixed values for frozen vars.
            res.push_back(clause{AssignedLit(v)});
            continue;
        }
        int lit_pos = v;
        int lit_neg = -v;
        const auto &pos = m_headClauses[LitIndex(lit_pos)];
        const auto &neg = m_headClauses[LitIndex(lit_neg)];
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
