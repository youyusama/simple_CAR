#include "solver.h"

namespace minicore {

namespace {

// Support sets are packed bitsets over the current unsat core.
constexpr size_t MLQPSupportMaskBits = 64;

inline size_t MLQPSupportChunks(size_t bits) {
    return (bits + MLQPSupportMaskBits - 1) / MLQPSupportMaskBits;
}

inline size_t MLQPSupportSize(const MLQPSupportSet &support) {
    size_t size = 0;
    for (MLQPSupportMask chunk : support) {
        size += static_cast<size_t>(__builtin_popcountll(static_cast<unsigned long long>(chunk)));
    }
    return size;
}

inline void SetMLQPUnitSupport(MLQPSupportSet &support, size_t chunks, size_t idx) {
    support.assign(chunks, 0);
    support[idx / MLQPSupportMaskBits] = MLQPSupportMask(1) << (idx % MLQPSupportMaskBits);
}

inline void SetMLQPFullSupport(MLQPSupportSet &support, size_t bits) {
    const size_t chunks = MLQPSupportChunks(bits);
    support.assign(chunks, ~MLQPSupportMask(0));
    const size_t tail_bits = bits % MLQPSupportMaskBits;
    if (tail_bits != 0) {
        support.back() = (MLQPSupportMask(1) << tail_bits) - MLQPSupportMask(1);
    }
}

inline void ClearMLQPSupport(MLQPSupportSet &support, size_t chunks) {
    support.assign(chunks, 0);
}

inline size_t MergeMLQPSupportCountNew(MLQPSupportSet &target, const MLQPSupportMask *source, size_t chunks) {
    size_t new_bits = 0;
    for (size_t i = 0; i < chunks; i++) {
        const MLQPSupportMask old_mask = target[i];
        const MLQPSupportMask new_mask = old_mask | source[i];
        target[i] = new_mask;
        new_bits += static_cast<size_t>(__builtin_popcountll(static_cast<unsigned long long>(new_mask ^ old_mask)));
    }
    return new_bits;
}

inline bool MLQPSupportHasBit(const MLQPSupportSet &support, size_t idx) {
    return support[idx / MLQPSupportMaskBits] & (MLQPSupportMask(1) << (idx % MLQPSupportMaskBits));
}

inline size_t MLQPPendingHash(CRef cr) {
    uint32_t x = cr >> 2;
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return static_cast<size_t>(x);
}

} // namespace


void Solver::initMLQPState(size_t core_size) {
    mlqp_core_size = core_size;
    mlqp_best_size = core_size;
    mlqp_support_chunks = MLQPSupportChunks(core_size);

    if (++mlqp_support_stamp == 0) {
        std::fill(mlqp_support_stamps.begin(), mlqp_support_stamps.end(), 0);
        std::fill(mlqp_pending_stamps.begin(), mlqp_pending_stamps.end(), 0);
        std::fill(mlqp_pending_seen_stamps.begin(), mlqp_pending_seen_stamps.end(), 0);
        ++mlqp_support_stamp;
    }
    mlqp_pending_seen_used = 0;
    const size_t lit_capacity = static_cast<size_t>(alloced_var) * 2 + 1;
    mlqp_assumption_indices.resize(lit_capacity, size_t(-1));
    for (size_t target : mlqp_bypass_targets)
        mlqp_bypass_rules[target].clear();
    mlqp_bypass_targets.clear();
    if (mlqp_bypass_rules.size() < core_size)
        mlqp_bypass_rules.resize(core_size);
    mlqp_support_pool.clear();
    if (mlqp_support_offsets.size() < lit_capacity) {
        mlqp_support_offsets.resize(lit_capacity);
        mlqp_support_sizes.resize(lit_capacity, 0);
        mlqp_support_stamps.resize(lit_capacity, 0);
    }
    if (m_mlqpStrategy > 0 && mlqp_pending_clauses.size() < lit_capacity) {
        mlqp_pending_clauses.resize(lit_capacity);
        mlqp_pending_stamps.resize(lit_capacity, 0);
    }
    if (m_mlqpStrategy > 0) {
        // markMLQPPendingClause grows this table as needed. Most queries touch
        // only a small part of the formula; avoid zeroing a formula-sized table.
        reserveMLQPPendingSeen(16);
    }

    ClearMLQPSupport(mlqp_zero_support, mlqp_support_chunks);
    SetMLQPFullSupport(mlqp_best_core, core_size);

    const size_t bucket_count = core_size + 1;
    if (mlqp_buckets.size() < bucket_count)
        mlqp_buckets.resize(bucket_count);
    for (size_t i = 0; i < bucket_count; i++) {
        mlqp_buckets[i].clear();
    }
    mlqp_bucket_heads.assign(bucket_count, 0);
}

MLQPSupportLookup Solver::lookupMLQPSupport(Lit p) const {
    const int key = toInt(p);
    if (key >= 0 && static_cast<size_t>(key) < mlqp_support_stamps.size() &&
        mlqp_support_stamps[key] == mlqp_support_stamp) {
        return {mlqp_support_pool.data() + mlqp_support_offsets[key], mlqp_support_sizes[key]};
    }

    if (value(p) == l_True && level(var(p)) == 0) {
        return {mlqp_zero_support.data(), 0};
    }

    return {};
}

void Solver::updateMLQPBestCore(const MLQPSupportSet &support) {
    updateMLQPBestCore(support, MLQPSupportSize(support));
}

void Solver::updateMLQPBestCore(const MLQPSupportSet &support, size_t support_size) {
    if (support_size < mlqp_best_size) {
        mlqp_best_size = support_size;
        mlqp_best_core = support;
    }
}

void Solver::updateMLQPSupport(Lit p, const MLQPSupportSet &cand_support, size_t cand_size) {
    const int key = toInt(p);
    assert(key >= 0);
    assert(static_cast<size_t>(key) < mlqp_support_offsets.size());

    // An alternative support need not improve the singleton {p}. Save it before
    // either size cutoff, but never record a rule that depends on its own head.
    const size_t assumption = mlqp_assumption_indices[key];
    if (m_mlqpStrategy == 2 && assumption != size_t(-1) &&
        !MLQPSupportHasBit(cand_support, assumption)) {
        auto &rules = mlqp_bypass_rules[assumption];
        // Repeated clause scans often rediscover the last rule. Avoid a full
        // dominance/deduplication search on this propagation hot path.
        if (rules.empty() || !std::equal(cand_support.begin(), cand_support.end(),
                                         rules.end() - mlqp_support_chunks)) {
            if (rules.empty()) mlqp_bypass_targets.push_back(assumption);
            rules.insert(rules.end(), cand_support.begin(), cand_support.end());
        }
    }
    if (cand_size >= mlqp_best_size)
        return;

    MLQPSupportLookup old_support = lookupMLQPSupport(p);
    if (old_support.bits != nullptr && cand_size >= old_support.size) {
        return;
    }
    if (mlqp_support_stamps[key] != mlqp_support_stamp) {
        mlqp_support_offsets[key] = mlqp_support_pool.size();
        mlqp_support_pool.insert(mlqp_support_pool.end(), cand_support.begin(), cand_support.end());
    } else {
        std::copy(cand_support.begin(), cand_support.end(),
                  mlqp_support_pool.begin() + mlqp_support_offsets[key]);
    }
    mlqp_support_sizes[key] = cand_size;
    mlqp_support_stamps[key] = mlqp_support_stamp;
    mlqp_buckets[cand_size].push_back(p);

    MLQPSupportLookup opp_support = lookupMLQPSupport(~p);
    if (opp_support.bits != nullptr) {
        mlqp_conflict_support = cand_support;
        const size_t conflict_size = cand_size + MergeMLQPSupportCountNew(mlqp_conflict_support, opp_support.bits, mlqp_support_chunks);
        updateMLQPBestCore(mlqp_conflict_support, conflict_size);
    }
}

void Solver::reduceMLQPBypassCore() {
    const size_t before_size = mlqp_best_size;
    // Rules are fixed now. If closure cannot derive u from C\{u}, shrinking C
    // cannot make it derivable later. Thus one sequential pass suffices; each
    // successful deletion still takes effect before testing the next candidate.
    for (size_t candidate : mlqp_bypass_targets) {
        if (!MLQPSupportHasBit(mlqp_best_core, candidate)) continue;

        mlqp_bypass_available = mlqp_best_core;
        const size_t chunk = candidate / MLQPSupportMaskBits;
        const MLQPSupportMask bit = MLQPSupportMask(1) << (candidate % MLQPSupportMaskBits);
        mlqp_bypass_available[chunk] &= ~bit;

        bool changed;
        do {
            changed = false;
            for (size_t target : mlqp_bypass_targets) {
                if (MLQPSupportHasBit(mlqp_bypass_available, target)) continue;
                const auto &rules = mlqp_bypass_rules[target];
                for (size_t offset = 0; offset < rules.size(); offset += mlqp_support_chunks) {
                    size_t i = 0;
                    while (i < mlqp_support_chunks &&
                           (rules[offset + i] & ~mlqp_bypass_available[i]) == 0)
                        ++i;
                    if (i != mlqp_support_chunks) continue;

                    mlqp_bypass_available[target / MLQPSupportMaskBits] |=
                        MLQPSupportMask(1) << (target % MLQPSupportMaskBits);
                    changed = true;
                    break;
                }
                if (mlqp_bypass_available[chunk] & bit) break;
            }
        } while (changed && !(mlqp_bypass_available[chunk] & bit));

        if (mlqp_bypass_available[chunk] & bit) {
            mlqp_best_core[chunk] &= ~bit;
            --mlqp_best_size;
        }
    }
    mlqp_bypass_reduced_literals = before_size - mlqp_best_size;
}

void Solver::addMLQPPendingClause(Lit p, CRef cr) {
    if (m_mlqpStrategy <= 0)
        return;

    const int key = toInt(p);
    assert(key >= 0);
    assert(static_cast<size_t>(key) < mlqp_pending_clauses.size());
    if (mlqp_pending_stamps[key] != mlqp_support_stamp) {
        mlqp_pending_clauses[key].clear();
        mlqp_pending_stamps[key] = mlqp_support_stamp;
    }
    mlqp_pending_clauses[key].push_back(cr);
}

void Solver::reserveMLQPPendingSeen(size_t min_entries) {
    size_t capacity = mlqp_pending_seen_keys.size();
    if (capacity != 0 && capacity * 3 >= min_entries * 4)
        return;

    size_t new_capacity = 16;
    while (new_capacity * 3 < min_entries * 4)
        new_capacity <<= 1;

    std::vector<CRef> old_keys = std::move(mlqp_pending_seen_keys);
    std::vector<uint32_t> old_stamps = std::move(mlqp_pending_seen_stamps);
    mlqp_pending_seen_keys.assign(new_capacity, CRef_Undef);
    mlqp_pending_seen_stamps.assign(new_capacity, 0);

    const uint32_t stamp = mlqp_support_stamp;
    const size_t mask = new_capacity - 1;
    mlqp_pending_seen_used = 0;
    for (size_t i = 0; i < old_keys.size(); i++) {
        if (old_stamps[i] != stamp)
            continue;

        CRef cr = old_keys[i];
        size_t pos = MLQPPendingHash(cr) & mask;
        while (mlqp_pending_seen_stamps[pos] == stamp)
            pos = (pos + 1) & mask;
        mlqp_pending_seen_keys[pos] = cr;
        mlqp_pending_seen_stamps[pos] = stamp;
        mlqp_pending_seen_used++;
    }
}

bool Solver::markMLQPPendingClause(CRef cr) {
    reserveMLQPPendingSeen(mlqp_pending_seen_used + 1);

    const uint32_t stamp = mlqp_support_stamp;
    const size_t mask = mlqp_pending_seen_keys.size() - 1;
    size_t pos = MLQPPendingHash(cr) & mask;
    while (mlqp_pending_seen_stamps[pos] == stamp) {
        if (mlqp_pending_seen_keys[pos] == cr)
            return false;
        pos = (pos + 1) & mask;
    }

    mlqp_pending_seen_keys[pos] = cr;
    mlqp_pending_seen_stamps[pos] = stamp;
    mlqp_pending_seen_used++;
    return true;
}

void Solver::mlqpScanClause(CRef cr) {
    if (isRemoved(cr))
        return;

    Clause &clause = ca->get_clause(cr);
    bool merged_initialized = false;
    size_t merged_size = 0;
    int unresolved = 0;
    Lit local = lit_Undef;
    Lit implied = lit_Undef;
    bool satisfied = false;
    bool pruned = false;

    mlqp_unresolved_clause_tmp.clear();

    for (size_t i = 0; i < clause.size(); i++) {
        Lit q = clause[i];
        if (!inDomain(var(q))) {
            satisfied = true;
            break;
        }
        MLQPSupportLookup neg_support = lookupMLQPSupport(~q);
        // literal is false
        if (neg_support.bits != nullptr) {
            if (!merged_initialized) {
                ClearMLQPSupport(mlqp_merged_support, mlqp_support_chunks);
                merged_initialized = true;
            }
            if (!pruned && neg_support.size != 0) {
                merged_size += MergeMLQPSupportCountNew(mlqp_merged_support, neg_support.bits, mlqp_support_chunks);
            }
            if (merged_size >= mlqp_best_size && mlqp_best_size < mlqp_core_size) {
                pruned = true;
            }
            continue;
        }

        if (lookupMLQPSupport(q).bits != nullptr) {
            if (m_mlqpStrategy == 2 && local == lit_Undef) {
                local = q;
                continue;
            } else {
                satisfied = true;
                break;
            }
        } // literal is true

        // literal is lit_Undef
        unresolved++;
        mlqp_unresolved_clause_tmp.push_back(q);
        implied = q;
    }

    if (!satisfied && !pruned && unresolved > 1 && markMLQPPendingClause(cr)) {
        for (Lit q : mlqp_unresolved_clause_tmp) {
            addMLQPPendingClause(~q, cr);
        }
    }

    if (satisfied || pruned || unresolved > 1)
        return;
    if (!merged_initialized) {
        ClearMLQPSupport(mlqp_merged_support, mlqp_support_chunks);
        merged_initialized = true;
    }
    if (local != lit_Undef) {
        if (unresolved == 0) updateMLQPSupport(local, mlqp_merged_support, merged_size);
    } else {
        if (unresolved == 0)
            updateMLQPBestCore(mlqp_merged_support, merged_size);
        else
            updateMLQPSupport(implied, mlqp_merged_support, merged_size);
    }
}

void Solver::mlqpPropagate(Lit p) {
    // In this solver, watches[p] is the frontier for clauses watching ~p.
    const std::vector<Watcher> &watched_on_neg_p = watches[p];
    mlqp_propagate_clauses.clear();
    for (const Watcher &watch : watched_on_neg_p) {
        Lit blocker = watch.blocker;
        if (!inDomain(var(blocker)))
            continue;
        if (m_mlqpStrategy == 1 && lookupMLQPSupport(blocker).bits != nullptr) // literal is true
            continue;
        mlqp_propagate_clauses.push_back(watch.cref);
    }

    const int key = toInt(p);
    assert(key >= 0);
    assert(static_cast<size_t>(key) < mlqp_pending_clauses.size());
    if (mlqp_pending_stamps[key] == mlqp_support_stamp) {
        auto &pending = mlqp_pending_clauses[key];
        mlqp_propagate_clauses.insert(mlqp_propagate_clauses.end(), pending.begin(), pending.end());
    }

    std::sort(mlqp_propagate_clauses.begin(), mlqp_propagate_clauses.end());
    CRef last = CRef_Undef;
    bool has_last = false;
    for (CRef cr : mlqp_propagate_clauses) {
        if (has_last && cr == last)
            continue;
        last = cr;
        has_last = true;
        mlqpScanClause(cr);
    }
}

bool Solver::mlqp(std::vector<Lit> &core_assumps) {
    mlqp_bypass_reduced_literals = 0;
    if (m_mlqpStrategy <= 0 || m_mlqpStrategy > 2)
        return false;
    if (core_assumps.size() < 3 || nVars() == 0)
        return false;

    initMLQPState(core_assumps.size());
    for (size_t i = 0; i < core_assumps.size(); ++i)
        mlqp_assumption_indices[toInt(core_assumps[i])] = i;

    if (temp_cls_activated) {
        ClearMLQPSupport(mlqp_work_support, mlqp_support_chunks);
        updateMLQPSupport(mkLit(temp_cls_act_var), mlqp_work_support, 0);
    }

    for (size_t i = 0; i < core_assumps.size(); i++) {
        SetMLQPUnitSupport(mlqp_work_support, mlqp_support_chunks, i);
        updateMLQPSupport(core_assumps[i], mlqp_work_support, 1);
    }

    for (size_t bucket_size = 0; bucket_size <= mlqp_core_size && bucket_size < mlqp_best_size; bucket_size++) {
        auto &bucket = mlqp_buckets[bucket_size];
        size_t &head = mlqp_bucket_heads[bucket_size];
        while (head < bucket.size()) {
            Lit p = bucket[head++];

            const int key = toInt(p);
            if (key < 0 || static_cast<size_t>(key) >= mlqp_support_stamps.size() ||
                mlqp_support_stamps[key] != mlqp_support_stamp ||
                mlqp_support_sizes[key] != bucket_size)
                continue;
            if (bucket_size >= mlqp_best_size)
                break;

            mlqpPropagate(p);
        }
    }
    // The input is already an UNSAT core: bypass can help even if propagation
    // did not find a smaller conflict.
    if (m_mlqpStrategy == 2) reduceMLQPBypassCore();
    for (Lit p : core_assumps)
        mlqp_assumption_indices[toInt(p)] = size_t(-1);

    if (mlqp_best_size >= core_assumps.size())
        return false;

    clearFailed();
    for (size_t i = 0; i < core_assumps.size(); i++) {
        if (MLQPSupportHasBit(mlqp_best_core, i)) {
            markFailed(core_assumps[i]);
        }
    }
    return true;
}

} // namespace minicore
