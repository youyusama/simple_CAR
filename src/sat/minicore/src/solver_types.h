#ifndef MINICORE_SOLVER_TYPES_H
#define MINICORE_SOLVER_TYPES_H

#include <algorithm>
#include <assert.h>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <memory_resource>
#include <stdint.h>
#include <vector>

namespace minicore {


// variable
typedef int Var;
const Var var_Undef = -1;

// literal
struct Lit {
    int x;

    friend Lit mkLit(Var var, bool sign);

    bool operator==(Lit p) const { return x == p.x; }
    bool operator!=(Lit p) const { return x != p.x; }
    bool operator<(Lit p) const { return x < p.x; } // '<' makes p, ~p adjacent in the ordering.
};
inline Lit mkLit(Var var, bool sign = false) {
    Lit p;
    p.x = var + var + (int)sign;
    return p;
}
inline Lit operator~(Lit p) {
    Lit q;
    q.x = p.x ^ 1;
    return q;
}
inline Lit operator^(Lit p, bool b) {
    Lit q;
    q.x = p.x ^ (unsigned int)b;
    return q;
}
inline bool sign(Lit p) { return p.x & 1; }
inline int var(Lit p) { return p.x >> 1; }

inline int toInt(Var v) { return v; }
inline int toInt(Lit p) { return p.x; }
inline Lit toLit(int i) {
    Lit p;
    p.x = i;
    return p;
}

struct LitHash {
    int operator()(const Lit &p) const {
        return p.x;
    }
};

const Lit lit_Undef = {-2}; // special constants
const Lit lit_Error = {-1}; //


// ternary logic
class lbool {
    uint8_t value;

  public:
    explicit lbool(uint8_t v) : value(v) {}

    lbool() : value(0) {}
    explicit lbool(bool x) : value(!x) {}

    bool operator==(lbool b) const { return ((b.value & 2) & (value & 2)) | (!(b.value & 2) & (value == b.value)); }
    bool operator!=(lbool b) const { return !(*this == b); }
    lbool operator^(bool b) const { return lbool((uint8_t)(value ^ (uint8_t)b)); }

    lbool operator&&(lbool b) const {
        uint8_t sel = (this->value << 1) | (b.value << 3);
        uint8_t v = (0xF7F755F4 >> sel) & 3;
        return lbool(v);
    }

    lbool operator||(lbool b) const {
        uint8_t sel = (this->value << 1) | (b.value << 3);
        uint8_t v = (0xFCFCF400 >> sel) & 3;
        return lbool(v);
    }

    friend int toInt(lbool l);
    friend lbool toLbool(int v);
};
inline int toInt(lbool l) { return l.value; }
inline lbool toLbool(int v) { return lbool((uint8_t)v); }

const lbool l_True((uint8_t)0);
const lbool l_False((uint8_t)1);
const lbool l_Undef((uint8_t)2);


// clause reference
using CRef = void *;
static constexpr CRef CRef_Undef = nullptr;


// clause
class Clause {
  public:
    struct Header {
        uint32_t mark : 2;
        uint32_t learnt : 1;
        uint32_t has_extra : 1;
        uint32_t reloced : 1;
        uint32_t size : 27;
    };

  private:
    Header header;

    Lit *data() noexcept {
        return reinterpret_cast<Lit *>(this + 1);
    }

    const Lit *data() const noexcept {
        return reinterpret_cast<const Lit *>(this + 1);
    }

  public:
    // Disable copying
    Clause(const Clause &) = delete;
    Clause &operator=(const Clause &) = delete;

    // Constructor for new clauses
    Clause(const std::vector<Lit> &ps, bool use_extra, bool learnt) {
        header = {0, static_cast<uint32_t>(learnt), static_cast<uint32_t>(use_extra), 0, static_cast<uint32_t>(ps.size())};
        if (!ps.empty()) {
            std::copy(ps.begin(), ps.end(), data());
        }
        if (header.has_extra) {
            if (header.learnt) {
                activity() = 0.0f;
            } else {
                calc_abstraction();
            }
        }
    }

    // Constructor for cloning clauses
    Clause(const Clause &other, bool use_extra) : header(other.header) {
        header.has_extra = use_extra;
        if (header.size > 0) {
            std::copy_n(other.data(), header.size, data());
        }
        if (header.has_extra && other.header.has_extra) {
            if (header.learnt) {
                activity() = other.activity();
            } else {
                abstraction() = other.abstraction();
            }
        }
    }

    void calc_abstraction() noexcept {
        assert(has_extra() && !learnt());
        uint32_t abs = 0;
        for (uint32_t i = 0; i < header.size; ++i) {
            abs |= 1 << (var(data()[i]) & 31);
        }
        extra_field() = abs;
    }

    // Accessors
    uint32_t size() const noexcept { return header.size; }
    bool learnt() const noexcept { return header.learnt; }
    bool has_extra() const noexcept { return header.has_extra; }
    uint32_t get_mark() const noexcept { return header.mark; }
    void set_mark(uint32_t m) noexcept { header.mark = m; }
    bool reloced() const noexcept { return header.reloced; }

    CRef relocation() const noexcept {
        assert(reloced());
        return *reinterpret_cast<const CRef *>(data());
    }

    void relocate(CRef c) noexcept {
        assert(!reloced());
        header.reloced = 1;
        *reinterpret_cast<CRef *>(data()) = c;
    }

    // Memory operations
    void shrink(uint32_t n) noexcept {
        assert(n <= size());
        if (has_extra() && n > 0) {
            extra_field() = *reinterpret_cast<uint32_t *>(data() + header.size);
        }
        header.size -= n;
    }

    void pop() noexcept { shrink(1); }

    // Element access
    const Lit &operator[](uint32_t i) const noexcept {
        assert(i < size());
        return data()[i];
    }

    Lit &operator[](uint32_t i) noexcept {
        assert(i < size());
        return data()[i];
    }

    operator const Lit *() const noexcept { return data(); }
    const Lit &last() const noexcept { return (*this)[size() - 1]; }

    // Extra field access
    uint32_t &activity() noexcept {
        assert(has_extra() && learnt());
        return *reinterpret_cast<uint32_t *>(data() + header.size);
    }

    uint32_t activity() const noexcept {
        assert(has_extra() && learnt());
        return *reinterpret_cast<const uint32_t *>(data() + header.size);
    }

    uint32_t &abstraction() noexcept {
        assert(has_extra() && !learnt());
        return *reinterpret_cast<uint32_t *>(data() + header.size);
    }

    uint32_t abstraction() const noexcept {
        assert(has_extra() && !learnt());
        return *reinterpret_cast<const uint32_t *>(data() + header.size);
    }

    std::string tostring() const noexcept {
        std::string res;
        for (uint32_t i = 0; i < size() && i < 10; i++) {
            Lit p = data()[i];
            res += std::to_string((!sign(p) ? var(p) : -var(p))) + " ";
        }
        if (learnt()) res += "learnt";
        return res;
    }

  private:
    uint32_t &extra_field() noexcept {
        return *reinterpret_cast<uint32_t *>(data() + header.size);
    }

    friend class ClauseAllocator;
};


class TrackingMemoryResource : public std::pmr::memory_resource {
  public:
    explicit TrackingMemoryResource(std::pmr::memory_resource *upstream =
                                        std::pmr::get_default_resource())
        : upstream_(upstream), allocated_(0), wasted_(0) {}

    size_t allocated() const noexcept { return allocated_; }
    size_t wasted() const noexcept { return wasted_; }
    void reset_stats() noexcept {
        allocated_ = 0;
        wasted_ = 0;
    }

    void *do_allocate(size_t bytes, size_t alignment) override {
        void *p = upstream_->allocate(bytes, alignment);
        if (p) allocated_ += bytes;
        return p;
    }

    void do_deallocate(void *p, size_t bytes, size_t alignment) override {
        if (p) {
            wasted_ += bytes;
            upstream_->deallocate(p, bytes, alignment);
        }
    }

    bool do_is_equal(const std::pmr::memory_resource &other) const noexcept override {
        return this == &other;
    }

  private:
    std::pmr::memory_resource *upstream_;
    size_t allocated_;
    size_t wasted_;
};


// clause allocator
class ClauseAllocator {
  private:
    // Constants
    static constexpr size_t INITIAL_BUFFER_SIZE = 16 * 1024;

    // Memory resource management
    TrackingMemoryResource tracking_resource_;
    std::pmr::monotonic_buffer_resource buffer_resource_;
    std::pmr::memory_resource *active_resource_;

    // Statistics
    struct {
        size_t clause_allocs = 0;
        size_t wasted_memory = 0;
    } stats_;

  public:
    explicit ClauseAllocator()
        : buffer_resource_(
              tracking_resource_.allocate(INITIAL_BUFFER_SIZE, alignof(std::max_align_t)),
              INITIAL_BUFFER_SIZE,
              &tracking_resource_),
          active_resource_(&buffer_resource_) {
    }

    explicit ClauseAllocator(size_t initial_size)
        : buffer_resource_(
              tracking_resource_.allocate(initial_size, alignof(std::max_align_t)),
              initial_size,
              &tracking_resource_),
          active_resource_(&buffer_resource_) {
    }

    ~ClauseAllocator() {
        buffer_resource_.release();
    }

    // Disallow copying
    ClauseAllocator(const ClauseAllocator &) = delete;
    ClauseAllocator &operator=(const ClauseAllocator &) = delete;

    // ================================
    // Public Interface
    // ================================

    // Allocate a new clause
    inline CRef alloc(const std::vector<Lit> &ps, bool learnt = false) {

        // Large clause handling
        return allocate_large(ps, learnt, learnt);
    }

    inline CRef alloc(const Clause &from) {
        const bool use_extra = from.learnt();

        // Large clause handling
        return allocate_large(from, use_extra);
    }

    // Release a clause
    void free(CRef ref) {
        // Large clause - handle through PMR
        release_large(ref);
    }

    // relocate clause in another allocator
    void reloc(CRef &cr, std::shared_ptr<ClauseAllocator> to) {
        Clause &c = get_clause(cr);

        if (c.reloced()) {
            cr = c.relocation();
            return;
        }

        cr = to->alloc(c);
        c.relocate(cr);
    }

    // Access clause data
    Clause &get_clause(CRef ref) const noexcept {
        return *reinterpret_cast<Clause *>(ref);
    }

    // Statistics access
    size_t allocated_memory() const noexcept { return tracking_resource_.allocated(); }
    size_t wasted_memory() const noexcept { return stats_.wasted_memory; }
    size_t active_memory() const noexcept { return allocated_memory() - wasted_memory(); }

    void reset_statistics() noexcept {
        stats_ = {};
        tracking_resource_.reset_stats();
    }

  private:
    // ================================
    // Private Implementation
    // ================================

    static inline size_t round_up(size_t x, size_t a) {
        return (x + a - 1) & ~(a - 1);
    }

    // Large clause allocation
    CRef allocate_large(const std::vector<Lit> &ps, bool learnt, bool use_extra) {
        stats_.clause_allocs++;

        // Calculate required memory
        const size_t base_size = sizeof(Clause);
        const size_t lits_size = sizeof(Lit) * ps.size();
        const size_t extra_size = use_extra ? sizeof(uint32_t) : 0;

        const size_t tail_padded = round_up(lits_size + extra_size, alignof(void *));
        const size_t total_size = base_size + tail_padded;

        // Align memory for Clause
        // constexpr size_t alignment = alignof(Clause);
        // const size_t aligned_size = (total_size + alignment - 1) & ~(alignment - 1);
        // assert(total_size == aligned_size);

        // Allocate memory through PMR
        void *mem = active_resource_->allocate(total_size, alignof(void *));

        // Construct clause in place
        Clause *clause = new (mem) Clause(ps, use_extra, learnt);
        return reinterpret_cast<CRef>(clause);
    }

    CRef allocate_large(const Clause &from, bool use_extra) {
        stats_.clause_allocs++;

        // Calculate required memory
        const size_t base_size = sizeof(Clause);
        const size_t lits_size = sizeof(Lit) * from.size();
        const size_t extra_size = use_extra ? sizeof(uint32_t) : 0;

        const size_t tail_padded = round_up(lits_size + extra_size, alignof(void *));
        const size_t total_size = base_size + tail_padded;

        // Align memory for Clause
        // constexpr size_t alignment = alignof(Clause);
        // const size_t aligned_size = (total_size + alignment - 1) & ~(alignment - 1);
        // assert(total_size == aligned_size);

        // Allocate memory through PMR
        void *mem = active_resource_->allocate(total_size, alignof(void *));

        // Construct clause in place
        Clause *clause = new (mem) Clause(from, use_extra);
        return reinterpret_cast<CRef>(clause);
    }


    // Large clause release
    void release_large(CRef ref) {
        Clause *clause = reinterpret_cast<Clause *>(ref);

        // Calculate memory size
        const bool use_extra = clause->has_extra();
        const size_t base_size = sizeof(Clause);
        const size_t lits_size = sizeof(Lit) * clause->size();
        const size_t extra_size = use_extra ? sizeof(uint32_t) : 0;
        const size_t total_size = base_size + lits_size + extra_size;

        // Align memory for deallocation
        // constexpr size_t alignment = alignof(Clause);
        // const size_t aligned_size = (total_size + alignment - 1) & ~(alignment - 1);
        // assert(total_size == aligned_size);

        // release memory
        stats_.wasted_memory += total_size;
        // active_resource_->deallocate(clause, aligned_size, alignment);
    }
};


struct VarData {
    CRef reason;
    size_t level;
};

static inline VarData mkVarData(CRef cr, size_t l) {
    VarData d = {cr, l};
    return d;
}

struct Watcher {
    CRef cref;
    Lit blocker;
    Watcher(CRef cr, Lit p) : cref(cr), blocker(p) {}
    Watcher() : cref(CRef_Undef), blocker(lit_Undef) {}
    bool operator==(const Watcher &w) const { return cref == w.cref; }
    bool operator!=(const Watcher &w) const { return cref != w.cref; }
};


struct VarOrderLt {
    std::vector<uint32_t> &act_;
    VarOrderLt(std::vector<uint32_t> &act) : act_(act) {}
    bool operator()(Var x, Var y) const { return act_[x] < act_[y]; }
};


struct reduceDB_lt {
    std::shared_ptr<ClauseAllocator> ca_;
    reduceDB_lt(std::shared_ptr<ClauseAllocator> ca) : ca_(ca) {}
    bool operator()(CRef x, CRef y) {
        return ca_->get_clause(x).size() > 2 &&
               (ca_->get_clause(y).size() == 2 ||
                ca_->get_clause(x).activity() < ca_->get_clause(y).activity());
    }
};

class OccLists {
    std::vector<std::vector<Watcher>> occs;
    std::vector<bool> dirty_flags;
    std::vector<Lit> dirty_indices;
    std::shared_ptr<ClauseAllocator> cls_allocator;

  public:
    explicit OccLists(std::shared_ptr<ClauseAllocator> ca) : cls_allocator(ca) {}

    void ensure(size_t key) {
        dirty_flags.resize(key, false);
        occs.resize(key, std::vector<Watcher>());
    }

    std::vector<Watcher> &operator[](Lit key) {
        return occs[key.x];
    }

    std::vector<Watcher> &on(Lit key) {
        if (dirty_flags[key.x]) {
            clean(key);
        }
        return occs[key.x];
    }

    void cleanAll() {
        for (Lit key : dirty_indices) clean(key);
        dirty_indices.clear();
    }

    bool deleted(Watcher w) {
        return cls_allocator->get_clause(w.cref).get_mark() == (uint32_t)1;
    }

    void clean(Lit key) {
        std::vector<Watcher> &vec = occs[key.x];
        auto new_end = std::remove_if(vec.begin(), vec.end(),
                                      [this](Watcher w) { return this->deleted(w); });
        vec.erase(new_end, vec.end());
        dirty_flags[key.x] = false;
    }

    void smudge(Lit key) {
        if (!dirty_flags[key.x]) {
            dirty_flags[key.x] = true;
            dirty_indices.emplace_back(key);
        }
    }

    void clear() {
        occs.clear();
        dirty_flags.clear();
        dirty_indices.clear();
    }
};


struct ShrinkStackElem {
    uint32_t i;
    Lit l;
    ShrinkStackElem(uint32_t _i, Lit _l) : i(_i), l(_l) {}
};


class DecisionBuckets {
  public:
    DecisionBuckets() : max_bucket_index_(0), var_inc_(0) {
        buckets_.resize(32);
    }
    ~DecisionBuckets() {}

    void resize(Var s) {
        var_bucket_.resize(s, -1);
        var_bucket_pos_.resize(s, -1);
        var_activity_.resize(s, 0);
    }

    void init_var(Var v) {
        if (var_inc_ > 1e9) rescale();

        var_activity_[v] = ++var_inc_;
        int bucket_index = getBucketIndex(v);
        buckets_[bucket_index].push_back(v);
        var_bucket_[v] = bucket_index;
        var_bucket_pos_[v] = buckets_[bucket_index].size() - 1;

        max_bucket_index_ = bucket_index;
    }

    void insert(Var v) {
        int bucket_index = getBucketIndex(v);

        if (var_bucket_[v] == -1) {
            buckets_[bucket_index].push_back(v);
            var_bucket_[v] = bucket_index;
            var_bucket_pos_[v] = buckets_[bucket_index].size() - 1;
        }

        if (bucket_index > max_bucket_index_) {
            max_bucket_index_ = bucket_index;
        }
    }

    bool empty() const {
        for (int i = max_bucket_index_; i >= 0; --i) {
            if (!buckets_[i].empty()) {
                return false;
            }
        }
        return true;
    }

    Var get_deci_var() {
        while (max_bucket_index_ >= 0 && buckets_[max_bucket_index_].empty()) {
            max_bucket_index_--;
        }

        Var v = buckets_[max_bucket_index_].back();
        buckets_[max_bucket_index_].pop_back();

        var_bucket_[v] = -1;

        return v;
    }

    void update(Var v) {
        if (var_inc_ > 1e9) rescale();

        var_activity_[v] = ++var_inc_;

        if (var_bucket_[v] != -1) {
            remove(v);

            int bucket_index = getBucketIndex(v);
            buckets_[bucket_index].push_back(v);
            var_bucket_[v] = bucket_index;
            var_bucket_pos_[v] = buckets_[bucket_index].size() - 1;

            if (bucket_index > max_bucket_index_) {
                max_bucket_index_ = bucket_index;
            }
        }
    }

  private:
    int getBucketIndex(Var v) const {
        uint32_t act = var_activity_[v];
        if (act == 0) return 0;
        int bucket_index = 0;
        while ((1ull << (bucket_index + 1)) <= act && bucket_index < buckets_.size()) {
            bucket_index++;
        }
        return bucket_index;
    }

    void remove(Var v) {
        int bucket_index = var_bucket_[v];
        int bucket_pos = var_bucket_pos_[v];

        auto &bucket = buckets_[bucket_index];
        std::swap(bucket[bucket_pos], bucket.back());
        var_bucket_pos_[bucket[bucket_pos]] = bucket_pos;
        bucket.pop_back();
        var_bucket_[v] = -1;
    }

    void rescale() {
        var_inc_ = 0;

        std::vector<Var> var_to_insert;
        for (auto &bucket : buckets_) {
            for (auto v : bucket) {
                var_activity_[v] = ++var_inc_;
                var_bucket_[v] = -1;
                var_to_insert.push_back(v);
            }
            bucket.clear();
        }
        for (auto v : var_to_insert) {
            insert(v);
        }
    }

    std::vector<std::vector<Var>> buckets_;
    std::vector<int> var_bucket_;
    std::vector<int> var_bucket_pos_;
    std::vector<uint32_t> var_activity_;
    int max_bucket_index_;
    uint32_t var_inc_;
};

} // namespace minicore

#endif