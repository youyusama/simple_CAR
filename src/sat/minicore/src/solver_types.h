#ifndef MINICORE_SOLVER_TYPES_H
#define MINICORE_SOLVER_TYPES_H

#include <algorithm>
#include <assert.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdint.h>
#include <vector>

namespace minicore {


// variable
typedef int32_t Var;
const Var var_Undef = -1;

// literal
struct Lit {
    int32_t x;

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
inline std::string str(Lit p) {
    if (var(p) == 0) return (sign(p) ? "-a" : "a");
    return (sign(p) ? "-" : "") + std::to_string(var(p));
}

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
using CRef = uint32_t;
const CRef CRef_Undef = UINT32_MAX;


// clause
class Clause {
  private:
    struct Header {
        uint32_t mark : 2;
        uint32_t learnt : 1;
        uint32_t has_extra : 1;
        uint32_t reloced : 1;
        uint32_t size : 27;
    };

    Header header_;

  public:
    Lit *lits() noexcept {
        return reinterpret_cast<Lit *>(this + 1);
    }

    const Lit *lits() const noexcept {
        return reinterpret_cast<const Lit *>(this + 1);
    }

    // Disable copying
    Clause(const Clause &) = delete;
    Clause &operator=(const Clause &) = delete;

    // Constructor for new clauses
    explicit Clause(const std::vector<Lit> &ps, bool learnt) {
        header_ = {0,
                   static_cast<uint32_t>(learnt),
                   static_cast<uint32_t>(learnt),
                   0,
                   static_cast<uint32_t>(ps.size())};
        if (!ps.empty()) {
            std::copy(ps.begin(), ps.end(), lits());
        }
        if (header_.has_extra) {
            if (header_.learnt)
                activity() = 0.0f;
        }
    }

    // Constructor for cloning clauses
    explicit Clause(const Clause &other, bool use_extra) : header_(other.header_) {
        header_.has_extra = use_extra;
        if (header_.size > 0) {
            std::copy_n(other.lits(), header_.size, lits());
        }
        if (header_.has_extra && other.header_.has_extra) {
            if (header_.learnt) {
                activity() = other.activity();
            }
        }
    }

    // Accessors
    uint32_t size() const noexcept { return header_.size; }
    bool learnt() const noexcept { return header_.learnt; }
    bool has_extra() const noexcept { return header_.has_extra; }
    uint32_t get_mark() const noexcept { return header_.mark; }
    void set_mark(uint32_t m) noexcept { header_.mark = m; }
    bool reloced() const noexcept { return header_.reloced; }

    CRef relocation() const noexcept {
        return *reinterpret_cast<const CRef *>(lits());
    }

    void relocate(CRef c) noexcept {
        assert(!reloced());
        header_.reloced = 1;
        *reinterpret_cast<CRef *>(lits()) = c;
    }

    // Memory operations
    void shrink(uint32_t n) noexcept {
        assert(n <= size());
        if (has_extra()) {
            lits()[header_.size - n] = lits()[header_.size];
        }
        header_.size -= n;
    }

    void pop() noexcept { shrink(1); }

    // Element access
    const Lit &operator[](uint32_t i) const noexcept {
        assert(i < size());
        return lits()[i];
    }

    Lit &operator[](uint32_t i) noexcept {
        assert(i < size());
        return lits()[i];
    }

    operator const Lit *() const noexcept { return lits(); }
    const Lit &last() const noexcept { return (*this)[size() - 1]; }

    // Extra field access
    float &activity() noexcept {
        assert(has_extra() && learnt());
        return *reinterpret_cast<float *>(lits() + header_.size);
    }

    float activity() const noexcept {
        assert(has_extra() && learnt());
        return *reinterpret_cast<const float *>(lits() + header_.size);
    }

    std::string tostring() const noexcept {
        std::string res;
        for (uint32_t i = 0; i < size() && i < 10; i++) {
            Lit p = lits()[i];
            res += str(p) + " ";
        }
        if (learnt()) res += "learnt";
        return res;
    }
};


class ClauseAllocator {
  private:
    uint8_t *base_;         // memory base pointer
    size_t capacity_;       // memory capacity
    uint32_t allocated_{0}; // pointer to next free location in memory
    size_t wasted_{0};      //  wasted bytes for freed clauses

    static constexpr size_t INITIAL_SIZE_ = 1 << 20;
    static constexpr uint8_t GROWTH_FACTOR_ = 2;
    static constexpr size_t MINIMAL_ALIGNMENT_ = 4;

    static inline uint32_t align_offset(uint32_t offset) {
        return (offset + MINIMAL_ALIGNMENT_ - 1) & ~(MINIMAL_ALIGNMENT_ - 1);
    }

    void grow(size_t required_size) {
        size_t new_capacity = capacity_ * GROWTH_FACTOR_;
        while (new_capacity < required_size) {
            new_capacity *= GROWTH_FACTOR_;
        }

        uint8_t *new_base = nullptr;
        try {
            new_base = new uint8_t[new_capacity];
        } catch (std::bad_alloc &e) {
            std::cerr << "Fatal Error: Memory allocation failed during ClauseAllocator expansion." << std::endl;
            std::cerr << "Required size: " << new_capacity << " bytes." << std::endl;
            std::cerr << "Exception: " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::memcpy(new_base, base_, allocated_);

        delete[] base_;
        base_ = new_base;
        capacity_ = new_capacity;
    }

  public:
    explicit ClauseAllocator()
        : capacity_(INITIAL_SIZE_) {
        try {
            base_ = new uint8_t[capacity_];
        } catch (std::bad_alloc &e) {
            std::cerr << "Fatal Error: Initial memory allocation failed for ClauseAllocator." << std::endl;
            std::cerr << "Required size: " << capacity_ << " bytes." << std::endl;
            std::cerr << "Exception: " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    explicit ClauseAllocator(size_t initial_size)
        : capacity_(initial_size) {
        try {
            base_ = new uint8_t[capacity_];
        } catch (std::bad_alloc &e) {
            std::cerr << "Fatal Error: Initial memory allocation failed for ClauseAllocator." << std::endl;
            std::cerr << "Required size: " << capacity_ << " bytes." << std::endl;
            std::cerr << "Exception: " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    ~ClauseAllocator() {
        delete[] base_;
    }

    ClauseAllocator(const ClauseAllocator &) = delete;
    ClauseAllocator &operator=(const ClauseAllocator &) = delete;

    Clause &get_clause(CRef ref) const noexcept {
        assert(ref < allocated_);
        return *reinterpret_cast<Clause *>(base_ + ref);
    }

    CRef get_ref(const Clause &c) const noexcept {
        const uint8_t *clause_ptr = reinterpret_cast<const uint8_t *>(&c);

        const uint8_t *base_ptr = base_;
        assert(clause_ptr >= base_ptr && clause_ptr < base_ptr + allocated_);
        size_t offset = clause_ptr - base_ptr;
        assert(offset <= std::numeric_limits<uint32_t>::max());

        return static_cast<CRef>(offset);
    }

    size_t clause_size(size_t size, bool use_extra) const noexcept {
        return (1 + size + (use_extra ? 1 : 0)) * sizeof(Lit);
    }

    CRef alloc(const std::vector<Lit> &ps, bool learnt) {
        size_t aligned_size = clause_size(ps.size(), learnt);
        size_t new_offset = align_offset(allocated_);

        if (new_offset + aligned_size > capacity_) {
            grow(new_offset + aligned_size);
        }

        CRef ref = new_offset;
        void *mem = base_ + ref;

        new (mem) Clause(ps, learnt);

        allocated_ = new_offset + aligned_size;

        return ref;
    }

    CRef alloc(const Clause &from) {
        size_t aligned_size = clause_size(from.size(), from.has_extra());
        size_t new_offset = align_offset(allocated_);

        if (new_offset + aligned_size > capacity_) {
            grow(new_offset + aligned_size);
        }

        CRef ref = new_offset;
        void *mem = base_ + ref;

        new (mem) Clause(from, from.has_extra());

        allocated_ = new_offset + aligned_size;

        return ref;
    }

    void free(CRef ref) {
        Clause &c = get_clause(ref);
        wasted_ += clause_size(c.size(), c.has_extra());
    }

    void reloc(CRef &cr, std::shared_ptr<ClauseAllocator> to) {
        Clause &c = get_clause(cr);

        if (c.reloced()) {
            cr = c.relocation();
            return;
        }

        CRef new_cr = to->alloc(c);

        c.relocate(new_cr);
        cr = new_cr;
    }

    size_t allocated_memory() const { return allocated_; }
    size_t wasted_memory() const { return wasted_; }
    size_t total_capacity() const { return capacity_; }
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
    void update(std::shared_ptr<ClauseAllocator> ca) { ca_ = ca; }
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

    void update(std::shared_ptr<ClauseAllocator> ca) { cls_allocator = ca; }

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