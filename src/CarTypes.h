#ifndef CAR_TYPES_H
#define CAR_TYPES_H

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ostream>
#include <type_traits>
#include <vector>

#include "sat/minicore/src/solver_types.h"

namespace car {

using Var = uint32_t;
constexpr Var VAR_UNDEF = 0;
using Lit = minicore::Lit;
using SignedVec = std::vector<int>;

inline constexpr Lit MkLit(Var var, bool sign = false) {
    return Lit{static_cast<int32_t>((var << 1) | static_cast<uint32_t>(sign))};
}

inline constexpr Lit LIT_FALSE{0};
inline constexpr Lit LIT_TRUE{1};

inline constexpr Var VarOf(Lit p) {
    return static_cast<Var>(static_cast<uint32_t>(p.x) >> 1);
}

inline constexpr bool Sign(Lit p) {
    return (p.x & 1) != 0;
}

inline constexpr bool IsConst(Lit p) {
    return VarOf(p) == 0;
}

inline constexpr bool IsConstFalse(Lit p) {
    return p.x == LIT_FALSE.x;
}

inline constexpr bool IsConstTrue(Lit p) {
    return p.x == LIT_TRUE.x;
}

inline int ToSigned(Lit p) {
    if (IsConst(p)) return IsConstTrue(p) ? 1 : 0;
    int id = static_cast<int>(VarOf(p));
    return Sign(p) ? -id : id;
}

inline Lit FromSigned(int lit) {
    assert(lit != 0);
    Var abs_lit = static_cast<Var>(lit > 0 ? lit : -lit);
    return MkLit(abs_lit, lit < 0);
}

inline size_t PackedIndex(Lit lit) {
    return static_cast<size_t>(static_cast<uint32_t>(lit.x));
}

inline Var AbsLit(int lit) {
    assert(lit != 0);
    return static_cast<Var>(lit > 0 ? lit : -lit);
}

inline Lit FromAigerLit(unsigned lit) {
    return Lit{static_cast<int32_t>(lit)};
}

inline unsigned ToAigerLit(Lit lit) {
    return static_cast<unsigned>(lit.x);
}

struct LitHash {
    size_t operator()(Lit lit) const noexcept {
        return static_cast<size_t>(static_cast<uint32_t>(lit.x));
    }
};

using Cube = std::vector<Lit>;
using Clause = std::vector<Lit>;

inline bool CubeComp(const Cube &c1, const Cube &c2) {
    if (c1.size() != c2.size()) return c1.size() < c2.size();
    for (size_t i = 0; i < c1.size(); ++i) {
        if (c1[i] < c2[i]) return true;
        if (c2[i] < c1[i]) return false;
    }
    return false;
}

struct CubeHash {
    size_t operator()(const Cube &cube) const noexcept {
        size_t seed = cube.size();
        for (Lit lit : cube) {
            seed ^= static_cast<size_t>(static_cast<uint32_t>(lit.x)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class LitSet {
  public:
    LitSet() = default;

    void NewSet(const Cube &cube) {
        Clear();
        for (Lit lit : cube) Insert(lit);
    }

    void Insert(Lit lit) {
        assert(lit.x != 0);
        std::size_t idx = PackedIndex(lit);
        if (idx >= m_has.size()) m_has.resize(idx + 1, 0);
        if (!m_has[idx]) {
            m_set.push_back(lit);
            m_has[idx] = 1;
        }
    }

    bool Has(Lit lit) const {
        assert(lit.x != 0);
        std::size_t idx = PackedIndex(lit);
        return idx < m_has.size() && m_has[idx];
    }

    void Clear() {
        for (Lit lit : m_set) m_has[PackedIndex(lit)] = 0;
        m_set.clear();
    }

    int Size() const { return static_cast<int>(m_set.size()); }

  private:
    std::vector<Lit> m_set;
    std::vector<uint8_t> m_has;
};

inline bool SubsumeSet(const Cube &a, const LitSet &b) {
    if (a.size() > static_cast<size_t>(b.Size())) return false;
    for (Lit lit : a) {
        if (!b.Has(lit)) return false;
    }
    return true;
}

inline bool CubeImplies(const Cube &a, const Cube &b) {
    if (a.size() > b.size()) return false;
    LitSet b_set;
    b_set.NewSet(b);
    return SubsumeSet(a, b_set);
}

inline std::ostream &operator<<(std::ostream &os, Lit lit) {
    os << ToSigned(lit);
    return os;
}

inline SignedVec ToSignedVec(const Cube &cube) {
    SignedVec out;
    out.reserve(cube.size());
    for (Lit lit : cube) out.emplace_back(ToSigned(lit));
    return out;
}

inline std::ostream &operator<<(std::ostream &os, const Cube &cb) {
    for (auto &l : cb) os << l << " ";
    return os;
}

inline Cube FromSignedVec(const SignedVec &cube) {
    Cube out;
    out.reserve(cube.size());
    for (int lit : cube) out.emplace_back(FromSigned(lit));
    return out;
}

static_assert(sizeof(Lit) == sizeof(uint32_t), "Lit must remain a 32-bit wrapper");
static_assert(alignof(Lit) == alignof(uint32_t), "Lit alignment should match uint32_t");
static_assert(std::is_trivially_copyable<Lit>::value, "Lit must stay trivially copyable");
static_assert(std::is_standard_layout<Lit>::value, "Lit must stay standard layout");

} // namespace car

#endif // CAR_TYPES_H
