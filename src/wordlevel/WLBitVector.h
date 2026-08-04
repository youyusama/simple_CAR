#pragma once

#include <cstdint>
#include <string>

struct BtorSimBitVector;

namespace car {

// Value-type C++ wrapper around btor2tools' fixed-width bit-vector operations.
class WLBitVector {
  public:
    using UnaryOperation =
        BtorSimBitVector *(*)(const BtorSimBitVector *);
    using BinaryOperation =
        BtorSimBitVector *(*)(const BtorSimBitVector *,
                              const BtorSimBitVector *);

    WLBitVector() = default;
    explicit WLBitVector(uint32_t width);
    WLBitVector(const WLBitVector &other);
    WLBitVector(WLBitVector &&other) noexcept;
    WLBitVector &operator=(const WLBitVector &other);
    WLBitVector &operator=(WLBitVector &&other) noexcept;
    ~WLBitVector();

    static WLBitVector Zero(uint32_t width);
    static WLBitVector One(uint32_t width);
    static WLBitVector Ones(uint32_t width);
    static WLBitVector FromUInt64(uint32_t width, uint64_t value);
    static WLBitVector FromBinary(uint32_t width, const std::string &value);
    static WLBitVector FromDecimal(uint32_t width, const std::string &value);
    static WLBitVector FromHex(uint32_t width, const std::string &value);

    uint32_t Width() const;
    bool GetBit(uint32_t bit) const;
    void SetBit(uint32_t bit, bool value);
    bool IsZero() const;
    bool IsOne() const;
    bool IsOnes() const;
    std::string ToBinary() const;

    WLBitVector Apply(UnaryOperation operation) const;
    WLBitVector Apply(BinaryOperation operation,
                      const WLBitVector &other) const;
    WLBitVector Slice(uint32_t upper, uint32_t lower) const;
    WLBitVector ZeroExtend(uint32_t amount) const;
    WLBitVector SignExtend(uint32_t amount) const;

    bool operator==(const WLBitVector &other) const;
    bool operator!=(const WLBitVector &other) const;

  private:
    explicit WLBitVector(BtorSimBitVector *value);

    BtorSimBitVector *m_value{nullptr};
};

} // namespace car
