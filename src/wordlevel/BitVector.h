#pragma once

#include <cstdint>
#include <string>

struct BtorSimBitVector;

namespace car {

// Value-type C++ wrapper around btor2tools' fixed-width bit-vector operations.
class BitVector {
  public:
    using UnaryOperation =
        BtorSimBitVector *(*)(const BtorSimBitVector *);
    using BinaryOperation =
        BtorSimBitVector *(*)(const BtorSimBitVector *,
                              const BtorSimBitVector *);

    BitVector() = default;
    explicit BitVector(uint32_t width);
    BitVector(const BitVector &other);
    BitVector(BitVector &&other) noexcept;
    BitVector &operator=(const BitVector &other);
    BitVector &operator=(BitVector &&other) noexcept;
    ~BitVector();

    static BitVector Zero(uint32_t width);
    static BitVector One(uint32_t width);
    static BitVector Ones(uint32_t width);
    static BitVector FromUInt64(uint32_t width, uint64_t value);
    static BitVector FromBinary(uint32_t width, const std::string &value);
    static BitVector FromDecimal(uint32_t width, const std::string &value);
    static BitVector FromHex(uint32_t width, const std::string &value);

    uint32_t Width() const;
    bool GetBit(uint32_t bit) const;
    void SetBit(uint32_t bit, bool value);
    bool IsZero() const;
    bool IsOne() const;
    bool IsOnes() const;
    std::string ToBinary() const;

    BitVector Apply(UnaryOperation operation) const;
    BitVector Apply(BinaryOperation operation,
                      const BitVector &other) const;
    BitVector Slice(uint32_t upper, uint32_t lower) const;
    BitVector ZeroExtend(uint32_t amount) const;
    BitVector SignExtend(uint32_t amount) const;

    bool operator==(const BitVector &other) const;
    bool operator!=(const BitVector &other) const;

  private:
    explicit BitVector(BtorSimBitVector *value);

    BtorSimBitVector *m_value{nullptr};
};

} // namespace car
