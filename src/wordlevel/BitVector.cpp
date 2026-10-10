#include "BitVector.h"

#include <btorsim/btorsimbv.h>

#include <cassert>
#include <utility>

namespace car {

BitVector::BitVector(uint32_t width) : m_value(btorsim_bv_new(width)) {}

BitVector::BitVector(BtorSimBitVector *value) : m_value(value) {
    assert(m_value);
}

BitVector::BitVector(const BitVector &other)
    : m_value(other.m_value ? btorsim_bv_copy(other.m_value) : nullptr) {}

BitVector::BitVector(BitVector &&other) noexcept
    : m_value(std::exchange(other.m_value, nullptr)) {}

BitVector &BitVector::operator=(const BitVector &other) {
    if (this == &other) return *this;
    BtorSimBitVector *copy =
        other.m_value ? btorsim_bv_copy(other.m_value) : nullptr;
    if (m_value) btorsim_bv_free(m_value);
    m_value = copy;
    return *this;
}

BitVector &BitVector::operator=(BitVector &&other) noexcept {
    if (this == &other) return *this;
    if (m_value) btorsim_bv_free(m_value);
    m_value = std::exchange(other.m_value, nullptr);
    return *this;
}

BitVector::~BitVector() {
    if (m_value) btorsim_bv_free(m_value);
}

BitVector BitVector::Zero(uint32_t width) { return BitVector(width); }

BitVector BitVector::One(uint32_t width) {
    return BitVector(btorsim_bv_one(width));
}

BitVector BitVector::Ones(uint32_t width) {
    return BitVector(btorsim_bv_ones(width));
}

BitVector BitVector::FromUInt64(uint32_t width, uint64_t value) {
    return BitVector(btorsim_bv_uint64_to_bv(value, width));
}

BitVector BitVector::FromBinary(uint32_t width,
                                    const std::string &value) {
    return BitVector(btorsim_bv_const(value.c_str(), width));
}

BitVector BitVector::FromDecimal(uint32_t width,
                                     const std::string &value) {
    return BitVector(btorsim_bv_constd(value.c_str(), width));
}

BitVector BitVector::FromHex(uint32_t width, const std::string &value) {
    return BitVector(btorsim_bv_consth(value.c_str(), width));
}

uint32_t BitVector::Width() const {
    assert(m_value);
    return m_value->width;
}

bool BitVector::GetBit(uint32_t bit) const {
    assert(m_value);
    return btorsim_bv_get_bit(m_value, bit) != 0;
}

void BitVector::SetBit(uint32_t bit, bool value) {
    assert(m_value);
    btorsim_bv_set_bit(m_value, bit, value);
}

bool BitVector::IsZero() const {
    assert(m_value);
    return btorsim_bv_is_zero(m_value);
}

bool BitVector::IsOne() const {
    assert(m_value);
    return btorsim_bv_is_one(m_value);
}

bool BitVector::IsOnes() const {
    assert(m_value);
    return btorsim_bv_is_ones(m_value);
}

std::string BitVector::ToBinary() const {
    assert(m_value);
    std::string result(Width(), '0');
    for (uint32_t bit = 0; bit < Width(); ++bit) {
        if (GetBit(bit)) result[Width() - bit - 1] = '1';
    }
    return result;
}

BitVector BitVector::Apply(UnaryOperation operation) const {
    assert(m_value && operation);
    return BitVector(operation(m_value));
}

BitVector BitVector::Apply(BinaryOperation operation,
                               const BitVector &other) const {
    assert(m_value && other.m_value && operation);
    return BitVector(operation(m_value, other.m_value));
}

BitVector BitVector::Slice(uint32_t upper, uint32_t lower) const {
    assert(m_value);
    return BitVector(btorsim_bv_slice(m_value, upper, lower));
}

BitVector BitVector::ZeroExtend(uint32_t amount) const {
    assert(m_value);
    return BitVector(btorsim_bv_uext(m_value, amount));
}

BitVector BitVector::SignExtend(uint32_t amount) const {
    assert(m_value);
    return BitVector(btorsim_bv_sext(m_value, amount));
}

bool BitVector::operator==(const BitVector &other) const {
    if (!m_value || !other.m_value) return m_value == other.m_value;
    return Width() == other.Width() &&
           btorsim_bv_compare(m_value, other.m_value) == 0;
}

bool BitVector::operator!=(const BitVector &other) const {
    return !(*this == other);
}

} // namespace car
