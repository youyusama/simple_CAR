#include "WLBitVector.h"

#include <btorsim/btorsimbv.h>

#include <cassert>
#include <utility>

namespace car {

WLBitVector::WLBitVector(uint32_t width) : m_value(btorsim_bv_new(width)) {}

WLBitVector::WLBitVector(BtorSimBitVector *value) : m_value(value) {
    assert(m_value);
}

WLBitVector::WLBitVector(const WLBitVector &other)
    : m_value(other.m_value ? btorsim_bv_copy(other.m_value) : nullptr) {}

WLBitVector::WLBitVector(WLBitVector &&other) noexcept
    : m_value(std::exchange(other.m_value, nullptr)) {}

WLBitVector &WLBitVector::operator=(const WLBitVector &other) {
    if (this == &other) return *this;
    BtorSimBitVector *copy =
        other.m_value ? btorsim_bv_copy(other.m_value) : nullptr;
    if (m_value) btorsim_bv_free(m_value);
    m_value = copy;
    return *this;
}

WLBitVector &WLBitVector::operator=(WLBitVector &&other) noexcept {
    if (this == &other) return *this;
    if (m_value) btorsim_bv_free(m_value);
    m_value = std::exchange(other.m_value, nullptr);
    return *this;
}

WLBitVector::~WLBitVector() {
    if (m_value) btorsim_bv_free(m_value);
}

WLBitVector WLBitVector::Zero(uint32_t width) { return WLBitVector(width); }

WLBitVector WLBitVector::One(uint32_t width) {
    return WLBitVector(btorsim_bv_one(width));
}

WLBitVector WLBitVector::Ones(uint32_t width) {
    return WLBitVector(btorsim_bv_ones(width));
}

WLBitVector WLBitVector::FromUInt64(uint32_t width, uint64_t value) {
    return WLBitVector(btorsim_bv_uint64_to_bv(value, width));
}

WLBitVector WLBitVector::FromBinary(uint32_t width,
                                    const std::string &value) {
    return WLBitVector(btorsim_bv_const(value.c_str(), width));
}

WLBitVector WLBitVector::FromDecimal(uint32_t width,
                                     const std::string &value) {
    return WLBitVector(btorsim_bv_constd(value.c_str(), width));
}

WLBitVector WLBitVector::FromHex(uint32_t width, const std::string &value) {
    return WLBitVector(btorsim_bv_consth(value.c_str(), width));
}

uint32_t WLBitVector::Width() const {
    assert(m_value);
    return m_value->width;
}

bool WLBitVector::GetBit(uint32_t bit) const {
    assert(m_value);
    return btorsim_bv_get_bit(m_value, bit) != 0;
}

void WLBitVector::SetBit(uint32_t bit, bool value) {
    assert(m_value);
    btorsim_bv_set_bit(m_value, bit, value);
}

bool WLBitVector::IsZero() const {
    assert(m_value);
    return btorsim_bv_is_zero(m_value);
}

bool WLBitVector::IsOne() const {
    assert(m_value);
    return btorsim_bv_is_one(m_value);
}

bool WLBitVector::IsOnes() const {
    assert(m_value);
    return btorsim_bv_is_ones(m_value);
}

std::string WLBitVector::ToBinary() const {
    assert(m_value);
    std::string result(Width(), '0');
    for (uint32_t bit = 0; bit < Width(); ++bit) {
        if (GetBit(bit)) result[Width() - bit - 1] = '1';
    }
    return result;
}

WLBitVector WLBitVector::Apply(UnaryOperation operation) const {
    assert(m_value && operation);
    return WLBitVector(operation(m_value));
}

WLBitVector WLBitVector::Apply(BinaryOperation operation,
                               const WLBitVector &other) const {
    assert(m_value && other.m_value && operation);
    return WLBitVector(operation(m_value, other.m_value));
}

WLBitVector WLBitVector::Slice(uint32_t upper, uint32_t lower) const {
    assert(m_value);
    return WLBitVector(btorsim_bv_slice(m_value, upper, lower));
}

WLBitVector WLBitVector::ZeroExtend(uint32_t amount) const {
    assert(m_value);
    return WLBitVector(btorsim_bv_uext(m_value, amount));
}

WLBitVector WLBitVector::SignExtend(uint32_t amount) const {
    assert(m_value);
    return WLBitVector(btorsim_bv_sext(m_value, amount));
}

bool WLBitVector::operator==(const WLBitVector &other) const {
    if (!m_value || !other.m_value) return m_value == other.m_value;
    return Width() == other.Width() &&
           btorsim_bv_compare(m_value, other.m_value) == 0;
}

bool WLBitVector::operator!=(const WLBitVector &other) const {
    return !(*this == other);
}

} // namespace car
