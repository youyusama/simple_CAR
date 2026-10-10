#include "WLSimulator.h"
#include "BitVector.h"

#include <btorsim/btorsimbv.h>

#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <unordered_set>

namespace car {

class WLSimulator::Impl {
  public:
    struct BitValue {
        BitVector value;

        static BitValue Zero(unsigned width) {
            return {BitVector::Zero(width)};
        }
        static BitValue FromUInt64(unsigned width, uint64_t value) {
            return {BitVector::FromUInt64(width, value)};
        }
        static BitValue FromBool(bool value) {
            return {value ? BitVector::One(1) : BitVector::Zero(1)};
        }
        unsigned Width() const { return value.Width(); }
        bool IsOne() const { return value.Width() == 1 && value.IsOne(); }
        bool IsZero() const { return value.IsZero(); }
    };

    struct ArrayStorage {
        unsigned indexWidth{0};
        unsigned elementWidth{0};
        BitValue defaultValue;
        std::unordered_map<std::string, BitValue> entries;
    };

    struct Value {
        bool isArray{false};
        BitValue bits;
        ArrayStorage array;

        static Value BV(BitValue bits) {
            Value value;
            value.bits = std::move(bits);
            return value;
        }
        static Value Array(ArrayStorage array) {
            Value value;
            value.isArray = true;
            value.array = std::move(array);
            return value;
        }
    };

    explicit Impl(const Btor2IR &ir) : m_ir(ir) { Index(); }

    WLSimulator::VerificationResult Verify(const WLTrace &trace) {
        WLSimulator::VerificationResult result;
        try {
            Execute(trace, {}, WLSimulator::MissingChoices::Reject, true, nullptr);
            result.kind = VerificationKind::Confirmed;
            m_verificationNode = m_bad;
        } catch (const VerificationFailure &failure) {
            result.kind = failure.kind;
            result.reason = failure.what();
        } catch (const Btor2Unsupported &error) {
            result.kind = VerificationKind::Unsupported;
            result.reason = error.what();
        } catch (const std::exception &error) {
            result.kind = VerificationKind::InternalError;
            result.reason = error.what();
        }
        result.time = m_time;
        result.nodeId = m_verificationNode;
        return result;
    }

    WLSimulator::Execution Simulate(const WLTrace &trace, const std::vector<int64_t> &observe,
                                   WLSimulator::MissingChoices missing,
                                   const WLSimulator::SimulationOptions &options) {
        try {
            return Execute(trace, observe, missing, false, &options);
        } catch (const std::exception &error) {
            throw std::runtime_error("IR simulation at frame " + std::to_string(m_time) +
                " node " + std::to_string(m_verificationNode) + ": " + error.what());
        }
    }

  private:
    class CurrentFrame final : public WLSimulator::Frame {
      public:
        explicit CurrentFrame(Impl &execution) : m_execution(execution) {}

        BitVector Scalar(int64_t id) const override {
            auto value = m_execution.EvalConcrete(id);
            if (value.isArray)
                m_execution.Fail(VerificationKind::Invalid, id,
                                 "scalar query requires a BV expression");
            return value.bits.value;
        }

        ArrayValue Array(int64_t id) const override {
            const auto value = m_execution.EvalConcrete(id);
            if (!value.isArray)
                m_execution.Fail(VerificationKind::Invalid, id, "array query requires an array expression");
            ArrayValue result;
            result.defaultValue = value.array.defaultValue.value;
            for (const auto &[address, entry] : value.array.entries)
                result.entries.push_back({BitVector::FromBinary(value.array.indexWidth, address), entry.value});
            return result;
        }

        BitVector ReadArray(int64_t id, const BitVector &address) const override {
            auto value = m_execution.EvalConcrete(id);
            if (!value.isArray || address.Width() != value.array.indexWidth)
                m_execution.Fail(VerificationKind::Invalid, id,
                                 "array query sort/width mismatch");
            return ArrayAt(value.array, address.ToBinary());
        }

        std::optional<BitVector> FindArrayDifference(
            int64_t lhs, int64_t rhs) const override {
            auto left = m_execution.EvalConcrete(lhs);
            auto right = m_execution.EvalConcrete(rhs);
            if (!left.isArray || !right.isArray)
                m_execution.Fail(VerificationKind::Invalid, lhs,
                                 "array difference requires two arrays");
            return ArrayDifference(left.array, right.array);
        }

      private:
        Impl &m_execution;
    };

    struct VerificationFailure : std::runtime_error {
        VerificationKind kind;
        VerificationFailure(VerificationKind kind, const std::string &reason)
            : std::runtime_error(reason), kind(kind) {}
    };

    [[noreturn]] void Fail(VerificationKind kind, int64_t id,
                           const std::string &reason) {
        m_verificationNode = id;
        throw VerificationFailure(kind, reason);
    }

    static bool CoversDomain(size_t count, unsigned width) {
        return width < std::numeric_limits<size_t>::digits &&
               count == (size_t{1} << width);
    }

    static const BitVector &ArrayAt(const ArrayStorage &array,
                                     const std::string &address) {
        auto it = array.entries.find(address);
        return it == array.entries.end() ? array.defaultValue.value : it->second.value;
    }

    // Search only the explicit union and its first gap. Fixed-width binary
    // strings sort by unsigned address, including addresses wider than uint64_t.
    // Different defaults are irrelevant when the explicit union covers the
    // whole domain, so never use a default mismatch alone as a witness.
    static std::optional<BitVector> ArrayDifference(const ArrayStorage &lhs,
                                                     const ArrayStorage &rhs) {
        if (lhs.indexWidth != rhs.indexWidth || lhs.elementWidth != rhs.elementWidth)
            throw std::runtime_error("array comparison sort mismatch");
        std::set<std::string> addresses;
        for (const auto &[address, value] : lhs.entries) addresses.insert(address);
        for (const auto &[address, value] : rhs.entries) addresses.insert(address);
        const bool differentDefaults = lhs.defaultValue.value != rhs.defaultValue.value;
        std::string gap(lhs.indexWidth, '0');
        bool exhausted = false;
        for (const auto &address : addresses) {
            if (differentDefaults && address != gap)
                return BitVector::FromBinary(lhs.indexWidth, gap);
            if (ArrayAt(lhs, address) != ArrayAt(rhs, address))
                return BitVector::FromBinary(lhs.indexWidth, address);
            if (differentDefaults) {
                // Increment without narrowing the index to a machine integer.
                size_t bit = gap.size();
                while (bit && gap[bit - 1] == '1') gap[--bit] = '0';
                if (bit) gap[bit - 1] = '1';
                else exhausted = true;
            }
        }
        if (differentDefaults && !exhausted)
            return BitVector::FromBinary(lhs.indexWidth, gap);
        return std::nullopt;
    }

    Value TraceArray(int64_t id, const ArrayValue &value) {
        ArrayStorage array = NewArray(id);
        for (const auto &entry : value.entries) {
            if (entry.index.Width() != array.indexWidth ||
                entry.value.Width() != array.elementWidth)
                Fail(VerificationKind::Invalid, id, "array entry width mismatch");
            auto [it, inserted] = array.entries.emplace(
                entry.index.ToBinary(), BitValue{entry.value});
            if (!inserted && it->second.value != entry.value)
                Fail(VerificationKind::Invalid, id, "conflicting duplicate array entries");
        }
        if (!value.defaultValue && !CoversDomain(array.entries.size(), array.indexWidth) &&
            m_missing == WLSimulator::MissingChoices::Reject)
            Fail(VerificationKind::Incomplete, id, "array assignment is not total");
        if (value.defaultValue && value.defaultValue->Width() != array.elementWidth)
            Fail(VerificationKind::Invalid, id, "array default width mismatch");
        array.defaultValue = {
            value.defaultValue.value_or(BitVector::Zero(array.elementWidth))};
        return Value::Array(std::move(array));
    }

    Value TraceValue(int64_t id, bool state) {
        const auto &step = m_candidateTrace->steps.at(m_time);
        if (IsArraySort(m_ir.Node(id).sortId)) {
            const auto &values = state ? step.arrayStateValues : step.arrayInputValues;
            auto it = values.find(id);
            if (it == values.end()) {
                if (m_missing == WLSimulator::MissingChoices::Zero) return Value::Array(NewArray(id));
                Fail(VerificationKind::Incomplete, id, "missing concrete array choice");
            }
            return TraceArray(id, it->second);
        }
        const auto &values = state ? step.stateValues : step.inputValues;
        auto it = values.find(id);
        if (it == values.end()) {
            if (m_missing == WLSimulator::MissingChoices::Zero) return Value::BV(BitValue::Zero(NodeWidth(id)));
            Fail(VerificationKind::Incomplete, id, "missing concrete bit-vector choice");
        }
        if (it->second.Width() != NodeWidth(id))
            Fail(VerificationKind::Invalid, id, "bit-vector width mismatch");
        return Value::BV({it->second});
    }

    bool HasTraceState(int64_t id) const {
        const auto &step = m_candidateTrace->steps.at(m_time);
        return step.stateValues.count(id) || step.arrayStateValues.count(id);
    }

    void CheckTracePorts() {
        // Reject stale/wrong-sort interface IDs rather than silently ignoring them.
        std::unordered_set<int64_t> ports;
        const auto check = [&](const auto &values, Btor2Tag tag, bool array) {
            for (const auto &[id, value] : values) {
                (void)value;
                m_verificationNode = id;
                // Node() throws for unknown IDs; these are malformed candidates,
                // not an unsupported source operation.
                try {
                    m_ir.Node(id);
                } catch (const std::exception &) {
                    Fail(VerificationKind::Invalid, id, "unknown concrete trace interface ID");
                }
                const auto &node = m_ir.Node(id);
                if (id <= 0 || node.tag != tag || IsArraySort(node.sortId) != array ||
                    !ports.insert(id).second)
                    Fail(VerificationKind::Invalid, id, "invalid concrete trace interface");
                TraceValue(id, tag == BTOR2_TAG_state);
            }
        };
        const auto &step = m_candidateTrace->steps.at(m_time);
        check(step.inputValues, BTOR2_TAG_input, false);
        check(step.arrayInputValues, BTOR2_TAG_input, true);
        check(step.stateValues, BTOR2_TAG_state, false);
        check(step.arrayStateValues, BTOR2_TAG_state, true);
        for (int64_t id : m_allInputs) TraceValue(id, false);
    }

    Value InitialState(int64_t id) {
        auto known = m_state.find(id);
        if (known != m_state.end()) return known->second;
        auto init = m_init.find(id);
        if (init == m_init.end())
            return m_state.emplace(id, TraceValue(id, true)).first->second;
        if (!m_initializing.insert(id).second)
            Fail(VerificationKind::Incomplete, id,
                 "cyclic initialization needs an explicit concrete state choice");
        Value value = EvalConcrete(init->second);
        if (IsArraySort(m_ir.Node(id).sortId) && !value.isArray)
            value = Value::Array(NewUniformArray(id, value.bits));
        m_initializing.erase(id);
        m_state.emplace(id, value);
        return value;
    }

    static bool ArraysEqual(const ArrayStorage &lhs, const ArrayStorage &rhs) {
        if (lhs.indexWidth != rhs.indexWidth || lhs.elementWidth != rhs.elementWidth)
            throw std::runtime_error("array comparison sort mismatch");
        std::unordered_set<std::string> addresses;
        for (const auto &[address, value] : lhs.entries) addresses.insert(address);
        for (const auto &[address, value] : rhs.entries) addresses.insert(address);
        for (const auto &address : addresses)
            if (ArrayAt(lhs, address) != ArrayAt(rhs, address)) return false;
        return CoversDomain(addresses.size(), lhs.indexWidth) ||
               lhs.defaultValue.value == rhs.defaultValue.value;
    }

    static bool ValuesEqual(const Value &lhs, const Value &rhs) {
        return lhs.isArray == rhs.isArray &&
               (lhs.isArray ? ArraysEqual(lhs.array, rhs.array) : lhs.bits.value == rhs.bits.value);
    }

    void CheckOptions() {
        if (!m_options) return;
        if (m_options->overrides.size() > m_candidateTrace->steps.size())
            Fail(VerificationKind::Invalid, 0, "override frame exceeds trace length");
        for (size_t time = 0; time < m_options->overrides.size(); ++time) {
            m_time = static_cast<unsigned>(time);
            for (const auto &[id, value] : m_options->overrides[time]) {
                m_verificationNode = id;
                if (id <= 0)
                    Fail(VerificationKind::Invalid, id, "override requires a positive expression ID");
                const auto &node = m_ir.Node(id);
                if (!node.sortId || IsArraySort(node.sortId) ||
                    node.tag == BTOR2_TAG_state || node.tag == BTOR2_TAG_init ||
                    node.tag == BTOR2_TAG_next)
                    Fail(VerificationKind::Invalid, id, "override requires a non-state BV expression");
                if (value.Width() != NodeWidth(id))
                    Fail(VerificationKind::Invalid, id, "override width mismatch");
            }
        }
        m_time = 0;
        m_verificationNode = 0;
    }

    WLSimulator::Execution Execute(const WLTrace &trace, const std::vector<int64_t> &observe,
                                  WLSimulator::MissingChoices missing, bool verify,
                                  const WLSimulator::SimulationOptions *options) {
        m_candidateTrace = &trace;
        m_options = options;
        m_missing = missing;
        m_time = 0;
        m_verificationNode = 0;
        struct Cleanup {
            Impl &self;
            ~Cleanup() {
                self.m_candidateTrace = nullptr;
                self.m_options = nullptr;
                self.m_initializing.clear();
                self.ClearStepCaches();
                self.m_state.clear();
                self.m_nextState.clear();
            }
        } cleanup{*this};
        m_ir.ValidateSupportedArrays();
        WLSimulator::Execution execution;
        if (m_candidateTrace->steps.empty())
            Fail(VerificationKind::Incomplete, 0, "concrete trace is empty");
        if (m_badCount != 1)
            Fail(VerificationKind::Unsupported, 0, "concrete verification requires exactly one bad");
        CheckOptions();
        m_state.clear();
        m_initializing.clear();
        ClearStepCaches();
        CheckTracePorts();
        // Supplied initial states are candidates, not trusted assignments. Seed
        // them together so simultaneous/cyclic init equations can be checked.
        for (int64_t id : m_states)
            if (HasTraceState(id)) m_state.emplace(id, TraceValue(id, true));
        for (int64_t id : m_states) InitialState(id);
        for (const auto &[id, expression] : m_init) {
            Value expected = EvalConcrete(expression);
            if (IsArraySort(m_ir.Node(id).sortId) && !expected.isArray)
                expected = Value::Array(NewUniformArray(id, expected.bits));
            if (!ValuesEqual(m_state.at(id), expected))
                Fail(VerificationKind::Invalid, id, "initial state violates init");
        }
        for (;;) {
            ClearStepCaches();
            for (int64_t id : m_states)
                if (HasTraceState(id) && !ValuesEqual(m_state.at(id), TraceValue(id, true)))
                    Fail(VerificationKind::Invalid, id, "reported state disagrees with computed transition");
            bool constraintsHold = true;
            for (int64_t id : m_constraints) {
                const bool holds = EvalConcrete(id).bits.IsOne();
                if (verify && !holds) Fail(VerificationKind::Invalid, id, "constraint is false");
                constraintsHold &= holds;
            }
            const bool bad = EvalConcrete(m_bad).bits.IsOne();
            if (!verify) {
                execution.constraintsHold.push_back(constraintsHold);
                execution.bad.push_back(bad);
                auto &step = execution.observations.emplace_back();
                for (int64_t id : observe) {
                    auto value = EvalConcrete(id);
                    if (value.isArray) Fail(VerificationKind::Invalid, id, "observation requires a BV expression");
                    step.emplace(id, std::move(value.bits.value));
                }
                if (m_options && m_options->onFrame) {
                    CurrentFrame frame(*this);
                    m_options->onFrame(m_time, frame);
                }
            }
            if (m_time + 1 == m_candidateTrace->steps.size()) {
                if (verify && !bad)
                    Fail(VerificationKind::Invalid, m_bad, "bad is false in the final frame");
                break;
            }
            // Evaluate every RHS in the old state before changing any latch.
            m_nextState.clear();
            for (int64_t id : m_states) {
                auto next = m_next.find(id);
                if (next != m_next.end()) m_nextState.emplace(id, EvalConcrete(next->second));
            }
            ++m_time;
            CheckTracePorts();
            for (int64_t id : m_states)
                if (!m_next.count(id)) m_nextState.emplace(id, TraceValue(id, true));
            m_state.swap(m_nextState);
        }
        return execution;
    }

    void Index() {
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_input:
                m_allInputs.push_back(node.id);
                break;
            case BTOR2_TAG_state: m_states.push_back(node.id); break;
            case BTOR2_TAG_init: m_init[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_next: m_next[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_bad: m_bad = node.args[0]; ++m_badCount; break;
            case BTOR2_TAG_constraint:
                m_constraints.push_back(node.args[0]);
                break;
            default: break;
            }
        }
    }

    bool IsArraySort(int64_t sortId) const {
        return sortId && m_ir.Sort(sortId).tag == BTOR2_TAG_SORT_array;
    }

    unsigned NodeWidth(int64_t id) const {
        return m_ir.Sort(m_ir.Node(id).sortId).width;
    }

    unsigned ArrayIndexWidth(int64_t memoryId) const {
        const Btor2IRSort &sort = m_ir.Sort(m_ir.Node(memoryId).sortId);
        return m_ir.Sort(sort.indexSort).width;
    }

    unsigned ArrayElementWidth(int64_t memoryId) const {
        const Btor2IRSort &sort = m_ir.Sort(m_ir.Node(memoryId).sortId);
        return m_ir.Sort(sort.elementSort).width;
    }

    ArrayStorage NewArray(int64_t stateId) const {
        ArrayStorage array;
        array.indexWidth = ArrayIndexWidth(stateId);
        array.elementWidth = ArrayElementWidth(stateId);
        array.defaultValue = BitValue::Zero(array.elementWidth);
        return array;
    }

    ArrayStorage NewUniformArray(int64_t stateId, BitValue initial) const {
        ArrayStorage array = NewArray(stateId);
        array.defaultValue = std::move(initial);
        return array;
    }

    void ClearStepCaches() { m_cache.clear(); }

    Value EvalConcrete(int64_t id) {
        auto found = m_cache.find(id);
        if (found != m_cache.end()) return found->second;
        Value result = EvalUncached(id);
        m_cache.emplace(id, result);
        return result;
    }

    Value EvalUncached(int64_t signedId) {
        m_verificationNode = signedId < 0 ? -signedId : signedId;
        if (signedId < 0) {
            Value value = EvalConcrete(-signedId);
            if (value.isArray)
                throw std::runtime_error("array value cannot be inverted");
            return Value::BV({value.bits.value.Apply(btorsim_bv_not)});
        }
        const auto &node = m_ir.Node(signedId);
        if (m_options && m_time < m_options->overrides.size()) {
            const auto &overrides = m_options->overrides[m_time];
            auto replacement = overrides.find(signedId);
            if (replacement != overrides.end()) return Value::BV({replacement->second});
        }
        if (node.tag == BTOR2_TAG_state) {
            if (m_time == 0) return InitialState(node.id);
            auto found = m_state.find(node.id);
            if (found != m_state.end()) return found->second;
            throw EvaluationError(node, "state has no simulated value");
        }
        if (node.tag == BTOR2_TAG_input) return TraceValue(node.id, false);
        return EvalOperation(node, [&](size_t index) {
            return EvalConcrete(node.args[index]);
        });
    }

    template<class Operand>
    Value EvalOperation(const Btor2IRNode &node, const Operand &arg) {
        switch (node.tag) {
        case BTOR2_TAG_const:
            return Value::BV({BitVector::FromBinary(
                NodeWidth(node.id), node.constant)});
        case BTOR2_TAG_constd:
            return Value::BV({BitVector::FromDecimal(
                NodeWidth(node.id), node.constant)});
        case BTOR2_TAG_consth:
            return Value::BV({BitVector::FromHex(
                NodeWidth(node.id), node.constant)});
        case BTOR2_TAG_zero:
            return Value::BV(BitValue::Zero(NodeWidth(node.id)));
        case BTOR2_TAG_one:
            return Value::BV(BitValue::FromUInt64(NodeWidth(node.id), 1));
        case BTOR2_TAG_ones:
            return Value::BV({BitVector::Ones(NodeWidth(node.id))});
        case BTOR2_TAG_read: return EvalConcreteRead(node);
        case BTOR2_TAG_write: {
            Value array = arg(0);
            Value index = arg(1);
            Value data = arg(2);
            if (!array.isArray || index.isArray || data.isArray)
                throw EvaluationError(node, "malformed array write");
            array.array.entries[index.bits.value.ToBinary()] = data.bits;
            return array;
        }
        case BTOR2_TAG_ite: {
            Value condition = arg(0);
            if (condition.isArray)
                throw EvaluationError(node, "array-valued ite condition");
            return condition.bits.IsOne() ? arg(1) : arg(2);
        }
        case BTOR2_TAG_slice: {
            Value value = arg(0);
            if (value.isArray)
                throw EvaluationError(node, "slice of array value");
            return Value::BV({value.bits.value.Slice(
                static_cast<unsigned>(node.args[1]),
                static_cast<unsigned>(node.args[2]))});
        }
        case BTOR2_TAG_uext: {
            Value value = arg(0);
            return Value::BV({value.bits.value.ZeroExtend(
                NodeWidth(node.id) - value.bits.Width())});
        }
        case BTOR2_TAG_sext: {
            Value value = arg(0);
            return Value::BV({value.bits.value.SignExtend(
                NodeWidth(node.id) - value.bits.Width())});
        }
        default: break;
        }

        if (node.nargs == 1)
            return EvalUnary(node, arg(0));
        if (node.nargs == 2)
            return EvalBinary(node, arg(0), arg(1));
        throw EvaluationError(node, "unsupported simulator operation");
    }

    Value EvalConcreteRead(const Btor2IRNode &read) {
        Value array = EvalConcrete(read.args[0]);
        Value index = EvalConcrete(read.args[1]);
        if (!array.isArray || index.isArray)
            throw EvaluationError(read, "malformed array read");
        const std::string key = index.bits.value.ToBinary();
        auto written = array.array.entries.find(key);
        if (written != array.array.entries.end())
            return Value::BV(written->second);
        return Value::BV(array.array.defaultValue);
    }

    Value EvalUnary(const Btor2IRNode &node, const Value &operand) const {
        if (operand.isArray)
            throw EvaluationError(node, "scalar operation consumes array");
        auto apply = [&](BitVector::UnaryOperation operation) {
            return Value::BV({operand.bits.value.Apply(operation)});
        };
        switch (node.tag) {
        case BTOR2_TAG_not: return apply(btorsim_bv_not);
        case BTOR2_TAG_inc: return apply(btorsim_bv_inc);
        case BTOR2_TAG_dec: return apply(btorsim_bv_dec);
        case BTOR2_TAG_neg: return apply(btorsim_bv_neg);
        case BTOR2_TAG_redand: return apply(btorsim_bv_redand);
        case BTOR2_TAG_redor: return apply(btorsim_bv_redor);
        case BTOR2_TAG_redxor: return apply(btorsim_bv_redxor);
        default: throw EvaluationError(node, "unsupported unary operation");
        }
    }

    Value EvalBinary(const Btor2IRNode &node,
                     const Value &lhs,
                     const Value &rhs) const {
        if (lhs.isArray && rhs.isArray &&
            (node.tag == BTOR2_TAG_eq || node.tag == BTOR2_TAG_neq)) {
            bool equal = ArraysEqual(lhs.array, rhs.array);
            return Value::BV(BitValue::FromBool(node.tag == BTOR2_TAG_eq ? equal : !equal));
        }
        if (lhs.isArray || rhs.isArray)
            throw EvaluationError(node, "scalar operation consumes array");
        const BitVector &x = lhs.bits.value;
        const BitVector &y = rhs.bits.value;
        auto apply = [&](BitVector::BinaryOperation operation) {
            return Value::BV({x.Apply(operation, y)});
        };
        auto reverse = [&](BitVector::BinaryOperation operation) {
            return Value::BV({y.Apply(operation, x)});
        };
        switch (node.tag) {
        case BTOR2_TAG_add: return apply(btorsim_bv_add);
        case BTOR2_TAG_sub: return apply(btorsim_bv_sub);
        case BTOR2_TAG_mul: return apply(btorsim_bv_mul);
        case BTOR2_TAG_and: return apply(btorsim_bv_and);
        case BTOR2_TAG_or: return apply(btorsim_bv_or);
        case BTOR2_TAG_xor: return apply(btorsim_bv_xor);
        case BTOR2_TAG_nand: return apply(btorsim_bv_nand);
        case BTOR2_TAG_nor: return apply(btorsim_bv_nor);
        case BTOR2_TAG_xnor: return apply(btorsim_bv_xnor);
        case BTOR2_TAG_eq:
        case BTOR2_TAG_iff: return apply(btorsim_bv_eq);
        case BTOR2_TAG_neq: return apply(btorsim_bv_neq);
        case BTOR2_TAG_implies: return apply(btorsim_bv_implies);
        case BTOR2_TAG_ult: return apply(btorsim_bv_ult);
        case BTOR2_TAG_ulte: return apply(btorsim_bv_ulte);
        case BTOR2_TAG_ugt: return reverse(btorsim_bv_ult);
        case BTOR2_TAG_ugte: return reverse(btorsim_bv_ulte);
        case BTOR2_TAG_slt: return apply(btorsim_bv_slt);
        case BTOR2_TAG_slte: return apply(btorsim_bv_slte);
        case BTOR2_TAG_sgt: return reverse(btorsim_bv_slt);
        case BTOR2_TAG_sgte: return reverse(btorsim_bv_slte);
        case BTOR2_TAG_udiv: return apply(btorsim_bv_udiv);
        case BTOR2_TAG_urem: return apply(btorsim_bv_urem);
        case BTOR2_TAG_sdiv: return apply(btorsim_bv_sdiv);
        case BTOR2_TAG_srem: return apply(btorsim_bv_srem);
        case BTOR2_TAG_smod: return apply(btorsim_bv_smod);
        case BTOR2_TAG_sll: return apply(btorsim_bv_sll);
        case BTOR2_TAG_srl: return apply(btorsim_bv_srl);
        case BTOR2_TAG_sra: return apply(btorsim_bv_sra);
        case BTOR2_TAG_concat: return apply(btorsim_bv_concat);
        case BTOR2_TAG_rol: return apply(btorsim_bv_rol);
        case BTOR2_TAG_ror: return apply(btorsim_bv_ror);
        case BTOR2_TAG_uaddo: {
            BitVector sum = x.Apply(btorsim_bv_add, y);
            return Value::BV(BitValue::FromBool(
                sum.Apply(btorsim_bv_ult, x).IsOne()));
        }
        case BTOR2_TAG_usubo:
            return Value::BV(BitValue::FromBool(
                x.Apply(btorsim_bv_ult, y).IsOne()));
        case BTOR2_TAG_umulo:
            return UnsignedMulOverflow(x, y);
        case BTOR2_TAG_saddo: return SignedAddOverflow(x, y);
        case BTOR2_TAG_ssubo: return SignedSubOverflow(x, y);
        case BTOR2_TAG_smulo: return SignedMulOverflow(x, y);
        case BTOR2_TAG_sdivo: return SignedDivOverflow(x, y);
        default: throw EvaluationError(node, "unsupported binary operation");
        }
    }

    static Value SignedAddOverflow(const BitVector &x,
                                   const BitVector &y) {
        BitVector sum = x.Apply(btorsim_bv_add, y);
        const bool sx = x.GetBit(x.Width() - 1);
        const bool sy = y.GetBit(y.Width() - 1);
        const bool sr = sum.GetBit(sum.Width() - 1);
        return Value::BV(BitValue::FromBool(sx == sy && sr != sx));
    }

    static Value SignedSubOverflow(const BitVector &x,
                                   const BitVector &y) {
        BitVector difference = x.Apply(btorsim_bv_sub, y);
        const bool sx = x.GetBit(x.Width() - 1);
        const bool sy = y.GetBit(y.Width() - 1);
        const bool sr = difference.GetBit(difference.Width() - 1);
        return Value::BV(BitValue::FromBool(sx != sy && sr != sx));
    }

    static Value SignedMulOverflow(const BitVector &x,
                                   const BitVector &y) {
        const unsigned width = x.Width();
        BitVector product = x.SignExtend(width).Apply(
            btorsim_bv_mul, y.SignExtend(width));
        BitVector low = product.Slice(width - 1, 0);
        return Value::BV(BitValue::FromBool(
            product != low.SignExtend(width)));
    }

    static Value UnsignedMulOverflow(const BitVector &x,
                                     const BitVector &y) {
        const unsigned width = x.Width();
        BitVector product = x.ZeroExtend(width).Apply(
            btorsim_bv_mul, y.ZeroExtend(width));
        return Value::BV(BitValue::FromBool(
            !product.Slice(2 * width - 1, width).IsZero()));
    }

    static Value SignedDivOverflow(const BitVector &x,
                                   const BitVector &y) {
        BitVector minimum = BitVector::Zero(x.Width());
        minimum.SetBit(x.Width() - 1, true);
        return Value::BV(BitValue::FromBool(
            x == minimum && y.IsOnes()));
    }

    static std::runtime_error EvaluationError(const Btor2IRNode &node,
                                              const std::string &message) {
        return std::runtime_error(
            "BTOR2 simulator error at line " + std::to_string(node.line) +
            " (id " + std::to_string(node.id) + "): " + message);
    }

    const Btor2IR &m_ir;
    const WLTrace *m_candidateTrace{nullptr};
    const WLSimulator::SimulationOptions *m_options{nullptr};
    WLSimulator::MissingChoices m_missing{WLSimulator::MissingChoices::Reject};
    int64_t m_verificationNode{0};
    std::unordered_set<int64_t> m_initializing;
    std::vector<int64_t> m_allInputs;
    std::vector<int64_t> m_states;
    std::unordered_map<int64_t, int64_t> m_init;
    std::unordered_map<int64_t, int64_t> m_next;
    int64_t m_bad{0};
    unsigned m_badCount{0};
    std::vector<int64_t> m_constraints;
    unsigned m_time{0};
    std::unordered_map<int64_t, Value> m_state;
    std::unordered_map<int64_t, Value> m_nextState;
    std::unordered_map<int64_t, Value> m_cache;
};

WLSimulator::WLSimulator(const Btor2IR &ir)
    : m_impl(std::make_unique<Impl>(ir)) {}

WLSimulator::~WLSimulator() = default;

WLSimulator::VerificationResult
WLSimulator::Verify(const WLTrace &trace) {
    return m_impl->Verify(trace);
}

WLSimulator::Execution WLSimulator::Simulate(const WLTrace &choices,
    const std::vector<int64_t> &observe, MissingChoices missing,
    const SimulationOptions &options) {
    return m_impl->Simulate(choices, observe, missing, options);
}

// COI drops only irrelevant choices. Restore those explicitly before replaying
// the full source model; never silently complete a missing retained SAT port.
void WLSimulator::CompleteCoiChoices(const Btor2IR &source, const Btor2IR &property,
                        WLTrace &trace) {
    std::unordered_set<int64_t> retained, initialized, updated;
    for (const auto &node : property.Nodes()) retained.insert(node.id);
    for (const auto &node : source.Nodes()) {
        if (node.tag == BTOR2_TAG_init) initialized.insert(node.args[0]);
        if (node.tag == BTOR2_TAG_next) updated.insert(node.args[0]);
    }
    for (size_t time = 0; time < trace.steps.size(); ++time) {
        auto &step = trace.steps[time];
        for (const auto &node : source.Nodes()) {
            if (retained.count(node.id)) continue;
            const bool state = node.tag == BTOR2_TAG_state;
            if (!state && node.tag != BTOR2_TAG_input) continue;
            if (state && (time == 0 ? initialized.count(node.id) : updated.count(node.id)))
                continue;
            const auto &sort = source.Sort(node.sortId);
            if (sort.tag == BTOR2_TAG_SORT_array) {
                auto &values = state ? step.arrayStateValues : step.arrayInputValues;
                ArrayValue value;
                value.defaultValue = BitVector::Zero(source.Sort(sort.elementSort).width);
                values.emplace(node.id, std::move(value));
            } else {
                auto &values = state ? step.stateValues : step.inputValues;
                values.emplace(node.id, BitVector::Zero(sort.width));
            }
        }
    }
}

} // namespace car
