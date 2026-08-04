#include "WLSimulator.h"
#include "WLBitVector.h"

#include <btorsim/btorsimbv.h>

#include <cstdlib>
#include <fstream>
#include <map>
#include <tuple>
#include <unordered_set>

namespace car {

class WLSimulator::Impl {
  public:
    struct BitValue {
        unsigned width{0};
        bool known{false};
        WLBitVector value;

        static BitValue Unknown(unsigned width) {
            return {width, false, WLBitVector::Zero(width)};
        }
        static BitValue Known(WLBitVector value) {
            const unsigned width = value.Width();
            return {width, true, std::move(value)};
        }
        static BitValue Known(unsigned width, uint64_t value) {
            return Known(WLBitVector::FromUInt64(width, value));
        }
        bool IsOne() const { return known && width == 1 && value.IsOne(); }
        bool IsZero() const { return known && value.IsZero(); }
    };

    struct ArrayValue {
        unsigned indexWidth{0};
        unsigned elementWidth{0};
        bool defaultKnown{false};
        BitValue defaultValue;
        std::unordered_map<std::string, BitValue> entries;
    };

    struct Value {
        bool isArray{false};
        BitValue bits;
        ArrayValue array;

        static Value BV(BitValue bits) {
            Value value;
            value.bits = std::move(bits);
            return value;
        }
        static Value Array(ArrayValue array) {
            Value value;
            value.isArray = true;
            value.array = std::move(array);
            return value;
        }
    };

    struct Frame {
        std::unordered_map<int64_t, BitValue> inputs;
        std::unordered_map<int64_t, BitValue> states;
        std::unordered_map<int64_t, BitValue> readMisses;
        std::unordered_map<int64_t, BitValue> arrayNextInputs;
        std::unordered_map<size_t, BitValue> selectors;
        std::unordered_map<size_t, BitValue> contents;
    };

    explicit Impl(const Btor2IR &ir) : m_ir(ir) { Index(); }

    void SetTracePairs(const std::vector<WLMemoryPair> &pairs) {
        // Pair order matches selector/content provenance emitted by bitblasting.
        m_tracePairs = pairs;
    }

    WLSimulator::Result Replay(
        const std::vector<std::pair<Cube, Cube>> &trace,
        const WLTraceMap &traceMap) {
        // Decode the bit-level trace into word-level frame values.
        WLSimulator::Result result;
        m_frames.clear();
        m_frames.resize(trace.size());
        for (size_t i = 0; i < trace.size(); ++i) {
            LoadTraceFrame(trace[i], traceMap, m_frames[i]);
        }
        if (m_frames.empty()) {
            result.incomplete = true;
            return result;
        }

        // Simulate the original BTOR2 transition system from frame zero.
        InitializeConcreteState();
        std::unordered_set<std::string> seenMismatch;
        // Compare concrete properties and reads with the abstract trace per frame.
        for (unsigned time = 0; time < m_frames.size(); ++time) {
            m_time = time;
            m_cache.clear();
            m_abstractCache.clear();

            for (int64_t constraint : m_constraints) {
                Value value = Eval(constraint);
                if (!value.bits.known) result.incomplete = true;
                if (value.bits.known && value.bits.IsZero()) {
                    result.incomplete = true;
                    return result;
                }
            }

            bool badKnown = false;
            bool badTrue = false;
            for (int64_t badId : m_bad) {
                Value value = Eval(badId);
                if (value.bits.known) {
                    badKnown = true;
                    badTrue = badTrue || value.bits.IsOne();
                }
            }

            // Read mismatches identify memory/address pairs for CEGAR refinement.
            for (int64_t readId : m_reads) {
                Value concrete = Eval(readId);
                Value abstract = EvalAbstract(readId);
                if (!concrete.bits.known || !abstract.bits.known) {
                    result.incomplete = true;
                    continue;
                }
                if (concrete.bits.value == abstract.bits.value) continue;
                const Btor2IRNode &read = m_ir.Node(readId);
                unsigned delay =
                    static_cast<unsigned>(m_frames.size() - 1 - time);
                std::string key = std::to_string(readId) + ":" +
                                  std::to_string(delay);
                if (seenMismatch.insert(key).second) {
                    result.mismatches.push_back(
                        {readId,
                         m_readMemory.at(readId),
                         read.args[1],
                         time,
                         delay});
                }
            }

            if (badTrue) {
                result.concreteCounterexample = true;
                result.badTime = time;
                return result;
            }
            if (!badKnown) result.incomplete = true;

            if (time + 1 < m_frames.size()) StepConcrete(time);
        }
        return result;
    }

    bool WriteCounterexample(const std::string &path) const {
        // Serialize replayed original inputs and states in BTOR2 witness format.
        if (m_frames.empty()) return false;
        std::ofstream out(path);
        if (!out) return false;
        out << "sat\nb0\n";
        for (size_t time = 0; time < m_frames.size(); ++time) {
            out << "#" << time << "\n";
            for (size_t i = 0; i < m_states.size(); ++i) {
                int64_t id = m_states[i];
                auto frameIt = m_stateTrace.find({id, time});
                if (frameIt == m_stateTrace.end()) continue;
                WriteWitnessValue(out, i, frameIt->second);
            }
            out << "@" << time << "\n";
            for (size_t i = 0; i < m_inputs.size(); ++i) {
                int64_t id = m_inputs[i];
                auto frameIt = m_frames[time].inputs.find(id);
                if (frameIt != m_frames[time].inputs.end() &&
                    frameIt->second.known) {
                    out << i << " " << frameIt->second.value.ToBinary()
                        << "\n";
                }
            }
        }
        out << ".\n";
        return true;
    }

  private:
    struct TimedStateKey {
        int64_t id{0};
        size_t time{0};
        bool operator==(const TimedStateKey &other) const {
            return id == other.id && time == other.time;
        }
    };

    struct TimedStateKeyHash {
        size_t operator()(const TimedStateKey &key) const {
            return std::hash<int64_t>{}(key.id) ^
                   (std::hash<size_t>{}(key.time) << 1);
        }
    };

    void Index() {
        // Pre-index transition metadata and ordered model interface nodes.
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_input:
                if (!IsArraySort(node.sortId)) {
                    m_inputPosition[node.id] = m_inputs.size();
                    m_inputs.push_back(node.id);
                }
                break;
            case BTOR2_TAG_state:
                m_statePosition[node.id] = m_states.size();
                m_states.push_back(node.id);
                break;
            case BTOR2_TAG_init: m_init[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_next: m_next[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_bad: m_bad.push_back(node.args[0]); break;
            case BTOR2_TAG_constraint:
                m_constraints.push_back(node.args[0]);
                break;
            case BTOR2_TAG_read:
                m_reads.push_back(node.id);
                m_readMemory[node.id] = FindMemory(node.args[0]);
                break;
            default: break;
            }
        }
    }

    bool IsArraySort(int64_t sortId) const {
        return sortId && m_ir.Sort(sortId).tag == BTOR2_TAG_SORT_array;
    }

    unsigned NodeWidth(int64_t signedId) const {
        const Btor2IRNode &node = m_ir.Node(signedId);
        return m_ir.Sort(node.sortId).width;
    }

    unsigned ArrayIndexWidth(int64_t memoryId) const {
        const Btor2IRSort &sort = m_ir.Sort(m_ir.Node(std::abs(memoryId)).sortId);
        return m_ir.Sort(sort.indexSort).width;
    }

    unsigned ArrayElementWidth(int64_t memoryId) const {
        const Btor2IRSort &sort = m_ir.Sort(m_ir.Node(std::abs(memoryId)).sortId);
        return m_ir.Sort(sort.elementSort).width;
    }

    BitValue &EnsureBits(std::unordered_map<int64_t, BitValue> &map,
                         int64_t id,
                         unsigned width) const {
        auto it = map.find(id);
        if (it == map.end()) {
            it = map.emplace(id, BitValue::Known(width, 0)).first;
        }
        return it->second;
    }

    BitValue &EnsurePairBits(std::unordered_map<size_t, BitValue> &map,
                             size_t id,
                             unsigned width) const {
        auto it = map.find(id);
        if (it == map.end()) {
            it = map.emplace(id, BitValue::Known(width, 0)).first;
        }
        return it->second;
    }

    void SetTraceBit(Frame &frame, const WLTraceBit &desc, bool value) const {
        // Route each AIGER bit to its original or abstraction-specific value.
        const uint32_t originalBit = desc.originalBitOffset + desc.bit;
        switch (desc.kind) {
        case WLTraceBitKind::OriginalInput: {
            BitValue &bits =
                EnsureBits(frame.inputs, desc.nodeId, NodeWidth(desc.nodeId));
            if (value) bits.value.SetBit(originalBit, true);
            break;
        }
        case WLTraceBitKind::OriginalState: {
            BitValue &bits =
                EnsureBits(frame.states, desc.nodeId, NodeWidth(desc.nodeId));
            if (value) bits.value.SetBit(originalBit, true);
            break;
        }
        case WLTraceBitKind::AbstractReadInput: {
            BitValue &bits = EnsureBits(
                frame.readMisses, desc.nodeId, NodeWidth(desc.nodeId));
            if (value) bits.value.SetBit(originalBit, true);
            break;
        }
        case WLTraceBitKind::ArrayNextInput: {
            BitValue &bits =
                EnsureBits(frame.arrayNextInputs,
                           desc.nodeId,
                           ArrayElementWidth(desc.nodeId));
            if (value) bits.value.SetBit(originalBit, true);
            break;
        }
        case WLTraceBitKind::SelectorState: {
            BitValue &bits = EnsurePairBits(
                frame.selectors, desc.pairIndex, ArrayIndexWidth(desc.nodeId));
            if (value) bits.value.SetBit(originalBit, true);
            break;
        }
        case WLTraceBitKind::ContentState: {
            BitValue &bits = EnsurePairBits(
                frame.contents, desc.pairIndex, ArrayElementWidth(desc.nodeId));
            if (value) bits.value.SetBit(originalBit, true);
            break;
        }
        default: break;
        }
    }

    void SetTraceSegment(Frame &frame,
                         const WLTraceBit &desc,
                         const WLBitVector &encoded) const {
        // The all-one package code denotes the original all-one constant;
        // every other code is injected by zero extension.
        const bool expandsToOnes = encoded.IsOnes();
        for (uint32_t bit = 0; bit < desc.originalSegmentWidth; ++bit) {
            WLTraceBit decodedBit = desc;
            decodedBit.bit = bit;
            decodedBit.resized = false;
            SetTraceBit(frame,
                        decodedBit,
                        expandsToOnes ||
                            (bit < encoded.Width() && encoded.GetBit(bit)));
        }
    }

    static std::unordered_map<Var, bool> CubeValues(const Cube &cube) {
        std::unordered_map<Var, bool> values;
        for (Lit lit : cube) {
            values[VarOf(lit)] = !Sign(lit);
        }
        return values;
    }

    void LoadTraceFrame(const std::pair<Cube, Cube> &traceFrame,
                        const WLTraceMap &traceMap,
                        Frame &frame) const {
        // Input and latch cubes use final AIGER variable IDs from WLTraceMap.
        auto inputValues = CubeValues(traceFrame.first);
        auto latchValues = CubeValues(traceFrame.second);
        using SegmentKey =
            std::tuple<int, int64_t, size_t, uint32_t, uint32_t, uint32_t>;
        struct EncodedSegment {
            WLTraceBit descriptor;
            WLBitVector value;
        };
        std::map<SegmentKey, EncodedSegment> encodedSegments;
        auto load = [&](const auto &descriptors, const auto &values) {
            for (const auto &[var, desc] : descriptors) {
                auto value = values.find(var);
                if (value == values.end()) continue;
                if (!desc.resized) {
                    SetTraceBit(frame, desc, value->second);
                    continue;
                }
                SegmentKey key{static_cast<int>(desc.kind),
                               desc.nodeId,
                               desc.pairIndex,
                               desc.originalBitOffset,
                               desc.originalSegmentWidth,
                               desc.encodedSegmentWidth};
                auto [it, inserted] = encodedSegments.emplace(
                    key,
                    EncodedSegment{
                        desc, WLBitVector::Zero(desc.encodedSegmentWidth)});
                (void)inserted;
                if (value->second)
                    it->second.value.SetBit(desc.bit, true);
            }
        };
        load(traceMap.inputBits, inputValues);
        load(traceMap.latchBits, latchValues);
        for (const auto &[key, segment] : encodedSegments) {
            (void)key;
            SetTraceSegment(frame, segment.descriptor, segment.value);
        }
    }

    void InitializeConcreteState() {
        // Evaluate scalar and array initial values before replaying frame zero.
        m_state.clear();
        m_stateTrace.clear();
        m_time = 0;
        m_cache.clear();
        for (int64_t stateId : m_states) {
            const Btor2IRNode &state = m_ir.Node(stateId);
            Value value;
            auto initIt = m_init.find(stateId);
            if (initIt != m_init.end()) {
                value = Eval(initIt->second);
                if (IsArraySort(state.sortId) &&
                    !IsArraySort(m_ir.Node(initIt->second).sortId)) {
                    ArrayValue array;
                    array.indexWidth = ArrayIndexWidth(stateId);
                    array.elementWidth = ArrayElementWidth(stateId);
                    array.defaultKnown = value.bits.known;
                    array.defaultValue = value.bits;
                    value = Value::Array(std::move(array));
                }
            } else if (IsArraySort(state.sortId)) {
                ArrayValue array;
                array.indexWidth = ArrayIndexWidth(stateId);
                array.elementWidth = ArrayElementWidth(stateId);
                array.defaultKnown = false;
                array.defaultValue = BitValue::Unknown(array.elementWidth);
                value = Value::Array(std::move(array));
            } else {
                auto it = m_frames[0].states.find(stateId);
                value = Value::BV(
                    it == m_frames[0].states.end()
                        ? BitValue::Known(NodeWidth(stateId), 0)
                        : it->second);
            }
            m_state[stateId] = value;
            m_stateTrace[{stateId, 0}] = value;
        }

        for (const Frame &frame : m_frames) {
            (void)frame;
        }
    }

    void StepConcrete(unsigned time) {
        // Evaluate each original next-state expression into the following frame.
        m_nextState.clear();
        for (int64_t stateId : m_states) {
            const Btor2IRNode &state = m_ir.Node(stateId);
            auto nextIt = m_next.find(stateId);
            Value value;
            if (nextIt != m_next.end()) {
                value = Eval(nextIt->second);
                if (IsArraySort(state.sortId) && !value.isArray) {
                    ArrayValue array;
                    array.indexWidth = ArrayIndexWidth(stateId);
                    array.elementWidth = ArrayElementWidth(stateId);
                    array.defaultKnown = value.bits.known;
                    array.defaultValue = value.bits;
                    value = Value::Array(std::move(array));
                }
            } else if (!IsArraySort(state.sortId)) {
                auto it = m_frames[time + 1].states.find(stateId);
                value = Value::BV(
                    it == m_frames[time + 1].states.end()
                        ? BitValue::Known(NodeWidth(stateId), 0)
                        : it->second);
            } else {
                value = m_state[stateId];
            }
            m_nextState[stateId] = value;
            m_stateTrace[{stateId, time + 1}] = value;
        }
        m_state.swap(m_nextState);
    }

    Value Eval(int64_t signedId) {
        // Concrete evaluation memoizes values within the current frame.
        auto cached = m_cache.find(signedId);
        if (cached != m_cache.end()) return cached->second;
        Value result = EvalUncached(signedId, false);
        m_cache.emplace(signedId, result);
        return result;
    }

    Value EvalAbstract(int64_t signedId) {
        // Abstract evaluation reads values reconstructed from the bit-level trace.
        auto cached = m_abstractCache.find(signedId);
        if (cached != m_abstractCache.end()) return cached->second;
        Value result = EvalUncached(signedId, true);
        m_abstractCache.emplace(signedId, result);
        return result;
    }

    Value EvalUncached(int64_t signedId, bool abstract) {
        // Shared evaluator selects concrete or decoded abstract state sources.
        if (signedId < 0) {
            Value value = abstract ? EvalAbstract(-signedId) : Eval(-signedId);
            if (value.isArray || !value.bits.known)
                return Value::BV(BitValue::Unknown(value.bits.width));
            return Value::BV(
                BitValue::Known(value.bits.value.Apply(btorsim_bv_not)));
        }

        const Btor2IRNode &node = m_ir.Node(signedId);
        if (node.tag == BTOR2_TAG_state) {
            if (abstract && !IsArraySort(node.sortId)) {
                auto it = m_frames[m_time].states.find(node.id);
                return Value::BV(it == m_frames[m_time].states.end()
                                     ? BitValue::Known(NodeWidth(node.id), 0)
                                     : it->second);
            }
            auto it = m_state.find(node.id);
            if (it != m_state.end()) return it->second;
        }
        if (node.tag == BTOR2_TAG_input) {
            if (IsArraySort(node.sortId)) {
                ArrayValue array;
                array.indexWidth = ArrayIndexWidth(node.id);
                array.elementWidth = ArrayElementWidth(node.id);
                array.defaultKnown = false;
                array.defaultValue = BitValue::Unknown(array.elementWidth);
                return Value::Array(std::move(array));
            }
            auto it = m_frames[m_time].inputs.find(node.id);
            return Value::BV(it == m_frames[m_time].inputs.end()
                                 ? BitValue::Known(NodeWidth(node.id), 0)
                                 : it->second);
        }
        if (abstract && node.tag == BTOR2_TAG_read) {
            return EvalAbstractRead(node);
        }

        auto arg = [&](size_t i) {
            return abstract ? EvalAbstract(node.args[i]) : Eval(node.args[i]);
        };

        switch (node.tag) {
        case BTOR2_TAG_const:
            return Value::BV(BitValue::Known(WLBitVector::FromBinary(
                NodeWidth(node.id), node.constant)));
        case BTOR2_TAG_constd:
            return Value::BV(BitValue::Known(WLBitVector::FromDecimal(
                NodeWidth(node.id), node.constant)));
        case BTOR2_TAG_consth:
            return Value::BV(BitValue::Known(WLBitVector::FromHex(
                NodeWidth(node.id), node.constant)));
        case BTOR2_TAG_zero:
            return Value::BV(BitValue::Known(NodeWidth(node.id), 0));
        case BTOR2_TAG_one:
            return Value::BV(BitValue::Known(NodeWidth(node.id), 1));
        case BTOR2_TAG_ones:
            return Value::BV(BitValue::Known(
                WLBitVector::Ones(NodeWidth(node.id))));
        case BTOR2_TAG_read: {
            Value array = arg(0);
            Value index = arg(1);
            if (!array.isArray || !index.bits.known)
                return Value::BV(BitValue::Unknown(NodeWidth(node.id)));
            std::string key = index.bits.value.ToBinary();
            auto it = array.array.entries.find(key);
            if (it != array.array.entries.end()) return Value::BV(it->second);
            return Value::BV(array.array.defaultKnown
                                 ? array.array.defaultValue
                                 : BitValue::Unknown(array.array.elementWidth));
        }
        case BTOR2_TAG_write: {
            Value array = arg(0);
            Value index = arg(1);
            Value data = arg(2);
            if (!array.isArray) return array;
            if (index.bits.known && data.bits.known) {
                array.array.entries[index.bits.value.ToBinary()] = data.bits;
            } else {
                array.array.defaultKnown = false;
            }
            return array;
        }
        case BTOR2_TAG_ite: {
            Value cond = arg(0);
            Value thenValue = arg(1);
            Value elseValue = arg(2);
            if (cond.bits.known)
                return cond.bits.IsOne() ? thenValue : elseValue;
            if (!thenValue.isArray && !elseValue.isArray &&
                thenValue.bits.known && elseValue.bits.known &&
                thenValue.bits.width == elseValue.bits.width &&
                thenValue.bits.value == elseValue.bits.value)
                return thenValue;
            if (!thenValue.isArray)
                return Value::BV(BitValue::Unknown(thenValue.bits.width));
            ArrayValue array = thenValue.array;
            array.defaultKnown = false;
            return Value::Array(std::move(array));
        }
        case BTOR2_TAG_slice: {
            Value value = arg(0);
            unsigned high = static_cast<unsigned>(node.args[1]);
            unsigned low = static_cast<unsigned>(node.args[2]);
            unsigned outWidth = high - low + 1;
            if (!value.bits.known)
                return Value::BV(BitValue::Unknown(outWidth));
            return Value::BV(
                BitValue::Known(value.bits.value.Slice(high, low)));
        }
        case BTOR2_TAG_uext: {
            Value value = arg(0);
            if (!value.bits.known)
                return Value::BV(BitValue::Unknown(NodeWidth(node.id)));
            return Value::BV(BitValue::Known(value.bits.value.ZeroExtend(
                NodeWidth(node.id) - value.bits.width)));
        }
        case BTOR2_TAG_sext: {
            Value value = arg(0);
            if (!value.bits.known)
                return Value::BV(BitValue::Unknown(NodeWidth(node.id)));
            return Value::BV(BitValue::Known(value.bits.value.SignExtend(
                NodeWidth(node.id) - value.bits.width)));
        }
        default: break;
        }

        if (node.nargs == 1) return EvalUnary(node.tag, arg(0), NodeWidth(node.id));
        if (node.nargs == 2)
            return EvalBinary(
                node.tag, arg(0), arg(1), NodeWidth(node.id));
        return Value::BV(BitValue::Unknown(NodeWidth(node.id)));
    }

    Value EvalUnary(Btor2Tag tag, const Value &a, unsigned width) const {
        if (a.isArray || !a.bits.known) return Value::BV(BitValue::Unknown(width));
        auto apply = [&](WLBitVector::UnaryOperation operation) {
            return Value::BV(BitValue::Known(a.bits.value.Apply(operation)));
        };
        switch (tag) {
        case BTOR2_TAG_not: return apply(btorsim_bv_not);
        case BTOR2_TAG_inc: return apply(btorsim_bv_inc);
        case BTOR2_TAG_dec: return apply(btorsim_bv_dec);
        case BTOR2_TAG_neg: return apply(btorsim_bv_neg);
        case BTOR2_TAG_redand: return apply(btorsim_bv_redand);
        case BTOR2_TAG_redor: return apply(btorsim_bv_redor);
        case BTOR2_TAG_redxor: return apply(btorsim_bv_redxor);
        default: return Value::BV(BitValue::Unknown(width));
        }
    }

    Value EvalBinary(Btor2Tag tag,
                     const Value &a,
                     const Value &b,
                     unsigned width) const {
        if (a.isArray || b.isArray || !a.bits.known || !b.bits.known)
            return Value::BV(BitValue::Unknown(width));
        const WLBitVector &x = a.bits.value;
        const WLBitVector &y = b.bits.value;
        auto apply = [&](WLBitVector::BinaryOperation operation) {
            return Value::BV(BitValue::Known(x.Apply(operation, y)));
        };
        auto reverse = [&](WLBitVector::BinaryOperation operation) {
            return Value::BV(BitValue::Known(y.Apply(operation, x)));
        };
        switch (tag) {
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
        default:
            return Value::BV(BitValue::Unknown(width));
        }
    }

    Value EvalAbstractRead(const Btor2IRNode &read) {
        // Reconstruct selected-slot reads from miss, selector, and content values.
        int64_t memoryId = m_readMemory.at(read.id);
        unsigned elementWidth = ArrayElementWidth(memoryId);
        auto missIt = m_frames[m_time].readMisses.find(read.id);
        BitValue result = missIt == m_frames[m_time].readMisses.end()
                              ? BitValue::Known(elementWidth, 0)
                              : missIt->second;
        if (memoryId < 0) return Value::BV(result);

        Value addressValue = EvalAbstract(read.args[1]);
        if (!addressValue.bits.known)
            return Value::BV(BitValue::Unknown(elementWidth));
        for (size_t idx = 0; idx < m_tracePairs.size(); ++idx) {
            const WLMemoryPair &pair = m_tracePairs[idx];
            if (pair.memoryStateId != memoryId) continue;
            auto selectorIt = m_frames[m_time].selectors.find(idx);
            auto contentIt = m_frames[m_time].contents.find(idx);
            if (selectorIt == m_frames[m_time].selectors.end() ||
                contentIt == m_frames[m_time].contents.end())
                continue;
            BitValue value = EvalArrayAt(read.args[0],
                                         memoryId,
                                         selectorIt->second,
                                         contentIt->second,
                                         true)
                                 .bits;
            if (selectorIt->second.known &&
                selectorIt->second.value == addressValue.bits.value) {
                result = value;
            }
        }
        return Value::BV(result);
    }

    Value EvalArrayAt(int64_t expressionId,
                      int64_t memoryId,
                      const BitValue &selector,
                      const BitValue &content,
                      bool abstract) {
        // Evaluate an array expression at one tracked selector without expansion.
        const Btor2IRNode &node = m_ir.Node(expressionId);
        switch (node.tag) {
        case BTOR2_TAG_state:
            return Value::BV(node.id == memoryId
                                 ? content
                                 : BitValue::Unknown(ArrayElementWidth(memoryId)));
        case BTOR2_TAG_write: {
            Value oldValue =
                EvalArrayAt(node.args[0], memoryId, selector, content, abstract);
            Value index = abstract ? EvalAbstract(node.args[1]) : Eval(node.args[1]);
            Value data = abstract ? EvalAbstract(node.args[2]) : Eval(node.args[2]);
            if (selector.known && index.bits.known &&
                selector.value == index.bits.value)
                return data;
            return oldValue;
        }
        case BTOR2_TAG_ite: {
            Value cond = abstract ? EvalAbstract(node.args[0]) : Eval(node.args[0]);
            if (cond.bits.known && cond.bits.IsOne())
                return EvalArrayAt(node.args[1], memoryId, selector, content, abstract);
            if (cond.bits.known && cond.bits.IsZero())
                return EvalArrayAt(node.args[2], memoryId, selector, content, abstract);
            return Value::BV(BitValue::Unknown(ArrayElementWidth(memoryId)));
        }
        default:
            return Value::BV(BitValue::Unknown(ArrayElementWidth(memoryId)));
        }
    }

    int64_t FindMemory(int64_t expressionId) const {
        // Recover the unique array state underlying read/write/ite expressions.
        const Btor2IRNode &node = m_ir.Node(expressionId);
        if (node.tag == BTOR2_TAG_state) return node.id;
        if (node.tag == BTOR2_TAG_write) return FindMemory(node.args[0]);
        if (node.tag == BTOR2_TAG_ite) {
            int64_t lhs = FindMemory(node.args[1]);
            int64_t rhs = FindMemory(node.args[2]);
            return lhs == rhs ? lhs : 0;
        }
        if (node.tag == BTOR2_TAG_input) return -node.id;
        return 0;
    }

    void WriteWitnessValue(std::ofstream &out,
                           size_t position,
                           const Value &value) const {
        if (!value.isArray) {
            if (value.bits.known)
                out << position << " " << value.bits.value.ToBinary() << "\n";
            return;
        }
        for (const auto &[index, data] : value.array.entries) {
            if (!data.known) continue;
            out << position << " [" << index << "] "
                << data.value.ToBinary() << "\n";
        }
    }

    const Btor2IR &m_ir;
    std::vector<int64_t> m_inputs;
    std::vector<int64_t> m_states;
    std::unordered_map<int64_t, size_t> m_inputPosition;
    std::unordered_map<int64_t, size_t> m_statePosition;
    std::unordered_map<int64_t, int64_t> m_init;
    std::unordered_map<int64_t, int64_t> m_next;
    std::vector<int64_t> m_bad;
    std::vector<int64_t> m_constraints;
    std::vector<int64_t> m_reads;
    std::unordered_map<int64_t, int64_t> m_readMemory;
    std::vector<WLMemoryPair> m_tracePairs;
    unsigned m_time{0};

    std::vector<Frame> m_frames;
    std::unordered_map<int64_t, Value> m_state;
    std::unordered_map<int64_t, Value> m_nextState;
    std::unordered_map<int64_t, Value> m_cache;
    std::unordered_map<int64_t, Value> m_abstractCache;
    std::unordered_map<TimedStateKey, Value, TimedStateKeyHash> m_stateTrace;
};

WLSimulator::WLSimulator(const Btor2IR &ir)
    : m_impl(std::make_unique<Impl>(ir)) {}

WLSimulator::~WLSimulator() = default;

WLSimulator::Result
WLSimulator::Replay(const std::vector<std::pair<Cube, Cube>> &trace,
                       const WLTraceMap &traceMap) {
    m_impl->SetTracePairs(traceMap.memoryPairs);
    return m_impl->Replay(trace, traceMap);
}

bool WLSimulator::WriteCounterexample(const std::string &path) const {
    return m_impl->WriteCounterexample(path);
}

} // namespace car
