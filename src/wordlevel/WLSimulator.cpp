#include "WLSimulator.h"
#include "WLBitVector.h"

#include <btorsim/btorsimbv.h>

#include <memory>
#include <stdexcept>
#include <unordered_set>

namespace car {

class WLSimulator::Impl {
  public:
    struct BitValue {
        WLBitVector value;

        static BitValue Zero(unsigned width) {
            return {WLBitVector::Zero(width)};
        }
        static BitValue FromUInt64(unsigned width, uint64_t value) {
            return {WLBitVector::FromUInt64(width, value)};
        }
        static BitValue FromBool(bool value) {
            return {value ? WLBitVector::One(1) : WLBitVector::Zero(1)};
        }
        unsigned Width() const { return value.Width(); }
        bool IsOne() const { return value.Width() == 1 && value.IsOne(); }
        bool IsZero() const { return value.IsZero(); }
    };

    struct InitialMemory {
        bool uniform{false};
        BitValue uniformValue;
        std::unordered_map<std::string, BitValue> entries;
    };

    struct ArrayValue {
        unsigned indexWidth{0};
        unsigned elementWidth{0};
        std::shared_ptr<InitialMemory> initial;
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

    struct TimedKey {
        int64_t id{0};
        unsigned time{0};

        bool operator==(const TimedKey &other) const {
            return id == other.id && time == other.time;
        }
    };

    struct TimedKeyHash {
        size_t operator()(const TimedKey &key) const {
            return std::hash<int64_t>{}(key.id) ^
                   (std::hash<unsigned>{}(key.time) << 1);
        }
    };

    explicit Impl(const Btor2IR &ir) : m_ir(ir) { Index(); }

    WLSimulator::Result Replay(const WLReplayTrace &trace) {
        if (trace.steps.empty())
            throw std::runtime_error("word-level replay trace is empty");

        m_steps = trace.steps;
        m_tracePairs = trace.memoryPairs;
        m_abstractReads.clear();
        m_concreteReads.clear();
        m_representedReads.clear();

        // Recompute the complete abstract execution from the initial state and
        // checker inputs. Intermediate checker latch cubes are not a replay
        // contract and may remain generalized.
        const std::unordered_set<TimedKey, TimedKeyHash> noCorrections;
        m_recordAbstractReplay = true;
        const bool abstractCounterexample =
            HybridCounterexampleSurvives(noCorrections);
        m_recordAbstractReplay = false;
        if (!abstractCounterexample)
            throw std::runtime_error(
                "recomputed checker trace does not reproduce the abstract bad state: " +
                m_hybridFailure);

        InitializeConcreteState();

        std::vector<WLReadMismatch> mismatches;
        bool constraintsHold = true;
        const unsigned failureTime =
            static_cast<unsigned>(m_steps.size() - 1);

        for (m_time = 0; m_time < m_steps.size(); ++m_time) {
            ClearStepCaches();

            for (int64_t readId : m_reads) {
                BitValue concreteValue = EvalConcrete(readId).bits;
                m_concreteReads[{readId, m_time}] = concreteValue;
                const BitValue &abstractValue =
                    m_abstractReads.at({readId, m_time});
                if (concreteValue.value == abstractValue.value) continue;
                const Btor2IRNode &read = m_ir.Node(readId);
                // A represented read can differ only because an upstream
                // unrepresented read already changed its address or data cone.
                // Refining it again would reproduce an existing pair.
                if (m_representedReads.count({readId, m_time})) continue;
                mismatches.push_back(
                    {readId,
                     m_readMemory.at(readId),
                     read.args[1],
                     m_time,
                     failureTime - m_time});
            }

            for (int64_t constraint : m_constraints) {
                if (EvalConcrete(constraint).bits.IsZero())
                    constraintsHold = false;
            }

            if (constraintsHold && EvalConcrete(m_bad).bits.IsOne()) {
                WLSimulator::Result result;
                result.kind = ReplayKind::ConcreteCounterexample;
                result.badTime = m_time;
                result.witnessTrace = BuildWitnessTrace(m_time);
                return result;
            }

            if (m_time + 1 < m_steps.size()) StepConcrete();
        }

        if (mismatches.empty())
            throw std::runtime_error(
                "spurious word-level counterexample contains no erroneous read");

        // Start with every erroneous read corrected, then greedily remove
        // corrections while the abstract counterexample remains eliminated.
        std::unordered_set<TimedKey, TimedKeyHash> forced;
        for (const WLReadMismatch &mismatch : mismatches)
            forced.insert({mismatch.readNodeId, mismatch.time});
        if (HybridCounterexampleSurvives(forced))
            throw std::runtime_error(
                "correcting every erroneous read did not eliminate the abstract trace");

        for (const WLReadMismatch &mismatch : mismatches) {
            TimedKey key{mismatch.readNodeId, mismatch.time};
            forced.erase(key);
            if (HybridCounterexampleSurvives(forced)) forced.insert(key);
        }

        WLSimulator::Result result;
        result.kind = ReplayKind::SpuriousCounterexample;
        result.badTime = failureTime;
        for (const WLReadMismatch &mismatch : mismatches) {
            if (forced.count({mismatch.readNodeId, mismatch.time}))
                result.refinementReads.push_back(mismatch);
        }
        if (result.refinementReads.empty())
            throw std::runtime_error(
                "greedy read refinement produced an empty correction set");
        return result;
    }

  private:
    enum class EvalMode { Concrete, Abstract, Hybrid };

    void Index() {
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_input:
                if (!IsArraySort(node.sortId)) m_inputs.push_back(node.id);
                break;
            case BTOR2_TAG_state: m_states.push_back(node.id); break;
            case BTOR2_TAG_init: m_init[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_next: m_next[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_bad: m_bad = node.args[0]; break;
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

    static BitValue LookupBits(
        const std::unordered_map<int64_t, WLBitVector> &values,
        int64_t id,
        unsigned width) {
        auto it = values.find(id);
        return it == values.end() ? BitValue::Zero(width)
                                  : BitValue{it->second};
    }

    static BitValue LookupPairBits(
        const std::unordered_map<size_t, WLBitVector> &values,
        size_t id,
        unsigned width) {
        auto it = values.find(id);
        return it == values.end() ? BitValue::Zero(width)
                                  : BitValue{it->second};
    }

    WLWitnessTrace BuildWitnessTrace(unsigned lastTime) const {
        WLWitnessTrace trace;
        trace.steps.resize(static_cast<size_t>(lastTime) + 1);
        for (unsigned time = 0; time <= lastTime; ++time) {
            WLWitnessStep &step = trace.steps[time];
            step.inputValues = m_steps[time].inputValues;
            for (int64_t stateId : m_states) {
                auto state = m_stateTrace.find({stateId, time});
                if (state == m_stateTrace.end()) continue;
                const Value &value = state->second;
                if (!value.isArray) {
                    step.stateValues.emplace(stateId, value.bits.value);
                    continue;
                }
                WLWitnessArrayValue array;
                if (value.array.initial)
                    for (const auto &[index, data] :
                         value.array.initial->entries)
                        array.entries[index] = data.value;
                for (const auto &[index, data] : value.array.entries)
                    array.entries[index] = data.value;
                if (!array.entries.empty())
                    step.arrayStateValues.emplace(stateId,
                                                  std::move(array));
            }
        }
        return trace;
    }

    ArrayValue NewArray(int64_t stateId) const {
        ArrayValue array;
        array.indexWidth = ArrayIndexWidth(stateId);
        array.elementWidth = ArrayElementWidth(stateId);
        array.initial = std::make_shared<InitialMemory>();
        return array;
    }

    ArrayValue NewUniformArray(int64_t stateId, BitValue initial) const {
        ArrayValue array = NewArray(stateId);
        array.initial->uniform = true;
        array.initial->uniformValue = std::move(initial);
        return array;
    }

    void InitializeConcreteState() {
        m_state.clear();
        m_stateTrace.clear();
        m_time = 0;

        // Decoded latch values provide concrete choices for scalar states;
        // array states first receive their sparse initial-memory object.
        for (int64_t stateId : m_states) {
            if (IsArraySort(m_ir.Node(stateId).sortId))
                m_state[stateId] = Value::Array(NewArray(stateId));
            else
                m_state[stateId] = Value::BV(LookupBits(
                    m_steps[0].stateValues, stateId, NodeWidth(stateId)));
        }

        ClearStepCaches();
        for (int64_t stateId : m_states) {
            auto init = m_init.find(stateId);
            if (init == m_init.end()) continue;
            const bool arrayState = IsArraySort(m_ir.Node(stateId).sortId);
            if (arrayState && IsArraySort(m_ir.Node(init->second).sortId))
                throw std::runtime_error(
                    "non-uniform array initialization is unsupported");
            Value initial = EvalConcrete(init->second);
            m_state[stateId] = arrayState
                                   ? Value::Array(NewUniformArray(
                                         stateId, initial.bits))
                                   : initial;
            m_cache.clear();
        }

        for (int64_t stateId : m_states)
            m_stateTrace[{stateId, 0}] = m_state.at(stateId);

        SeedTrackedInitialContents();
    }

    void SeedTrackedInitialContents() {
        // Represented slots are projections of the same concrete initial
        // memory, not independent values chosen later by individual reads.
        for (size_t index = 0; index < m_tracePairs.size(); ++index) {
            const int64_t memoryId = m_tracePairs[index].memoryStateId;
            auto memory = m_state.find(memoryId);
            if (memory == m_state.end() || !memory->second.isArray)
                throw std::runtime_error(
                    "tracked pair references an unavailable memory state");
            ArrayValue &array = memory->second.array;
            BitValue selector = LookupPairBits(m_steps[0].selectorValues,
                                               index,
                                               array.indexWidth);
            BitValue content = LookupPairBits(m_steps[0].contentValues,
                                              index,
                                              array.elementWidth);
            const std::string address = selector.value.ToBinary();
            if (array.initial->uniform) {
                if (array.initial->uniformValue.value != content.value)
                    throw std::runtime_error(
                        "tracked content conflicts with uniform memory initialization");
                continue;
            }
            auto [entry, inserted] =
                array.initial->entries.emplace(address, content);
            if (!inserted && entry->second.value != content.value)
                throw std::runtime_error(
                    "equal selectors have inconsistent initial contents");
        }
    }

    void StepConcrete() {
        m_nextState.clear();
        for (int64_t stateId : m_states) {
            auto next = m_next.find(stateId);
            if (next != m_next.end()) {
                m_nextState[stateId] = EvalConcrete(next->second);
            } else if (IsArraySort(m_ir.Node(stateId).sortId)) {
                m_nextState[stateId] = m_state.at(stateId);
            } else {
                m_nextState[stateId] = Value::BV(LookupBits(
                    m_steps[m_time + 1].stateValues,
                    stateId,
                    NodeWidth(stateId)));
            }
            m_stateTrace[{stateId, m_time + 1}] = m_nextState.at(stateId);
        }
        m_state.swap(m_nextState);
    }

    void ClearStepCaches() {
        m_cache.clear();
        m_abstractCache.clear();
        m_hybridCache.clear();
    }

    Value EvalConcrete(int64_t id) { return Eval(id, EvalMode::Concrete); }
    Value EvalAbstract(int64_t id) { return Eval(id, EvalMode::Abstract); }
    Value EvalHybrid(int64_t id) { return Eval(id, EvalMode::Hybrid); }

    Value Eval(int64_t id, EvalMode mode) {
        auto &cache = mode == EvalMode::Concrete
                          ? m_cache
                          : mode == EvalMode::Abstract ? m_abstractCache
                                                       : m_hybridCache;
        auto found = cache.find(id);
        if (found != cache.end()) return found->second;
        Value result = EvalUncached(id, mode);
        cache.emplace(id, result);
        return result;
    }

    Value EvalUncached(int64_t signedId, EvalMode mode) {
        if (signedId < 0) {
            Value value = Eval(-signedId, mode);
            if (value.isArray)
                throw std::runtime_error("array value cannot be inverted");
            return Value::BV(
                {value.bits.value.Apply(btorsim_bv_not)});
        }

        const Btor2IRNode &node = m_ir.Node(signedId);
        if (node.tag == BTOR2_TAG_state) {
            const auto &states = mode == EvalMode::Hybrid ? m_hybridState
                                                          : m_state;
            if (mode == EvalMode::Abstract && !IsArraySort(node.sortId))
                return Value::BV(LookupBits(
                    m_steps[m_time].stateValues,
                    node.id,
                    NodeWidth(node.id)));
            auto found = states.find(node.id);
            if (found != states.end()) return found->second;
            throw EvaluationError(node, "state has no simulated value");
        }
        if (node.tag == BTOR2_TAG_input)
            return Value::BV(LookupBits(
                m_steps[m_time].inputValues,
                node.id,
                NodeWidth(node.id)));
        if (node.tag == BTOR2_TAG_read) {
            if (mode == EvalMode::Abstract) return EvalAbstractRead(node);
            if (mode == EvalMode::Hybrid) return EvalHybridRead(node);
        }

        auto arg = [&](size_t index) { return Eval(node.args[index], mode); };

        switch (node.tag) {
        case BTOR2_TAG_const:
            return Value::BV({WLBitVector::FromBinary(
                NodeWidth(node.id), node.constant)});
        case BTOR2_TAG_constd:
            return Value::BV({WLBitVector::FromDecimal(
                NodeWidth(node.id), node.constant)});
        case BTOR2_TAG_consth:
            return Value::BV({WLBitVector::FromHex(
                NodeWidth(node.id), node.constant)});
        case BTOR2_TAG_zero:
            return Value::BV(BitValue::Zero(NodeWidth(node.id)));
        case BTOR2_TAG_one:
            return Value::BV(BitValue::FromUInt64(NodeWidth(node.id), 1));
        case BTOR2_TAG_ones:
            return Value::BV({WLBitVector::Ones(NodeWidth(node.id))});
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
        if (!array.array.initial)
            throw EvaluationError(read, "array has no initial store");
        if (array.array.initial->uniform)
            return Value::BV(array.array.initial->uniformValue);
        auto initial = array.array.initial->entries.find(key);
        if (initial != array.array.initial->entries.end())
            return Value::BV(initial->second);

        // A missing entry in an uninitialized memory is chosen to match the
        // abstract read. Future aliases of the same address reuse this value.
        auto abstract = m_abstractReads.find({read.id, m_time});
        if (abstract == m_abstractReads.end())
            throw EvaluationError(
                read, "abstract replay has no value for uninitialized read");
        BitValue chosen = abstract->second;
        array.array.initial->entries.emplace(key, chosen);
        return Value::BV(chosen);
    }

    Value EvalUnary(const Btor2IRNode &node, const Value &operand) const {
        if (operand.isArray)
            throw EvaluationError(node, "scalar operation consumes array");
        auto apply = [&](WLBitVector::UnaryOperation operation) {
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
        if (lhs.isArray || rhs.isArray)
            throw EvaluationError(node, "scalar operation consumes array");
        const WLBitVector &x = lhs.bits.value;
        const WLBitVector &y = rhs.bits.value;
        auto apply = [&](WLBitVector::BinaryOperation operation) {
            return Value::BV({x.Apply(operation, y)});
        };
        auto reverse = [&](WLBitVector::BinaryOperation operation) {
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
            WLBitVector sum = x.Apply(btorsim_bv_add, y);
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

    static Value SignedAddOverflow(const WLBitVector &x,
                                   const WLBitVector &y) {
        WLBitVector sum = x.Apply(btorsim_bv_add, y);
        const bool sx = x.GetBit(x.Width() - 1);
        const bool sy = y.GetBit(y.Width() - 1);
        const bool sr = sum.GetBit(sum.Width() - 1);
        return Value::BV(BitValue::FromBool(sx == sy && sr != sx));
    }

    static Value SignedSubOverflow(const WLBitVector &x,
                                   const WLBitVector &y) {
        WLBitVector difference = x.Apply(btorsim_bv_sub, y);
        const bool sx = x.GetBit(x.Width() - 1);
        const bool sy = y.GetBit(y.Width() - 1);
        const bool sr = difference.GetBit(difference.Width() - 1);
        return Value::BV(BitValue::FromBool(sx != sy && sr != sx));
    }

    static Value SignedMulOverflow(const WLBitVector &x,
                                   const WLBitVector &y) {
        const unsigned width = x.Width();
        WLBitVector product = x.SignExtend(width).Apply(
            btorsim_bv_mul, y.SignExtend(width));
        WLBitVector low = product.Slice(width - 1, 0);
        return Value::BV(BitValue::FromBool(
            product != low.SignExtend(width)));
    }

    static Value UnsignedMulOverflow(const WLBitVector &x,
                                     const WLBitVector &y) {
        const unsigned width = x.Width();
        WLBitVector product = x.ZeroExtend(width).Apply(
            btorsim_bv_mul, y.ZeroExtend(width));
        return Value::BV(BitValue::FromBool(
            !product.Slice(2 * width - 1, width).IsZero()));
    }

    static Value SignedDivOverflow(const WLBitVector &x,
                                   const WLBitVector &y) {
        WLBitVector minimum = WLBitVector::Zero(x.Width());
        minimum.SetBit(x.Width() - 1, true);
        return Value::BV(BitValue::FromBool(
            x == minimum && y.IsOnes()));
    }

    Value EvalAbstractRead(const Btor2IRNode &read) {
        const int64_t memoryId = m_readMemory.at(read.id);
        BitValue result = LookupBits(m_steps[m_time].abstractReadValues,
                                     read.id,
                                     ArrayElementWidth(memoryId));
        const BitValue address = EvalAbstract(read.args[1]).bits;
        // Match the priority chain built by WLArrayAbstraction: lower slot
        // indices dominate when selectors alias.
        for (size_t index = m_tracePairs.size(); index-- > 0;) {
            if (m_tracePairs[index].memoryStateId != memoryId) continue;
            BitValue selector = LookupPairBits(
                m_steps[m_time].selectorValues,
                index,
                ArrayIndexWidth(memoryId));
            BitValue content = LookupPairBits(
                m_steps[m_time].contentValues,
                index,
                ArrayElementWidth(memoryId));
            if (selector.value == address.value)
                result = EvalArrayAt(read.args[0],
                                     memoryId,
                                     selector,
                                     content,
                                     EvalMode::Abstract)
                             .bits;
        }
        return Value::BV(result);
    }

    Value EvalHybridRead(const Btor2IRNode &read) {
        TimedKey key{read.id, m_time};
        if (m_forcedReads.count(key)) {
            auto concrete = m_concreteReads.find(key);
            if (concrete == m_concreteReads.end())
                throw EvaluationError(
                    read, "trace has no concrete value for forced read");
            return Value::BV(concrete->second);
        }

        const int64_t memoryId = m_readMemory.at(read.id);
        BitValue result = LookupBits(m_steps[m_time].abstractReadValues,
                                     read.id,
                                     ArrayElementWidth(memoryId));
        const BitValue address = EvalHybrid(read.args[1]).bits;
        for (size_t index = m_tracePairs.size(); index-- > 0;) {
            if (m_tracePairs[index].memoryStateId != memoryId) continue;
            if (m_hybridSelectors[index].value != address.value) continue;
            if (m_recordAbstractReplay)
                m_representedReads.insert({read.id, m_time});
            result = EvalArrayAt(read.args[0],
                                 memoryId,
                                 m_hybridSelectors[index],
                                 m_hybridContents[index],
                                 EvalMode::Hybrid)
                         .bits;
        }
        return Value::BV(result);
    }

    Value EvalArrayAt(int64_t expressionId,
                      int64_t memoryId,
                      const BitValue &selector,
                      const BitValue &content,
                      EvalMode mode) {
        const Btor2IRNode &node = m_ir.Node(expressionId);
        switch (node.tag) {
        case BTOR2_TAG_state:
            if (node.id != memoryId)
                throw EvaluationError(node, "array expression mixes memories");
            return Value::BV(content);
        case BTOR2_TAG_write: {
            Value old = EvalArrayAt(
                node.args[0], memoryId, selector, content, mode);
            Value index = Eval(node.args[1], mode);
            return selector.value == index.bits.value
                       ? Eval(node.args[2], mode)
                       : old;
        }
        case BTOR2_TAG_ite:
            return Eval(node.args[0], mode).bits.IsOne()
                       ? EvalArrayAt(
                             node.args[1], memoryId, selector, content, mode)
                       : EvalArrayAt(
                             node.args[2], memoryId, selector, content, mode);
        default:
            throw EvaluationError(
                node, "array expression is outside the remodellable subset");
        }
    }

    bool HybridCounterexampleSurvives(
        const std::unordered_set<TimedKey, TimedKeyHash> &forced) {
        m_forcedReads = forced;
        m_hybridState.clear();
        for (int64_t stateId : m_states) {
            if (IsArraySort(m_ir.Node(stateId).sortId)) continue;
            m_hybridState[stateId] = Value::BV(LookupBits(
                m_steps[0].stateValues, stateId, NodeWidth(stateId)));
        }
        m_time = 0;
        m_hybridCache.clear();
        for (int64_t stateId : m_states) {
            if (IsArraySort(m_ir.Node(stateId).sortId)) continue;
            auto init = m_init.find(stateId);
            if (init == m_init.end()) continue;
            m_hybridState[stateId] = EvalHybrid(init->second);
            m_hybridCache.clear();
        }
        m_hybridSelectors.clear();
        m_hybridContents.clear();
        m_hybridSelectors.reserve(m_tracePairs.size());
        m_hybridContents.reserve(m_tracePairs.size());
        for (size_t index = 0; index < m_tracePairs.size(); ++index) {
            const int64_t memoryId = m_tracePairs[index].memoryStateId;
            m_hybridSelectors.push_back(LookupPairBits(
                m_steps[0].selectorValues,
                index,
                ArrayIndexWidth(memoryId)));
            m_hybridContents.push_back(LookupPairBits(
                m_steps[0].contentValues,
                index,
                ArrayElementWidth(memoryId)));
        }

        std::vector<std::unordered_map<int64_t, BitValue>> addressHistory(
            m_steps.size());
        bool constraintsHold = true;
        bool badSeen = false;
        bool guardFailure = false;
        bool counterexampleSeen = false;
        m_hybridFailure.clear();
        for (m_time = 0; m_time < m_steps.size(); ++m_time) {
            m_hybridCache.clear();
            for (int64_t readId : m_reads) {
                BitValue value = EvalHybrid(readId).bits;
                if (m_recordAbstractReplay)
                    m_abstractReads[{readId, m_time}] = value;
            }
            for (const WLMemoryPair &pair : m_tracePairs) {
                addressHistory[m_time].emplace(
                    pair.addressNodeId,
                    EvalHybrid(pair.addressNodeId).bits);
            }
            for (int64_t constraint : m_constraints) {
                if (EvalHybrid(constraint).bits.IsZero()) {
                    constraintsHold = false;
                    if (m_hybridFailure.empty())
                        m_hybridFailure =
                            "constraint " + std::to_string(constraint) +
                            " is false at time step " + std::to_string(m_time);
                }
            }
            if (EvalHybrid(m_bad).bits.IsOne()) {
                badSeen = true;
                if (constraintsHold) {
                    if (HybridGuardsHold(m_time, addressHistory)) {
                        if (!m_recordAbstractReplay) return true;
                        counterexampleSeen = true;
                    } else {
                        guardFailure = true;
                    }
                }
            }

            if (m_time + 1 < m_steps.size()) StepHybrid();
        }
        if (counterexampleSeen) return true;
        if (m_hybridFailure.empty()) {
            if (!badSeen)
                m_hybridFailure = "the bad expression is never true";
            else if (guardFailure)
                m_hybridFailure = "a selector guard is false";
            else
                m_hybridFailure = "no valid bad time step was found";
        }
        return false;
    }

    bool HybridGuardsHold(
        unsigned time,
        const std::vector<std::unordered_map<int64_t, BitValue>>
            &addressHistory) const {
        for (size_t index = 0; index < m_tracePairs.size(); ++index) {
            const WLMemoryPair &pair = m_tracePairs[index];
            // Before a delayed guard has received a value its latch is
            // unconstrained. Keeping it enabled is conservative for shrinking.
            if (pair.delay > time) continue;
            auto address =
                addressHistory[time - pair.delay].find(pair.addressNodeId);
            if (address == addressHistory[time - pair.delay].end() ||
                m_hybridSelectors[index].value != address->second.value)
                return false;
        }
        return true;
    }

    void StepHybrid() {
        std::unordered_map<int64_t, Value> nextState;
        for (int64_t stateId : m_states) {
            if (IsArraySort(m_ir.Node(stateId).sortId)) continue;
            auto next = m_next.find(stateId);
            if (next == m_next.end()) {
                nextState[stateId] = Value::BV(LookupBits(
                    m_steps[m_time + 1].stateValues,
                    stateId,
                    NodeWidth(stateId)));
            } else {
                nextState[stateId] = EvalHybrid(next->second);
            }
        }

        std::vector<BitValue> nextContents;
        nextContents.reserve(m_tracePairs.size());
        for (size_t index = 0; index < m_tracePairs.size(); ++index) {
            const WLMemoryPair &pair = m_tracePairs[index];
            auto next = m_next.find(pair.memoryStateId);
            if (next == m_next.end())
                throw std::runtime_error(
                    "tracked memory has no next-state expression");
            nextContents.push_back(
                EvalArrayAt(next->second,
                            pair.memoryStateId,
                            m_hybridSelectors[index],
                            m_hybridContents[index],
                            EvalMode::Hybrid)
                    .bits);
        }
        m_hybridState.swap(nextState);
        m_hybridContents.swap(nextContents);
    }

    int64_t FindMemory(int64_t expressionId) const {
        const Btor2IRNode &node = m_ir.Node(expressionId);
        if (node.tag == BTOR2_TAG_state) return node.id;
        if (node.tag == BTOR2_TAG_write) return FindMemory(node.args[0]);
        if (node.tag == BTOR2_TAG_ite) {
            int64_t lhs = FindMemory(node.args[1]);
            int64_t rhs = FindMemory(node.args[2]);
            if (lhs == rhs) return lhs;
        }
        throw EvaluationError(
            node, "array expression does not have one underlying memory");
    }

    static std::runtime_error EvaluationError(const Btor2IRNode &node,
                                              const std::string &message) {
        return std::runtime_error(
            "BTOR2 simulator error at line " + std::to_string(node.line) +
            " (id " + std::to_string(node.id) + "): " + message);
    }

    const Btor2IR &m_ir;
    std::vector<int64_t> m_inputs;
    std::vector<int64_t> m_states;
    std::unordered_map<int64_t, int64_t> m_init;
    std::unordered_map<int64_t, int64_t> m_next;
    int64_t m_bad{0};
    std::vector<int64_t> m_constraints;
    std::vector<int64_t> m_reads;
    std::unordered_map<int64_t, int64_t> m_readMemory;
    std::vector<WLMemoryPair> m_tracePairs;

    unsigned m_time{0};
    std::vector<WLReplayStep> m_steps;
    std::unordered_map<int64_t, Value> m_state;
    std::unordered_map<int64_t, Value> m_nextState;
    std::unordered_map<int64_t, Value> m_hybridState;
    std::vector<BitValue> m_hybridSelectors;
    std::vector<BitValue> m_hybridContents;
    std::unordered_map<int64_t, Value> m_cache;
    std::unordered_map<int64_t, Value> m_abstractCache;
    std::unordered_map<int64_t, Value> m_hybridCache;
    std::unordered_map<TimedKey, BitValue, TimedKeyHash> m_abstractReads;
    std::unordered_map<TimedKey, BitValue, TimedKeyHash> m_concreteReads;
    std::unordered_set<TimedKey, TimedKeyHash> m_forcedReads;
    std::unordered_set<TimedKey, TimedKeyHash> m_representedReads;
    bool m_recordAbstractReplay{false};
    std::string m_hybridFailure;
    std::unordered_map<TimedKey, Value, TimedKeyHash> m_stateTrace;
};

WLSimulator::WLSimulator(const Btor2IR &ir)
    : m_impl(std::make_unique<Impl>(ir)) {}

WLSimulator::~WLSimulator() = default;

WLSimulator::Result
WLSimulator::Replay(const WLReplayTrace &trace) {
    return m_impl->Replay(trace);
}

} // namespace car
