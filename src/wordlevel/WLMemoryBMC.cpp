#include "WLMemoryBMC.h"

#include "Btor2Frontend.h"
#include "Log.h"
#include "model/WLBitblastor.h"
#include "model/WLModel.h"

#ifdef KISSAT
extern "C" {
#include "kissat/src/kissat.h"
}
#endif

#include <algorithm>
#include <climits>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace car {
namespace {

// A small direct-CNF builder.  Literal 0 is false and INT_MAX is true.
class CnfFormula {
  public:
    static constexpr int kFalse = 0;
    static constexpr int kTrue = INT_MAX;

    int NewVar() { return ++m_maxVar; }

    static int Not(int lit) {
        if (lit == kFalse) return kTrue;
        if (lit == kTrue) return kFalse;
        return -lit;
    }

    void AddClause(std::initializer_list<int> literals) {
        AddClause(std::vector<int>(literals));
    }

    void AddClause(std::vector<int> literals) {
        std::vector<int> clause;
        clause.reserve(literals.size());
        for (int literal : literals) {
            if (literal == kTrue) return;
            if (literal == kFalse) continue;
            if (std::find(clause.begin(), clause.end(), literal) !=
                clause.end())
                continue;
            if (std::find(clause.begin(), clause.end(), -literal) !=
                clause.end())
                return;
            clause.push_back(literal);
        }
        m_clauses.push_back(std::move(clause));
    }

    int MakeAnd(int lhs, int rhs) {
        if (lhs == kFalse || rhs == kFalse) return kFalse;
        if (lhs == kTrue) return rhs;
        if (rhs == kTrue) return lhs;
        if (lhs == rhs) return lhs;
        if (lhs == -rhs) return kFalse;
        int result = NewVar();
        DefineAnd(result, lhs, rhs);
        return result;
    }

    void DefineAnd(int result, int lhs, int rhs) {
        AddClause({-result, lhs});
        AddClause({-result, rhs});
        AddClause({result, Not(lhs), Not(rhs)});
    }

    int AddressEqual(const std::vector<int> &lhs,
                     const std::vector<int> &rhs) {
        if (lhs.size() != rhs.size())
            throw std::runtime_error("memory address width mismatch");
        if (lhs.empty()) return kTrue;

        // This is the paper's 4m+1-clause address-equality encoding.
        int equal = NewVar();
        std::vector<int> finalClause;
        finalClause.reserve(lhs.size() + 1);
        for (size_t bit = 0; bit < lhs.size(); ++bit) {
            int bitEqual = NewVar();
            AddClause({-equal, lhs[bit], Not(rhs[bit])});
            AddClause({-equal, Not(lhs[bit]), rhs[bit]});
            AddClause({bitEqual, lhs[bit], rhs[bit]});
            AddClause({bitEqual, Not(lhs[bit]), Not(rhs[bit])});
            finalClause.push_back(-bitEqual);
        }
        finalClause.push_back(equal);
        AddClause(std::move(finalClause));
        return equal;
    }

    int AigLiteral(uint64_t aigLiteral) {
        if (aigLiteral == 0) return kFalse;
        if (aigLiteral == 1) return kTrue;
        const uint64_t node = aigLiteral & ~UINT64_C(1);
        auto [it, inserted] = m_aigVars.emplace(node, 0);
        if (inserted) it->second = NewVar();
        return (aigLiteral & 1U) ? -it->second : it->second;
    }

    void AddAigAnd(uint64_t node, uint64_t child0, uint64_t child1) {
        int result = AigLiteral(node);
        if (result <= 0 || result == kTrue)
            throw std::runtime_error("invalid Boolector AIG AND node");
        DefineAnd(result, AigLiteral(child0), AigLiteral(child1));
    }

    unsigned NumVars() const { return static_cast<unsigned>(m_maxVar); }
    const std::vector<std::vector<int>> &Clauses() const { return m_clauses; }

  private:
    int m_maxVar{0};
    std::vector<std::vector<int>> m_clauses;
    std::unordered_map<uint64_t, int> m_aigVars;
};

#ifdef KISSAT
class RawKissat {
  public:
    explicit RawKissat(const CnfFormula &formula)
        : m_solver(kissat_init()) {
        if (!m_solver) throw std::runtime_error("failed to initialize Kissat");
        kissat_reserve(m_solver, static_cast<int>(formula.NumVars()));
        for (const auto &clause : formula.Clauses()) {
            for (int literal : clause) kissat_add(m_solver, literal);
            kissat_add(m_solver, 0);
        }
    }

    ~RawKissat() { kissat_release(m_solver); }

    int Solve() { return kissat_solve(m_solver); }
    bool Value(int variable) const {
        if (variable <= 0 || variable == CnfFormula::kTrue)
            throw std::runtime_error("invalid Kissat model variable");
        return kissat_value(m_solver, variable) > 0;
    }

  private:
    kissat *m_solver;
};
#endif

} // namespace

class WLMemoryBMC::Impl {
  public:
    Impl(const Btor2IR &ir, Log &log)
        : m_ir(ir),
          m_log(log),
          m_bitblastor(ir) {
        IndexModel();
    }

    ~Impl() = default;

    enum class Result { Sat, Unsat };

    Result Check(unsigned target, WLWitnessTrace &witness) {
#ifndef KISSAT
        (void)target;
        (void)witness;
        throw std::runtime_error("WL memory BMC requires Kissat support");
#else
        BuildBound(target);
        EncodeAigGates();

        LOG_L(m_log,
              1,
              "WL memory BMC bound ",
              target,
              ": ",
              m_cnf.NumVars(),
              " variables, ",
              m_cnf.Clauses().size(),
              " clauses");

        RawKissat kissatEngine(m_cnf);
        int result = kissatEngine.Solve();
        if (result == 20) return Result::Unsat;
        if (result != 10)
            throw std::runtime_error(
                "Kissat returned an unexpected result code " +
                std::to_string(result));

        witness = ExtractWitness(target, kissatEngine);
        return Result::Sat;
#endif
    }

  private:
    struct WriteCandidate {
        BoolectorNode *enable{nullptr};
        BoolectorNode *address{nullptr};
        BoolectorNode *data{nullptr};
    };

    struct NormalizedArray {
        int64_t memoryId{0};
        std::vector<WriteCandidate> writes;
    };

    struct InitialRead {
        int64_t memoryId{0};
        int select{CnfFormula::kFalse};
        std::vector<int> address;
        std::vector<int> data;
    };

    void IndexModel() {
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_state:
                if (m_ir.Sort(node.sortId).tag == BTOR2_TAG_SORT_array)
                    m_arrayStates.push_back(node.id);
                else
                    m_scalarStates.push_back(node.id);
                break;
            case BTOR2_TAG_input: m_inputs.push_back(node.id); break;
            case BTOR2_TAG_read: m_reads.push_back(node.id); break;
            case BTOR2_TAG_init: m_init[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_next: m_next[node.args[0]] = node.args[1]; break;
            case BTOR2_TAG_bad: m_bad = node.args[0]; break;
            case BTOR2_TAG_constraint:
                m_constraints.push_back(node.args[0]);
                break;
            default: break;
            }
        }
        if (!m_bad) throw std::runtime_error("WL memory BMC has no bad property");

        for (int64_t memoryId : m_arrayStates) {
            if (!m_next.count(memoryId)) {
                throw std::runtime_error(
                    "WL memory BMC requires array states to have next functions");
            }
            auto init = m_init.find(memoryId);
            if (init != m_init.end() &&
                m_ir.Sort(m_ir.Node(init->second).sortId).tag ==
                    BTOR2_TAG_SORT_array) {
                throw std::runtime_error(
                    "WL memory BMC does not support non-uniform array init");
            }
        }
    }

    void BuildBound(unsigned target) {
        // Materialize scalar variables and expressions at every time step.
        for (unsigned time = 0; time <= target; ++time) {
            for (int64_t input : m_inputs) Evaluate(input, time);
            for (int64_t state : m_scalarStates) Evaluate(state, time);
            for (const Btor2IRNode &node : m_ir.Nodes()) {
                if (!node.sortId ||
                    m_ir.Sort(node.sortId).tag != BTOR2_TAG_SORT_bitvec)
                    continue;
                switch (node.tag) {
                case BTOR2_TAG_init:
                case BTOR2_TAG_next:
                case BTOR2_TAG_bad:
                case BTOR2_TAG_constraint:
                case BTOR2_TAG_output:
                case BTOR2_TAG_fair:
                case BTOR2_TAG_justice:
                    continue;
                default: Evaluate(node.id, time); break;
                }
            }
        }

        // Scalar state equations retain BTOR2's symbolic init/next semantics.
        for (int64_t state : m_scalarStates) {
            auto init = m_init.find(state);
            if (init != m_init.end())
                RequireEqual(Evaluate(state, 0), Evaluate(init->second, 0));
            auto next = m_next.find(state);
            if (next != m_next.end()) {
                for (unsigned time = 0; time < target; ++time)
                    RequireEqual(Evaluate(state, time + 1),
                                 Evaluate(next->second, time));
            }
        }

        // Each dynamic read receives exact forwarding constraints.
        for (unsigned time = 0; time <= target; ++time) {
            for (int64_t read : m_reads) EncodeRead(read, time);
        }
        EncodeInitialMemoryConsistency();

        // Constraints hold at every visited time; bad is queried at target.
        for (unsigned time = 0; time <= target; ++time) {
            for (int64_t constraint : m_constraints)
                RequireTrue(Evaluate(constraint, time));
        }
        RequireTrue(Evaluate(m_bad, target));

        // Reserve every value needed for a witness before Kissat sees the CNF.
        for (unsigned time = 0; time <= target; ++time) {
            for (int64_t input : m_inputs) Bits(Evaluate(input, time));
            for (int64_t state : m_scalarStates) Bits(Evaluate(state, time));
            for (int64_t read : m_reads) Bits(Evaluate(read, time));
        }
    }

    BoolectorNode *FreshValue(int64_t id,
                              unsigned time,
                              int64_t sortId,
                              const char *kind) {
        std::string symbol = std::string("wlbmc.") + kind + "." +
                             std::to_string(id) + "." +
                             std::to_string(time);
        return m_bitblastor.Variable(sortId, symbol.c_str());
    }

    WLBitblastor::ScalarContext &ScalarContext(unsigned time) {
        while (m_scalarContexts.size() <= time) {
            const unsigned contextTime = m_scalarContexts.size();
            m_scalarContexts.push_back(
                m_bitblastor.CreateScalarContext(
                    [this, contextTime](const Btor2IRNode &node) {
                        switch (node.tag) {
                        case BTOR2_TAG_input:
                            return FreshValue(node.id,
                                              contextTime,
                                              node.sortId,
                                              "input");
                        case BTOR2_TAG_state:
                            // States without next are per-step choices.
                            return FreshValue(node.id,
                                              contextTime,
                                              node.sortId,
                                              "state");
                        case BTOR2_TAG_read:
                            return FreshValue(node.id,
                                              contextTime,
                                              node.sortId,
                                              "read");
                        default:
                            throw std::runtime_error(
                                "unexpected scalar BMC leaf node " +
                                std::to_string(node.id));
                        }
                    }));
        }
        return *m_scalarContexts[time];
    }

    BoolectorNode *Evaluate(int64_t signedId, unsigned time) {
        return ScalarContext(time).Lower(signedId);
    }

    BoolectorNode *And(BoolectorNode *lhs, BoolectorNode *rhs) {
        return boolector_and(m_bitblastor.BtorInstance(), lhs, rhs);
    }

    NormalizedArray NormalizeArray(int64_t expressionId,
                                   unsigned time,
                                   BoolectorNode *path) {
        const Btor2IRNode &node = m_ir.Node(expressionId);
        if (node.tag == BTOR2_TAG_state) return {node.id, {}};
        if (node.tag == BTOR2_TAG_write) {
            NormalizedArray result =
                NormalizeArray(node.args[0], time, path);
            result.writes.insert(result.writes.begin(),
                                 {path,
                                  Evaluate(node.args[1], time),
                                  Evaluate(node.args[2], time)});
            return result;
        }
        if (node.tag == BTOR2_TAG_ite) {
            BoolectorNode *condition = Evaluate(node.args[0], time);
            NormalizedArray thenArray = NormalizeArray(
                node.args[1], time, And(path, condition));
            NormalizedArray elseArray = NormalizeArray(
                node.args[2],
                time,
                And(path,
                    boolector_not(m_bitblastor.BtorInstance(), condition)));
            if (thenArray.memoryId != elseArray.memoryId)
                throw std::runtime_error(
                    "array ite combines different memory states");
            thenArray.writes.insert(thenArray.writes.end(),
                                    elseArray.writes.begin(),
                                    elseArray.writes.end());
            return thenArray;
        }
        throw std::runtime_error(
            "WL memory BMC encountered unsupported array expression");
    }

    std::pair<int64_t, std::vector<WriteCandidate>>
    BuildWriteHistory(int64_t expressionId, unsigned time) {
        Btor *btor = m_bitblastor.BtorInstance();
        BoolectorSort boolSort = boolector_bitvec_sort(btor, 1);
        BoolectorNode *enabled = boolector_one(btor, boolSort);
        NormalizedArray current =
            NormalizeArray(expressionId, time, enabled);
        int64_t memoryId = current.memoryId;
        std::vector<WriteCandidate> writes = std::move(current.writes);

        while (time > 0) {
            --time;
            auto next = m_next.find(memoryId);
            if (next == m_next.end()) continue;
            NormalizedArray previous =
                NormalizeArray(next->second, time, enabled);
            if (previous.memoryId != memoryId)
                throw std::runtime_error(
                    "array next expression changes underlying memory state");
            writes.insert(writes.end(),
                          previous.writes.begin(),
                          previous.writes.end());
        }
        return {memoryId, std::move(writes)};
    }

    std::vector<int> Bits(BoolectorNode *node) {
        std::vector<uint64_t> raw = m_bitblastor.Bitblast(node);
        std::vector<int> result;
        result.reserve(raw.size());
        for (uint64_t literal : raw)
            result.push_back(m_cnf.AigLiteral(literal));
        return result;
    }

    void RequireTrue(BoolectorNode *node) {
        std::vector<int> bits = Bits(node);
        if (bits.size() != 1)
            throw std::runtime_error("expected one-bit BMC condition");
        m_cnf.AddClause({bits.front()});
    }

    void RequireEqual(BoolectorNode *lhs, BoolectorNode *rhs) {
        RequireTrue(
            boolector_eq(m_bitblastor.BtorInstance(), lhs, rhs));
    }

    void EncodeSelectedData(int select,
                            const std::vector<int> &readData,
                            const std::vector<int> &sourceData) {
        if (readData.size() != sourceData.size())
            throw std::runtime_error("memory data width mismatch");
        for (size_t bit = 0; bit < readData.size(); ++bit) {
            m_cnf.AddClause(
                {CnfFormula::Not(select),
                 CnfFormula::Not(readData[bit]),
                 sourceData[bit]});
            m_cnf.AddClause(
                {CnfFormula::Not(select),
                 readData[bit],
                 CnfFormula::Not(sourceData[bit])});
        }
    }

    void EncodeRead(int64_t readId, unsigned time) {
        const Btor2IRNode &read = m_ir.Node(readId);
        std::vector<int> readData = Bits(Evaluate(readId, time));
        std::vector<int> readAddress = Bits(Evaluate(read.args[1], time));
        auto [memoryId, writes] = BuildWriteHistory(read.args[0], time);

        int prefix = CnfFormula::kTrue;
        for (const WriteCandidate &write : writes) {
            int addressEqual =
                m_cnf.AddressEqual(Bits(write.address), readAddress);
            std::vector<int> enableBits = Bits(write.enable);
            if (enableBits.size() != 1)
                throw std::runtime_error("memory write enable is not Boolean");
            int match = m_cnf.MakeAnd(enableBits.front(), addressEqual);
            int select = m_cnf.MakeAnd(match, prefix);
            EncodeSelectedData(select, readData, Bits(write.data));
            prefix = m_cnf.MakeAnd(CnfFormula::Not(match), prefix);
        }

        auto init = m_init.find(memoryId);
        if (init != m_init.end()) {
            EncodeSelectedData(prefix,
                               readData,
                               Bits(Evaluate(init->second, 0)));
            return;
        }

        InitialRead initial;
        initial.memoryId = memoryId;
        initial.select = prefix;
        initial.address = std::move(readAddress);
        initial.data.reserve(readData.size());
        for (size_t bit = 0; bit < readData.size(); ++bit)
            initial.data.push_back(m_cnf.NewVar());
        EncodeSelectedData(initial.select, readData, initial.data);
        m_initialReads.push_back(std::move(initial));
    }

    void EncodeInitialMemoryConsistency() {
        for (size_t i = 0; i < m_initialReads.size(); ++i) {
            for (size_t j = i + 1; j < m_initialReads.size(); ++j) {
                const InitialRead &lhs = m_initialReads[i];
                const InitialRead &rhs = m_initialReads[j];
                if (lhs.memoryId != rhs.memoryId) continue;
                int sameAddress =
                    m_cnf.AddressEqual(lhs.address, rhs.address);
                for (size_t bit = 0; bit < lhs.data.size(); ++bit) {
                    m_cnf.AddClause(
                        {CnfFormula::Not(lhs.select),
                         CnfFormula::Not(rhs.select),
                         CnfFormula::Not(sameAddress),
                         CnfFormula::Not(lhs.data[bit]),
                         rhs.data[bit]});
                    m_cnf.AddClause(
                        {CnfFormula::Not(lhs.select),
                         CnfFormula::Not(rhs.select),
                         CnfFormula::Not(sameAddress),
                         lhs.data[bit],
                         CnfFormula::Not(rhs.data[bit])});
                }
            }
        }
    }

    void EncodeAigGates() {
        for (const WLAigGate &gate : m_bitblastor.Gates())
            m_cnf.AddAigAnd(gate.node, gate.child0, gate.child1);
    }

#ifdef KISSAT
    bool ModelLiteral(int literal,
                      const RawKissat &kissatEngine) const {
        if (literal == CnfFormula::kTrue) return true;
        if (literal == CnfFormula::kFalse) return false;
        return literal > 0 ? kissatEngine.Value(literal)
                           : !kissatEngine.Value(-literal);
    }

    WLBitVector ModelCnfBits(const std::vector<int> &literals,
                             const RawKissat &kissatEngine) const {
        WLBitVector value = WLBitVector::Zero(literals.size());
        for (size_t bit = 0; bit < literals.size(); ++bit)
            value.SetBit(static_cast<uint32_t>(bit),
                         ModelLiteral(literals[bit], kissatEngine));
        return value;
    }

    WLBitVector ModelBits(BoolectorNode *node,
                          const RawKissat &kissatEngine) {
        return ModelCnfBits(Bits(node), kissatEngine);
    }

    WLWitnessTrace ExtractWitness(unsigned target,
                                  const RawKissat &kissatEngine) {
        WLWitnessTrace trace;
        trace.steps.resize(static_cast<size_t>(target) + 1);
        for (unsigned time = 0; time <= target; ++time) {
            WLWitnessStep &step = trace.steps[time];
            for (int64_t input : m_inputs)
                step.inputValues.emplace(
                    input,
                    ModelBits(Evaluate(input, time), kissatEngine));
            for (int64_t state : m_scalarStates)
                step.stateValues.emplace(
                    state,
                    ModelBits(Evaluate(state, time), kissatEngine));
        }

        // An uninitialized memory needs only the queried initial locations;
        // all successor array values are reconstructed from BTOR2 transitions.
        std::unordered_map<int64_t,
                           std::unordered_map<std::string, WLBitVector>>
            initialEntries;
        for (const InitialRead &initial : m_initialReads) {
            if (!ModelLiteral(initial.select, kissatEngine)) continue;
            WLBitVector address =
                ModelCnfBits(initial.address, kissatEngine);
            WLBitVector data =
                ModelCnfBits(initial.data, kissatEngine);
            initialEntries[initial.memoryId].insert_or_assign(
                address.ToBinary(), std::move(data));
        }
        for (auto &[memoryId, entries] : initialEntries) {
            WLWitnessArrayValue value;
            value.entries.reserve(entries.size());
            for (auto &[address, data] : entries) {
                value.entries.push_back(
                    {WLBitVector::FromBinary(address.size(), address),
                     std::move(data)});
            }
            trace.steps.front().arrayStateValues.emplace(
                memoryId, std::move(value));
        }
        return trace;
    }
#endif

    const Btor2IR &m_ir;
    Log &m_log;
    WLBitblastor m_bitblastor;
    int64_t m_bad{0};
    std::vector<int64_t> m_inputs;
    std::vector<int64_t> m_scalarStates;
    std::vector<int64_t> m_arrayStates;
    std::vector<int64_t> m_reads;
    std::vector<int64_t> m_constraints;
    std::unordered_map<int64_t, int64_t> m_init;
    std::unordered_map<int64_t, int64_t> m_next;
    std::vector<std::unique_ptr<WLBitblastor::ScalarContext>>
        m_scalarContexts;
    std::vector<InitialRead> m_initialReads;
    CnfFormula m_cnf;
};

WLMemoryBMC::WLMemoryBMC(const Settings &settings,
                         WLModel &model,
                         Log &log)
    : m_settings(settings),
      m_model(model),
      m_log(log) {}

WLMemoryBMC::~WLMemoryBMC() = default;

CheckResult WLMemoryBMC::Run(unsigned bound) {
    m_witnessTrace = {};

    for (unsigned depth = 0;; ++depth) {
        // Kissat is non-incremental, so every depth receives a fresh formula.
        m_impl = std::make_unique<Impl>(m_model.PropertyIR(), m_log);
        Impl::Result result = m_impl->Check(depth, m_witnessTrace);
        if (result == Impl::Result::Sat) return CheckResult::Unsafe;
        if (depth == bound) break;
    }

    LOG_L(m_log,
          1,
          "WL memory BMC found no counterexample through bound ",
          bound);
    return CheckResult::Unknown;
}

} // namespace car
