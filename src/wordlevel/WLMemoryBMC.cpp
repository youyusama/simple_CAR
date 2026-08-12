#include "WLMemoryBMC.h"

#include "Btor2Frontend.h"
#include "Log.h"
#include "model/WLModel.h"
#include "WLSimulator.h"

#include <boolector/boolector.h>

#ifdef KISSAT
extern "C" {
#include "kissat/src/kissat.h"
}
#endif

#include <algorithm>
#include <array>
#include <climits>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace car {
namespace {

using UnaryFn = BoolectorNode *(*)(Btor *, BoolectorNode *);
using BinaryFn = BoolectorNode *(*)(Btor *, BoolectorNode *, BoolectorNode *);

const std::unordered_map<Btor2Tag, UnaryFn> kUnaryOps{
    {BTOR2_TAG_dec, boolector_dec},       {BTOR2_TAG_inc, boolector_inc},
    {BTOR2_TAG_neg, boolector_neg},       {BTOR2_TAG_not, boolector_not},
    {BTOR2_TAG_redand, boolector_redand}, {BTOR2_TAG_redor, boolector_redor},
    {BTOR2_TAG_redxor, boolector_redxor},
};

const std::unordered_map<Btor2Tag, BinaryFn> kBinaryOps{
    {BTOR2_TAG_add, boolector_add},       {BTOR2_TAG_and, boolector_and},
    {BTOR2_TAG_concat, boolector_concat}, {BTOR2_TAG_eq, boolector_eq},
    {BTOR2_TAG_iff, boolector_iff},       {BTOR2_TAG_implies, boolector_implies},
    {BTOR2_TAG_mul, boolector_mul},       {BTOR2_TAG_nand, boolector_nand},
    {BTOR2_TAG_neq, boolector_ne},        {BTOR2_TAG_nor, boolector_nor},
    {BTOR2_TAG_or, boolector_or},         {BTOR2_TAG_rol, boolector_rol},
    {BTOR2_TAG_ror, boolector_ror},       {BTOR2_TAG_saddo, boolector_saddo},
    {BTOR2_TAG_sdiv, boolector_sdiv},     {BTOR2_TAG_sdivo, boolector_sdivo},
    {BTOR2_TAG_sgt, boolector_sgt},       {BTOR2_TAG_sgte, boolector_sgte},
    {BTOR2_TAG_sll, boolector_sll},       {BTOR2_TAG_slt, boolector_slt},
    {BTOR2_TAG_slte, boolector_slte},     {BTOR2_TAG_smod, boolector_smod},
    {BTOR2_TAG_smulo, boolector_smulo},   {BTOR2_TAG_sra, boolector_sra},
    {BTOR2_TAG_srem, boolector_srem},     {BTOR2_TAG_srl, boolector_srl},
    {BTOR2_TAG_ssubo, boolector_ssubo},   {BTOR2_TAG_sub, boolector_sub},
    {BTOR2_TAG_uaddo, boolector_uaddo},   {BTOR2_TAG_udiv, boolector_udiv},
    {BTOR2_TAG_ugt, boolector_ugt},       {BTOR2_TAG_ugte, boolector_ugte},
    {BTOR2_TAG_ult, boolector_ult},       {BTOR2_TAG_ulte, boolector_ulte},
    {BTOR2_TAG_umulo, boolector_umulo},   {BTOR2_TAG_urem, boolector_urem},
    {BTOR2_TAG_usubo, boolector_usubo},   {BTOR2_TAG_xnor, boolector_xnor},
    {BTOR2_TAG_xor, boolector_xor},
};

struct TimedId {
    int64_t id{0};
    unsigned time{0};

    bool operator==(const TimedId &other) const {
        return id == other.id && time == other.time;
    }
};

struct TimedIdHash {
    size_t operator()(const TimedId &key) const {
        return std::hash<int64_t>{}(key.id) ^
               (std::hash<unsigned>{}(key.time) << 1);
    }
};

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

struct AigGateCollector {
    std::vector<std::tuple<uint64_t, uint64_t, uint64_t>> gates;
    std::unordered_set<uint64_t> visited;
};

void CollectAigGate(void *state,
                    bool isPost,
                    uint64_t node,
                    const char *,
                    uint64_t child0,
                    uint64_t child1) {
    if (!isPost || !child0) return;
    auto *collector = static_cast<AigGateCollector *>(state);
    const uint64_t normalized = node & ~UINT64_C(1);
    if (!collector->visited.insert(normalized).second) return;
    collector->gates.emplace_back(normalized, child0, child1);
}

} // namespace

class WLMemoryBMC::Impl {
  public:
    Impl(const Btor2IR &ir, const Btor2IR &sourceIr, Log &log)
        : m_ir(ir),
          m_sourceIr(sourceIr),
          m_log(log),
          m_btor(boolector_new()) {
        IndexModel();
    }

    ~Impl() {
        if (m_aigManager) boolector_aig_delete(m_aigManager);
        boolector_release_all(m_btor);
        boolector_delete(m_btor);
    }

    enum class Result { Sat, Unsat, Unknown };

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

        RawKissat solver(m_cnf);
        int result = solver.Solve();
        if (result == 20) return Result::Unsat;
        if (result != 10) return Result::Unknown;

        WLReplayTrace replay = ExtractReplay(target, solver);
        // Validate and serialize the SAT trace against the complete source model.
        WLSimulator simulator(m_sourceIr);
        WLSimulator::Result simulated = simulator.Replay(replay);
        if (simulated.kind !=
            WLSimulator::ReplayKind::ConcreteCounterexample) {
            throw std::runtime_error(
                "exact WL memory BMC trace failed concrete replay");
        }
        witness = std::move(simulated.witnessTrace);
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

    BoolectorSort Sort(int64_t sortId) {
        auto found = m_sorts.find(sortId);
        if (found != m_sorts.end()) return found->second;
        const Btor2IRSort &sort = m_ir.Sort(sortId);
        if (sort.tag != BTOR2_TAG_SORT_bitvec)
            throw std::runtime_error("array sort reached scalar BMC lowering");
        BoolectorSort result = boolector_bitvec_sort(m_btor, sort.width);
        m_sorts.emplace(sortId, result);
        return result;
    }

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
        m_aigManager = boolector_aig_new(m_btor);

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
        return boolector_var(m_btor, Sort(sortId), symbol.c_str());
    }

    BoolectorNode *Evaluate(int64_t signedId, unsigned time) {
        if (signedId < 0)
            return boolector_not(m_btor, Evaluate(-signedId, time));
        TimedId key{signedId, time};
        auto cached = m_values.find(key);
        if (cached != m_values.end()) return cached->second;

        const Btor2IRNode &node = m_ir.Node(signedId);
        if (!node.sortId ||
            m_ir.Sort(node.sortId).tag != BTOR2_TAG_SORT_bitvec) {
            throw std::runtime_error("scalar evaluation reached an array node");
        }

        BoolectorNode *result = nullptr;
        auto arg = [&](size_t index) {
            return Evaluate(node.args[index], time);
        };
        switch (node.tag) {
        case BTOR2_TAG_input:
            result = FreshValue(node.id, time, node.sortId, "input");
            break;
        case BTOR2_TAG_state:
            // States without next are independent per-step choices.
            result = FreshValue(node.id, time, node.sortId, "state");
            break;
        case BTOR2_TAG_read:
            result = FreshValue(node.id, time, node.sortId, "read");
            break;
        case BTOR2_TAG_const:
            result = boolector_const(m_btor, node.constant.c_str());
            break;
        case BTOR2_TAG_constd:
            result = boolector_constd(
                m_btor, Sort(node.sortId), node.constant.c_str());
            break;
        case BTOR2_TAG_consth:
            result = boolector_consth(
                m_btor, Sort(node.sortId), node.constant.c_str());
            break;
        case BTOR2_TAG_zero:
            result = boolector_zero(m_btor, Sort(node.sortId));
            break;
        case BTOR2_TAG_one:
            result = boolector_one(m_btor, Sort(node.sortId));
            break;
        case BTOR2_TAG_ones:
            result = boolector_ones(m_btor, Sort(node.sortId));
            break;
        case BTOR2_TAG_slice:
            result = boolector_slice(m_btor,
                                     arg(0),
                                     static_cast<uint32_t>(node.args[1]),
                                     static_cast<uint32_t>(node.args[2]));
            break;
        case BTOR2_TAG_uext:
            result = boolector_uext(
                m_btor, arg(0), static_cast<uint32_t>(node.args[1]));
            break;
        case BTOR2_TAG_sext:
            result = boolector_sext(
                m_btor, arg(0), static_cast<uint32_t>(node.args[1]));
            break;
        case BTOR2_TAG_ite:
            result = boolector_cond(m_btor, arg(0), arg(1), arg(2));
            break;
        default:
            if (node.nargs == 1) {
                auto operation = kUnaryOps.find(node.tag);
                if (operation != kUnaryOps.end())
                    result = operation->second(m_btor, arg(0));
            } else if (node.nargs == 2) {
                auto operation = kBinaryOps.find(node.tag);
                if (operation != kBinaryOps.end())
                    result = operation->second(m_btor, arg(0), arg(1));
            }
            break;
        }
        if (!result) {
            throw std::runtime_error(
                "unsupported WL memory BMC scalar node " +
                std::to_string(node.id));
        }
        m_values.emplace(key, result);
        return result;
    }

    BoolectorNode *And(BoolectorNode *lhs, BoolectorNode *rhs) {
        return boolector_and(m_btor, lhs, rhs);
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
                And(path, boolector_not(m_btor, condition)));
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
        BoolectorSort boolSort = boolector_bitvec_sort(m_btor, 1);
        BoolectorNode *enabled = boolector_one(m_btor, boolSort);
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
        RegisterRoot(node);
        const size_t width = boolector_get_width(m_btor, node);
        uint64_t *raw = boolector_aig_get_bits(m_aigManager, node);
        std::vector<int> result(width);
        for (size_t bit = 0; bit < width; ++bit)
            result[bit] = m_cnf.AigLiteral(raw[width - bit - 1]);
        boolector_aig_free_bits(m_aigManager, raw, width);
        return result;
    }

    void RequireTrue(BoolectorNode *node) {
        std::vector<int> bits = Bits(node);
        if (bits.size() != 1)
            throw std::runtime_error("expected one-bit BMC condition");
        m_cnf.AddClause({bits.front()});
    }

    void RequireEqual(BoolectorNode *lhs, BoolectorNode *rhs) {
        RequireTrue(boolector_eq(m_btor, lhs, rhs));
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

    void RegisterRoot(BoolectorNode *node) {
        if (!m_registeredRoots.insert(node).second) return;
        boolector_aig_bitblast(m_aigManager, node);
        boolector_aig_visit(
            m_aigManager, node, CollectAigGate, &m_gateCollector);
    }

    void EncodeAigGates() {
        for (const auto &[node, child0, child1] : m_gateCollector.gates)
            m_cnf.AddAigAnd(node, child0, child1);
    }

#ifdef KISSAT
    WLBitVector ModelBits(BoolectorNode *node, const RawKissat &solver) {
        std::vector<int> literals = Bits(node);
        WLBitVector value = WLBitVector::Zero(literals.size());
        for (size_t bit = 0; bit < literals.size(); ++bit) {
            int literal = literals[bit];
            bool bitValue = literal == CnfFormula::kTrue ||
                            (literal != CnfFormula::kFalse &&
                             (literal > 0 ? solver.Value(literal)
                                          : !solver.Value(-literal)));
            value.SetBit(static_cast<uint32_t>(bit), bitValue);
        }
        return value;
    }

    WLReplayTrace ExtractReplay(unsigned target, const RawKissat &solver) {
        WLReplayTrace trace;
        trace.steps.resize(static_cast<size_t>(target) + 1);
        for (unsigned time = 0; time <= target; ++time) {
            WLReplayStep &step = trace.steps[time];
            for (int64_t input : m_inputs)
                step.inputValues.emplace(
                    input, ModelBits(Evaluate(input, time), solver));
            for (int64_t state : m_scalarStates)
                step.stateValues.emplace(
                    state, ModelBits(Evaluate(state, time), solver));
            for (int64_t read : m_reads)
                step.abstractReadValues.emplace(
                    read, ModelBits(Evaluate(read, time), solver));
        }
        return trace;
    }
#endif

    const Btor2IR &m_ir;
    const Btor2IR &m_sourceIr;
    Log &m_log;
    Btor *m_btor{nullptr};
    BoolectorAIGMgr *m_aigManager{nullptr};
    int64_t m_bad{0};
    std::vector<int64_t> m_inputs;
    std::vector<int64_t> m_scalarStates;
    std::vector<int64_t> m_arrayStates;
    std::vector<int64_t> m_reads;
    std::vector<int64_t> m_constraints;
    std::unordered_map<int64_t, int64_t> m_init;
    std::unordered_map<int64_t, int64_t> m_next;
    std::unordered_map<int64_t, BoolectorSort> m_sorts;
    std::unordered_map<TimedId, BoolectorNode *, TimedIdHash> m_values;
    std::vector<InitialRead> m_initialReads;
    std::unordered_set<BoolectorNode *> m_registeredRoots;
    AigGateCollector m_gateCollector;
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
    m_completedBound = false;
    m_checkedBound = 0;
    m_witnessTrace = {};

    try {
        for (unsigned depth = 0;; ++depth) {
            // Kissat is non-incremental, so every depth receives a fresh formula.
            m_impl = std::make_unique<Impl>(
                m_model.PropertyIR(), m_model.SourceIR(), m_log);
            Impl::Result result = m_impl->Check(depth, m_witnessTrace);
            if (result == Impl::Result::Sat) return CheckResult::Unsafe;
            if (result == Impl::Result::Unknown) return CheckResult::Unknown;
            m_checkedBound = depth;
            if (depth == bound) break;
        }
    } catch (const std::exception &error) {
        LOG_L(m_log, 0, "WL memory BMC failed: ", error.what());
        return CheckResult::Unknown;
    }

    m_completedBound = true;
    LOG_L(m_log,
          1,
          "WL memory BMC found no counterexample through bound ",
          bound);
    return CheckResult::Unknown;
}

} // namespace car
