#include "WLMemoryBMC.h"

#include "Btor2Frontend.h"
#include "Log.h"
#include "WLSimulator.h"
#include "model/WLArrayEqualityEncoder.h"
#include "model/WLBoundedUnroller.h"
#include "model/WLBitblastor.h"
#include "model/WLModel.h"

extern "C" {
#include "kissat/src/kissat.h"
}

#include <algorithm>
#include <climits>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace car {
namespace {

// A small direct-CNF builder.  Literal 0 is false and INT_MAX is true.
class CnfFormula {
  public:
    static constexpr int kFalse = 0;
    static constexpr int kTrue = INT_MAX;

    int NewVar() {
        if (m_maxVar >= INT_MAX - 1)
            throw std::runtime_error("WL memory BMC exhausted CNF variable IDs");
        return ++m_maxVar;
    }

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

    void DefineAnd(int result, int lhs, int rhs) {
        AddClause({-result, lhs});
        AddClause({-result, rhs});
        AddClause({result, Not(lhs), Not(rhs)});
    }

    int And(int lhs, int rhs) {
        if (lhs == kFalse || rhs == kFalse || lhs == Not(rhs)) return kFalse;
        if (lhs == kTrue || lhs == rhs) return rhs;
        if (rhs == kTrue) return lhs;
        const int result = NewVar();
        DefineAnd(result, lhs, rhs);
        return result;
    }

    int Or(int lhs, int rhs) { return Not(And(Not(lhs), Not(rhs))); }

    int AddressEqual(const std::vector<int> &lhs,
                     const std::vector<int> &rhs) {
        if (lhs.size() != rhs.size())
            throw std::runtime_error("memory address width mismatch");
        if (lhs == rhs) return kTrue;
        for (size_t bit = 0; bit < lhs.size(); ++bit)
            if (lhs[bit] == Not(rhs[bit])) return kFalse;

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

class RawKissat {
  public:
    RawKissat(const CnfFormula &formula, int queryLiteral)
        : m_solver(kissat_init()) {
        if (!m_solver) throw std::runtime_error("failed to initialize Kissat");
        kissat_reserve(m_solver, static_cast<int>(formula.NumVars()));
        for (const auto &clause : formula.Clauses()) {
            for (int literal : clause) kissat_add(m_solver, literal);
            kissat_add(m_solver, 0);
        }
        // The bad property is asserted only for this query.
        if (queryLiteral != CnfFormula::kTrue) {
            if (queryLiteral != CnfFormula::kFalse)
                kissat_add(m_solver, queryLiteral);
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

// One equality-free, already unrolled formula. All IDs belong to that IR;
// this encoder neither interprets state transitions nor reconstructs time steps.
class MemoryQuery {
  public:
    MemoryQuery(const Btor2IR &ir, Log &log)
        : m_ir(ir), m_log(log), m_bitblastor(ir) {
        IndexModel();
        m_scalar = m_bitblastor.CreateScalarContext(
            [this](const Btor2IRNode &node) {
                const std::string name = "wlbmc.value." + std::to_string(node.id);
                switch (node.tag) {
                case BTOR2_TAG_input:
                    return m_bitblastor.Variable(node.sortId, name.c_str());
                case BTOR2_TAG_read: {
                    auto *value = m_bitblastor.Variable(node.sortId, name.c_str());
                    m_readRequests.push_back({node.id, value});
                    return value;
                }
                default:
                    throw std::runtime_error("unexpected bounded scalar leaf " +
                                             std::to_string(node.id));
                }
            });
    }

    enum class Result { Sat, Unsat, Unknown };

    Result Check(WLTraceStep &flat,
                 const std::vector<int64_t> &observations,
                 std::map<int64_t, WLBitVector> &observedValues,
                 bool recoverArrays) {
        LowerRequiredValues(observations);
        for (int64_t constraint : m_constraints)
            RequireTrue(Evaluate(constraint));
        for (int64_t input : m_inputs)
            m_inputBits.emplace(input, Bits(Evaluate(input)));
        std::map<int64_t, std::vector<int>> modelBits;
        for (int64_t id : observations)
            modelBits.emplace(id, Bits(Evaluate(id)));
        const int queryLiteral = BooleanLiteral(Evaluate(m_bad));
        DrainReadRequests();
        EncodeAigGates();

        LOG_L(m_log, 1, "WL memory BMC formula: ", m_cnf.NumVars(),
              " variables, ", m_cnf.Clauses().size() +
                  (queryLiteral == CnfFormula::kTrue ? 0 : 1),
              " clauses, ", m_arrayCount, " array DAG nodes, ",
              m_rootReads.size(), " root reads");
        RawKissat kissatEngine(m_cnf, queryLiteral);
        const int result = kissatEngine.Solve();
        if (result == 20) return Result::Unsat;
        if (result == 0) return Result::Unknown;
        if (result != 10)
            throw std::runtime_error("Kissat returned an unexpected result code " +
                                     std::to_string(result));
        observedValues.clear();
        for (const auto &[id, bits] : modelBits)
            observedValues.emplace(id, ModelCnfBits(bits, kissatEngine));
        flat = {};
        for (const auto &[id, bits] : m_inputBits)
            flat.inputValues.emplace(id, ModelCnfBits(bits, kissatEngine));
        if (recoverArrays) CompleteArrayValues(flat, kissatEngine);
        return Result::Sat;
    }

  private:
    friend struct EMMEncodingTestAccess;

    void LowerRequiredValues(const std::vector<int64_t> &observations) {
        std::vector<int64_t> work = observations;
        work.push_back(m_bad);
        work.insert(work.end(), m_constraints.begin(), m_constraints.end());
        work.insert(work.end(), m_inputs.begin(), m_inputs.end());
        // Uniform initializers may themselves read arrays. Keep their equations
        // even if the initialized array is not read by the property.
        for (const auto &[array, data] : m_uniformData) work.push_back(data);
        std::unordered_set<int64_t> needed;
        while (!work.empty()) {
            const int64_t id = std::abs(work.back());
            work.pop_back();
            if (!needed.insert(id).second) continue;
            const auto &node = m_ir.Node(id);
            // Follow array operands too: EMM will need their write addresses,
            // write data and choice conditions when it drains a required read.
            for (uint32_t i = 0; i < node.nargs; ++i) work.push_back(node.args[i]);
        }
        // Retain declaration order, so scalar dependencies are normally cached
        // before their consumers reach the recursive scalar lowering API.
        for (const auto &node : m_ir.Nodes())
            if (needed.count(node.id) && node.sortId && node.tag != BTOR2_TAG_init &&
                m_ir.Sort(node.sortId).tag == BTOR2_TAG_SORT_bitvec)
                Evaluate(node.id);
    }

    struct RootRead {
        int64_t root;
        int select;
        std::vector<int> address;
        std::vector<int> data;
    };
    struct ReadRequest {
        int64_t nodeId;
        BoolectorNode *result;
    };

    void IndexModel() {
        for (const auto &node : m_ir.Nodes()) {
            if ((node.tag == BTOR2_TAG_eq || node.tag == BTOR2_TAG_neq) &&
                m_ir.Sort(m_ir.Node(node.args[0]).sortId).tag == BTOR2_TAG_SORT_array)
                throw std::runtime_error("EMM requires equality-free IR");
            const bool array = node.sortId &&
                m_ir.Sort(node.sortId).tag == BTOR2_TAG_SORT_array;
            if (array && node.tag != BTOR2_TAG_init) ++m_arrayCount;
            switch (node.tag) {
            case BTOR2_TAG_next:
                throw std::runtime_error("EMM requires bounded IR without next");
            case BTOR2_TAG_state:
                if (!array)
                    throw std::runtime_error("EMM requires bounded IR without scalar states");
                break;
            case BTOR2_TAG_init:
                if (!array ||
                    m_ir.Node(node.args[0]).tag != BTOR2_TAG_state ||
                    m_ir.Node(node.args[0]).sortId != node.sortId ||
                    m_ir.Node(node.args[1]).sortId != m_ir.Sort(node.sortId).elementSort)
                    throw std::runtime_error("bounded init must describe a uniform array");
                m_uniformData.emplace(node.args[0], node.args[1]);
                break;
            case BTOR2_TAG_input:
                (array ? m_arrayInputs : m_inputs).push_back(node.id);
                break;
            case BTOR2_TAG_bad: m_bad = node.args[0]; break;
            case BTOR2_TAG_constraint: m_constraints.push_back(node.args[0]); break;
            default: break;
            }
        }
        for (const auto &node : m_ir.Nodes())
            if (node.tag == BTOR2_TAG_state && !m_uniformData.count(node.id))
                throw std::runtime_error("bounded array state must describe a uniform array");
        if (!m_bad) throw std::runtime_error("WL memory BMC has no bad property");
    }

    BoolectorNode *Evaluate(int64_t id) { return m_scalar->Lower(id); }

    std::vector<int64_t> ArrayOrder(int64_t array) const {
        // Reverse postorder puts every write/choice before its array operands.
        // All incoming path conditions are therefore available before a shared
        // node is encoded. Visit DAG nodes once, never enumerate their paths.
        std::vector<int64_t> order;
        std::vector<int64_t> stack{array};
        std::unordered_set<int64_t> active{array};
        std::unordered_set<int64_t> visited;
        while (!stack.empty()) {
            const auto &node = m_ir.Node(stack.back());
            const unsigned first = node.tag == BTOR2_TAG_write ? 0 : 1;
            const unsigned end = node.tag == BTOR2_TAG_write ? 1 :
                                 node.tag == BTOR2_TAG_ite ? 3 : first;
            bool pending = false;
            for (unsigned arg = first; arg < end; ++arg) {
                const int64_t child = node.args[arg];
                if (visited.count(child)) continue;
                if (!active.insert(child).second)
                    throw std::runtime_error("cyclic bounded array expression");
                stack.push_back(child);
                pending = true;
                break;
            }
            if (pending) continue;
            order.push_back(node.id);
            visited.insert(node.id);
            active.erase(node.id);
            stack.pop_back();
        }
        std::reverse(order.begin(), order.end());
        return order;
    }

    void EncodeRead(const ReadRequest &request) {
        const auto &read = m_ir.Node(request.nodeId);
        const auto address = Bits(Evaluate(read.args[1]));
        const auto data = Bits(request.result);
        const auto order = ArrayOrder(read.args[0]);
        std::unordered_map<int64_t, int> enabled{{read.args[0], CnfFormula::kTrue}};
        auto enable = [&](int64_t array, int condition) {
            auto [it, inserted] = enabled.emplace(array, condition);
            if (!inserted) it->second = m_cnf.Or(it->second, condition);
        };

        // Ganai et al., equations (3)-(4): match is s, prefix is PS and
        // select is S. BTOR2 reads are total, so the read enable starts at true.
        // Path conditions act as write enables. They do NOT include write misses:
        // the shared prefix excludes all earlier (higher-priority) candidates.
        // For each assignment, enabled candidates lie on one array ancestry path;
        // parent-before-operand order therefore gives the newest write priority.
        int prefix = CnfFormula::kTrue;
        std::vector<int> sources;
        auto selectSource = [&](int match) {
            const int select = m_cnf.And(prefix, match);
            prefix = m_cnf.And(prefix, CnfFormula::Not(match));
            if (select != CnfFormula::kFalse) sources.push_back(select);
            return select;
        };
        for (int64_t id : order) {
            if (prefix == CnfFormula::kFalse) break;
            const auto found = enabled.find(id);
            if (found == enabled.end() || found->second == CnfFormula::kFalse)
                continue;
            const int path = found->second;
            const auto &node = m_ir.Node(id);
            switch (node.tag) {
            case BTOR2_TAG_input: {
                const int select = selectSource(path);
                // When selected, the final read value is an observation of this
                // root. Inactive candidates must not constrain its contents.
                if (select != CnfFormula::kFalse)
                    RegisterRootRead({node.id, select, address, data});
                break;
            }
            case BTOR2_TAG_state:
                EncodeSelectedData(selectSource(path), data,
                                   Bits(Evaluate(m_uniformData.at(node.id))));
                break;
            case BTOR2_TAG_write: {
                const int equal =
                    m_cnf.AddressEqual(address, Bits(Evaluate(node.args[1])));
                const int match = m_cnf.And(path, equal);
                EncodeSelectedData(selectSource(match), data,
                                   Bits(Evaluate(node.args[2])));
                enable(node.args[0], path);
                break;
            }
            case BTOR2_TAG_ite: {
                const int condition = BooleanLiteral(Evaluate(node.args[0]));
                enable(node.args[1], m_cnf.And(path, condition));
                enable(node.args[2], m_cnf.And(path, CnfFormula::Not(condition)));
                break;
            }
            default:
                throw std::runtime_error("unsupported bounded array source " +
                                         std::to_string(node.id));
            }
        }
        // The paper's read-validity clause: exactly one source must supply RD.
        // At-most-one follows from the shared prefix, without pairwise clauses.
        m_cnf.AddClause(std::move(sources));
    }

    void DrainReadRequests() {
        while (m_encodedReads < m_readRequests.size()) {
            // Evaluate may append more requests, so do not retain queue references.
            const ReadRequest request = m_readRequests[m_encodedReads++];
            EncodeRead(request);
        }
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
        m_cnf.AddClause({BooleanLiteral(node)});
    }

    int BooleanLiteral(BoolectorNode *node) {
        std::vector<int> bits = Bits(node);
        if (bits.size() != 1)
            throw std::runtime_error("expected one-bit BMC condition");
        return bits.front();
    }

    void EncodeSelectedData(int select,
                            const std::vector<int> &readData,
                            const std::vector<int> &sourceData) {
        if (readData.size() != sourceData.size())
            throw std::runtime_error("memory data width mismatch");
        if (select == CnfFormula::kFalse) return;
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

    void RegisterRootRead(RootRead read) {
        // Enforce congruence only when both reads select this free root.
        // The unroller has already distinguished roots from different frames.
        auto &prior = m_readsByRoot[read.root];
        for (size_t index : prior) {
            const RootRead &previous = m_rootReads[index];
            int selected = m_cnf.And(previous.select, read.select);
            if (selected == CnfFormula::kFalse) continue;
            int sameAddress =
                m_cnf.AddressEqual(previous.address, read.address);
            EncodeSelectedData(m_cnf.And(selected, sameAddress),
                               read.data, previous.data);
        }
        prior.push_back(m_rootReads.size());
        m_rootReads.push_back(std::move(read));
    }

    void EncodeAigGates() {
        const std::vector<WLAigGate> &gates = m_bitblastor.Gates();
        for (const WLAigGate &gate : gates) {
            m_cnf.AddAigAnd(gate.node, gate.child0, gate.child1);
        }
    }

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

    void CompleteArrayValues(WLTraceStep &flat, const RawKissat &kissatEngine) {
        for (int64_t input : m_arrayInputs) {
            const auto &sort = m_ir.Sort(m_ir.Node(input).sortId);
            flat.arrayInputValues[input].defaultValue =
                WLBitVector::Zero(m_ir.Sort(sort.elementSort).width);
        }
        std::map<int64_t, std::map<std::string, WLBitVector>> entriesByRoot;
        for (const RootRead &read : m_rootReads) {
            if (!ModelLiteral(read.select, kissatEngine)) continue;
            const auto address = ModelCnfBits(read.address, kissatEngine);
            const auto data = ModelCnfBits(read.data, kissatEngine);
            auto [entry, inserted] = entriesByRoot[read.root].emplace(address.ToBinary(), data);
            if (!inserted && entry->second != data)
                throw std::runtime_error("SAT model has inconsistent root reads");
        }
        for (auto &[root, entries] : entriesByRoot) {
            auto &value = flat.arrayInputValues.at(root);
            for (auto &[address, data] : entries)
                value.entries.push_back(
                    {WLBitVector::FromBinary(address.size(), address), std::move(data)});
        }
    }

    const Btor2IR &m_ir;
    Log &m_log;
    WLBitblastor m_bitblastor;
    std::unique_ptr<WLBitblastor::ScalarContext> m_scalar;
    int64_t m_bad{0};
    std::vector<int64_t> m_inputs, m_arrayInputs, m_constraints;
    // Synthetic state/init pairs are just the bounded IR's uniform-array syntax.
    std::unordered_map<int64_t, int64_t> m_uniformData;
    size_t m_arrayCount{0};
    std::vector<RootRead> m_rootReads;
    std::unordered_map<int64_t, std::vector<size_t>> m_readsByRoot;
    std::vector<ReadRequest> m_readRequests;
    size_t m_encodedReads{0};
    std::map<int64_t, std::vector<int>> m_inputBits;
    CnfFormula m_cnf;
};

} // namespace

WLMemoryBMC::WLMemoryBMC(WLModel &model,
                         Log &log)
    : m_model(model),
      m_log(log) {}

WLBoundedResult WLMemoryBMC::CheckThrough(unsigned bound) {
    m_trace = {};
    WLBoundedResult result;
    const char *phase = "property preparation";
    unsigned depth = 0;
    try {
        // WLModel validates the full SourceIR before preparing its property cone.
        const Btor2IR &source = m_model.PropertyIR();
        for (;; ++depth) {
            LOG_L(m_log, 1, "WL memory BMC bound ", depth, ":");
            phase = "bounded unrolling";
            WLBoundedUnroller unrolled(source, depth);
            std::unique_ptr<WLArrayEqualityEncoder> equality;
            if (WLArrayEqualityEncoder::HasArrayComparisons(unrolled.IR())) {
                // Equality auxiliaries belong to this finite formula. Rebuild
                // them per query so background witnesses never restrict later bounds.
                phase = "equality elimination";
                equality = std::make_unique<WLArrayEqualityEncoder>(unrolled.IR());
                const auto &stats = equality->Stats();
                LOG_L(m_log, 1, "WL array equality bound ", depth, ": ",
                      stats.comparisons, " comparisons, ", stats.points, " points, ",
                      stats.queries, " address closures, ", stats.implications,
                      " implications; equality-free IR");
            }
            phase = "EMM encoding/query";
            MemoryQuery emm(equality ? equality->IR() : unrolled.IR(), m_log);
            WLTraceStep flat;
            std::map<int64_t, WLBitVector> values;
            const auto query = emm.Check(flat,
                equality ? equality->ModelTerms() : std::vector<int64_t>{},
                values, !equality);
            if (query == MemoryQuery::Result::Sat) {
                if (equality) {
                    phase = "equality model recovery";
                    const auto arrays = equality->Complete(
                        [&](int64_t id) { return values.at(id); });
                    // Retain only ports of the pre-elimination bounded IR.
                    // EMM-private root reads must not constrain this completion.
                    for (const auto &node : unrolled.IR().Nodes())
                        if (node.tag == BTOR2_TAG_input &&
                            unrolled.IR().Sort(node.sortId).tag == BTOR2_TAG_SORT_array)
                            flat.arrayInputValues.emplace(node.id, arrays.at(node.id));
                }
                phase = "trace decoding";
                m_trace = unrolled.DecodeTrace(flat);
                phase = "SourceIR verification";
                m_model.RestoreSourceTrace(m_trace);
                WLSimulator simulator(m_model.SourceIR());
                const auto verified = simulator.Verify(m_trace);
                if (verified.kind != WLSimulator::VerificationKind::Confirmed)
                    throw std::runtime_error(
                        "concrete replay failed at frame " + std::to_string(verified.time) +
                        " (node " + std::to_string(verified.nodeId) + "): " + verified.reason);
                LOG_L(m_log, 1, "WL memory BMC concrete replay confirmed at depth ", depth);
                result.status = WLBoundedStatus::Counterexample;
                result.badDepth = depth;
                return result;
            }
            if (query == MemoryQuery::Result::Unknown) {
                result.reason = "SAT solver did not complete depth " +
                                std::to_string(depth);
                return result;
            }
            result.checkedThrough = depth;
            if (depth == bound) break;
        }
    } catch (const std::exception &error) {
        m_trace = {};
        result.reason = std::string("memory BMC / ") + phase + " at depth " +
                        std::to_string(depth) + ": " + error.what();
        return result;
    }

    result.status = WLBoundedStatus::PrefixSafe;
    LOG_L(m_log,
          1,
          "WL memory BMC found no counterexample through bound ",
          bound);
    return result;
}

} // namespace car
