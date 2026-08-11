#include "WLBoolectorBitblast.h"

#include "CircuitGraph.h"

#include <boolector/boolector.h>

#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace car {
namespace {

using UnaryFn = BoolectorNode *(*)(Btor *, BoolectorNode *);
using BinaryFn = BoolectorNode *(*)(Btor *,
                                    BoolectorNode *,
                                    BoolectorNode *);

const std::unordered_map<Btor2Tag, UnaryFn> kUnaryOps{
    {BTOR2_TAG_dec, boolector_dec},
    {BTOR2_TAG_inc, boolector_inc},
    {BTOR2_TAG_neg, boolector_neg},
    {BTOR2_TAG_not, boolector_not},
    {BTOR2_TAG_redand, boolector_redand},
    {BTOR2_TAG_redor, boolector_redor},
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

class BoolectorModel {
  public:
    BoolectorModel() : btor(boolector_new()) {}
    ~BoolectorModel() {
        boolector_release_all(btor);
        boolector_delete(btor);
    }

    void AddNode(int64_t id, BoolectorNode *node) {
        if (!nodes.emplace(id, node).second) {
            throw std::runtime_error("duplicate word-level node id");
        }
    }

    void SetTraceSource(BoolectorNode *node,
                        const WLValueOrigin &source) {
        // Keep word-level provenance attached to the complete variable.
        traceSources[node] = source;
    }

    Btor *btor;
    std::vector<BoolectorNode *> inputs;
    std::vector<std::pair<int64_t, BoolectorNode *>> states;
    std::unordered_map<BoolectorNode *, WLValueOrigin> traceSources;
    std::unordered_map<int64_t, BoolectorNode *> init;
    std::unordered_map<int64_t, BoolectorNode *> next;
    BoolectorNode *bad{nullptr};
    std::vector<BoolectorNode *> constraints;
    std::unordered_map<int64_t, BoolectorNode *> nodes;
    std::unordered_map<int64_t, BoolectorSort> sorts;
};

class Lowering {
  public:
    Lowering(const Btor2IR &ir, const WLIRTraceMap &traceSources)
        : m_ir(ir), m_traceSources(traceSources) {
        if (m_ir.HasArrays()) {
            throw std::runtime_error(
                "standard word-level bitblasting requires array-free IR");
        }
        // Lower the optimized IR in dependency order into a Boolector transition system.
        CreateVariables();
        BuildExpressions();
        BuildTransitionSystem();
    }

    BoolectorModel &Model() { return m_model; }

  private:
    BoolectorSort Sort(int64_t sortId) {
        // Lazily create Boolector sorts from the final widths stored in the IR.
        auto it = m_model.sorts.find(sortId);
        if (it != m_model.sorts.end()) return it->second;
        const Btor2IRSort &sort = m_ir.Sort(sortId);
        if (sort.tag != BTOR2_TAG_SORT_bitvec) {
            throw std::runtime_error(
                "standard bitblasting encountered a non-bit-vector sort");
        }
        BoolectorSort result =
            boolector_bitvec_sort(m_model.btor, sort.width);
        m_model.sorts.emplace(sortId, result);
        return result;
    }

    void CreateVariables() {
        // Inputs and states are the only primary Boolector variables.
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            if (node.tag != BTOR2_TAG_input && node.tag != BTOR2_TAG_state)
                continue;
            const char *symbol =
                node.symbol.empty() ? nullptr : node.symbol.c_str();
            BoolectorNode *var =
                boolector_var(m_model.btor, Sort(node.sortId), symbol);
            m_model.AddNode(node.id, var);
            auto trace = m_traceSources.find(node.id);
            if (trace != m_traceSources.end()) {
                m_model.SetTraceSource(var, trace->second);
            }
            if (node.tag == BTOR2_TAG_input)
                m_model.inputs.push_back(var);
            else
                m_model.states.emplace_back(node.id, var);
        }
    }

    void BuildExpressions() {
        // Force construction of all combinational nodes before transition metadata.
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_input:
            case BTOR2_TAG_state:
            case BTOR2_TAG_init:
            case BTOR2_TAG_next:
            case BTOR2_TAG_bad:
            case BTOR2_TAG_constraint:
            case BTOR2_TAG_output:
            case BTOR2_TAG_fair:
            case BTOR2_TAG_justice:
                continue;
            default: Evaluate(node.id); break;
            }
        }
    }

    void BuildTransitionSystem() {
        // Attach init/next functions, bad properties, and constraints to variables.
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_init:
                m_model.init[node.args[0]] = Evaluate(node.args[1]);
                break;
            case BTOR2_TAG_next:
                m_model.next[node.args[0]] = Evaluate(node.args[1]);
                break;
            case BTOR2_TAG_bad:
                m_model.bad = Evaluate(node.args[0]);
                break;
            case BTOR2_TAG_constraint:
                m_model.constraints.push_back(Evaluate(node.args[0]));
                break;
            default: break;
            }
        }
    }

    BoolectorNode *Evaluate(int64_t signedId) {
        // Recursively lower scalar expressions with memoization and signed-ID inversion.
        auto cached = m_model.nodes.find(signedId);
        if (cached != m_model.nodes.end()) return cached->second;
        if (signedId < 0) {
            BoolectorNode *result =
                boolector_not(m_model.btor, Evaluate(-signedId));
            m_model.AddNode(signedId, result);
            return result;
        }

        const Btor2IRNode &node = m_ir.Node(signedId);
        BoolectorNode *result = nullptr;
        auto arg = [&](size_t index) {
            return Evaluate(node.args[index]);
        };
        switch (node.tag) {
        case BTOR2_TAG_const:
            result =
                boolector_const(m_model.btor, node.constant.c_str());
            break;
        case BTOR2_TAG_constd:
            result = boolector_constd(
                m_model.btor, Sort(node.sortId), node.constant.c_str());
            break;
        case BTOR2_TAG_consth:
            result = boolector_consth(
                m_model.btor, Sort(node.sortId), node.constant.c_str());
            break;
        case BTOR2_TAG_zero:
            result = boolector_zero(m_model.btor, Sort(node.sortId));
            break;
        case BTOR2_TAG_one:
            result = boolector_one(m_model.btor, Sort(node.sortId));
            break;
        case BTOR2_TAG_ones:
            result = boolector_ones(m_model.btor, Sort(node.sortId));
            break;
        case BTOR2_TAG_slice:
            result = boolector_slice(
                m_model.btor, arg(0), node.args[1], node.args[2]);
            break;
        case BTOR2_TAG_uext:
            result = boolector_uext(
                m_model.btor, arg(0), node.args[1]);
            break;
        case BTOR2_TAG_sext:
            result = boolector_sext(
                m_model.btor, arg(0), node.args[1]);
            break;
        case BTOR2_TAG_ite:
            result = boolector_cond(
                m_model.btor, arg(0), arg(1), arg(2));
            break;
        default:
            if (node.nargs == 1) {
                auto it = kUnaryOps.find(node.tag);
                if (it != kUnaryOps.end())
                    result = it->second(m_model.btor, arg(0));
            } else if (node.nargs == 2) {
                auto it = kBinaryOps.find(node.tag);
                if (it != kBinaryOps.end())
                    result =
                        it->second(m_model.btor, arg(0), arg(1));
            }
            break;
        }
        if (!result) {
            throw std::runtime_error(
                "unsupported scalar word-level node " +
                std::to_string(node.id));
        }
        m_model.AddNode(node.id, result);
        return result;
    }

    const Btor2IR &m_ir;
    const WLIRTraceMap &m_traceSources;
    BoolectorModel m_model;
};

struct AigVisitor {
    aiger *aig;
    std::unordered_set<uint64_t> visited;
};

void VisitAig(void *state,
              bool isPost,
              uint64_t node,
              const char *,
              uint64_t child0,
              uint64_t child1) {
    // Boolector visits gates post-order; emit each AIG AND node exactly once.
    if (!isPost || !child0) return;
    auto *visitor = static_cast<AigVisitor *>(state);
    if (!visitor->visited.insert(node).second) return;
    aiger_add_and(visitor->aig, node, child0, child1);
}

struct PendingTraceSpan {
    WLValueOrigin origin;
    uint32_t firstInterfaceIndex{0};
    uint32_t encodedWidth{0};
};

void RecordTraceSpan(
    BoolectorNode *node,
    uint32_t firstInterfaceIndex,
    uint32_t width,
    const std::unordered_map<BoolectorNode *, WLValueOrigin> &traceSources,
    std::vector<PendingTraceSpan> &spans) {
    auto trace = traceSources.find(node);
    if (trace == traceSources.end()) return;
    spans.push_back({trace->second, firstInterfaceIndex, width});
}

void AddInput(
    Btor *btor,
    BoolectorAIGMgr *manager,
    aiger *aig,
    BoolectorNode *input,
    std::vector<PendingTraceSpan> &inputSpans,
    const std::unordered_map<BoolectorNode *, WLValueOrigin> &traceSources) {
    // Emit each word-level input contiguously from logical LSB to MSB.
    const size_t width = boolector_get_width(btor, input);
    const uint32_t firstInput = aig->num_inputs;
    uint64_t *bits = boolector_aig_get_bits(manager, input);
    for (size_t bit = 0; bit < width; ++bit) {
        const size_t i = width - bit - 1;
        aiger_add_input(aig,
                        bits[i],
                        boolector_aig_get_symbol(manager, bits[i])
                            ? boolector_aig_get_symbol(manager, bits[i])
                            : "");
    }
    RecordTraceSpan(input,
                    firstInput,
                    static_cast<uint32_t>(width),
                    traceSources,
                    inputSpans);
    boolector_aig_free_bits(manager, bits, width);
}

std::vector<WLTraceSpan>
FinalizeTraceSpans(const aiger_symbol *symbols,
                   uint32_t symbolCount,
                   const std::vector<PendingTraceSpan> &pending) {
    // Resolve interface offsets after AIGER reencoding and verify contiguity.
    std::vector<WLTraceSpan> result;
    result.reserve(pending.size());
    for (const PendingTraceSpan &span : pending) {
        if (!span.encodedWidth ||
            span.firstInterfaceIndex + span.encodedWidth > symbolCount) {
            throw std::runtime_error("invalid AIGER trace span");
        }
        const uint32_t firstVar =
            symbols[span.firstInterfaceIndex].lit / 2;
        for (uint32_t bit = 0; bit < span.encodedWidth; ++bit) {
            const uint32_t var =
                symbols[span.firstInterfaceIndex + bit].lit / 2;
            if (var != firstVar + bit) {
                throw std::runtime_error(
                    "AIGER trace span is not contiguous in LSB-first order");
            }
        }
        const uint32_t originalWidth =
            span.origin.originalSegmentWidth
                ? span.origin.originalSegmentWidth
                : span.encodedWidth;
        WLValueOrigin origin = span.origin;
        origin.originalSegmentWidth = originalWidth;
        result.push_back(
            {std::move(origin), firstVar, span.encodedWidth});
    }
    return result;
}

unsigned MakeAnd(aiger *aig, unsigned lhs, unsigned rhs) {
    unsigned result = (aig->maxvar + 1) * 2;
    aiger_add_and(aig, result, lhs, rhs);
    return result;
}

unsigned MakeEq(aiger *aig, unsigned lhs, unsigned rhs) {
    unsigned bothTrue = MakeAnd(aig, lhs, rhs);
    unsigned bothFalse = MakeAnd(aig, aiger_not(lhs), aiger_not(rhs));
    return aiger_not(
        MakeAnd(aig, aiger_not(bothTrue), aiger_not(bothFalse)));
}

std::shared_ptr<aiger> Bitblast(BoolectorModel &model,
                               WLTraceMap &traceMap) {
    // Bitblast the complete Boolector transition system into one in-memory AIGER.
    std::shared_ptr<aiger> result(aiger_init(), AigerDeleter);
    aiger *aig = result.get();
    BoolectorAIGMgr *manager = boolector_aig_new(model.btor);
    AigVisitor visitor{aig, {}};
    std::vector<std::pair<uint64_t, uint64_t>> symbolicInits;
    std::vector<PendingTraceSpan> inputSpans;
    std::vector<PendingTraceSpan> latchSpans;

    auto bitblast = [&](BoolectorNode *node) {
        boolector_aig_bitblast(manager, node);
        boolector_aig_visit(manager, node, VisitAig, &visitor);
    };

    // Emit primary input bits before latches to establish stable CNF variable order.
    for (BoolectorNode *input : model.inputs) {
        boolector_aig_bitblast(manager, input);
        AddInput(model.btor,
                 manager,
                 aig,
                 input,
                 inputSpans,
                 model.traceSources);
    }
    // Emit state bits as latches, or as inputs when no next function is defined.
    for (const auto &[id, state] : model.states) {
        (void)id;
        boolector_aig_bitblast(manager, state);
    }
    for (const auto &[id, init] : model.init) {
        (void)id;
        bitblast(init);
    }
    for (const auto &[id, next] : model.next) {
        (void)id;
        bitblast(next);
    }

    for (const auto &[id, state] : model.states) {
        BoolectorNode *next =
            model.next.count(id) ? model.next.at(id) : nullptr;
        BoolectorNode *init =
            model.init.count(id) ? model.init.at(id) : nullptr;
        const size_t width = boolector_get_width(model.btor, state);
        uint64_t *stateBits = boolector_aig_get_bits(manager, state);
        uint64_t *nextBits =
            next ? boolector_aig_get_bits(manager, next) : nullptr;
        uint64_t *initBits =
            init ? boolector_aig_get_bits(manager, init) : nullptr;
        const uint32_t firstInput = aig->num_inputs;
        const uint32_t firstLatch = aig->num_latches;
        for (size_t bit = 0; bit < width; ++bit) {
            const size_t i = width - bit - 1;
            const char *name =
                boolector_aig_get_symbol(manager, stateBits[i]);
            if (!nextBits) {
                aiger_add_input(aig, stateBits[i], name ? name : "");
                // A state without next is a per-step nondeterministic input;
                // its optional init still constrains the initial step.
                if (initBits)
                    symbolicInits.emplace_back(stateBits[i], initBits[i]);
                continue;
            }
            aiger_add_latch(aig, stateBits[i], nextBits[i], name ? name : "");
            if (!initBits || initBits[i] == 0 || initBits[i] == 1) {
                aiger_add_reset(
                    aig, stateBits[i], initBits ? initBits[i] : stateBits[i]);
            } else {
                aiger_add_reset(aig, stateBits[i], stateBits[i]);
                symbolicInits.emplace_back(stateBits[i], initBits[i]);
            }
        }
        RecordTraceSpan(state,
                        next ? firstLatch : firstInput,
                        static_cast<uint32_t>(width),
                        model.traceSources,
                        next ? latchSpans : inputSpans);
        boolector_aig_free_bits(manager, stateBits, width);
        if (nextBits) boolector_aig_free_bits(manager, nextBits, width);
        if (initBits) boolector_aig_free_bits(manager, initBits, width);
    }

    // Properties and constraints are single-bit Boolector expressions.
    for (BoolectorNode *constraint : model.constraints) {
        bitblast(constraint);
        uint64_t *bits = boolector_aig_get_bits(manager, constraint);
        aiger_add_constraint(aig, bits[0], "");
        boolector_aig_free_bits(manager, bits, 1);
    }
    bitblast(model.bad);
    uint64_t *badBits = boolector_aig_get_bits(manager, model.bad);
    aiger_add_bad(aig, badBits[0], "");
    boolector_aig_free_bits(manager, badBits, 1);

    // Encode symbolic resets as constraints active only in the initial step.
    if (!symbolicInits.empty()) {
        unsigned initialStep = (aig->maxvar + 1) * 2;
        aiger_add_latch(aig, initialStep, 0, "btor2.initial_step");
        aiger_add_reset(aig, initialStep, 1);
        for (const auto &[stateBit, initBit] : symbolicInits) {
            unsigned equal = MakeEq(aig, stateBit, initBit);
            unsigned constraint =
                aiger_not(MakeAnd(aig, initialStep, aiger_not(equal)));
            aiger_add_constraint(aig, constraint, "");
        }
    }

    boolector_aig_delete(manager);
    if (const char *error = aiger_check(aig)) {
        throw std::runtime_error(std::string("generated AIG is invalid: ") +
                                 error);
    }
    if (!aiger_is_reencoded(aig)) aiger_reencode(aig);

    // Store one final AIGER range for each word-level segment.
    traceMap.inputSpans = FinalizeTraceSpans(
        aig->inputs, aig->num_inputs, inputSpans);
    traceMap.latchSpans = FinalizeTraceSpans(
        aig->latches, aig->num_latches, latchSpans);
    return result;
}

} // namespace

std::shared_ptr<aiger> GenerateWLAig(const Btor2IR &ir,
                                     const WLIRTraceMap &traceSources,
                                     WLTraceMap &traceMap) {
    Lowering lowering(ir, traceSources);
    return Bitblast(lowering.Model(), traceMap);
}

} // namespace car
