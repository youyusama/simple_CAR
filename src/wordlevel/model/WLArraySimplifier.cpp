#include "WLArraySimplifier.h"

#include "WLSimulator.h"
#include <algorithm>
#include <cstdlib>
#include <set>
#include <sstream>
#include <unordered_set>

namespace car {
namespace {
using Definitions = std::map<int64_t, int64_t>;
struct DefinitionCycle : std::runtime_error {
    DefinitionCycle() : std::runtime_error("cyclic array definition") {}
};

bool Statement(Btor2Tag tag) {
    return tag == BTOR2_TAG_init || tag == BTOR2_TAG_next ||
           tag == BTOR2_TAG_bad || tag == BTOR2_TAG_constraint;
}

size_t Comparisons(const Btor2IR &ir) {
    size_t count = 0;
    for (const auto &n : ir.Nodes())
        if ((n.tag == BTOR2_TAG_eq || n.tag == BTOR2_TAG_neq) &&
            ir.Sort(ir.Node(std::abs(n.args[0])).sortId).tag == BTOR2_TAG_SORT_array)
            ++count;
    return count;
}

// A summary of a positive Boolean subformula F. Under residual, definitions
// satisfy F; conversely every satisfying assignment of F is preserved by the
// definitions (fallback ports take the old input values). No DNF is formed.
struct Summary {
    int64_t residual;
    Definitions definitions;
};

class Builder {
  public:
    explicit Builder(const Btor2IR &source) : ir(source) {
        for (const auto &n : source.Nodes()) {
            if (n.tag == BTOR2_TAG_init) init[n.args[0]] = n.args[1];
            if (n.tag == BTOR2_TAG_next) next[n.args[0]] = n.args[1];
            if (Statement(n.tag)) statements.push_back(n);
            if (n.tag == BTOR2_TAG_state) states.push_back(n.id);
            if (n.tag != BTOR2_TAG_input && n.tag != BTOR2_TAG_state && !Statement(n.tag))
                intern.emplace(Key(n), n.id);
        }
        Btor2IRSort sort;
        sort.id = ir.FreshId(); sort.width = 1;
        boolean = ir.AddSort(sort);
        Btor2IRNode node;
        node.tag = BTOR2_TAG_one; node.sortId = boolean;
        yes = Intern(node);
    }

    Btor2IR ir;
    Definitions definitions;
    std::vector<int64_t> privateInputs;
    std::set<std::string> skippedRules;
    std::string context{"algebraic"};

    void Run() {
        // First normalize expressions, including signed Boolean literals. Port
        // and statement IDs remain stable; only expression nodes are interned.
        Rewriter algebra(*this, {});
        for (auto &n : statements) {
            const unsigned arg = (n.tag == BTOR2_TAG_init || n.tag == BTOR2_TAG_next) ? 1 : 0;
            n.args[arg] = algebra(n.args[arg]);
            if (n.tag == BTOR2_TAG_init) init[n.args[0]] = n.args[1];
            if (n.tag == BTOR2_TAG_next) next[n.args[0]] = n.args[1];
        }
        std::vector<int64_t> constraints;
        for (const auto &n : statements)
            if (n.tag == BTOR2_TAG_constraint) constraints.push_back(n.args[0]);
        if (!constraints.empty()) {
            auto summary = Analyze(Fold(BTOR2_TAG_and, constraints));
            if (!summary.definitions.empty() && CloseDefinitions(summary) && InitialAcyclic(summary.definitions)) {
                definitions = std::move(summary.definitions);
                Apply(definitions);
                // All original constraints were conjuncts of the summary.
                bool first = true;
                for (auto &n : statements) if (n.tag == BTOR2_TAG_constraint) {
                    n.args[0] = first ? summary.residual : yes;
                    first = false;
                }
                context = "constraints";
                return;
            }
        }
        FunctionalizeValid();
    }

    Btor2IR Output(bool recovery) const {
        Btor2IR output;
        output.CopySortsFrom(ir);
        output.ReserveFreshIdsAfter(ir);
        std::set<int64_t> live;
        std::vector<int64_t> todo = states;
        for (const auto &n : statements) {
            for (unsigned i = 0; i < n.nargs; ++i) todo.push_back(n.args[i]);
        }
        if (recovery)
            for (const auto &[port, expression] : definitions) todo.push_back(expression);
        while (!todo.empty()) {
            const auto id = std::abs(todo.back()); todo.pop_back();
            if (!live.insert(id).second) continue;
            const auto &n = ir.Node(id);
            for (unsigned i = 0; i < n.nargs; ++i) todo.push_back(n.args[i]);
        }
        for (const auto &n : ir.Nodes())
            if (live.count(n.id) && !Statement(n.tag)) output.AddNode(n);
        for (const auto &n : statements) output.AddNode(n);
        return output;
    }

  private:
    int64_t boolean, yes;
    Definitions init, next, raw;
    std::vector<int64_t> states;
    std::vector<Btor2IRNode> statements;
    std::unordered_map<std::string, int64_t> intern;
    std::map<int64_t, Summary> summaries;

    static std::string Key(const Btor2IRNode &n) {
        std::ostringstream s;
        s << n.tag << ':' << n.sortId << ':' << n.nargs;
        // args beyond nargs contain extension widths / slice bounds.
        for (auto a : n.args) s << ':' << a;
        s << ':' << n.constant;
        return s.str();
    }
    bool Array(int64_t id) const {
        return ir.Sort(ir.Node(std::abs(id)).sortId).tag == BTOR2_TAG_SORT_array;
    }
    bool Boolean(int64_t id) const {
        const auto &s = ir.Sort(ir.Node(std::abs(id)).sortId);
        return s.tag == BTOR2_TAG_SORT_bitvec && s.width == 1;
    }
    int64_t Intern(Btor2IRNode n) {
        auto key = Key(n);
        auto found = intern.find(key);
        if (found != intern.end()) return found->second;
        n.id = ir.FreshId();
        n.symbol.clear();
        ir.AddNode(n);
        intern.emplace(std::move(key), n.id);
        return n.id;
    }
    int64_t Make(Btor2IRNode n) {
        auto a = n.args[0], b = n.args[1], c = n.args[2];
        if (n.tag == BTOR2_TAG_neq) {
            n.tag = BTOR2_TAG_eq;
            return -Make(std::move(n));
        }
        if (n.sortId == boolean) {
            if (n.tag == BTOR2_TAG_zero) return -yes;
            if (n.tag == BTOR2_TAG_one || n.tag == BTOR2_TAG_ones) return yes;
            if (n.tag == BTOR2_TAG_const) return WLBitVector::FromBinary(1, n.constant).IsZero() ? -yes : yes;
            if (n.tag == BTOR2_TAG_constd) return WLBitVector::FromDecimal(1, n.constant).IsZero() ? -yes : yes;
            if (n.tag == BTOR2_TAG_consth) return WLBitVector::FromHex(1, n.constant).IsZero() ? -yes : yes;
            if (n.tag == BTOR2_TAG_not) return -a;
            if (n.tag == BTOR2_TAG_and || n.tag == BTOR2_TAG_or) {
                const bool conjunction = n.tag == BTOR2_TAG_and;
                const auto unit = conjunction ? yes : -yes;
                if (a == unit) return b;
                if (b == unit || a == b) return a;
                if (a == -unit || b == -unit || a == -b) return -unit;
            }
        }
        if (n.tag == BTOR2_TAG_ite) {
            if (a == yes || b == c) return b;
            if (a == -yes) return c;
            if (Boolean(b) && b == yes && c == -yes) return a;
            if (Boolean(b) && b == -yes && c == yes) return -a;
        }
        if (n.tag == BTOR2_TAG_read && a > 0) {
            const auto base = ir.Node(a);
            if (base.tag == BTOR2_TAG_write && base.args[1] == b) return base.args[2];
        }
        if (n.tag == BTOR2_TAG_write) {
            const auto base = ir.Node(a), value = ir.Node(std::abs(c));
            if (c > 0 && value.tag == BTOR2_TAG_read && value.args[0] == a && value.args[1] == b)
                return a;
            if (base.tag == BTOR2_TAG_write && base.args[1] == b) n.args[0] = base.args[0];
        }
        if (n.tag == BTOR2_TAG_eq) {
            if (a == b) return yes;
            if (Array(a)) {
                auto left = ir.Node(a), right = ir.Node(b);
                if (left.tag != BTOR2_TAG_write && right.tag == BTOR2_TAG_write) {
                    std::swap(a, b); std::swap(left, right);
                }
                if (left.tag == BTOR2_TAG_write && left.args[0] == b) {
                    const auto &s = ir.Sort(ir.Node(b).sortId);
                    b = Op(BTOR2_TAG_read, s.elementSort, {b, left.args[1]});
                    a = left.args[2];
                } else if (left.tag == BTOR2_TAG_write && right.tag == BTOR2_TAG_write &&
                           left.args[0] == right.args[0] && left.args[1] == right.args[1]) {
                    a = left.args[2]; b = right.args[2];
                }
                if (a == b) return yes;
                n.args[0] = a; n.args[1] = b;
            }
        }
        if (n.tag == BTOR2_TAG_and || n.tag == BTOR2_TAG_or ||
            n.tag == BTOR2_TAG_eq)
            if (n.args[0] > n.args[1]) std::swap(n.args[0], n.args[1]);
        return Intern(std::move(n));
    }
    int64_t Op(Btor2Tag tag, int64_t sort, std::initializer_list<int64_t> args) {
        Btor2IRNode n;
        n.tag = tag; n.sortId = sort; n.nargs = args.size();
        std::copy(args.begin(), args.end(), n.args.begin());
        return Make(std::move(n));
    }
    int64_t Fold(Btor2Tag tag, const std::vector<int64_t> &args) {
        int64_t result = tag == BTOR2_TAG_and ? yes : -yes;
        for (auto a : args) result = Op(tag, boolean, {result, a});
        return result;
    }

    // Iterative substitution and hash-consing. State/port boundaries stop DAG
    // traversal; state initialization cycles are checked separately below.
    class Rewriter {
      public:
        Rewriter(Builder &b, const Definitions &d) : b(b), substitutions(d) {}
        int64_t operator()(int64_t root) {
            std::vector<int64_t> todo{std::abs(root)};
            std::unordered_set<int64_t> pending;
            while (!todo.empty()) {
                auto id = todo.back();
                if (cache.count(id)) { todo.pop_back(); continue; }
                const auto replacement = substitutions.find(id);
                const auto n = b.ir.Node(id);
                std::vector<int64_t> dependencies;
                if (replacement != substitutions.end()) dependencies.push_back(replacement->second);
                else for (unsigned i = 0; i < n.nargs; ++i) dependencies.push_back(n.args[i]);
                if (pending.insert(id).second) {
                    for (auto child : dependencies) {
                        child = std::abs(child);
                        if (pending.count(child)) throw DefinitionCycle();
                        if (!cache.count(child)) todo.push_back(child);
                    }
                    continue;
                }
                auto lookup = [&](int64_t child) { return (child < 0 ? -1 : 1) * cache.at(std::abs(child)); };
                int64_t value;
                if (replacement != substitutions.end()) value = lookup(replacement->second);
                else if (n.tag == BTOR2_TAG_input || n.tag == BTOR2_TAG_state) value = id;
                else {
                    auto copy = n;
                    for (unsigned i = 0; i < n.nargs; ++i) copy.args[i] = lookup(n.args[i]);
                    value = b.Make(std::move(copy));
                }
                cache[id] = value;
                pending.erase(id); todo.pop_back();
            }
            return (root < 0 ? -1 : 1) * cache.at(std::abs(root));
        }
      private:
        Builder &b;
        Definitions substitutions;
        std::unordered_map<int64_t, int64_t> cache;
    };

    std::vector<int64_t> Flatten(int64_t root, Btor2Tag tag) const {
        std::vector<int64_t> result, todo{root};
        std::unordered_set<int64_t> seen;
        while (!todo.empty()) {
            auto id = todo.back(); todo.pop_back();
            if (!seen.insert(id).second) continue;
            const auto &n = ir.Node(std::abs(id));
            if (id > 0 && n.tag == tag) {
                todo.push_back(n.args[1]); todo.push_back(n.args[0]);
            } else result.push_back(id);
        }
        return result;
    }
    bool Depends(int64_t root, const Definitions &ports) const {
        std::vector<int64_t> todo{root};
        std::unordered_set<int64_t> seen;
        while (!todo.empty()) {
            auto id = std::abs(todo.back()); todo.pop_back();
            if (ports.count(id)) return true;
            if (!seen.insert(id).second) continue;
            const auto &n = ir.Node(id);
            for (unsigned i = 0; i < n.nargs; ++i) todo.push_back(n.args[i]);
        }
        return false;
    }
    int64_t Raw(int64_t port) {
        auto found = raw.find(port);
        if (found != raw.end()) return found->second;
        auto n = ir.Node(port);
        n.id = ir.FreshId(); n.symbol = "wl.array.fallback." + std::to_string(port);
        ir.AddNode(n); raw.emplace(port, n.id);
        privateInputs.push_back(n.id);
        return n.id;
    }
    Summary Leaf(int64_t id) {
        Summary result{id, {}};
        if (id < 0) return result;
        const auto n = ir.Node(id);
        if (n.tag != BTOR2_TAG_eq || !Array(n.args[0])) return result;
        auto a = n.args[0], b = n.args[1];
        // Orient aliases consistently. Never define state variables.
        if (a < b) std::swap(a, b);
        if (ir.Node(a).tag != BTOR2_TAG_input) std::swap(a, b);
        if (ir.Node(a).tag == BTOR2_TAG_input && !Depends(b, {{a, b}})) {
            result.residual = yes; result.definitions.emplace(a, b);
        }
        return result;
    }
    Summary Conjunction(int64_t root, const std::vector<int64_t> &children) {
        Summary result{root, {}};
        std::vector<int64_t> residuals;
        for (auto child : children) {
            const auto &s = summaries.at(child);
            residuals.push_back(s.residual);
            for (const auto &[port, value] : s.definitions) {
                auto [it, inserted] = result.definitions.emplace(port, value);
                // Conflicting definitions remain relational. Do not guess an
                // ordering that would silently drop one of the conjuncts.
                if (!inserted && it->second != value) {
                    skippedRules.insert("conflicting-definitions");
                    return {root, {}};
                }
            }
        }
        if (result.definitions.empty()) return result;
        result.residual = Fold(BTOR2_TAG_and, residuals);
        return CloseDefinitions(result) ? result : Summary{root, {}};
    }
    bool CloseDefinitions(Summary &summary) {
        // OR merges can introduce dependencies between different branches.
        // Encoding and trace recovery must use the very same expanded terms.
        try {
            Rewriter expand(*this, summary.definitions);
            for (auto &[port, value] : summary.definitions) value = expand(port);
            summary.residual = expand(summary.residual);
            return true;
        } catch (const DefinitionCycle &) {
            skippedRules.insert("cyclic-definitions");
            return false;
        }
    }
    Summary Disjunction(int64_t root, const std::vector<int64_t> &children) {
        Definitions ports;
        std::vector<int64_t> guards;
        std::vector<std::set<int64_t>> literals;
        for (auto child : children) {
            const auto &s = summaries.at(child);
            ports.insert(s.definitions.begin(), s.definitions.end());
            guards.push_back(s.residual);
            auto flat = Flatten(s.residual, BTOR2_TAG_and);
            literals.emplace_back(flat.begin(), flat.end());
        }
        if (ports.empty()) return {root, {}};
        for (auto g : guards) if (Depends(g, ports)) {
            skippedRules.insert("guard-depends-on-defined-input");
            return {root, {}};
        }
        const auto value = [&](size_t i, int64_t port) {
            const auto &d = summaries.at(children[i]).definitions;
            auto found = d.find(port);
            return found == d.end() ? Raw(port) : found->second;
        };
        for (size_t i = 0; i < children.size(); ++i) {
            for (size_t j = 0; j < i; ++j) {
                bool disjoint = guards[i] == -yes || guards[j] == -yes;
                for (auto literal : literals[i]) disjoint |= literals[j].count(-literal) != 0;
                // Flattening (a & b) loses its parent ID. Still recognize the
                // complementary branch !(a & b) from the conjuncts a and b.
                auto both = literals[i];
                both.insert(literals[j].begin(), literals[j].end());
                for (auto literal : both) {
                    if (literal >= 0 || ir.Node(-literal).tag != BTOR2_TAG_and) continue;
                    auto conjuncts = Flatten(-literal, BTOR2_TAG_and);
                    disjoint |= std::all_of(conjuncts.begin(), conjuncts.end(),
                        [&](int64_t term) { return both.count(term) != 0; });
                }
                if (disjoint) continue;
                for (const auto &[port, unused] : ports)
                    if (value(i, port) != value(j, port)) {
                        skippedRules.insert("unproved-branch-compatibility");
                        return {root, {}};
                    }
            }
        }
        Summary result{Fold(BTOR2_TAG_or, guards), {}};
        for (const auto &[port, unused] : ports) {
            auto expression = Raw(port);
            // Last guard need not hold: residual already requires some branch.
            // Keeping the fallback also preserves values off the valid path.
            for (size_t i = children.size(); i-- > 0;)
                expression = Op(BTOR2_TAG_ite, ir.Node(port).sortId,
                                {guards[i], value(i, port), expression});
            result.definitions.emplace(port, expression);
        }
        return result;
    }
    Summary Analyze(int64_t root) {
        std::vector<std::pair<int64_t, bool>> todo{{root, false}};
        while (!todo.empty()) {
            auto [id, finish] = todo.back(); todo.pop_back();
            if (summaries.count(id)) continue;
            const auto tag = ir.Node(std::abs(id)).tag;
            if (id < 0 || (tag != BTOR2_TAG_and && tag != BTOR2_TAG_or)) {
                summaries.emplace(id, Leaf(id)); continue;
            }
            auto children = Flatten(id, tag);
            if (!finish) {
                todo.emplace_back(id, true);
                for (auto child : children) if (!summaries.count(child)) todo.emplace_back(child, false);
                continue;
            }
            summaries.emplace(id, tag == BTOR2_TAG_and ? Conjunction(id, children) : Disjunction(id, children));
        }
        return summaries.at(root);
    }
    bool InitialAcyclic(const Definitions &d) {
        // Include init dependencies, not next: an input defined by a state
        // initialized from that same input is an algebraic initialization loop.
        auto initial = d;
        initial.insert(init.begin(), init.end());
        try {
            Rewriter check(*this, initial);
            for (const auto &[port, unused] : d) check(port);
            return true;
        } catch (const DefinitionCycle &) {
            skippedRules.insert("initialization-cycle");
            return false;
        }
    }
    void Apply(const Definitions &d) {
        Rewriter rewrite(*this, d);
        for (auto &n : statements) {
            const unsigned arg = (n.tag == BTOR2_TAG_init || n.tag == BTOR2_TAG_next) ? 1 : 0;
            n.args[arg] = rewrite(n.args[arg]);
        }
    }
    void FunctionalizeValid() {
        // Structural recognition only: startup latch r'=1, r0=0; v0=0;
        // v'|r=1 = v & T; every bad requires v. No symbol-name assumptions.
        for (auto reset : states) {
            if (!Boolean(reset) || !init.count(reset) || init.at(reset) != -yes ||
                !next.count(reset) || next.at(reset) != yes) continue;
            for (auto valid : states) {
                if (valid == reset || !Boolean(valid) || !init.count(valid) ||
                    init.at(valid) != -yes || !next.count(valid)) continue;
                bool guarded = true, hasBad = false;
                for (const auto &n : statements) if (n.tag == BTOR2_TAG_bad) {
                    hasBad = true;
                    auto leaves = Flatten(n.args[0], BTOR2_TAG_and);
                    guarded &= std::find(leaves.begin(), leaves.end(), valid) != leaves.end();
                }
                if (!guarded || !hasBad) continue;
                Rewriter active(*this, {{reset, yes}}), startup(*this, {{reset, -yes}});
                auto factors = Flatten(active(next.at(valid)), BTOR2_TAG_and);
                auto v = std::find(factors.begin(), factors.end(), valid);
                if (v == factors.end()) continue;
                factors.erase(v);
                auto summary = Analyze(Fold(BTOR2_TAG_and, factors));
                if (summary.definitions.empty() || !CloseDefinitions(summary)) continue;
                // At the final bad frame T is not required. Global substitution
                // is allowed only if terminal observations/constraints do not
                // depend on the eliminated input choices. Init is kept outside
                // the rewrite too; this also excludes new initialization loops.
                bool escapes = false;
                for (const auto &n : statements)
                    if (n.tag == BTOR2_TAG_bad || n.tag == BTOR2_TAG_constraint || n.tag == BTOR2_TAG_init)
                        escapes |= Depends(n.args[n.tag == BTOR2_TAG_init ? 1 : 0], summary.definitions);
                if (escapes) {
                    skippedRules.insert("definition-escapes-valid-scope");
                    continue;
                }
                auto initial = startup(next.at(valid));
                auto enabled = Op(BTOR2_TAG_and, boolean, {reset, valid});
                definitions = summary.definitions;
                for (auto &[port, expression] : definitions)
                    expression = Op(BTOR2_TAG_ite, ir.Node(port).sortId,
                                    {enabled, expression, Raw(port)});
                Apply(definitions);
                Rewriter rewrite(*this, definitions);
                for (auto &n : statements) if (n.tag == BTOR2_TAG_next && n.args[0] == valid)
                    n.args[1] = Op(BTOR2_TAG_ite, boolean,
                        {reset, Op(BTOR2_TAG_and, boolean, {valid, summary.residual}), rewrite(initial)});
                context = "valid-history";
                return;
            }
        }
    }
};
} // namespace

WLArraySimplifier::WLArraySimplifier(const Btor2IR &source) {
    m_stats.comparisonsBefore = Comparisons(source);
    Builder builder(source);
    builder.Run();
    m_ir = builder.Output(false);
    if (!builder.definitions.empty()) m_replayIr = builder.Output(true);
    m_definitions = std::move(builder.definitions);
    m_privateInputs = std::move(builder.privateInputs);
    m_stats.comparisonsAfter = Comparisons(m_ir);
    m_stats.definedInputs = m_definitions.size();
    m_stats.context = builder.context;
    m_stats.skippedRules.assign(builder.skippedRules.begin(), builder.skippedRules.end());
}

void WLArraySimplifier::RestoreTrace(WLTrace &trace, const Btor2IR &property) const {
    if (m_definitions.empty()) return;
    auto choices = trace;
    WLSimulator::CompleteCoiChoices(m_replayIr, property, choices);
    WLSimulator::SimulationOptions options;
    options.onFrame = [&](size_t time, const WLSimulator::Frame &frame) {
        for (const auto &[port, expression] : m_definitions)
            trace.steps[time].arrayInputValues.insert_or_assign(port, frame.Array(expression));
    };
    WLSimulator(m_replayIr).Simulate(choices, {}, WLSimulator::MissingChoices::Reject, options);
    for (auto &step : trace.steps)
        for (auto id : m_privateInputs) step.arrayInputValues.erase(id);
}

} // namespace car
