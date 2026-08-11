#include "WitnessBuilder.h"

#include "Btor2Frontend.h"
#include "Log.h"
#include "Model.h"
#include "WLModel.h"
#include "WLTypes.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace car {

void AigerDeleter(aiger *aig);

namespace {

unsigned GetBadRaw(const aiger *model_aig) {
    if (model_aig->num_bad == 1) {
        return model_aig->bad[0].lit;
    }
    if (model_aig->num_outputs == 1) {
        return model_aig->outputs[0].lit;
    }
    assert(false);
    return 0;
}

class FoldedAigBuilder {
  public:
    explicit FoldedAigBuilder()
        : aig_ptr(aiger_init(), AigerDeleter),
          aig(aig_ptr.get()) {}

    unsigned TrueLit() const { return ToAigerLit(LIT_TRUE); }
    unsigned FalseLit() const { return ToAigerLit(LIT_FALSE); }
    unsigned Negate(unsigned lit) const { return lit ^ 1U; }

    unsigned AddInput(const std::string &name) {
        unsigned lit = NewLit();
        aiger_add_input(aig, lit, name.empty() ? nullptr : name.c_str());
        return lit;
    }

    unsigned AddLatch(const std::string &name) {
        unsigned lit = NewLit();
        latch_indices[lit] = latches.size();
        latches.push_back(PendingLatch{lit, lit, lit, name});
        return lit;
    }

    void SetLatchNext(unsigned latch, unsigned next) {
        auto it = latch_indices.find(latch);
        assert(it != latch_indices.end());
        latches[it->second].next = next;
    }

    void SetLatchReset(unsigned latch, unsigned reset) {
        auto it = latch_indices.find(latch);
        assert(it != latch_indices.end());
        latches[it->second].reset = reset;
    }

    unsigned BuildAnd(unsigned a, unsigned b) {
        if (a == TrueLit()) return b;
        if (b == TrueLit()) return a;
        if (a == FalseLit() || b == FalseLit()) return FalseLit();
        if (a == b) return a;
        if (a == Negate(b)) return FalseLit();
        unsigned lhs = NewLit();
        aiger_add_and(aig, lhs, a, b);
        return lhs;
    }

    unsigned BuildAnd(const std::vector<unsigned> &lits) {
        if (lits.empty()) return TrueLit();
        unsigned result = lits.front();
        for (size_t i = 1; i < lits.size(); ++i) {
            result = BuildAnd(result, lits[i]);
        }
        return result;
    }

    unsigned BuildOr(unsigned a, unsigned b) {
        return Negate(BuildAnd(Negate(a), Negate(b)));
    }

    unsigned BuildOr(const std::vector<unsigned> &lits) {
        if (lits.empty()) return FalseLit();
        unsigned result = lits.front();
        for (size_t i = 1; i < lits.size(); ++i) {
            result = BuildOr(result, lits[i]);
        }
        return result;
    }

    unsigned BuildXor(unsigned a, unsigned b) {
        unsigned t1 = BuildAnd(a, Negate(b));
        unsigned t2 = BuildAnd(Negate(a), b);
        return BuildOr(t1, t2);
    }

    unsigned BuildXnor(unsigned a, unsigned b) {
        return Negate(BuildXor(a, b));
    }

    unsigned BuildImp(unsigned a, unsigned b) {
        return BuildOr(Negate(a), b);
    }

    void FlushLatches() {
        for (const PendingLatch &latch : latches) {
            aiger_add_latch(aig, latch.lit, latch.next,
                            latch.name.empty() ? nullptr : latch.name.c_str());
            aiger_add_reset(aig, latch.lit, latch.reset);
        }
    }

    std::shared_ptr<aiger> aig_ptr;
    aiger *aig{nullptr};

  private:
    struct PendingLatch {
        unsigned lit;
        unsigned next;
        unsigned reset;
        std::string name;
    };

    unsigned NewLit() {
        ++max_var;
        return max_var * 2U;
    }

    unsigned max_var{0};
    std::vector<PendingLatch> latches;
    std::unordered_map<unsigned, size_t> latch_indices;
};

std::string SymbolName(const aiger_symbol &symbol,
                       const std::string &prefix,
                       int time) {
    std::string name = symbol.name ? symbol.name : prefix + std::to_string(symbol.lit / 2U);
    return name + "@" + std::to_string(time);
}

std::string OriginalLitSymbol(unsigned lit) {
    return "= " + std::to_string(lit);
}

} // namespace

WitnessBuilder::WitnessBuilder(const Settings &settings,
                               Log &log,
                               const Model &model)
    : m_settings(settings),
      m_log(log),
      m_model(&model),
      m_modelAig(model.GetAiger().get()) {
    assert(m_modelAig != nullptr);
    m_numInputs = static_cast<int>(m_modelAig->num_inputs);
    m_numLatches = static_cast<int>(m_modelAig->num_latches);
}

WitnessBuilder::WitnessBuilder(const Settings &settings,
                               Log &log,
                               const WLModel &model)
    : m_settings(settings),
      m_log(log),
      m_model(&model.BitModel()),
      m_wlModel(&model),
      m_modelAig(model.BitModel().GetAiger().get()) {
    assert(m_modelAig != nullptr);
    m_numInputs = static_cast<int>(m_modelAig->num_inputs);
    m_numLatches = static_cast<int>(m_modelAig->num_latches);
}

void WitnessBuilder::BeginWitness() {
    m_witnessAigPtr = CloneBaseAig(m_modelAig);
    m_witnessAig = m_witnessAigPtr.get();
    m_propertyLit = Negate(GetBadRaw(m_modelAig));
}


bool WitnessBuilder::WriteWitness() {
    if (m_witnessAig == nullptr) {
        BeginWitness();
    }
    return WriteAigWitness(m_modelAig, m_propertyLit);
}


bool WitnessBuilder::WriteCounterexample(const std::vector<std::pair<Cube, Cube>> &trace) {
    return WriteAigerCounterexample(trace);
}

bool WitnessBuilder::WriteCounterexample(const WLWitnessTrace &trace) {
    return WriteBtor2Counterexample(trace);
}

bool WitnessBuilder::WriteAigerCounterexample(
    const std::vector<std::pair<Cube, Cube>> &trace) {
    if (trace.empty()) {
        LOG_L(m_log, 1, "WitnessBuilder: counterexample trace is empty.");
        return false;
    }

    std::ofstream cex_file(GetWitnessPath(".cex"));
    if (!cex_file.is_open()) {
        LOG_L(m_log, 1, "WitnessBuilder: failed to open counterexample file.");
        return false;
    }

    cex_file << "1" << std::endl
             << (GetMCAlgorithmProperty(m_settings.alg) == MCAlgorithmProperty::Liveness ? "j0" : "b0") << std::endl;
    cex_file << CubeToLatchString(trace.front().second) << std::endl;
    for (const auto &step : trace) {
        cex_file << CubeToInputString(step.first) << std::endl;
    }
    cex_file << "." << std::endl;
    return true;
}

bool WitnessBuilder::WriteBtor2Counterexample(
    const WLWitnessTrace &trace) {
    if (m_wlModel == nullptr) {
        LOG_L(m_log, 1, "WitnessBuilder: BTOR2 counterexample needs a word-level model.");
        return false;
    }
    if (trace.steps.empty()) {
        LOG_L(m_log, 1, "WitnessBuilder: counterexample trace is empty.");
        return false;
    }

    const Btor2IR &ir = m_wlModel->SourceIR();
    std::vector<int64_t> states;
    std::vector<int64_t> inputs;
    for (const Btor2IRNode &node : ir.Nodes()) {
        if (node.tag == BTOR2_TAG_state)
            states.push_back(node.id);
        else if (node.tag == BTOR2_TAG_input)
            inputs.push_back(node.id);
    }

    std::ofstream output(GetBtor2CounterexamplePath());
    if (!output) {
        LOG_L(m_log, 1, "WitnessBuilder: failed to open BTOR2 counterexample file.");
        return false;
    }

    output << "sat\nb0\n";
    for (size_t time = 0; time < trace.steps.size(); ++time) {
        const WLWitnessStep &step = trace.steps[time];
        output << "#" << time << "\n";
        for (size_t position = 0; position < states.size(); ++position) {
            const int64_t stateId = states[position];
            auto scalar = step.stateValues.find(stateId);
            if (scalar != step.stateValues.end()) {
                output << position << " " << scalar->second.ToBinary()
                       << "\n";
                continue;
            }
            auto array = step.arrayStateValues.find(stateId);
            if (array == step.arrayStateValues.end()) continue;
            std::map<std::string, WLBitVector> ordered(
                array->second.entries.begin(),
                array->second.entries.end());
            for (const auto &[index, value] : ordered)
                output << position << " [" << index << "] "
                       << value.ToBinary() << "\n";
        }

        output << "@" << time << "\n";
        for (size_t position = 0; position < inputs.size(); ++position) {
            auto input = step.inputValues.find(inputs[position]);
            if (input != step.inputValues.end())
                output << position << " " << input->second.ToBinary()
                       << "\n";
        }
    }
    output << ".\n";
    return static_cast<bool>(output);
}

unsigned WitnessBuilder::BuildCube(const Cube &cube) {
    std::vector<unsigned> lits;
    lits.reserve(cube.size());
    for (Lit lit : cube) {
        lits.push_back(ToAigerLit(lit));
    }
    return BuildAnd(lits);
}


unsigned WitnessBuilder::BuildClause(const Clause &clause) {
    std::vector<unsigned> negated_clause;
    negated_clause.reserve(clause.size());
    for (Lit lit : clause) {
        negated_clause.push_back(ToAigerLit(~lit));
    }
    return Negate(BuildAnd(negated_clause));
}


unsigned WitnessBuilder::BuildAnd(const std::vector<unsigned> &lits) {
    assert(m_witnessAig != nullptr);
    if (lits.empty()) return TrueLit();

    unsigned result = lits.front();
    for (size_t i = 1; i < lits.size(); ++i) {
        unsigned new_gate = (m_witnessAig->maxvar + 1) * 2;
        aiger_add_and(m_witnessAig, new_gate, result, lits[i]);
        result = new_gate;
    }
    return result;
}


unsigned WitnessBuilder::BuildOr(const std::vector<unsigned> &lits) {
    assert(m_witnessAig != nullptr);
    if (lits.empty()) return FalseLit();

    std::vector<unsigned> negated;
    negated.reserve(lits.size());
    for (unsigned lit : lits) {
        negated.push_back(Negate(lit));
    }
    return Negate(BuildAnd(negated));
}

void WitnessBuilder::RegisterEquivalenceWitness(const EquivalenceWitness &witness) {
    m_equivalenceWitness = witness;
    m_hasEquivalenceWitness = true;
}

void WitnessBuilder::BuildKInductionWitness(int safeK) {
    assert(m_modelAig != nullptr);
    const int n = std::max(1, safeK);

    FoldedAigBuilder folded;

    // Build a folded copy of the original AIG over the time window [0, n-1].
    // Time n-1 is the live frontier; earlier input copies are stored as latches.
    std::unordered_map<unsigned, unsigned> input_index;
    std::unordered_map<unsigned, unsigned> latch_index;
    for (unsigned i = 0; i < m_modelAig->num_inputs; ++i) {
        input_index[m_modelAig->inputs[i].lit] = i;
    }
    for (unsigned i = 0; i < m_modelAig->num_latches; ++i) {
        latch_index[m_modelAig->latches[i].lit] = i;
    }

    std::vector<std::vector<unsigned>> inputs_at(static_cast<size_t>(n));
    std::vector<std::vector<unsigned>> latches_at(static_cast<size_t>(n));
    std::vector<std::unordered_map<unsigned, unsigned>> clone_memo(static_cast<size_t>(n));

    // X^{n-1} are real inputs. X^0..X^{n-2} and all L^0..L^{n-1}
    // are folded-state latches.
    for (int t = 0; t < n; ++t) {
        inputs_at[t].resize(m_modelAig->num_inputs);
        for (unsigned i = 0; i < m_modelAig->num_inputs; ++i) {
            const aiger_symbol &input = m_modelAig->inputs[i];
            if (t == n - 1) {
                inputs_at[t][i] = folded.AddInput(OriginalLitSymbol(input.lit));
            } else {
                inputs_at[t][i] = folded.AddLatch(SymbolName(input, "hist_input", t));
            }
        }

        latches_at[t].resize(m_modelAig->num_latches);
        for (unsigned i = 0; i < m_modelAig->num_latches; ++i) {
            const aiger_symbol &latch = m_modelAig->latches[i];
            const std::string name = (t == n - 1) ? OriginalLitSymbol(latch.lit)
                                                  : SymbolName(latch, "latch", t);
            latches_at[t][i] = folded.AddLatch(name);
        }
    }

    std::vector<unsigned> valid(static_cast<size_t>(n));
    for (int t = 0; t < n; ++t) {
        valid[t] = folded.AddLatch("kind_valid@" + std::to_string(t));
    }

    // Substitute original literals into time t:
    // input x -> X^t, latch l -> L^t, gate g -> recursively cloned g^t.
    std::function<unsigned(unsigned, int)> clone_lit_at = [&](unsigned lit, int t) -> unsigned {
        if (lit == folded.FalseLit() || lit == folded.TrueLit()) return lit;

        const bool sign = (lit & 1U) != 0;
        const unsigned stripped = lit & ~1U;
        auto &memo = clone_memo[t];
        auto memo_it = memo.find(stripped);
        unsigned cloned = 0;
        if (memo_it != memo.end()) {
            cloned = memo_it->second;
        } else {
            auto input_it = input_index.find(stripped);
            if (input_it != input_index.end()) {
                cloned = inputs_at[t][input_it->second];
            } else {
                auto latch_it = latch_index.find(stripped);
                if (latch_it != latch_index.end()) {
                    cloned = latches_at[t][latch_it->second];
                } else if (aiger_and *gate = aiger_is_and(m_modelAig, stripped)) {
                    unsigned rhs0 = clone_lit_at(gate->rhs0, t);
                    unsigned rhs1 = clone_lit_at(gate->rhs1, t);
                    cloned = folded.BuildAnd(rhs0, rhs1);
                } else {
                    assert(false && "literal is not defined in source AIG");
                    cloned = folded.FalseLit();
                }
            }
            memo.emplace(stripped, cloned);
        }
        return sign ? folded.Negate(cloned) : cloned;
    };

    auto clone_car_lit_at = [&](Lit lit, int t) {
        return clone_lit_at(ToAigerLit(lit), t);
    };

    auto build_cube_at = [&](const Cube &cube, int t) {
        std::vector<unsigned> lits;
        lits.reserve(cube.size());
        for (Lit lit : cube) {
            lits.push_back(clone_car_lit_at(lit, t));
        }
        return folded.BuildAnd(lits);
    };

    auto build_clause_at = [&](const Clause &clause, int t) {
        std::vector<unsigned> lits;
        lits.reserve(clause.size());
        for (Lit lit : clause) {
            lits.push_back(clone_car_lit_at(lit, t));
        }
        return folded.BuildOr(lits);
    };

    auto build_preprocess_at = [&](int t) {
        if (!m_hasEquivalenceWitness) return folded.TrueLit();

        // Cons_pre^t := (& equivalence_clauses^t) & (| reached_state_cubes^t).
        // This brings Model preprocessing/equivalence assumptions back to
        // the original AIG variable space, following the certifaiger style.
        std::vector<unsigned> terms;
        terms.reserve(m_equivalenceWitness.equivalence_clauses.size() + 1);
        for (const Clause &clause : m_equivalenceWitness.equivalence_clauses) {
            terms.push_back(build_clause_at(clause, t));
        }

        if (m_equivalenceWitness.has_reached_state_region) {
            std::vector<unsigned> state_terms;
            state_terms.reserve(m_equivalenceWitness.reached_state_cubes.size());
            for (const Cube &cube : m_equivalenceWitness.reached_state_cubes) {
                state_terms.push_back(build_cube_at(cube, t));
            }
            terms.push_back(folded.BuildOr(state_terms));
        }

        return folded.BuildAnd(terms);
    };

    auto build_constraints_at = [&](int t) {
        // C^t := conjunction of original AIGER constraints at time t.
        std::vector<unsigned> terms;
        terms.reserve(m_modelAig->num_constraints);
        for (unsigned i = 0; i < m_modelAig->num_constraints; ++i) {
            terms.push_back(clone_lit_at(m_modelAig->constraints[i].lit, t));
        }
        return folded.BuildAnd(terms);
    };

    auto build_reset_state_at = [&](int t) {
        // R_state^t := &_{l with deterministic reset} (L_l^t <-> reset_l^t).
        // AIGER reset equal to the latch literal denotes nondeterministic init.
        std::vector<unsigned> terms;
        terms.reserve(m_modelAig->num_latches);
        for (unsigned i = 0; i < m_modelAig->num_latches; ++i) {
            const aiger_symbol &latch = m_modelAig->latches[i];
            if (latch.reset == latch.lit) continue;
            unsigned reset = clone_lit_at(latch.reset, t);
            terms.push_back(folded.BuildXnor(latches_at[t][i], reset));
        }
        return folded.BuildAnd(terms);
    };

    // Shift-register transition:
    //   next(X^t) = X^{t+1}, t < n-1
    //   next(L^t) = L^{t+1}, t < n-1
    //   next(L^{n-1}) = F(X^{n-1}, L^{n-1})
    //   next(b^t) = b^{t+1}, next(b^{n-1}) = true
    for (int t = 0; t < n; ++t) {
        for (unsigned i = 0; i < m_modelAig->num_inputs; ++i) {
            if (t < n - 1) {
                folded.SetLatchReset(inputs_at[t][i], inputs_at[t][i]);
                folded.SetLatchNext(inputs_at[t][i], inputs_at[t + 1][i]);
            }
        }

        for (unsigned i = 0; i < m_modelAig->num_latches; ++i) {
            const aiger_symbol &latch = m_modelAig->latches[i];
            unsigned latch_copy = latches_at[t][i];
            if (t == n - 1) {
                unsigned reset = (latch.reset == latch.lit) ? latch_copy : clone_lit_at(latch.reset, t);
                folded.SetLatchReset(latch_copy, reset);
                folded.SetLatchNext(latch_copy, clone_lit_at(latch.next, t));
            } else {
                folded.SetLatchReset(latch_copy, latch_copy);
                folded.SetLatchNext(latch_copy, latches_at[t + 1][i]);
            }
        }

        folded.SetLatchReset(valid[t], t == n - 1 ? folded.TrueLit() : folded.FalseLit());
        folded.SetLatchNext(valid[t], t == n - 1 ? folded.TrueLit() : valid[t + 1]);
    }

    unsigned bad = GetBadRaw(m_modelAig);
    std::vector<unsigned> property_terms;
    property_terms.reserve(static_cast<size_t>(5 * n));

    // p0: monotonicity of validity bits, &_{t=0}^{n-2} (b^t -> b^{t+1}).
    for (int t = 0; t < n - 1; ++t) {
        property_terms.push_back(folded.BuildImp(valid[t], valid[t + 1]));
    }

    // p1: stored-history transition legality,
    // &_{t=0}^{n-2} (b^t -> &_{l} (L_l^{t+1} <-> F_l(X^t, L^t))).
    for (int t = 0; t < n - 1; ++t) {
        std::vector<unsigned> trans_terms;
        trans_terms.reserve(m_modelAig->num_latches);
        for (unsigned i = 0; i < m_modelAig->num_latches; ++i) {
            unsigned next = clone_lit_at(m_modelAig->latches[i].next, t);
            trans_terms.push_back(folded.BuildXnor(latches_at[t + 1][i], next));
        }
        property_terms.push_back(folded.BuildImp(valid[t], folded.BuildAnd(trans_terms)));
    }

    // p2: safety plus preprocessing bridge for every valid time:
    // &_{t=0}^{n-1} (b^t -> (!Bad^t & Cons_pre^t)).
    // p_constraints: &_{t=0}^{n-1} (b^t -> C^t).
    for (int t = 0; t < n; ++t) {
        unsigned safe = folded.BuildAnd(folded.Negate(clone_lit_at(bad, t)), build_preprocess_at(t));
        property_terms.push_back(folded.BuildImp(valid[t], safe));
        property_terms.push_back(folded.BuildImp(valid[t], build_constraints_at(t)));
    }

    // p3: initialization boundary legality,
    // &_{t=1}^{n-1} ((!b^{t-1} & b^t) -> R_state^t).
    for (int t = 1; t < n; ++t) {
        unsigned boundary = folded.BuildAnd(folded.Negate(valid[t - 1]), valid[t]);
        property_terms.push_back(folded.BuildImp(boundary, build_reset_state_at(t)));
    }

    // p4: the newest frame must exist, b^{n-1}.
    property_terms.push_back(valid[n - 1]);
    unsigned property = folded.BuildAnd(property_terms);

    folded.FlushLatches();

    m_witnessAigPtr = folded.aig_ptr;
    m_witnessAig = m_witnessAigPtr.get();
    m_propertyLit = property;
    m_numInputs = static_cast<int>(m_witnessAig->num_inputs);
    m_numLatches = static_cast<int>(m_witnessAig->num_latches);
}


std::shared_ptr<aiger> WitnessBuilder::CloneBaseAig(const aiger *src) {
    std::shared_ptr<aiger> aig_ptr(aiger_init(), AigerDeleter);
    aiger *dst = aig_ptr.get();
    for (unsigned i = 0; i < src->num_inputs; ++i) {
        const aiger_symbol &input = src->inputs[i];
        aiger_add_input(dst, input.lit, input.name);
    }
    for (unsigned i = 0; i < src->num_latches; ++i) {
        const aiger_symbol &latch = src->latches[i];
        aiger_add_latch(dst, latch.lit, latch.next, latch.name);
        aiger_add_reset(dst, latch.lit, latch.reset);
    }
    for (unsigned i = 0; i < src->num_ands; ++i) {
        const aiger_and &gate = src->ands[i];
        aiger_add_and(dst, gate.lhs, gate.rhs0, gate.rhs1);
    }
    for (unsigned i = 0; i < src->num_constraints; ++i) {
        const aiger_symbol &constraint = src->constraints[i];
        aiger_add_constraint(dst, constraint.lit, constraint.name);
    }
    assert(src->maxvar == dst->maxvar);
    return aig_ptr;
}


std::string WitnessBuilder::GetWitnessPath(const std::string &suffix) const {
    auto start_index = m_settings.aigFilePath.find_last_of("/\\");
    if (start_index == std::string::npos) {
        start_index = 0;
    } else {
        ++start_index;
    }
    auto end_index = m_settings.aigFilePath.find_last_of(".");
    assert(end_index != std::string::npos);
    std::string aig_name = m_settings.aigFilePath.substr(start_index, end_index - start_index);
    return m_settings.witnessOutputDir + aig_name + suffix;
}

std::string WitnessBuilder::GetBtor2CounterexamplePath() const {
    std::filesystem::path outputDir(m_settings.witnessOutputDir);
    return (outputDir / (std::filesystem::path(m_settings.aigFilePath)
                             .filename()
                             .string() +
                         ".cexb"))
        .string();
}

bool WitnessBuilder::WriteAigWitness(const aiger *model_aig, unsigned invariant_lit) {
    assert(m_witnessAig != nullptr);

    if (model_aig->num_bad == 1) {
        aiger_add_bad(m_witnessAig, Negate(invariant_lit), model_aig->bad[0].name);
    } else if (model_aig->num_outputs == 1) {
        aiger_add_output(m_witnessAig, Negate(invariant_lit), model_aig->outputs[0].name);
    } else {
        assert(false);
    }

    if (const char *err = aiger_check(m_witnessAig)) {
        std::cerr << "invalid witness aig: " << err << std::endl;
        return false;
    }

    aiger_reencode(m_witnessAig);
    if (!aiger_open_and_write_to_file(m_witnessAig, GetWitnessPath(".w.aig").c_str())) {
        if (const char *err = aiger_error(m_witnessAig)) {
            std::cerr << "aiger write error: " << err << std::endl;
        } else {
            std::cerr << "aiger write error: failed to write safe witness" << std::endl;
        }
        return false;
    }
    return true;
}


std::string WitnessBuilder::CubeToInputString(const Cube &cube) const {
    std::string result(static_cast<size_t>(m_numInputs), 'x');
    for (Lit lit : cube) {
        Var var = VarOf(lit);
        if (var >= 1 && var <= static_cast<Var>(m_numInputs)) {
            result[static_cast<size_t>(var - 1)] = Sign(lit) ? '0' : '1';
        }
    }
    return result;
}


std::string WitnessBuilder::CubeToLatchString(const Cube &cube) const {
    std::string result(static_cast<size_t>(m_numLatches), 'x');
    Var latch_begin = static_cast<Var>(m_numInputs) + 1;
    Var latch_end = latch_begin + static_cast<Var>(m_numLatches);
    for (Lit lit : cube) {
        Var var = VarOf(lit);
        if (var >= latch_begin && var < latch_end) {
            result[static_cast<size_t>(var - latch_begin)] = Sign(lit) ? '0' : '1';
        }
    }
    return result;
}

} // namespace car
