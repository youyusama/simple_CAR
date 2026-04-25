#include "WitnessBuilder.h"

#include "Log.h"

#include <fstream>
#include <iostream>

namespace car {

void AigerDeleter(aiger *aig);

namespace {

unsigned GetBadLit(const aiger *model_aig) {
    if (model_aig->num_bad == 1) {
        return model_aig->bad[0].lit;
    }
    if (model_aig->num_outputs == 1) {
        return model_aig->outputs[0].lit;
    }
    assert(false);
    return 0;
}

} // namespace

WitnessBuilder::WitnessBuilder(const Settings &settings, Log &log, const aiger *model_aig)
    : m_settings(settings), m_log(log), m_modelAig(model_aig) {
    assert(m_modelAig != nullptr);
    m_numInputs = static_cast<int>(m_modelAig->num_inputs);
    m_numLatches = static_cast<int>(m_modelAig->num_latches);
}


void WitnessBuilder::BeginWitness() {
    m_witnessAigPtr = CloneBaseAig(m_modelAig);
    m_witnessAig = m_witnessAigPtr.get();
    m_propertyLit = Negate(GetBadLit(m_modelAig));
}


bool WitnessBuilder::WriteWitness() {
    if (m_witnessAig == nullptr) {
        BeginWitness();
    }
    return WriteAigWitness(m_modelAig, m_propertyLit);
}


bool WitnessBuilder::WriteCounterexample(const std::vector<std::pair<Cube, Cube>> &trace) {
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
             << "b0" << std::endl;
    cex_file << CubeToLatchString(trace.front().second) << std::endl;
    for (const auto &step : trace) {
        cex_file << CubeToInputString(step.first) << std::endl;
    }
    cex_file << "." << std::endl;
    return true;
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
