// created by Jianwen Li
// Kissat API for BMC
#include "KissatSolver.h"
#include <cstdlib>
#include <iostream>

namespace car {
KissatSolver::KissatSolver(Model &m) : m_model(m) {
    m_maxId = m_model.NumVar() + 1;
    m_solver = kissat_init();
    assert(m_solver != nullptr);
    kissat_reserve(m_solver, m_maxId);
}

[[noreturn]] void KissatSolver::Unsupported(const char *fn) {
    throw std::runtime_error(std::string("KissatSolver does not support ") + fn);
}

void KissatSolver::EnsureReserved(const Cube &cls) {
    Var maxVar = 0;
    for (Lit lit : cls) {
        Var litVar = VarOf(lit);
        if (litVar > maxVar) {
            maxVar = litVar;
        }
    }
    if (maxVar > m_maxId) {
        m_maxId = maxVar;
        kissat_reserve(m_solver, static_cast<int>(m_maxId));
    }
}

bool KissatSolver::Solve() {
    int ret = kissat_solve(m_solver);
    if (ret == 10) { // satisfiable
        return true;
    } else if (ret == 20) { // unsatisfiable
        return false;
    } else { // UNKNOWN
        std::cerr << "Kissat solve returned unknown status: " << ret << '\n';
        std::exit(1);
    }
}

void KissatSolver::AddClause(const Cube &cls) {
    EnsureReserved(cls);
    for (Lit l : cls) {
        kissat_add(m_solver, ToSigned(l));
    }
    kissat_add(m_solver, 0); // end of a Clause
}

void KissatSolver::AddAssumption(const Cube &assumption) {
    if (!assumption.empty()) {
        Unsupported("AddAssumption");
    }
}

bool KissatSolver::Solve(const Cube &assumption) {
    if (!assumption.empty()) {
        Unsupported("Solve(const Cube&)");
    }
    return Solve();
}

pair<Cube, Cube> KissatSolver::GetAssignment(bool prime) {
    (void)prime;
    Unsupported("GetAssignment");
}

unordered_set<Lit, LitHash> KissatSolver::GetConflict() {
    Unsupported("GetConflict");
}

Var KissatSolver::GetNewVar() {
    ++m_maxId;
    kissat_reserve(m_solver, static_cast<int>(m_maxId));
    return m_maxId;
}

void KissatSolver::AddTempClause(const Cube &cls) {
    if (!cls.empty()) {
        Unsupported("AddTempClause");
    }
}

void KissatSolver::ReleaseTempClause() {}

void KissatSolver::ClearAssumption() {}

void KissatSolver::PushAssumption(Lit a) {
    (void)a;
    Unsupported("PushAssumption");
}

Lit KissatSolver::PopAssumption() {
    Unsupported("PopAssumption");
}

} // namespace car
