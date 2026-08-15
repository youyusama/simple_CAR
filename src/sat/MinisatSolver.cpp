#include "MinisatSolver.h"
#include <algorithm>

namespace car {
MinisatSolver::MinisatSolver(Model &m) : m_model(m) {
    m_maxId = m_model.NumVar() + 1; // reserve variable numbers for one step reachability check
    m_tempVar = 0;
}

MinisatSolver::~MinisatSolver() {}

bool MinisatSolver::Solve() {
    ClearFailed();
    if (m_tempVar != 0) m_assumptions.push(GetLit(m_tempVar));
    Minisat::lbool result = solveLimited(m_assumptions);
    if (result == Minisat::l_True) {
        return true;
    } else {
        assert(result == Minisat::l_False);
        CacheFailed();
        return false;
    }
}


bool MinisatSolver::Solve(const Cube &assumption) {
    m_assumptions.clear();
    for (auto it : assumption) {
        m_assumptions.push(GetLit(it));
    }
    return Solve();
}


void MinisatSolver::AddClause(const Cube &cls) {
    Minisat::vec<Minisat::Lit> literals;
    for (Lit l : cls) {
        literals.push(GetLit(l));
        if (VarOf(l) > m_maxId) m_maxId = VarOf(l) + 1;
    }
    bool result = addClause(literals);
    // result may be false when the Clause is already conflict
    // assert(result != false);
}


pair<Cube, Cube> MinisatSolver::GetAssignment(bool prime) {
    assert(m_model.GetNumInputs() < nVars());
    Cube inputs;
    Cube latches;
    inputs.reserve(m_model.GetNumInputs());
    latches.reserve(m_model.GetNumLatches());
    for (Var i : m_model.GetModelInputs()) {
        if (model[static_cast<int>(i)] == Minisat::l_True) {
            inputs.emplace_back(MkLit(i));
        } else {
            assert(model[static_cast<int>(i)] == Minisat::l_False);
            inputs.emplace_back(~MkLit(i));
        }
    }
    for (Var i : m_model.GetModelLatches()) {
        if (!prime) {
            if (model[static_cast<int>(i)] == Minisat::l_True) {
                latches.emplace_back(MkLit(i));
            } else {
                assert(model[static_cast<int>(i)] == Minisat::l_False);
                latches.emplace_back(~MkLit(i));
            }
        } else {
            Lit p = m_model.LookupPrime(MkLit(i));
            Minisat::lbool val = model[static_cast<int>(VarOf(p))];
            if ((val == Minisat::l_True && !Sign(p)) || (val == Minisat::l_False && Sign(p))) {
                latches.emplace_back(MkLit(i));
            } else {
                latches.emplace_back(~MkLit(i));
            }
        }
    }
    for (Var i : m_model.GetInnards()) {
        if (!prime) {
            if (model[static_cast<int>(i)] == Minisat::l_True) {
                latches.emplace_back(MkLit(i));
            } else {
                assert(model[static_cast<int>(i)] == Minisat::l_False);
                latches.emplace_back(~MkLit(i));
            }
        } else {
            Lit p = m_model.LookupPrime(MkLit(i));
            Minisat::lbool val = model[static_cast<int>(VarOf(p))];
            if ((val == Minisat::l_True && !Sign(p)) || (val == Minisat::l_False && Sign(p))) {
                latches.emplace_back(MkLit(i));
            } else {
                latches.emplace_back(~MkLit(i));
            }
        }
    }
    return pair<Cube, Cube>(inputs, latches);
}

void MinisatSolver::ClearFailed() {
    if (++m_failedEpoch == 0) {
        std::fill(m_failedStamp.begin(), m_failedStamp.end(), 0);
        m_failedEpoch = 1;
    }
}


void MinisatSolver::CacheFailed() {
    if (m_failedStamp.size() < static_cast<size_t>(2 * nVars()))
        m_failedStamp.resize(static_cast<size_t>(2 * nVars()), 0);
    for (int i = 0; i < conflict.size(); ++i) {
        Lit assumption = ~GetLiteral(conflict[i]);
        m_failedStamp[PackedIndex(assumption)] = m_failedEpoch;
    }
}


bool MinisatSolver::Failed(Lit assumption) {
    const size_t index = PackedIndex(assumption);
    return index < m_failedStamp.size() && m_failedStamp[index] == m_failedEpoch;
}


inline Lit MinisatSolver::GetLiteral(const Minisat::Lit &l) {
    return MkLit(static_cast<Var>(Minisat::var(l)), Minisat::sign(l));
}


void MinisatSolver::AddTempClause(const Cube &cls) {
    m_tempVar = GetNewVar();
    Cube temp_cls = cls;
    temp_cls.push_back(~MkLit(m_tempVar));
    AddClause(temp_cls);
}


void MinisatSolver::ReleaseTempClause() {
    if (m_tempVar == 0) return;
    releaseVar(~GetLit(MkLit(m_tempVar)));
    m_tempVar = 0;
}

} // namespace car
