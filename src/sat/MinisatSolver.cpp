#include "MinisatSolver.h"
#include <algorithm>

namespace car {
MinisatSolver::MinisatSolver(Model &m) : m_model(m) {
    m_maxId = m_model.NumVar() + 1; // reserve variable numbers for one step reachability check
    m_tempVar = 0;
}

MinisatSolver::~MinisatSolver() {}

bool MinisatSolver::Solve() {
    if (m_tempVar != 0) m_assumptions.push(GetLit(m_tempVar));
    Minisat::lbool result = solveLimited(m_assumptions);
    if (result == Minisat::l_True) {
        return true;
    } else {
        assert(result == Minisat::l_False);
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


void MinisatSolver::AddAssumption(const Cube &assumption) {
    for (auto it : assumption) {
        m_assumptions.push(GetLit(it));
    }
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

unordered_set<Lit, LitHash> MinisatSolver::GetConflict() {
    unordered_set<Lit, LitHash> conflict_set;
    for (int i = 0; i < conflict.size(); ++i) {
        conflict_set.insert(~GetLiteral(conflict[i]));
    }
    return conflict_set;
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
    assert(m_tempVar != 0);
    releaseVar(~GetLit(MkLit(m_tempVar)));
    m_tempVar = 0;
}


void MinisatSolver::ClearAssumption() {
    m_assumptions.clear();
}


void MinisatSolver::PushAssumption(Lit a) {
    m_assumptions.push(GetLit(a));
}


Lit MinisatSolver::PopAssumption() {
    Minisat::Lit p = m_assumptions.last();
    m_assumptions.pop();
    return GetLiteral(p);
}

} // namespace car
