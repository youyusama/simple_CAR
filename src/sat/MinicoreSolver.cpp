#include "MinicoreSolver.h"
#include <algorithm>

namespace car {
MinicoreSolver::MinicoreSolver(Model &m) : m_model(m) {
    m_maxId = m_model.NumVar() + 1; // reserve variable numbers for one step reachability check
    // verbosity = 1;
}

MinicoreSolver::~MinicoreSolver() {}

bool MinicoreSolver::Solve() {
    if (!m_tempClause.empty()) {
        addTempClause(m_tempClause);
    }
    return solve(m_assumptions) == minicore::l_True;
}


bool MinicoreSolver::Solve(const Cube &assumption) {
    m_assumptions = assumption;
    return Solve();
}


void MinicoreSolver::AddAssumption(const Cube &assumption) {
    m_assumptions.insert(m_assumptions.end(), assumption.begin(), assumption.end());
}


void MinicoreSolver::AddClause(const Cube &cls) {
    for (Lit l : cls) {
        if (VarOf(l) > m_maxId) m_maxId = VarOf(l) + 1;
        while (static_cast<int>(VarOf(l)) >= nVars()) newVar();
    }
    bool result = addClause(cls);
}


pair<Cube, Cube> MinicoreSolver::GetAssignment(bool prime) {
    Cube inputs;
    Cube latches;
    inputs.reserve(m_model.GetNumInputs());
    latches.reserve(m_model.GetNumLatches());
    for (Var i : m_model.GetModelInputs()) {
        if (value(static_cast<int>(i)) == minicore::l_True) {
            inputs.emplace_back(MkLit(i));
        } else if (value(static_cast<int>(i)) == minicore::l_False) {
            inputs.emplace_back(~MkLit(i));
        }
    }
    for (Var i : m_model.GetModelLatches()) {
        if (!prime) {
            if (value(static_cast<int>(i)) == minicore::l_True) {
                latches.emplace_back(MkLit(i));
            } else if (value(static_cast<int>(i)) == minicore::l_False) {
                latches.emplace_back(~MkLit(i));
            }
        } else {
            Lit p = m_model.LookupPrime(MkLit(i));
            minicore::lbool val = value(static_cast<int>(VarOf(p)));
            if ((val == minicore::l_True && !Sign(p)) || (val == minicore::l_False && Sign(p))) {
                latches.emplace_back(MkLit(i));
            } else if ((val == minicore::l_True && Sign(p)) || (val == minicore::l_False && !Sign(p))) {
                latches.emplace_back(~MkLit(i));
            }
        }
    }
    for (Var i : m_model.GetInnards()) {
        if (!prime) {
            if (value(static_cast<int>(i)) == minicore::l_True) {
                latches.emplace_back(MkLit(i));
            } else if (value(static_cast<int>(i)) == minicore::l_False) {
                latches.emplace_back(~MkLit(i));
            }
        } else {
            Lit p = m_model.LookupPrime(MkLit(i));
            minicore::lbool val = value(static_cast<int>(VarOf(p)));
            if ((val == minicore::l_True && !Sign(p)) || (val == minicore::l_False && Sign(p))) {
                latches.emplace_back(MkLit(i));
            } else if ((val == minicore::l_True && Sign(p)) || (val == minicore::l_False && !Sign(p))) {
                latches.emplace_back(~MkLit(i));
            }
        }
    }
    return pair<Cube, Cube>(inputs, latches);
}


unordered_set<Lit, LitHash> MinicoreSolver::GetConflict() {
    unordered_set<Lit, LitHash> conflict_set;
    for (minicore::Lit l : conflict) {
        conflict_set.insert(~l);
    }
    return conflict_set;
}


void MinicoreSolver::AddTempClause(const Cube &cls) {
    for (Lit l : cls) {
        if (VarOf(l) > m_maxId) m_maxId = VarOf(l) + 1;
        while (static_cast<int>(VarOf(l)) >= nVars()) newVar();
    }
    m_tempClause = cls;
}


void MinicoreSolver::ReleaseTempClause() {
    m_tempClause.clear();
}


void MinicoreSolver::ClearAssumption() {
    m_assumptions.clear();
}


void MinicoreSolver::PushAssumption(Lit a) {
    m_assumptions.emplace_back(a);
}


Lit MinicoreSolver::PopAssumption() {
    minicore::Lit p = m_assumptions.back();
    m_assumptions.pop_back();
    return p;
}

} // namespace car
