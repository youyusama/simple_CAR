#include "CadicalSolver.h"
#include <algorithm>

namespace car {
CadicalSolver::CadicalSolver(Model &m) : m_model(m) {
    m_maxId = m_model.NumVar() + 1; // reserve variable numbers for one step reachability check
    m_tempClause.clear();
}

CadicalSolver::~CadicalSolver() {}

bool CadicalSolver::Solve() {
    for (auto it : m_assumptions) {
        assume(ToSigned(it));
    }
    if (m_tempClause.size() > 0) {
        for (Lit l : m_tempClause) {
            constrain(ToSigned(l));
        }
        constrain(0);
    }
    int result = solve();
    if (result == 10) {
        return true;
    } else {
        assert(result == 20);
        return false;
    }
}


bool CadicalSolver::Solve(const Cube &assumption) {
    m_assumptions.clear();
    m_assumptions = assumption;
    return Solve();
}


void CadicalSolver::AddAssumption(const Cube &assumption) {
    for (auto it : assumption) {
        m_assumptions.push_back(it);
    }
}


void CadicalSolver::AddClause(const Cube &cls) {
    for (Lit l : cls)
        if (VarOf(l) > m_maxId) m_maxId = VarOf(l) + 1;
    clause(ToSignedVec(cls));
}


pair<Cube, Cube> CadicalSolver::GetAssignment(bool prime) {
    Cube inputs;
    Cube latches;
    inputs.reserve(m_model.GetNumInputs());
    latches.reserve(m_model.GetNumLatches());
    for (Var i : m_model.GetModelInputs()) {
        if (val(static_cast<int>(i)) > 0) {
            inputs.emplace_back(MkLit(i));
        } else {
            assert(val(static_cast<int>(i)) < 0);
            inputs.emplace_back(~MkLit(i));
        }
    }
    for (Var i : m_model.GetModelLatches()) {
        if (!prime) {
            if (val(static_cast<int>(i)) > 0) {
                latches.emplace_back(MkLit(i));
            } else {
                assert(val(static_cast<int>(i)) < 0);
                latches.emplace_back(~MkLit(i));
            }
        } else {
            Lit p = m_model.LookupPrime(MkLit(i));
            if ((val(ToSigned(p)) > 0 && !Sign(p)) || (val(ToSigned(p)) < 0 && Sign(p))) {
                latches.emplace_back(MkLit(i));
            } else {
                latches.emplace_back(~MkLit(i));
            }
        }
    }
    for (Var i : m_model.GetInnards()) {
        if (!prime) {
            if (val(static_cast<int>(i)) > 0) {
                latches.emplace_back(MkLit(i));
            } else {
                assert(val(static_cast<int>(i)) < 0);
                latches.emplace_back(~MkLit(i));
            }
        } else {
            Lit p = m_model.LookupPrime(MkLit(i));
            if ((val(ToSigned(p)) > 0 && !Sign(p)) || (val(ToSigned(p)) < 0 && Sign(p))) {
                latches.emplace_back(MkLit(i));
            } else {
                assert(val(ToSigned(p)) < 0);
                latches.emplace_back(~MkLit(i));
            }
        }
    }
    return pair<Cube, Cube>(inputs, latches);
}


unordered_set<Lit, LitHash> CadicalSolver::GetConflict() {
    unordered_set<Lit, LitHash> conflict_set;
    for (auto v : m_assumptions) {
        if (failed(ToSigned(v))) {
            conflict_set.insert(v);
        }
    }
    return conflict_set;
}

void CadicalSolver::AddTempClause(const Cube &cls) {
    m_tempClause.clear();
    m_tempClause.reserve(cls.size());
    for (Lit lit : cls) {
        m_tempClause.emplace_back(lit);
    }
}


void CadicalSolver::ReleaseTempClause() {
    m_tempClause.clear();
}


void CadicalSolver::ClearAssumption() {
    m_assumptions.clear();
}


void CadicalSolver::PushAssumption(Lit a) {
    m_assumptions.push_back(a);
}


Lit CadicalSolver::PopAssumption() {
    Lit p = m_assumptions.back();
    m_assumptions.pop_back();
    return p;
}


} // namespace car
