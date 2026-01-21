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


bool MinicoreSolver::Solve(const cube &assumption) {
    m_assumptions.clear();
    for (auto it : assumption) {
        m_assumptions.emplace_back(GetLit(it));
    }
    return Solve();
}


void MinicoreSolver::AddAssumption(const cube &assumption) {
    for (auto it : assumption) {
        m_assumptions.emplace_back(GetLit(it));
    }
}


void MinicoreSolver::AddClause(const cube &cls) {
    vector<minicore::Lit> lit_cls;
    for (int l : cls) {
        lit_cls.emplace_back(GetLit(l));
        if (abs(l) > m_maxId) m_maxId = abs(l) + 1;
    }
    bool result = addClause(lit_cls);
}


pair<cube, cube> MinicoreSolver::GetAssignment(bool prime) {
    cube inputs;
    cube latches;
    inputs.reserve(m_model.GetNumInputs());
    latches.reserve(m_model.GetNumLatches());
    for (int i : m_model.GetModelInputs()) {
        if (value(i) == minicore::l_True) {
            inputs.emplace_back(i);
        } else if (value(i) == minicore::l_False) {
            inputs.emplace_back(-i);
        }
    }
    for (int i : m_model.GetModelLatches()) {
        if (!prime) {
            if (value(i) == minicore::l_True) {
                latches.emplace_back(i);
            } else if (value(i) == minicore::l_False) {
                latches.emplace_back(-i);
            }
        } else {
            int p = m_model.GetPrime(i);
            minicore::lbool val = value(abs(p));
            if ((val == minicore::l_True && p > 0) || (val == minicore::l_False && p < 0)) {
                latches.emplace_back(i);
            } else if ((val == minicore::l_True && p < 0) || (val == minicore::l_False && p > 0)) {
                latches.emplace_back(-i);
            }
        }
    }
    for (int i : m_model.GetInnards()) {
        if (!prime) {
            if (value(i) == minicore::l_True) {
                latches.emplace_back(i);
            } else if (value(i) == minicore::l_False) {
                latches.emplace_back(-i);
            }
        } else {
            int p = m_model.GetPrime(i);
            minicore::lbool val = value(abs(p));
            if ((val == minicore::l_True && p > 0) || (val == minicore::l_False && p < 0)) {
                latches.emplace_back(i);
            } else if ((val == minicore::l_True && p < 0) || (val == minicore::l_False && p > 0)) {
                latches.emplace_back(-i);
            }
        }
    }
    return pair<cube, cube>(inputs, latches);
}


unordered_set<int> MinicoreSolver::GetConflict() {
    unordered_set<int> conflictSet;
    for (minicore::Lit l : conflict) {
        int val = -GetLiteralId(l);
        conflictSet.insert(val);
    }
    return conflictSet;
}


inline int MinicoreSolver::GetLiteralId(const minicore::Lit &l) {
    return sign(l) ? -var(l) : var(l);
}


void MinicoreSolver::AddTempClause(const cube &cls) {
    m_tempClause.clear();
    for (int l : cls) {
        m_tempClause.emplace_back(GetLit(l));
        if (abs(l) > m_maxId) m_maxId = abs(l) + 1;
    }
}


void MinicoreSolver::ReleaseTempClause() {
    m_tempClause.clear();
}


void MinicoreSolver::ClearAssumption() {
    m_assumptions.clear();
}


void MinicoreSolver::PushAssumption(int a) {
    m_assumptions.emplace_back(GetLit(a));
}


int MinicoreSolver::PopAssumption() {
    minicore::Lit p = m_assumptions.back();
    m_assumptions.pop_back();
    return GetLiteralId(p);
}

} // namespace car
