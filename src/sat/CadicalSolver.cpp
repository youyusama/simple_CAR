#include "CadicalSolver.h"
#include <algorithm>

namespace car {
CadicalSolver::CadicalSolver(Model &m) : m_model(m) {
    m_maxId = m_model.NumVar() + 1; // reserve variable numbers for one step reachability check
    m_tempClause = Cube();
}

CadicalSolver::~CadicalSolver() {}

bool CadicalSolver::Solve() {
    for (auto it : m_assumptions) {
        assume(it);
    }
    if (m_tempClause.size() > 0) {
        for (int l : m_tempClause) {
            constrain(l);
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
    m_assumptions.resize(assumption.size());
    std::copy(assumption.begin(), assumption.end(), m_assumptions.begin());
    return Solve();
}


void CadicalSolver::AddAssumption(const Cube &assumption) {
    for (auto it : assumption) {
        m_assumptions.push_back(it);
    }
}


void CadicalSolver::AddClause(const Cube &cls) {
    for (int l : cls)
        if (abs(l) > m_maxId) m_maxId = abs(l) + 1;
    clause(cls);
}


pair<Cube, Cube> CadicalSolver::GetAssignment(bool prime) {
    Cube inputs;
    Cube latches;
    inputs.reserve(m_model.GetNumInputs());
    latches.reserve(m_model.GetNumLatches());
    for (int i : m_model.GetModelInputs()) {
        if (val(i) > 0) {
            inputs.emplace_back(i);
        } else {
            assert(val(i) < 0);
            inputs.emplace_back(-i);
        }
    }
    for (int i : m_model.GetModelLatches()) {
        if (!prime) {
            if (val(i) > 0) {
                latches.emplace_back(i);
            } else {
                assert(val(i) < 0);
                latches.emplace_back(-i);
            }
        } else {
            int p = m_model.GetPrime(i);
            if ((val(p) > 0 && p > 0) || (val(p) < 0 && p < 0)) {
                latches.emplace_back(i);
            } else {
                latches.emplace_back(-i);
            }
        }
    }
    for (int i : m_model.GetInnards()) {
        if (!prime) {
            if (val(i) > 0) {
                latches.emplace_back(i);
            } else {
                assert(val(i) < 0);
                latches.emplace_back(-i);
            }
        } else {
            int p = m_model.GetPrime(i);
            if ((val(p) > 0 && p > 0) || (val(p) < 0 && p < 0)) {
                latches.emplace_back(i);
            } else {
                assert(val(p) < 0);
                latches.emplace_back(-i);
            }
        }
    }
    return pair<Cube, Cube>(inputs, latches);
}


unordered_set<int> CadicalSolver::GetConflict() {
    unordered_set<int> conflict_set;
    for (auto v : m_assumptions) {
        if (failed(v)) {
            conflict_set.insert(v);
        }
    }
    return conflict_set;
}

void CadicalSolver::AddTempClause(const Cube &cls) {
    m_tempClause = cls;
}


void CadicalSolver::ReleaseTempClause() {
    m_tempClause.clear();
}


void CadicalSolver::ClearAssumption() {
    m_assumptions.clear();
}


void CadicalSolver::PushAssumption(int a) {
    m_assumptions.push_back(a);
}


int CadicalSolver::PopAssumption() {
    int p = m_assumptions.back();
    m_assumptions.pop_back();
    return p;
}


} // namespace car
