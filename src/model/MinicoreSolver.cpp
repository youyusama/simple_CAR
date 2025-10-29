#include "MinicoreSolver.h"
#include <algorithm>

namespace car {
MinicoreSolver::MinicoreSolver(shared_ptr<Model> m) {
    m_model = m;
    m_maxId = m_model->TrueId() + 1; // reserve variable numbers for one step reachability check
    // verbosity = 1;
}

MinicoreSolver::~MinicoreSolver() {}

bool MinicoreSolver::Solve() {
    if (!m_tempClause.empty()) {
        addTempClause(m_tempClause);
    }
    return solve(m_assumptions) == minicore::l_True;
}


bool MinicoreSolver::Solve(const shared_ptr<cube> assumption) {
    m_assumptions.clear();
    for (auto it : *assumption) {
        m_assumptions.emplace_back(GetLit(it));
    }
    return Solve();
}


void MinicoreSolver::AddAssumption(const shared_ptr<cube> assumption) {
    for (auto it : *assumption) {
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


pair<shared_ptr<cube>, shared_ptr<cube>> MinicoreSolver::GetAssignment(bool prime) {
    shared_ptr<cube> inputs(new cube());
    shared_ptr<cube> latches(new cube());
    inputs->reserve(m_model->GetNumInputs());
    latches->reserve(m_model->GetNumLatches());
    for (int i : m_model->GetModelInputs()) {
        if (model[i] == minicore::l_True) {
            inputs->emplace_back(i);
        } else if (model[i] == minicore::l_False) {
            inputs->emplace_back(-i);
        }
    }
    for (int i : m_model->GetModelLatches()) {
        if (!prime) {
            if (model[i] == minicore::l_True) {
                latches->emplace_back(i);
            } else if (model[i] == minicore::l_False) {
                latches->emplace_back(-i);
            }
        } else {
            int p = m_model->GetPrime(i);
            minicore::lbool val = model[abs(p)];
            if ((val == minicore::l_True && p > 0) || (val == minicore::l_False && p < 0)) {
                latches->emplace_back(i);
            } else if ((val == minicore::l_True && p < 0) || (val == minicore::l_False && p > 0)) {
                latches->emplace_back(-i);
            }
        }
    }
    for (int i : m_model->GetInnards()) {
        if (!prime) {
            if (model[i] == minicore::l_True) {
                latches->emplace_back(i);
            } else if (model[i] == minicore::l_False) {
                latches->emplace_back(-i);
            }
        } else {
            int p = m_model->GetPrime(i);
            minicore::lbool val = model[abs(p)];
            if ((val == minicore::l_True && p > 0) || (val == minicore::l_False && p < 0)) {
                latches->emplace_back(i);
            } else if ((val == minicore::l_True && p < 0) || (val == minicore::l_False && p > 0)) {
                latches->emplace_back(-i);
            }
        }
    }
    return pair<shared_ptr<cube>, shared_ptr<cube>>(inputs, latches);
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


inline void MinicoreSolver::SetSolveInDomain() {
    solve_in_domain = true;
}


inline void MinicoreSolver::SetDomain(const shared_ptr<cube> domain) {
    std::vector<minicore::Var> d;
    for (auto v : *domain) d.emplace_back(v);
    setDomain(d);
}


inline void MinicoreSolver::SetTempDomain(const shared_ptr<cube> domain) {
    std::vector<minicore::Var> d;
    for (auto v : *domain) d.emplace_back(v);
    setTempDomain(d);
}


inline void MinicoreSolver::ResetTempDomain() {
    resetTempDomain();
}


} // namespace car