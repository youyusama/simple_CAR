#include "MinisatSolver.h"
#include <algorithm>

namespace car {
MinisatSolver::MinisatSolver() {
    m_tempVar = 0;
}

MinisatSolver::~MinisatSolver() {}

bool MinisatSolver::Solve() {
    lbool result = solveLimited(m_assumptions);
    if (result == l_True) {
        return true;
    } else {
        assert(result == l_False);
        return false;
    }
}


bool MinisatSolver::Solve(const shared_ptr<vector<int>> assumption) {
    m_assumptions.clear();
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
    if (m_tempVar != 0) m_assumptions.push(GetLit(m_tempVar));
    return Solve();
}


void MinisatSolver::AddAssumption(const shared_ptr<vector<int>> assumption) {
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
}


void MinisatSolver::AddClause(const vector<int> &cls) {
    vec<Lit> literals;
    for (int i = 0; i < cls.size(); ++i) {
        literals.push(GetLit(cls[i]));
    }
    bool result = addClause(literals);
    assert(result != false);
}


pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> MinisatSolver::GetAssignment(bool prime) {
    assert(m_model->GetNumInputs() < nVars());
    shared_ptr<vector<int>> inputs(new vector<int>());
    shared_ptr<vector<int>> latches(new vector<int>());
    inputs->reserve(m_model->GetNumInputs());
    latches->reserve(m_model->GetNumLatches());
    for (int i = 0; i < m_model->GetNumInputs(); ++i) {
        if (model[i] == l_True) {
            inputs->emplace_back(i + 1);
        } else {
            assert(model[i] == l_False);
            inputs->emplace_back(-i - 1);
        }
    }
    for (int i = m_model->GetNumInputs(), end = m_model->GetNumInputs() + m_model->GetNumLatches(); i < end; ++i) {
        if (!prime) {
            if (model[i] == l_True) {
                latches->emplace_back(i + 1);
            } else {
                assert(model[i] == l_False);
                latches->emplace_back(-i - 1);
            }
        } else {
            int p = m_model->GetPrime(i + 1);
            lbool val = model[abs(p) - 1];
            if ((val == l_True && p > 0) || (val == l_False && p < 0)) {
                latches->emplace_back(i + 1);
            } else {
                latches->emplace_back(-i - 1);
            }
        }
    }
    return pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>>(inputs, latches);
}


shared_ptr<vector<int>> MinisatSolver::GetUC(bool prime) {
    shared_ptr<vector<int>> uc(new vector<int>());
    uc->reserve(conflict.size());
    if (prime) {
        for (int i = 0; i < conflict.size(); ++i) {
            int val = -GetLiteralId(conflict[i]);
            vector<int> ids = m_model->GetPrevious(val);
            if (val > 0) {
                for (auto x : ids) {
                    uc->push_back(x);
                }
            } else {
                for (auto x : ids) {
                    uc->push_back(-x);
                }
            }
        }
    } else {
        for (int i = 0; i < conflict.size(); ++i) {
            int val = -GetLiteralId(conflict[i]);
            if (m_model->IsLatch(val)) {
                uc->emplace_back(val);
            }
        }
    }

    sort(uc->begin(), uc->end(), cmp);
    return uc;
}


inline int MinisatSolver::GetLiteralId(const Minisat::Lit &l) {
    return sign(l) ? -(var(l) + 1) : var(l) + 1;
}


void MinisatSolver::AddTempClause(const vector<int> &cls) {
    m_tempVar = GetNewVar();
    vector<int> temp_cls = cls;
    temp_cls.push_back(-m_tempVar);
}


void MinisatSolver::ReleaseTempClause() {
    releaseVar(~GetLit(m_tempVar));
    m_tempVar = 0;
}

} // namespace car