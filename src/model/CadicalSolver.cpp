#include "CadicalSolver.h"
#include <algorithm>

namespace car {
CadicalSolver::CadicalSolver() {}

CadicalSolver::~CadicalSolver() {}

bool CadicalSolver::Solve() {
    for (auto it : *m_assumptions) {
        assume(it);
    }
    int result = solve();
    if (result == 10) {
        return true;
    } else {
        assert(result == 20);
        return false;
    }
}


bool CadicalSolver::Solve(const shared_ptr<vector<int>> assumption) {
    m_assumptions = assumption;
    if (m_tempClause.size() > 0) {
        for (int l : m_tempClause) {
            constrain(l);
        }
        constrain(0);
    }
    return Solve();
}


void CadicalSolver::AddAssumption(const shared_ptr<vector<int>> assumption) {
    for (auto it : *assumption) {
        m_assumptions->push_back(it);
    }
}


void CadicalSolver::AddClause(const vector<int> &cls) {
    clause(cls);
}


pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>> CadicalSolver::GetAssignment(bool prime) {
    shared_ptr<vector<int>> inputs(new vector<int>());
    shared_ptr<vector<int>> latches(new vector<int>());
    inputs->reserve(m_model->GetNumInputs());
    latches->reserve(m_model->GetNumLatches());
    for (int i = 1; i < m_model->GetNumInputs() + 1; ++i) {
        if (val(i) > 0) {
            inputs->emplace_back(i);
        } else {
            assert(val(i) < 0);
            inputs->emplace_back(-i);
        }
    }
    for (int i = m_model->GetNumInputs() + 1, end = m_model->GetNumInputs() + m_model->GetNumLatches() + 1; i < end; ++i) {
        if (!prime) {
            if (val(i) > 0) {
                latches->emplace_back(i);
            } else {
                assert(val(i) < 0);
                latches->emplace_back(-i);
            }
        } else {
            int p = m_model->GetPrime(i);
            if ((val(p) > 0 && p > 0) || (val(p) < 0 && p < 0)) {
                latches->emplace_back(i);
            } else {
                latches->emplace_back(-i);
            }
        }
    }
    return pair<shared_ptr<vector<int>>, shared_ptr<vector<int>>>(inputs, latches);
}


shared_ptr<vector<int>> CadicalSolver::GetUC(bool prime) {
    shared_ptr<vector<int>> uc(new vector<int>());
    if (prime) {
        for (auto v : *m_assumptions) {
            if (failed(v)) {
                vector<int> ids = m_model->GetPrevious(v);
                if (v > 0) {
                    for (auto x : ids) {
                        uc->push_back(x);
                    }
                } else {
                    for (auto x : ids) {
                        uc->push_back(-x);
                    }
                }
            }
        }
    } else {
        for (auto v : *m_assumptions) {
            if (failed(v) && m_model->IsLatch(v)) {
                uc->emplace_back(v);
            }
        }
    }

    sort(uc->begin(), uc->end(), cmp);
    return uc;
}


void CadicalSolver::AddTempClause(const vector<int> &cls) {
    m_tempClause = cls;
}


void CadicalSolver::ReleaseTempClause() {
    m_tempClause.clear();
}


} // namespace car