#include "CadicalSolver.h"
#include <algorithm>

namespace car {
CadicalSolver::CadicalSolver() {
    m_assumptions = make_shared<cube>();
    m_tempClause = cube();
}

CadicalSolver::~CadicalSolver() {}

bool CadicalSolver::Solve() {
    for (auto it : *m_assumptions) {
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


bool CadicalSolver::Solve(const shared_ptr<cube> assumption) {
    m_assumptions->clear();
    m_assumptions->resize(assumption->size());
    std::copy(assumption->begin(), assumption->end(), m_assumptions->begin());
    return Solve();
}


void CadicalSolver::AddAssumption(const shared_ptr<cube> assumption) {
    for (auto it : *assumption) {
        m_assumptions->push_back(it);
    }
}


void CadicalSolver::AddClause(const cube &cls) {
    clause(cls);
}


pair<shared_ptr<cube>, shared_ptr<cube>> CadicalSolver::GetAssignment(bool prime) {
    shared_ptr<cube> inputs(new cube());
    shared_ptr<cube> latches(new cube());
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
            if (val(p) > 0) {
                latches->emplace_back(i);
            } else {
                assert(val(p) < 0);
                latches->emplace_back(-i);
            }
        }
    }
    return pair<shared_ptr<cube>, shared_ptr<cube>>(inputs, latches);
}


shared_ptr<cube> CadicalSolver::GetUC(bool prime) {
    shared_ptr<cube> uc(new cube());
    if (prime) {
        for (auto v : *m_assumptions) {
            if (failed(v)) {
                cube ids = m_model->GetPrevious(v);
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


void CadicalSolver::AddTempClause(const cube &cls) {
    m_tempClause = cls;
}


void CadicalSolver::ReleaseTempClause() {
    m_tempClause.clear();
}


} // namespace car