#include "MinisatSolver.h"
#include <algorithm>

namespace car {
MinisatSolver::MinisatSolver(shared_ptr<Model> m) {
    m_model = m;
    m_maxId = m_model->GetMaxId();
    m_tempVar = 0;
}

MinisatSolver::~MinisatSolver() {}

bool MinisatSolver::Solve() {
    if (m_tempVar != 0) m_assumptions.push(GetLit(m_tempVar));
    lbool result = solveLimited(m_assumptions);
    if (result == l_True) {
        return true;
    } else {
        assert(result == l_False);
        return false;
    }
}


bool MinisatSolver::Solve(const shared_ptr<cube> assumption) {
    m_assumptions.clear();
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
    return Solve();
}


void MinisatSolver::AddAssumption(const shared_ptr<cube> assumption) {
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
}


void MinisatSolver::AddClause(const cube &cls) {
    vec<Lit> literals;
    for (int i = 0; i < cls.size(); ++i) {
        literals.push(GetLit(cls[i]));
    }
    bool result = addClause(literals);
    assert(result != false);
}


pair<shared_ptr<cube>, shared_ptr<cube>> MinisatSolver::GetAssignment(bool prime) {
    assert(m_model->GetNumInputs() < nVars());
    shared_ptr<cube> inputs(new cube());
    shared_ptr<cube> latches(new cube());
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
    for (int i : *m_model->GetInnards()) {
        if (!prime) {
            if (model[i - 1] == l_True) {
                latches->emplace_back(i);
            } else {
                assert(model[i - 1] == l_False);
                latches->emplace_back(-i);
            }
        } else {
            int p = m_model->GetPrime(i);
            lbool val = model[abs(p) - 1];
            if ((val == l_True && p > 0) || (val == l_False && p < 0)) {
                latches->emplace_back(i);
            } else {
                latches->emplace_back(-i);
            }
        }
    }
    return pair<shared_ptr<cube>, shared_ptr<cube>>(inputs, latches);
}


shared_ptr<cube> MinisatSolver::GetUC(bool prime) {
    shared_ptr<cube> uc(new cube());
    uc->reserve(conflict.size());
    if (prime) {
        for (int i = 0; i < conflict.size(); ++i) {
            int val = -GetLiteralId(conflict[i]);
            cube ids = m_model->GetPrevious(val);
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
            if (m_model->IsLatch(val) || m_model->IsInnard(val)) {
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


void MinisatSolver::AddTempClause(const cube &cls) {
    m_tempVar = GetNewVar();
    cube temp_cls = cls;
    temp_cls.push_back(-m_tempVar);
    AddClause(temp_cls);
}


void MinisatSolver::ReleaseTempClause() {
    assert(m_tempVar != 0);
    releaseVar(~GetLit(m_tempVar));
    m_tempVar = 0;
}


void MinisatSolver::ClearAssumption() {
    m_assumptions.clear();
}


void MinisatSolver::PushAssumption(int a) {
    m_assumptions.push(GetLit(a));
}


int MinisatSolver::PopAssumption() {
    Lit p = m_assumptions.last();
    m_assumptions.pop();
    return GetLiteralId(p);
}

} // namespace car