#include "CarSolver.h"
#include <algorithm>
using namespace Minisat;

namespace car {
CarSolver::CarSolver() {}

CarSolver::~CarSolver() {
    ;
}


bool CarSolver::Solve() {
    lbool result = solveLimited(m_assumptions);
    if (result == l_True) {
        return true;
    } else {
        assert(result == l_False);
        return false;
    }
}


bool CarSolver::Solve(const shared_ptr<cube> assumption) {
    m_assumptions.clear();
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
    return Solve();
}


bool CarSolver::Solve(const shared_ptr<cube> assumption, int frameLevel) {
    m_assumptions.clear();
    m_assumptions.push(GetLit(GetFrameFlag(frameLevel)));
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
    return Solve();
}


void CarSolver::AddAssumption(const shared_ptr<cube> assumption) {
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
}


void CarSolver::AddClause(const clause &cls) {
    vec<Lit> literals;
    for (int i = 0; i < cls.size(); ++i) {
        literals.push(GetLit(cls[i]));
    }
    bool result = addClause(literals);
    assert(result != false);
}


void CarSolver::AddUC(const cube &uc, int frameLevel) {
    int flag = GetFrameFlag(frameLevel);
    vec<Lit> literals;
    literals.push(GetLit(-flag));
    for (int i = 0; i < uc.size(); ++i) {
        literals.push(GetLit(-uc[i]));
    }
    bool result = addClause(literals);
    assert(result != false);
}


void CarSolver::AddConstraintOr(const vector<shared_ptr<cube>> frame) {
    clause cls;
    for (int i = 0; i < frame.size(); ++i) {
        int flag = GetNewVar();
        cls.push_back(flag);
        for (int j = 0; j < frame[i]->size(); ++j) {
            AddClause(clause{-flag, frame[i]->at(j)});
        }
    }
    AddClause(cls);
}


void CarSolver::AddConstraintAnd(const vector<shared_ptr<cube>> frame) {
    int flag = GetNewVar();
    for (int i = 0; i < frame.size(); ++i) {
        clause cls;
        for (int j = 0; j < frame[i]->size(); ++j) {
            cls.push_back(-frame[i]->at(j));
        }
        cls.push_back(-flag);
        AddClause(cls);
    }
    AddAssumption(flag);
}


void CarSolver::FlipLastConstrain() {
    Lit lit = m_assumptions.last();
    m_assumptions.pop();
    releaseVar(~lit);
}


pair<shared_ptr<cube>, shared_ptr<cube>> CarSolver::GetAssignment(bool prime) {
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
    return pair<shared_ptr<cube>, shared_ptr<cube>>(inputs, latches);
}


shared_ptr<cube> CarSolver::GetUC(bool prime) {
    shared_ptr<cube> uc(new cube());
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


inline int CarSolver::GetFrameFlag(int frameLevel) {
    assert(frameLevel >= 0);
    while (m_frameFlags.size() <= frameLevel) {
        m_frameFlags.emplace_back(GetNewVar());
    }
    return m_frameFlags[frameLevel];
}


inline int CarSolver::GetLiteralId(const Minisat::Lit &l) {
    return sign(l) ? -(var(l) + 1) : var(l) + 1;
}


} // namespace car