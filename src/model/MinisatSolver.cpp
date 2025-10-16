#include "MinisatSolver.h"
#include <algorithm>

namespace car {
MinisatSolver::MinisatSolver(shared_ptr<Model> m) {
    m_model = m;
    m_maxId = m_model->TrueId() + 1; // reserve variable numbers for one step reachability check
    m_tempVar = 0;
}

MinisatSolver::~MinisatSolver() {}

bool MinisatSolver::Solve() {
    if (m_tempVar != 0) m_assumptions.push(GetLit(m_tempVar));
    Minisat::lbool result = solveLimited(m_assumptions);
    if (result == Minisat::l_True) {
        return true;
    } else {
        assert(result == Minisat::l_False);
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
    Minisat::vec<Minisat::Lit> literals;
    for (int l : cls) {
        literals.push(GetLit(l));
        if (abs(l) > m_maxId) m_maxId = abs(l) + 1;
    }
    bool result = addClause(literals);
    // result may be false when the clause is already conflict
    // assert(result != false);
}


pair<shared_ptr<cube>, shared_ptr<cube>> MinisatSolver::GetAssignment(bool prime) {
    assert(m_model->GetNumInputs() < nVars());
    shared_ptr<cube> inputs(new cube());
    shared_ptr<cube> latches(new cube());
    inputs->reserve(m_model->GetNumInputs());
    latches->reserve(m_model->GetNumLatches());
    for (int i : m_model->GetModelInputs()) {
        if (model[i] == Minisat::l_True) {
            inputs->emplace_back(i);
        } else {
            assert(model[i] == Minisat::l_False);
            inputs->emplace_back(-i);
        }
    }
    for (int i : m_model->GetModelLatches()) {
        if (!prime) {
            if (model[i] == Minisat::l_True) {
                latches->emplace_back(i);
            } else {
                assert(model[i] == Minisat::l_False);
                latches->emplace_back(-i);
            }
        } else {
            int p = m_model->GetPrime(i);
            Minisat::lbool val = model[abs(p)];
            if ((val == Minisat::l_True && p > 0) || (val == Minisat::l_False && p < 0)) {
                latches->emplace_back(i);
            } else {
                latches->emplace_back(-i);
            }
        }
    }
    for (int i : m_model->GetInnards()) {
        if (!prime) {
            if (model[i] == Minisat::l_True) {
                latches->emplace_back(i);
            } else {
                assert(model[i - 1] == Minisat::l_False);
                latches->emplace_back(-i);
            }
        } else {
            int p = m_model->GetPrime(i);
            Minisat::lbool val = model[abs(p)];
            if ((val == Minisat::l_True && p > 0) || (val == Minisat::l_False && p < 0)) {
                latches->emplace_back(i);
            } else {
                latches->emplace_back(-i);
            }
        }
    }
    return pair<shared_ptr<cube>, shared_ptr<cube>>(inputs, latches);
}

unordered_set<int> MinisatSolver::GetConflict() {
    unordered_set<int> conflictSet;
    for (int i = 0; i < conflict.size(); ++i) {
        int val = -GetLiteralId(conflict[i]);
        conflictSet.insert(val);
    }
    return conflictSet;
}


inline int MinisatSolver::GetLiteralId(const Minisat::Lit &l) {
    return sign(l) ? -var(l) : var(l);
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
    Minisat::Lit p = m_assumptions.last();
    m_assumptions.pop();
    return GetLiteralId(p);
}

} // namespace car