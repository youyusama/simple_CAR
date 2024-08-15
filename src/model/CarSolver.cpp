#include "CarSolver.h"
#include <algorithm>
using namespace Minisat;

namespace car {
CarSolver::CarSolver() {}

CarSolver::~CarSolver() {
    ;
}


bool CarSolver::SolveWithAssumption() {
    lbool result = solveLimited(m_assumptions);
    if (result == l_True) {
        return true;
    } else {
        assert(result == l_False);
        return false;
    }
}


bool CarSolver::SolveWithAssumption(const shared_ptr<cube> assumption, int frameLevel) {
    m_assumptions.clear();
    m_assumptions.push(GetLit(GetFrameFlag(frameLevel)));
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
    lbool result = solveLimited(m_assumptions);
    if (result == l_True) {
        return true;
    } else {
        assert(result == l_False);
        return false;
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


void CarSolver::AddUnsatisfiableCore(const cube &uc, int frameLevel) {
    int flag = GetFrameFlag(frameLevel);
    vec<Lit> literals;
    literals.push(GetLit(-flag));
    if (m_isForward) {
        for (int i = 0; i < uc.size(); ++i) {
            literals.push(GetLit(-uc[i]));
        }
    } else {
        for (int i = 0; i < uc.size(); ++i) {
            literals.push(GetLit(-m_model->GetPrime(uc[i])));
        }
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


shared_ptr<vector<int>> CarSolver::GetModel() {
    shared_ptr<vector<int>> res(new vector<int>());
    res->resize(nVars(), 0);
    for (int i = 0; i < nVars(); i++) {
        if (model[i] == l_True) {
            res->at(i) = i + 1;
        } else {
            assert(model[i] == l_False);
            res->at(i) = -(i + 1);
        }
    }
    return res;
}


pair<shared_ptr<cube>, shared_ptr<cube>> CarSolver::GetAssignment() {
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
        if (m_isForward) {
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


void CarSolver::GetUnsatisfiableCoreFromBad(shared_ptr<cube> uc, int badId) {
    uc->reserve(conflict.size());
    int val;

    for (int i = 0; i < conflict.size(); ++i) {
        val = -GetLiteralId(conflict[i]);
        if (m_model->IsLatch(val) && val != badId) {
            uc->emplace_back(val);
        }
    }
    sort(uc->begin(), uc->end(), cmp);
}

void CarSolver::GetUnsatisfiableCore(shared_ptr<cube> uc) {
    uc->reserve(conflict.size());
    int val;
    if (m_isForward) {
        for (int i = 0; i < conflict.size(); ++i) {
            val = -GetLiteralId(conflict[i]);
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
            val = -GetLiteralId(conflict[i]);
            if (m_model->IsLatch(val)) {
                uc->emplace_back(val);
            }
        }
    }

    sort(uc->begin(), uc->end(), cmp);
}


// ================================================================================
// @brief: after sat solve, get uc
// @input:
// @output:
// ================================================================================
shared_ptr<cube> CarSolver::Getuc(bool minimal) {
    // get conflict as assumption
    LSet ass;
    for (int i = 0; i < conflict.size(); i++)
        ass.insert(~conflict[i]);

    // compute muc
    if (minimal) Getmuc(ass);

    // get <int> form muc
    int val;
    shared_ptr<cube> muc(new cube());
    if (m_isForward) {
        for (int i = 0; i < ass.size(); ++i) {
            val = GetLiteralId(ass[i]);
            vector<int> ids = m_model->GetPrevious(val);
            if (val > 0) {
                for (auto x : ids) {
                    muc->push_back(x);
                }
            } else {
                for (auto x : ids) {
                    muc->push_back(-x);
                }
            }
        }
    } else {
        for (int i = 0; i < ass.size(); ++i) {
            val = GetLiteralId(ass[i]);
            if (m_model->IsLatch(val)) {
                muc->emplace_back(val);
            }
        }
    }
    sort(muc->begin(), muc->end(), cmp);
    return muc;
}


// ================================================================================
// @brief: get muc by uc
// @input: assumption
// @output:
// ================================================================================
void CarSolver::Getmuc(LSet &ass) {
    if (ass.size() > 215) return;
    vec<Lit> tass;
    LSet mass;

    while (ass.size() > 0) {
        int sz = ass.size();
        Lit t = ass[sz - 1];
        tass.clear();
        for (int i = 0; i < sz - 1; i++) tass.push(ass[i]);
        for (int i = 0; i < mass.size(); i++) tass.push(mass[i]);
        // tass.push(GetLit(GetFrameFlag(level)));
        lbool result = solveLimited(tass);
        if (result == l_True) {
            mass.insert(t);
            ass.clear();
            for (int i = 0; i < sz - 1; i++) ass.insert(tass[i]);
        } else if (result == l_False) {
            ass.clear();
            for (int i = 0; i < conflict.size(); i++) {
                if (!mass.has(~conflict[i]))
                    ass.insert(~conflict[i]);
            }
        } else {
            assert(false);
        }
    }
    for (int i = 0; i < mass.size(); i++) ass.insert(mass[i]);
}


shared_ptr<vector<int>> CarSolver::justGetUC() {
    // get conflict as assumption
    LSet ass;
    for (int i = 0; i < conflict.size(); i++)
        ass.insert(~conflict[i]);

    // compute muc
    // Getmuc(ass);

    shared_ptr<vector<int>> uc(new vector<int>());
    uc->reserve(ass.size());
    int val;
    for (int i = 0; i < ass.size(); ++i) {
        val = GetLiteralId(ass[i]);
        if (m_model->IsLatch(val)) {
            uc->emplace_back(val);
        }
    }
    sort(uc->begin(), uc->end(), cmp);
    return uc;
}


void CarSolver::CleanAssumptions() {
    m_assumptions.clear();
}


void CarSolver::AddNewFrame(const vector<shared_ptr<cube>> &frame, int frameLevel) {
    for (int i = 0; i < frame.size(); ++i) {
        AddUnsatisfiableCore(*frame[i], frameLevel);
    }
}


bool CarSolver::SolveWithAssumptionAndBad(const shared_ptr<cube> assumption, int badId) {
    m_assumptions.clear();
    m_assumptions.push(GetLit(badId));
    for (auto it : *assumption) {
        m_assumptions.push(GetLit(it));
    }
    lbool result = solveLimited(m_assumptions);
    if (result == l_True) {
        return true;
    } else {
        assert(result == l_False);
        return false;
    }
}


int CarSolver::GetTempFlag() {
    return GetNewVar();
}


void CarSolver::AddTempClause(clause *cls, int temp_flag, bool is_primed) {
    vector<int> *temp_cls = new vector<int>();
    temp_cls->emplace_back(-temp_flag);
    for (int l : *cls) {
        if (is_primed)
            temp_cls->emplace_back(m_model->GetPrime(l));
        else
            temp_cls->emplace_back(l);
    }
    AddClause(*temp_cls);
}


void CarSolver::ReleaseTempClause(int temp_flag) {
    releaseVar(~GetLit(temp_flag));
}


#pragma region private


inline int CarSolver::GetFrameFlag(int frameLevel) {
    assert(frameLevel >= 0);
    while (m_frameFlags.size() <= frameLevel) {
        m_frameFlags.emplace_back(m_maxFlag++);
    }
    return m_frameFlags[frameLevel];
}

inline int CarSolver::GetLiteralId(const Minisat::Lit &l) {
    return sign(l) ? -(var(l) + 1) : var(l) + 1;
}


#pragma endregion


} // namespace car