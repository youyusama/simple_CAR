// created by Jianwen Li
// Kissat API for BMC
#include "KissatSolver.h"
#include <algorithm>

namespace car {
KissatSolver::KissatSolver(shared_ptr<Model> m) {
    m_model = m;
    m_maxId = m_model->TrueId() + 1;
    // m_tempVar = 0;
    m_solver = kissat_init();
    kissat_reserve(m_solver, m_maxId);
    assert(!m_solver);
}


bool KissatSolver::Solve() {
    int ret = kissat_solve(m_solver);
    if (ret == 10) // satisfiable
        return true;
    else if (ret == 20) // unsatisfiable
        return false;
    else { // UNKONW
        cout << "Kissat solve value UNKNOW!" << endl;
        exit(0);
    }
}

void KissatSolver::AddClause(const cube &cls) {
    for (int l : cls) {
        kissat_add(m_solver, l);
    }
    if (abs(cls[0]) > m_maxId) {
        m_maxId += m_maxId;
        kissat_reserve(m_solver, m_maxId);
    }
    kissat_add(m_solver, 0); // end of a clause
}


} // namespace car