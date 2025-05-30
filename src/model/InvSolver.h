#ifndef INVSOLVER_H
#define INVSOLVER_H

#ifdef CADICAL
#include "CadicalSolver.h"
#else
#include "MinisatSolver.h"
#endif

namespace car {

class InvSolver :
#ifdef CADICAL
    public CadicalSolver
#else
    public MinisatSolver
#endif
{
  public:
    InvSolver(std::shared_ptr<Model> model) {
        m_model = model;
        m_maxId = model->GetMaxId();
        // auto &clauses = m_model->GetClauses();
        // for (int i = 0, end = model->GetOutputsStart(); i < end; i++) {
        //     AddClause(clauses[i]);
        // }
        // for (auto c : m_model->GetConstraints()) {
        //     AddClause({c});
        // }
    }

    void AddConstraintOr(const vector<shared_ptr<cube>> frame) {
        cube cls;
        for (int i = 0; i < frame.size(); ++i) {
            int flag = GetNewVar();
            cls.push_back(flag);
            for (int j = 0; j < frame[i]->size(); ++j) {
                AddClause(cube{-flag, frame[i]->at(j)});
            }
        }
        AddClause(cls);
    }

    void AddConstraintAnd(const vector<shared_ptr<cube>> frame) {
        int flag = GetNewVar();
        for (int i = 0; i < frame.size(); ++i) {
            cube cls;
            for (int j = 0; j < frame[i]->size(); ++j) {
                cls.push_back(-frame[i]->at(j));
            }
            cls.push_back(-flag);
            AddClause(cls);
        }
        shared_ptr<cube> f = make_shared<cube>();
        f->push_back(flag);
        AddAssumption(f);
    }


    void FlipLastConstrain() {
#ifdef CADICAL
        int v = m_assumptions->back();
        m_assumptions->pop_back();
        clause(-v);
#else
        Lit lit = m_assumptions.last();
        m_assumptions.pop();
        releaseVar(~lit);
#endif
    }

  private:
};

} // namespace car

#endif