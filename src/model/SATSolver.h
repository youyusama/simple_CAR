#ifndef SATSOLVER_H
#define SATSOLVER_H

#ifdef CADICAL
#include "CadicalSolver.h"
#endif
#ifdef KISSAT
#include "KissatSolver.h"
#endif
#include "ISolver.h"
#include "MinicoreSolver.h"
#include "MinisatSolver.h"
#include "Model.h"
#include <memory>

namespace car {

class SATSolver : public ISolver {
  public:
    SATSolver(shared_ptr<Model> model, MCSATSolver slv_kind);
    ~SATSolver() {}

    SATSolver(const SATSolver &src)
        : m_model(src.m_model),
          m_slvKind(src.m_slvKind),
          m_solveInDomain(src.m_solveInDomain),
          m_frameFlags(src.m_frameFlags)
    {
        auto cloned = src.m_slv->Clone();
        if (!cloned) {
            throw std::invalid_argument("Cloning only supported for CaDiCaL solver");
        }
        m_slv = cloned;
    }

    explicit SATSolver(const std::shared_ptr<SATSolver> &other)
        : SATSolver(*other) {}

    int GetOption(const char* name) override {
        return m_slv->GetOption(name);
    }
    bool SetOption(const char* name, int val) override {
        cout << "name: " << name << ", val: " << val << endl;
        bool success = m_slv->SetOption(name, val);
        cout << "success: " << success << endl;
        return success;
    }
    int Simplify(int rounds = 3) override {
        cout << "simplify rounds: " << rounds << endl;
        return m_slv->Simplify(rounds);
    }

    // general SAT interface
    void AddClause(const cube &cls) {
        m_slv->AddClause(cls);
    }

    void AddAssumption(const shared_ptr<cube> assumption) {
        m_slv->AddAssumption(assumption);
    }

    bool Solve() {
        return m_slv->Solve();
    }

    bool Solve(const shared_ptr<cube> assumption) {
        return m_slv->Solve(assumption);
    }

    pair<shared_ptr<cube>, shared_ptr<cube>> GetAssignment(bool prime) {
        return m_slv->GetAssignment(prime);
    }

    shared_ptr<cube> GetUC(bool prime) {
        return m_slv->GetUC(prime);
    }

    unordered_set<int> GetConflict() {
        return m_slv->GetConflict();
    }

    int GetNewVar() {
        return m_slv->GetNewVar();
    }

    void AddTempClause(const cube &cls) {
        m_slv->AddTempClause(cls);
    }

    void ReleaseTempClause() {
        m_slv->ReleaseTempClause();
    }

    bool GetModel(int id) {
        return m_slv->GetModel(id);
    }

    void ClearAssumption() {
        m_slv->ClearAssumption();
    }

    void PushAssumption(int a) {
        m_slv->PushAssumption(a);
    }

    int PopAssumption() {
        return m_slv->PopAssumption();
    }

    // special interface in minicore
    void SetSolveInDomain() {
        m_solveInDomain = true;
        if (m_slvKind != MCSATSolver::minicore) return;
        m_slv->SetSolveInDomain();
    }


    void SetDomain(const shared_ptr<cube> domain) {
        if (m_slvKind != MCSATSolver::minicore) return;
        shared_ptr<cube> d = make_shared<cube>();
        for (int v : *domain) d->emplace_back(abs(v));
        m_slv->SetDomain(d);
    }

    void SetTempDomain(const shared_ptr<cube> domain) {
        if (m_slvKind != MCSATSolver::minicore) return;
        shared_ptr<cube> d = make_shared<cube>();
        for (int v : *domain) d->emplace_back(abs(v));
        m_slv->SetTempDomain(d);
    }

    void ResetTempDomain() {
        if (m_slvKind != MCSATSolver::minicore) return;
        m_slv->ResetTempDomain();
    }

    void SetDomainCOI(const shared_ptr<cube> c) {
        if (m_slvKind != MCSATSolver::minicore) return;
        shared_ptr<cube> domain = m_model->GetCOIDomain(c);
        SetDomain(domain);
    }

    void SetTempDomainCOI(const shared_ptr<cube> c) {
        if (m_slvKind != MCSATSolver::minicore) return;
        shared_ptr<cube> domain = m_model->GetCOIDomain(c);
        SetTempDomain(domain);
    }

    // SAT interface for IC3/CAR
    void AddTrans();

    void AddTransK(int k);

    void AddConstraints();

    void AddConstraintsK(int k);

    void AddBad();

    void AddBadk(int k);

    void AddInitialClauses();

    bool SolveFrame(const shared_ptr<cube> assumption, int lvl);

    void AddUC(const shared_ptr<cube> uc, int lvl);

    void AddUC(const cube &uc, int lvl);

    void AddUC(const shared_ptr<cube> uc);

    void AddProperty();

    void FlipLastConstrain();

    void UpdateStartSolverFlag();

  protected:
    shared_ptr<Model> m_model;
    MCSATSolver m_slvKind;
    shared_ptr<ISolver> m_slv;
    bool m_solveInDomain;

    vector<int> m_frameFlags;
    inline int GetFrameFlag(int lvl) {
        assert(lvl >= 0);
        while (m_frameFlags.size() <= lvl) {
            m_frameFlags.emplace_back(GetNewVar());
        }
        return m_frameFlags[lvl];
    }
};

} // namespace car

#endif