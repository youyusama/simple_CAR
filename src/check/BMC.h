#ifndef BMC_H
#define BMC_H

#include "BaseChecker.h"
#include "CarSolver.h"
#include "Log.h"

namespace car {


class CNFWriter {
  public:
    CNFWriter(string filePath) {
        m_maxId = 0;
        m_filePath = filePath;
    };
    void AppendClause(clause c) {
        m_clauses.push_back(c);
        for (int v : c)
            if (abs(v) > m_maxId) m_maxId = abs(v);
    }
    void WriteFile();

  private:
    string m_filePath;
    vector<clause> m_clauses;
    int m_maxId;
};


class BMC : public BaseChecker {
  public:
    BMC(Settings settings,
        shared_ptr<AigerModel> model,
        shared_ptr<Log> log);

    bool Run();
    bool Check(int badId);
    void Build(int badId);

  private:
    Settings m_settings;
    shared_ptr<Log> m_log;
    shared_ptr<AigerModel> m_model;
    int m_k;
    int m_maxK;
    shared_ptr<State> m_initialState;
    int m_badId;
    shared_ptr<CarSolver> m_Solver;

    void Init(int badId);
    void OutputCounterExample(int bad);
    void GetClausesK(int m_k, shared_ptr<vector<clause>> clauses);
    int GetBadK(int m_k);
    vector<int> GetConstraintsK(int m_k);

    shared_ptr<CNFWriter> m_cnfw;
};

} // namespace car

#endif