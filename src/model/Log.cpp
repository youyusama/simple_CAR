#include "Log.h"

namespace car {

void signalHandler(int signum) {
    GLOBAL_LOG->PrintStatistics();
    GLOBAL_LOG->L(0, "Unknown");
    exit(signum);
}


shared_ptr<Log> GLOBAL_LOG = nullptr;


string CubeToStr(const shared_ptr<vector<int>> c) {
    string s;
    for (int l : *c) s.append(to_string(l) + " ");
    return s;
}


void compress_vector(shared_ptr<vector<int>> res, const shared_ptr<vector<int>> v) {
    int count = 0;
    int tempi = 0;
    for (int l : *v) {
        if (l > 0)
            tempi = (tempi << 1) + 1;
        else
            tempi <<= 1;
        count++;
        if (count == 32 || l == v->back()) {
            res->emplace_back(tempi);
            tempi = 0;
            count = 0;
        }
    }
}


string CubeToStrShort(const shared_ptr<vector<int>> c) {
    shared_ptr<vector<int>> s(new vector<int>());
    compress_vector(s, c);
    return CubeToStr(s);
}


void Log::PrintStatistics() {
    if (m_verbosity == 0) return;

    double mainSolverTime = GetTimeDouble(m_mainSolverTime);
    cout << endl
         << "TransSolver    called: " << left << setw(12) << m_mainSolverCalls
         << "takes: " << fixed << setprecision(3) << setw(10) << mainSolverTime
         << "per: " << fixed << setprecision(5) << mainSolverTime / m_mainSolverCalls << endl;

    double propagationTime = GetTimeDouble(m_propagationTime);
    cout << "Propagation    called: " << left << setw(12) << m_propagation
         << "takes: " << fixed << setprecision(3) << setw(10) << propagationTime
         << "per: " << fixed << setprecision(5) << propagationTime / m_propagation << endl;

    double liftSolverTime = GetTimeDouble(m_liftSolverTime);
    cout << "LiftSolver     called: " << left << setw(12) << m_liftSolverCalls
         << "takes: " << fixed << setprecision(3) << setw(10) << liftSolverTime
         << "per: " << fixed << setprecision(5) << liftSolverTime / m_liftSolverCalls << endl;

    double invSolverTime = GetTimeDouble(m_invSolverTime);
    cout << "InvSolver      called: " << left << setw(12) << m_invSolverCalls
         << "takes: " << fixed << setprecision(3) << setw(10) << invSolverTime
         << "per: " << fixed << setprecision(5) << invSolverTime / m_invSolverCalls << endl;

    double startSolverTime = GetTimeDouble(m_enumerateStartStateTime);
    cout << "StartSolver    called: " << left << setw(12) << m_enumerateStartState
         << "takes: " << fixed << setprecision(3) << setw(10) << startSolverTime
         << "per: " << fixed << setprecision(5) << startSolverTime / m_enumerateStartState << endl;

    double getNewLevelTime = GetTimeDouble(m_getNewLevelTime);
    cout << "Locating lvl   called: " << left << setw(12) << m_getNewLevel
         << "takes: " << fixed << setprecision(3) << setw(10) << getNewLevelTime
         << "per: " << fixed << setprecision(5) << getNewLevelTime / m_getNewLevel << endl;

    double updateUcTime = GetTimeDouble(m_updateUcTime);
    cout << "Updating UC    called: " << left << setw(12) << m_updateUc
         << "takes: " << fixed << setprecision(3) << setw(10) << updateUcTime
         << "per: " << fixed << setprecision(5) << updateUcTime / m_updateUc << endl;

    cout << "Initialization takes: " << fixed << setprecision(5) << GetTimeDouble(m_initTime) << endl;
    cout << "Innards        takes: " << fixed << setprecision(5) << GetTimeDouble(m_internalSignalsTime) << endl;
    cout << "Total Time     spent: " << fixed << setprecision(5) << GetTimeDouble(chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - m_begin)) << endl;
}
} // namespace car