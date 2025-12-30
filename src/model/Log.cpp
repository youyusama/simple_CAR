#include "Log.h"
#include <algorithm>
#include <utility>
#include <vector>

namespace car {

void signalHandler(int signum) {
    GLOBAL_LOG->PrintStatistics();
    GLOBAL_LOG->L(0, "Unknown");
    exit(signum);
}

Log::ScopedTimer::ScopedTimer(Log &log, string name)
    : m_log(&log),
      m_active(true) {
    m_log->BeginSection(name);
}

Log::ScopedTimer::ScopedTimer(Log::ScopedTimer &&other) noexcept
    : m_log(other.m_log),
      m_active(other.m_active) {
    other.m_active = false;
}

Log::ScopedTimer &Log::ScopedTimer::operator=(Log::ScopedTimer &&other) noexcept {
    if (this != &other) {
        if (m_active && m_log != nullptr) {
            m_log->EndSection();
        }
        m_log = other.m_log;
        m_active = other.m_active;
        other.m_active = false;
    }
    return *this;
}

Log::ScopedTimer::~ScopedTimer() {
    if (m_active && m_log != nullptr) {
        m_log->EndSection();
    }
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


void Log::AddCustomTime(const string &name, chrono::microseconds time) {
    auto &stat = m_customStats[name];
    stat.calls++;
    stat.total += time;
}


void Log::BeginSection(const string &name) {
    auto now = chrono::steady_clock::now();
    if (!m_timerStack.empty()) {
        auto &current = m_timerStack.back();
        current.elapsed += chrono::duration_cast<chrono::microseconds>(now - current.start);
    }
    m_timerStack.push_back({name, now, chrono::microseconds{0}});
}


void Log::EndSection() {
    if (m_timerStack.empty()) return;
    auto now = chrono::steady_clock::now();
    auto current = m_timerStack.back();
    m_timerStack.pop_back();
    current.elapsed += chrono::duration_cast<chrono::microseconds>(now - current.start);
    AddCustomTime(current.name, current.elapsed);
    if (!m_timerStack.empty()) {
        m_timerStack.back().start = now; // resume parent from now to avoid overlap
    }
}


void Log::PrintCustomStatistics() {
    if (m_customStats.empty()) return;

    vector<pair<string, CustomTimeStat>> sorted(m_customStats.begin(), m_customStats.end());
    sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) {
        return a.second.total > b.second.total;
    });

    cout << "Detailed Timers:" << endl;
    double totalSum = 0.0;
    for (const auto &entry : sorted) {
        double total = GetTimeDouble(entry.second.total);
        double per = entry.second.calls == 0 ? 0.0 : total / entry.second.calls;
        cout << "  " << left << setw(18) << entry.first << "called: " << setw(10) << entry.second.calls
             << "takes: " << fixed << setprecision(3) << setw(10) << total
             << "per: " << fixed << setprecision(5) << per << endl;
        totalSum += total;
    }
    cout << "  " << left << setw(18) << "Sum"
         << "called: " << setw(10) << "-"
         << "takes: " << fixed << setprecision(3) << setw(10) << totalSum
         << "per: " << fixed << setprecision(5) << 0.0 << endl;
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

    PrintCustomStatistics();

    cout << "Total Time     spent: " << fixed << setprecision(5) << GetTimeDouble(chrono::duration_cast<std::chrono::microseconds>(chrono::steady_clock::now() - m_begin)) << endl;
}
} // namespace car
