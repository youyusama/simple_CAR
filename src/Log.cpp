#include "Log.h"
#include <algorithm>
#include <utility>
#include <vector>

namespace car {

void signalHandler(int signum) {
    if (GLOBAL_LOG != nullptr) {
        GLOBAL_LOG->PrintCustomStatistics();
        GLOBAL_LOG->L(0, "Unknown");
    }
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


Log *GLOBAL_LOG = nullptr;


string CubeToStr(const vector<int> &c) {
    string s;
    for (int l : c) s.append(to_string(l) + " ");
    return s;
}


void compress_vector(vector<int> &res, const vector<int> &v) {
    int count = 0;
    int tempi = 0;
    for (int l : v) {
        if (l > 0)
            tempi = (tempi << 1) + 1;
        else
            tempi <<= 1;
        count++;
        if (count == 32 || l == v.back()) {
            res.emplace_back(tempi);
            tempi = 0;
            count = 0;
        }
    }
}


string CubeToStrShort(const vector<int> &c) {
    vector<int> s;
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


void Log::PrintTotalTime() {
    if (m_verbosity == 0) return;
    cout << "Time spent: " << fixed << setprecision(2)
         << GetTimeDouble(chrono::duration_cast<std::chrono::microseconds>(
                chrono::steady_clock::now() - m_begin))
         << endl;
}


void Log::PrintCustomStatistics() {
    if (!m_detailedTimers) return;

    vector<pair<string, CustomTimeStat>> sorted(m_customStats.begin(), m_customStats.end());
    sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) {
        return a.second.total > b.second.total;
    });

    if (!sorted.empty()) {
        cout << endl
             << "Detailed Timers:" << endl;
    }
    double totalSum = 0.0;
    for (const auto &entry : sorted) {
        double total = GetTimeDouble(entry.second.total);
        double per = entry.second.calls == 0 ? 0.0 : total / entry.second.calls;
        cout << "  " << left << setw(18) << entry.first << "called: " << setw(10) << entry.second.calls
             << "takes: " << fixed << setprecision(3) << setw(10) << total
             << "per: " << fixed << setprecision(6) << per << endl;
        totalSum += total;
    }
    if (!sorted.empty()) {
        cout << "  " << left << setw(18) << "Sum"
             << "called: " << setw(10) << "-"
             << "takes: " << fixed << setprecision(3) << setw(10) << totalSum
             << "per: " << fixed << setprecision(6) << 0.0 << endl;
    }

    cout << "Total Time     spent: " << fixed << setprecision(2)
         << GetTimeDouble(chrono::duration_cast<std::chrono::microseconds>(
                chrono::steady_clock::now() - m_begin))
         << endl;
}
} // namespace car
