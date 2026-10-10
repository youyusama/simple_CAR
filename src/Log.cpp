#include "Log.h"
#include <algorithm>
#include <utility>
#include <vector>

namespace car {

void SignalHandler(int signum) {
    if (global_log != nullptr) {
        global_log->PrintCustomStatistics();
        LOG_LP(global_log, 0, "Unknown");
    }
    exit(signum);
}

Log::ScopedTimer::ScopedTimer(Log &log, std::string name)
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


Log *global_log = nullptr;


std::string CubeToStr(const Cube &c) {
    std::string s;
    for (Lit lit : c) s.append(std::to_string(ToSigned(lit)) + " ");
    return s;
}

namespace {

void CompressVector(std::vector<int> &res, const Cube &v) {
    int count = 0;
    int packed = 0;
    for (size_t i = 0; i < v.size(); ++i) {
        packed <<= 1;
        if (!Sign(v[i])) packed += 1;
        count++;
        if (count == 32 || i + 1 == v.size()) {
            res.emplace_back(packed);
            packed = 0;
            count = 0;
        }
    }
}

} // namespace

std::string CubeToStrShort(const Cube &c) {
    std::vector<int> compressed;
    CompressVector(compressed, c);
    std::string s;
    for (int value : compressed) s.append(std::to_string(value) + " ");
    return s;
}


void Log::AddCustomTime(const std::string &name, std::chrono::microseconds time) {
    auto &stat = m_customStats[name];
    stat.calls++;
    stat.total += time;
}


void Log::BeginSection(const std::string &name) {
    auto now = std::chrono::steady_clock::now();
    if (!m_timerStack.empty()) {
        auto &current = m_timerStack.back();
        current.elapsed += std::chrono::duration_cast<std::chrono::microseconds>(now - current.start);
    }
    m_timerStack.push_back({name, now, std::chrono::microseconds{0}});
}


void Log::EndSection() {
    if (m_timerStack.empty()) return;
    auto now = std::chrono::steady_clock::now();
    auto current = m_timerStack.back();
    m_timerStack.pop_back();
    current.elapsed += std::chrono::duration_cast<std::chrono::microseconds>(now - current.start);
    AddCustomTime(current.name, current.elapsed);
    if (!m_timerStack.empty()) {
        m_timerStack.back().start = now; // resume parent from now to avoid overlap
    }
}


void Log::PrintTotalTime() {
    if (m_verbosity == 0) return;
    std::cout << "Time spent: " << std::fixed << std::setprecision(2)
         << GetTimeDouble(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - m_begin))
         << std::endl;
}


void Log::PrintCustomStatistics() {
    if (!m_detailedTimers) return;

    std::vector<std::pair<std::string, CustomTimeStat>> sorted(m_customStats.begin(), m_customStats.end());
    std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) {
        return a.second.total > b.second.total;
    });

    if (!sorted.empty()) {
        std::cout << std::endl
             << "Detailed Timers:" << std::endl;
    }
    double total_sum = 0.0;
    for (const auto &entry : sorted) {
        double total = GetTimeDouble(entry.second.total);
        double per = entry.second.calls == 0 ? 0.0 : total / entry.second.calls;
        std::cout << "  " << std::left << std::setw(18) << entry.first << "called: " << std::setw(10) << entry.second.calls
             << "takes: " << std::fixed << std::setprecision(3) << std::setw(10) << total
             << "per: " << std::fixed << std::setprecision(6) << per << std::endl;
        total_sum += total;
    }
    if (!sorted.empty()) {
        std::cout << "  " << std::left << std::setw(18) << "Sum"
             << "called: " << std::setw(10) << "-"
             << "takes: " << std::fixed << std::setprecision(3) << std::setw(10) << total_sum
             << "per: " << std::fixed << std::setprecision(6) << 0.0 << std::endl;
    }

    std::cout << "Total Time     spent: " << std::fixed << std::setprecision(2)
         << GetTimeDouble(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - m_begin))
         << std::endl;
}
} // namespace car
