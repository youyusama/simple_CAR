#ifndef LOG_H
#define LOG_H

#include "Settings.h"
#include "signal.h"
#include <assert.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace car {

string CubeToStr(const vector<int> &c);

void compress_vector(vector<int> &res, const vector<int> &v);

string CubeToStrShort(const vector<int> &c);

void signalHandler(int signum);

class Log {
  public:
    struct CustomTimeStat {
        uint64_t calls = 0;
        chrono::microseconds total{0};
    };

    class ScopedTimer {
      public:
        ScopedTimer() : m_log(nullptr), m_active(false) {}
        ScopedTimer(Log &log, string name);
        ~ScopedTimer();
        ScopedTimer(const ScopedTimer &) = delete;
        ScopedTimer &operator=(const ScopedTimer &) = delete;
        ScopedTimer(ScopedTimer &&other) noexcept;
        ScopedTimer &operator=(ScopedTimer &&other) noexcept;

      private:
        Log *m_log;
        bool m_active = false;
    };

    Log(int verb, bool detailedTimers) : m_verbosity(verb),
                                         m_detailedTimers(detailedTimers) {
        m_begin = chrono::steady_clock::now();
        m_tick = chrono::steady_clock::now();
    }

    ~Log() {}

    template <typename... Args>
    void L(int messageVerbosity, const Args &...args) {
        if (messageVerbosity <= m_verbosity) {
            ostringstream oss;
            logHelper(oss, args...);
            cout << oss.str() << endl;
        }
    }

    void PrintTotalTime();

    void PrintCustomStatistics();

    ScopedTimer Section(const string &name) {
        if (!m_detailedTimers) return ScopedTimer();
        return ScopedTimer(*this, name);
    }

    inline void Tick() {
        m_tick = chrono::steady_clock::now();
    }

    inline double Tock() {
        return chrono::duration_cast<chrono::duration<double>>(
                   chrono::steady_clock::now() - m_tick)
            .count();
    }

    inline double GetTimeDouble(chrono::microseconds time) {
        return chrono::duration_cast<chrono::duration<double>>(time).count();
    }

    void SetVerbosity(int verb) { m_verbosity = verb; }

  private:
    struct ActiveSection {
        string name;
        chrono::time_point<chrono::steady_clock> start;
        chrono::microseconds elapsed{0};
    };

    friend class ScopedTimer;

    template <typename T, typename... Args>
    void logHelper(std::ostringstream &oss, const T &first, const Args &...args) {
        oss << first;
        logHelper(oss, args...);
    }

    template <typename T>
    void logHelper(std::ostringstream &oss, const T &last) {
        oss << last;
    }

    int m_verbosity;
    bool m_detailedTimers;

    chrono::time_point<chrono::steady_clock> m_tick;
    chrono::time_point<chrono::steady_clock> m_begin;

    unordered_map<string, CustomTimeStat> m_customStats;
    vector<ActiveSection> m_timerStack;

    void AddCustomTime(const string &name, chrono::microseconds time);
    void BeginSection(const string &name);
    void EndSection();
};

extern Log *GLOBAL_LOG;

} // namespace car

#endif
