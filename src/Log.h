#pragma once

#include "CarTypes.h"
#include "Settings.h"
#include "signal.h"
#include <assert.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace car {

std::string CubeToStr(const Cube &c);

std::string CubeToStrShort(const Cube &c);

void SignalHandler(int signum);

class Log {
  public:
    struct CustomTimeStat {
        uint64_t calls = 0;
        std::chrono::microseconds total{0};
    };

    class ScopedTimer {
      public:
        ScopedTimer() : m_log(nullptr), m_active(false) {}
        ScopedTimer(Log &log, std::string name);
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
        m_begin = std::chrono::steady_clock::now();
        m_tick = std::chrono::steady_clock::now();
    }

    ~Log() {}

    template <typename... Args>
    void L(const Args &...args) {
        std::ostringstream oss;
        LogHelper(oss, args...);
        std::cout << oss.str() << std::endl;
    }

    void PrintTotalTime();

    void PrintCustomStatistics();

    ScopedTimer Section(const std::string &name) {
        if (!m_detailedTimers) return ScopedTimer();
        return ScopedTimer(*this, name);
    }

    inline void Tick() {
        m_tick = std::chrono::steady_clock::now();
    }

    inline double Tock() {
        return std::chrono::duration_cast<std::chrono::duration<double>>(
                   std::chrono::steady_clock::now() - m_tick)
            .count();
    }

    inline double GetTimeDouble(std::chrono::microseconds time) {
        return std::chrono::duration_cast<std::chrono::duration<double>>(time).count();
    }

    void SetVerbosity(int verb) { m_verbosity = verb; }
    int Verbosity() const { return m_verbosity; }

  private:
    struct ActiveSection {
        std::string name;
        std::chrono::time_point<std::chrono::steady_clock> start;
        std::chrono::microseconds elapsed{0};
    };

    friend class ScopedTimer;

    template <typename T, typename... Args>
    void LogHelper(std::ostringstream &oss, const T &first, const Args &...args) {
        oss << first;
        LogHelper(oss, args...);
    }

    template <typename T>
    void LogHelper(std::ostringstream &oss, const T &last) {
        oss << last;
    }

    int m_verbosity;
    bool m_detailedTimers;

    std::chrono::time_point<std::chrono::steady_clock> m_tick;
    std::chrono::time_point<std::chrono::steady_clock> m_begin;

    std::unordered_map<std::string, CustomTimeStat> m_customStats;
    std::vector<ActiveSection> m_timerStack;

    void AddCustomTime(const std::string &name, std::chrono::microseconds time);
    void BeginSection(const std::string &name);
    void EndSection();
};

extern Log *global_log;

} // namespace car

#define LOG_L(log, level, ...)                         \
    do {                                               \
        auto &__log = (log);                           \
        if ((level) <= __log.Verbosity()) {            \
            __log.L(__VA_ARGS__);                      \
        }                                              \
    } while (0)

#define LOG_LP(logptr, level, ...)                       \
    do {                                                 \
        auto *__logp = (logptr);                         \
        if (__logp && (level) <= __logp->Verbosity()) {  \
            __logp->L(__VA_ARGS__);                      \
        }                                                \
    } while (0)
