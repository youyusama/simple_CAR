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

namespace car {

string CubeToStr(const shared_ptr<vector<int>> c);

void compress_vector(shared_ptr<vector<int>> res, const shared_ptr<vector<int>> v);

string CubeToStrShort(const shared_ptr<vector<int>> c);

void signalHandler(int signum);

class Log {
  public:
    Log(int verb) : m_verbosity(verb) {
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

    void PrintStatistics();

    inline void Tick() {
        m_tick = chrono::steady_clock::now();
    }

    inline void StatMainSolver() {
        m_mainSolverTime += chrono::duration_cast<std::chrono::microseconds>(
            chrono::steady_clock::now() - m_tick);
        m_mainSolverCalls++;
    }

    inline void StatInvSolver() {
        m_invSolverTime += chrono::duration_cast<std::chrono::microseconds>(
            chrono::steady_clock::now() - m_tick);
        m_invSolverCalls++;
    }

    inline void StatLiftSolver() {
        m_liftSolverTime += chrono::duration_cast<std::chrono::microseconds>(
            chrono::steady_clock::now() - m_tick);
        m_liftSolverCalls++;
    }

    void StatInit() {
        m_initTime += chrono::duration_cast<std::chrono::microseconds>(
            chrono::steady_clock::now() - m_tick);
    }

    inline void StatGetNewLevel() {
        m_getNewLevelTime += chrono::duration_cast<std::chrono::microseconds>(
            chrono::steady_clock::now() - m_tick);
        m_getNewLevel++;
    }

    inline void StatPropagation() {
        m_propagationTime += chrono::duration_cast<std::chrono::microseconds>(
            chrono::steady_clock::now() - m_tick);
        m_propagation++;
    }

    inline void StatStartSolver() {
        m_enumerateStartStateTime += chrono::duration_cast<std::chrono::microseconds>(
            chrono::steady_clock::now() - m_tick);
        m_enumerateStartState++;
    }

    inline void StatUpdateUc() {
        m_updateUcTime += chrono::duration_cast<std::chrono::microseconds>(
            chrono::steady_clock::now() - m_tick);
        m_updateUc++;
    }

    void StatInternalSignals() {
        m_internalSignalsTime += chrono::duration_cast<std::chrono::microseconds>(
            chrono::steady_clock::now() - m_tick);
    }

    inline double GetTimeDouble(chrono::microseconds time) {
        return chrono::duration_cast<chrono::duration<double>>(time).count();
    }

  private:
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

    uint32_t m_mainSolverCalls = 0;
    chrono::microseconds m_mainSolverTime{0};
    uint32_t m_invSolverCalls = 0;
    chrono::microseconds m_invSolverTime{0};
    uint32_t m_propagation = 0;
    chrono::microseconds m_propagationTime{0};
    uint32_t m_enumerateStartState = 0;
    chrono::microseconds m_enumerateStartStateTime{0};
    uint32_t m_liftSolverCalls = 0;
    chrono::microseconds m_liftSolverTime{0};
    uint32_t m_getNewLevel = 0;
    chrono::microseconds m_getNewLevelTime{0};
    uint32_t m_updateUc = 0;
    chrono::microseconds m_updateUcTime{0};

    chrono::microseconds m_initTime{0};
    chrono::microseconds m_internalSignalsTime{0};

    chrono::time_point<chrono::steady_clock> m_tick;
    chrono::time_point<chrono::steady_clock> m_begin;
};

extern shared_ptr<Log> GLOBAL_LOG;

} // namespace car

#endif