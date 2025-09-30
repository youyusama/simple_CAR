#ifndef LOG_H
#define LOG_H

#include "Settings.h"
#include "signal.h"
#include <assert.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <time.h>

namespace car {

string CubeToStr(const shared_ptr<vector<int>> c);

void compress_vector(shared_ptr<vector<int>> res, const shared_ptr<vector<int>> v);

string CubeToStrShort(const shared_ptr<vector<int>> c);

void signalHandler(int signum);

class Log {
  public:
    Log(int verb, MCAlgorithm algo) : m_verbosity(verb), m_algorithm(algo) {
        m_begin = clock();
        m_tick = clock();
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

    void PrintStatistics() {
        if (m_verbosity == 0) return;
        if (m_algorithm != MCAlgorithm::IC3) {
            cout << endl
                 << "TransSolver    called: " << left << setw(12) << m_mainSolverCalls
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_mainSolverTime
                 << "per: " << fixed << setprecision(5) << m_mainSolverTime / m_mainSolverCalls << endl;
            cout << "Propagation    called: " << left << setw(12) << m_propagation
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_propagationTime
                 << "per: " << fixed << setprecision(5) << m_propagationTime / m_propagation << endl;
            cout << "LiftSolver     called: " << left << setw(12) << m_liftSolverCalls
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_liftSolverTime
                 << "per: " << fixed << setprecision(5) << m_liftSolverTime / m_liftSolverCalls << endl;
            cout << "InvSolver      called: " << left << setw(12) << m_invSolverCalls
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_invSolverTime
                 << "per: " << fixed << setprecision(5) << m_invSolverTime / m_invSolverCalls << endl;
            cout << "StartSolver    called: " << left << setw(12) << m_enumerateStartState
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_enumerateStartStateTime
                 << "per: " << fixed << setprecision(5) << m_enumerateStartStateTime / m_enumerateStartState << endl;
            cout << "Locating lvl   called: " << left << setw(12) << m_getNewLevel
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_getNewLevelTime
                 << "per: " << fixed << setprecision(5) << m_getNewLevelTime / m_getNewLevel << endl;
            cout << "Updating UC    called: " << left << setw(12) << m_updateUc
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_updateUcTime
                 << "per: " << fixed << setprecision(5) << m_updateUcTime / m_updateUc << endl;
        } else {
            constexpr double MS = 1000.0;
            auto avg = [](double t, uint32_t c) { return c ? (t / c) : 0.0; };
            cout << endl
                 << "FrameSolver    called: " << left << setw(12) << m_frameSolverCalls
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_frameSolverTime
                 << "per(ms): " << fixed << setprecision(5) << avg(m_frameSolverTime, m_frameSolverCalls) * MS << endl;
            cout << "GetBlocker     called: " << left << setw(12) << m_getBlockerCalls
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_getBlockerTime
                 << "per(ms): " << fixed << setprecision(5) << avg(m_getBlockerTime, m_getBlockerCalls) * MS << endl;
            cout << "AddBlockCube   called: " << left << setw(12) << m_addBlockingCubeCalls
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_addBlockingCubeTime
                 << "per(ms): " << fixed << setprecision(5) << avg(m_addBlockingCubeTime, m_addBlockingCubeCalls) * MS << endl;
            cout << "LazyCheck      called: " << left << setw(12) << m_lazyCheckCalls
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_lazyCheckTime
                 << "per(ms): " << fixed << setprecision(5) << avg(m_lazyCheckTime, m_lazyCheckCalls) * MS << endl;
            cout << "SolverCreate   called: " << left << setw(12) << m_solverCreates
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_solverCreateTime
                 << "per(ms): " << fixed << setprecision(5) << avg(m_solverCreateTime, m_solverCreates) * MS << endl;
            cout << "CadicalSimplify called: " << left << setw(12) << m_cadicalSimplify
                 << "takes: " << fixed << setprecision(3) << setw(10) << m_cadicalSimplifyTime
                 << "per(ms): " << fixed << setprecision(5) << avg(m_cadicalSimplifyTime, m_cadicalSimplify) * MS << endl;
        }

        cout << "Initialization takes: " << fixed << setprecision(3) << m_initTime << endl;
        cout << "Innards        takes: " << fixed << setprecision(3) << m_internalSignalsTime << endl;
        cout << "Total Time     spent: " << fixed << setprecision(3) << static_cast<double>(clock() - m_begin) / CLOCKS_PER_SEC << endl;
    }

    inline void Tick() {
        m_tick = clock();
    }

    inline void StatMainSolver() {
        m_mainSolverTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_mainSolverCalls++;
    }

    inline void StatInvSolver() {
        m_invSolverTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_invSolverCalls++;
    }

    inline void StatLiftSolver() {
        m_liftSolverTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_liftSolverCalls++;
    }

    void StatInit() {
        m_initTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
    }

    inline void StatGetNewLevel() {
        m_getNewLevelTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_getNewLevel++;
    }

    inline void StatPropagation() {
        m_propagationTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_propagation++;
    }

    inline void StatStartSolver() {
        m_enumerateStartStateTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_enumerateStartState++;
    }

    inline void StatUpdateUc() {
        m_updateUcTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_updateUc++;
    }

    inline void StatFrameSolver() {
        m_frameSolverTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_frameSolverCalls++;
    }

    inline void StatGetBlocker() {
        m_getBlockerTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_getBlockerCalls++;
    }

    inline void StatAddBlockingCube() {
        m_addBlockingCubeTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_addBlockingCubeCalls++;
    }

    inline void StatLazyCheck() {
        m_lazyCheckTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_lazyCheckCalls++;
    }

    inline void StatSolverCreate() {
        m_solverCreateTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_solverCreates++;
    }

    inline void StatCadicalSimplify() {
        m_cadicalSimplifyTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_cadicalSimplify++;
    }

    void StatInternalSignals() {
        m_internalSignalsTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
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
    MCAlgorithm m_algorithm;

    uint32_t m_mainSolverCalls = 0;
    double m_mainSolverTime = 0;
    uint32_t m_invSolverCalls = 0;
    double m_invSolverTime = 0;
    uint32_t m_propagation = 0;
    double m_propagationTime = 0;
    uint32_t m_enumerateStartState = 0;
    double m_enumerateStartStateTime = 0;
    uint32_t m_liftSolverCalls = 0;
    double m_liftSolverTime = 0;
    uint32_t m_getNewLevel = 0;
    double m_getNewLevelTime = 0;
    uint32_t m_updateUc = 0;
    double m_updateUcTime = 0;

    // for IC3
    uint32_t m_frameSolverCalls = 0;
    double m_frameSolverTime = 0;
    uint32_t m_getBlockerCalls = 0;
    double m_getBlockerTime = 0;
    uint32_t m_addBlockingCubeCalls = 0;
    double m_addBlockingCubeTime = 0;
    uint32_t m_lazyCheckCalls = 0;
    double m_lazyCheckTime = 0;
    uint32_t m_solverCreates = 0;
    double m_solverCreateTime = 0;
    uint32_t m_cadicalSimplify = 0;
    double m_cadicalSimplifyTime = 0;

    double m_initTime = 0;
    double m_internalSignalsTime = 0;

    clock_t m_tick;
    clock_t m_begin;
};

extern shared_ptr<Log> GLOBAL_LOG;

} // namespace car

#endif