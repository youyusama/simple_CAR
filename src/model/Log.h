#ifndef LOG_H
#define LOG_H

#include "AigerModel.h"
#include "Settings.h"
#include "State.h"
#include "signal.h"
#include <assert.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stack>
#include <time.h>

namespace car {

string CubeToStr(const shared_ptr<cube> c);

void compress_vector(shared_ptr<cube> res, const shared_ptr<cube> v);

string CubeToStrShort(const shared_ptr<cube> c);

void signalHandler(int signum);

class Log {
  public:
    Log(int verb) : m_verbosity(verb) {
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
        cout << endl
             << "TransSolver    called:\t" << m_mainSolverCalls << endl;
        cout << "TransSolver    takes:\t" << m_mainSolverTime << endl;
        cout << "LiftSolver     takes:\t" << m_liftSolverTime << endl;
        cout << "InvSolver      takes:\t" << m_invSolverTime << endl;
        cout << "StartSolver    takes:\t" << m_enumerateStartStateTime << endl;
        cout << "Locating Level takes:\t" << m_getNewLevelTime << endl;
        cout << "Updating UC    takes:\t" << m_updateUcTime << endl;
        cout << "Propagation    takes:\t" << m_propagationTime << endl;
        cout << "Initialization takes:\t" << m_initTime << endl;
        cout << "Total Time     spent:\t" << static_cast<double>(clock() - m_begin) / CLOCKS_PER_SEC << endl;
    }

    inline void Tick() {
        m_tick = clock();
    }

    void StatMainSolver() {
        m_mainSolverTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_mainSolverCalls++;
    }

    void StatInvSolver() {
        m_invSolverTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
    }

    void StatLiftSolver() {
        m_liftSolverTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
    }

    void StatInit() {
        m_initTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
    }

    void StatGetNewLevel() {
        m_getNewLevelTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
    }

    void StatPropagation() {
        m_propagationTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
    }

    void StatStartSolver() {
        m_enumerateStartStateTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
    }

    void StatUpdateUc() {
        m_updateUcTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
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

    int m_mainSolverCalls = 0;
    int m_invSolverCalls = 0;
    int m_restartTimes = 0;
    double m_mainSolverTime = 0;
    double m_liftSolverTime = 0;
    double m_invSolverTime = 0;
    double m_getNewLevelTime = 0;
    double m_updateUcTime = 0;
    double m_propagationTime = 0;
    double m_enumerateStartStateTime = 0;
    double m_initTime = 0;

    clock_t m_tick;
    clock_t m_begin;
};

extern shared_ptr<Log> GLOBAL_LOG;

} // namespace car

#endif