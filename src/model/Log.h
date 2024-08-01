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
#include <stack>
#include <time.h>
namespace car {

class Log {
  public:
    Log(Settings settings, shared_ptr<AigerModel> model) : m_settings(settings) {
        string outPath = settings.outputDir + GetFileName(settings.aigFilePath);
        m_model = model;
        m_res.open(outPath + ".res");
        m_log.open(outPath + ".log");
        if (settings.debug) {
            m_debug.open(outPath + ".debug");
        }
        lastState = nullptr;
        m_begin = clock();
    }

    ~Log() {
        m_res.close();
        m_log.close();
        if (m_settings.debug) {
            m_debug.close();
        }
    }

    void PrintSth(string s);

    void DebugPrintSth(string s);

    bool IsDebug();

    void PrintInDebug(string str);

    void PrintCounterExample(int badNo, bool isForward);

    void PrintSafe(int badNo);

    void DebugPrintVector(vector<int> &uc, string text = "");

    void PrintStateShort(shared_ptr<State> s);

    void PrintSAT(vector<int> &vec, int frameLevel);

    void PrintLitOrder(vector<float> order);

    void PrintPineInfo(shared_ptr<State> state, shared_ptr<vector<int>> uc);

    void StatPineInfo(shared_ptr<State> state, shared_ptr<vector<int>> uc_pine, shared_ptr<vector<int>> uc);

    void PrintStatistics() {
        m_log << endl
              << "MainSolverCalls:\t" << m_mainSolverCalls << endl;
        m_log << "MainSolver takes:\t" << m_mainSolverTime << " seconds" << endl;
        m_log << "InvSolver takes:\t" << m_invSolverTime << " seconds" << endl;
        m_log << "StartSolver takes:\t" << m_enumerateStartStateTime << " seconds" << endl;
        m_log << "GetNewLevel Procedure takes:\t" << m_getNewLevelTime << " seconds" << endl;
        m_log << "Update uc takes:\t" << m_updateUcTime << " seconds" << endl;
        m_log << "muc takes:\t" << m_mucTime << " seconds" << endl;
        m_log << "partial takes:\t" << m_partialTime << " seconds" << endl;
        if (m_settings.propagation) m_log << "Propagation Time:\t" << m_propagationTime << endl;
        m_log << "Init Time:\t" << m_initTime << " seconds" << endl;
        m_log << "Total Time:\t" << static_cast<double>(clock() - m_begin) / CLOCKS_PER_SEC << " seconds" << endl;
    }

    void ResetClock() {
        m_begin = clock();
        m_mainSolverTime = 0;
        m_mainSolverCalls = 0;
        m_invSolverCalls = 0;
        m_mainSolverTime = 0;
        m_invSolverTime = 0;
        m_getNewLevelTime = 0;
        m_updateUcTime = 0;
    }

    void Tick() {
        m_tick = clock();
    }

    void stackTick() {
        m_ticks.push(clock());
    }

    void StatMainSolver() {
        m_mainSolverTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
        m_mainSolverCalls++;
    }

    void StatInvSolver() {
        m_invSolverTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
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

    void Statmuc() {
        m_mucTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
    }

    void Statpartial() {
        m_partialTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
    }

    void set_o_same_rate(float rate);

    void CountRestartTimes() { m_restartTimes++; }

    shared_ptr<State> lastState;
    ofstream m_res;
    ofstream m_debug;

    string GetFileName(string filePath) {
        auto startIndex = filePath.find_last_of("/");
        if (startIndex == string::npos) {
            startIndex = 0;
        } else {
            startIndex++;
        }
        auto endIndex = filePath.find_last_of(".");
        assert(endIndex != string::npos);
        return filePath.substr(startIndex, endIndex - startIndex);
    }

  private:
    int m_mainSolverCalls = 0;
    int m_invSolverCalls = 0;
    int m_restartTimes = 0;
    double m_mainSolverTime = 0;
    double m_invSolverTime = 0;
    double m_getNewLevelTime = 0;
    double m_updateUcTime = 0;
    double m_propagationTime = 0;
    double m_mucTime = 0;
    double m_partialTime = 0;
    double m_enumerateStartStateTime = 0;
    double m_initTime = 0;
    double m_succpropstatTime = 1;
    int m_prop_times = 0;
    int m_succ_prop_times = 0;
    int m_gen_times = 0;
    int m_gen_good_times = 0;

    shared_ptr<AigerModel> m_model;
    clock_t m_tick;
    clock_t m_begin;
    stack<clock_t> m_ticks;

    ofstream m_log;
    Settings m_settings;

    float m_o_same_rate = 0;
};

} // namespace car

#endif