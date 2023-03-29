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
  Log(Settings settings, std::shared_ptr<AigerModel> model) : m_settings(settings) {
    string outPath = settings.outputDir + GetFileName(settings.aigFilePath);
    m_model = model;
    m_res.open(outPath + ".res");
    m_log.open(outPath + ".log");
    if (settings.debug) {
      m_debug.open(outPath + ".debug");
    }
    m_timelimit = static_cast<double>(settings.timelimit / model->GetNumOutputs());
    lastState = nullptr;
    m_begin = clock();
    m_restartTimes = 0;
    m_good_lemma_num_frame.reset(new std::vector<int>());
    m_lemma_num_frame.reset(new std::vector<int>());
  }

  ~Log() {
    m_res.close();
    m_log.close();
    if (m_settings.debug) {
      m_debug.close();
    }
  }

  void PrintSth(std::string s);

  void DebugPrintSth(std::string s);

  bool IsDebug();

  void PrintInDebug(std::string str);

  void PrintCounterExample(int badNo, bool isForward);

  void PrintSafe(int badNo);

  void DebugPrintVector(std::vector<int> &uc, std::string text = "");

  void PrintStateShort(std::shared_ptr<State> s);

  void PrintSAT(std::vector<int> &vec, int frameLevel);

  void PrintLitOrder(std::vector<float> order);

  void PrintPineInfo(std::shared_ptr<State> state, std::shared_ptr<std::vector<int>> uc);

  void StatPineInfo(std::shared_ptr<State> state, std::shared_ptr<std::vector<int>> uc_pine, std::shared_ptr<std::vector<int>> uc);

  void PrintStatistics() {
    m_log << std::endl
          << "MainSolverCalls:\t" << m_mainSolverCalls << std::endl;
    m_log << "MainSolver takes:\t" << m_mainSolverTime << " seconds" << std::endl;
    m_log << "InvSolver takes:\t" << m_invSolverTime << " seconds" << std::endl;
    m_log << "StartSolver takes:\t" << m_enumerateStartStateTime << " seconds" << std::endl;
    m_log << "GetNewLevel Procedure takes:\t" << m_getNewLevelTime << " seconds" << std::endl;
    m_log << "Update uc takes:\t" << m_updateUcTime << " seconds" << std::endl;
    m_log << "muc takes:\t" << m_mucTime << " seconds" << std::endl;
    m_log << "partial takes:\t" << m_partialTime << " seconds" << std::endl;
    if (m_settings.restart) m_log << "Restart Times:\t" << m_restartTimes << std::endl;
    if (m_settings.propagation) m_log << "Propagation Time:\t" << m_propagationTime << std::endl;
    if (m_settings.pine) {
      m_log << "Pine Times:\t" << m_pineTime << std::endl;
      m_log << "Pine Called Times:\t" << m_pineCalled << std::endl;
      m_log << "Pine uc is short Times:\t" << m_pineIsShort << std::endl;
      m_log << "Pine l1 is <20% Times:\t" << m_pineL1isShort << std::endl;
    }
    if (m_settings.stat) {
      m_log << "CLS in O same as parent rate:\t" << m_o_same_rate << std::endl;
      m_log << "gen good lemma rate:\t" << (float)m_gen_good_times / (float)m_gen_times << std::endl;
      m_log << "prop succ rate:\t" << (float)m_succ_prop_times / (float)m_prop_times << std::endl;
      // m_log << "frame lemmas, good lemmas" << std::endl;
      // for (int i = 0; i < m_lemma_num_frame->size(); i++) {
      //   m_log << m_lemma_num_frame->at(i) << " " << m_good_lemma_num_frame->at(i) << std::endl;
      // }
      // m_log << "cls in fixpoint ratio:\t" << m_cls_in_fixpoint_ratio << std::endl;
      m_log << "monotone degree:\t" << 1- ((float)monotone_degree_un/(float) monotone_degree_all) << std::endl;
    }
    m_log << "Init Time:\t" << m_initTime << " seconds" << std::endl;
    m_log << "Total Time:\t" << static_cast<double>(clock() - m_begin) / CLOCKS_PER_SEC << " seconds" << std::endl;
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
    m_pineTime = 0;
  }

  void Timeout() {
    PrintStatistics();
    exit(0);
  }

  bool IsTimeout() {
    clock_t current = clock();
    double seconds = static_cast<double>(current - m_begin) / (CLOCKS_PER_SEC);
    return seconds > m_timelimit;
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

  void StatPine() {
    m_pineTime += static_cast<double>(clock() - m_tick) / CLOCKS_PER_SEC;
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

  void StatSuccProp(bool is_succ) {
    if (!m_settings.stat) return;
    m_prop_times++;
    if (is_succ)
      m_succ_prop_times++;
  }

  void StatGenGood(bool is_good) {
    if (!m_settings.stat) return;
    m_gen_times++;
    if (is_good)
      m_gen_good_times++;
  }

  bool need_same_stat() {
    if (m_settings.stat)
      return true;
    else
      return false;
  }

  void set_o_same_rate(float rate);

  void CountRestartTimes() { m_restartTimes++; }

  std::shared_ptr<State> lastState;
  std::ofstream m_res;
  std::ofstream m_debug;
  sptr<std::vector<int>> m_lemma_num_frame;
  sptr<std::vector<int>> m_good_lemma_num_frame;
  double m_cls_in_fixpoint_ratio = 0;
  int monotone_degree_all = 0;
  int monotone_degree_un = 0;

private:
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

  int m_mainSolverCalls = 0;
  int m_invSolverCalls = 0;
  int m_restartTimes = 0;
  double m_mainSolverTime = 0;
  double m_invSolverTime = 0;
  double m_getNewLevelTime = 0;
  double m_updateUcTime = 0;
  double m_propagationTime = 0;
  double m_timelimit = 0;
  double m_mucTime = 0;
  double m_partialTime = 0;
  double m_enumerateStartStateTime = 0;
  double m_initTime = 0;
  double m_succpropstatTime = 1;
  int m_prop_times = 0;
  int m_succ_prop_times = 0;
  int m_gen_times = 0;
  int m_gen_good_times = 0;
  // pine
  double m_pineTime = 0;
  int m_pineCalled = 0;
  int m_pineIsShort = 0;
  int m_pineL1isShort = 0;

  std::shared_ptr<AigerModel> m_model;
  clock_t m_tick;
  clock_t m_begin;
  std::stack<clock_t> m_ticks;

  std::ofstream m_log;
  Settings m_settings;

  float m_o_same_rate = 0;
};

} // namespace car

#endif