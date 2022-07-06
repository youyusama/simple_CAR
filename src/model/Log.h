#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <fstream>
#include "State.h"
#include <time.h>
#include <stack>
#include "OverSequence.h"
#include "OverSequenceNI.h"
#include "AigerModel.h"
#include <memory>
#include "Settings.h"
#include <assert.h>
namespace car
{

class Log
{
public:
    Log(Settings settings, std::shared_ptr<AigerModel> model) : m_settings(settings)
    {
        string outPath = settings.outputDir + GetFileName(settings.aigFilePath);
        m_model = model;
        m_res.open(outPath + ".res");
        m_log.open(outPath + ".log");
        if (settings.debug)
        {
            m_debug.open(outPath + ".debug");
        } 
        m_timelimit = static_cast<double>(settings.timelimit/model->GetNumOutputs());
        lastState = nullptr;
        m_begin = clock();
        m_restartTimes = 0;
    }

    ~Log()
    {
        m_res.close();
        m_log.close();
        if (m_settings.debug)
        {
            m_debug.close();
        } 
    }
    
    void PrintSth(std::string s);

    void DebugPrintSth(std::string s);

    void PrintInDebug(std::string str);

    void PrintFramesInfo(OverSequenceNI* sequence);
    void PrintFramesInfo(IOverSequence* sequence);

    void PrintCounterExample(int badNo, bool isForward = false);

    void PrintSafe(int badNo);

    void DebugPrintVector(std::vector<int> &uc, std::string text = "");

    void PrintOSequence(OverSequenceNI* sequence);
    void PrintOSequence(IOverSequence* sequence);

    void Log::PrintStateShort(std::shared_ptr<State> s);

    void PrintUcNums(std::vector<int> &uc, OverSequenceNI* sequence);
    void PrintUcNums(std::vector<int> &uc, IOverSequence* sequence);

    void PrintSAT(std::vector<int>& vec, int frameLevel);

    void PrintPineInfo(std::shared_ptr<State> state, std::shared_ptr<std::vector<int>> uc);

    void StatPineInfo(std::shared_ptr<State> state, std::shared_ptr<std::vector<int>> uc_pine, std::shared_ptr<std::vector<int>> uc);

    void PrintStatistics()
    {
        m_log<<std::endl<<"MainSolverCalls:\t"<<m_mainSolverCalls<<std::endl;
        m_log<<"MainSolver takes:\t"<<m_mainSolverTime<<" seconds"<<std::endl;
        m_log<<"InvSolver takes:\t"<<m_invSolverTime<<" seconds"<<std::endl;
        m_log<<"GetNewLevel Procedure takes:\t"<<m_getNewLevelTime<<" seconds"<<std::endl;
        m_log<<"Update uc takes:\t"<<m_updateUcTime<<" seconds"<<std::endl;
        m_log<<"Restart Times:\t"<<m_restartTimes<<std::endl;
        if (m_settings.pine){
            m_log<<"Pine Times:\t"<<m_pineTime<<std::endl;
            m_log<<"Pine Called Times:\t"<<m_pineCalled<<std::endl;
            m_log<<"Pine uc is short Times:\t"<<m_pineIsShort<<std::endl;
            m_log<<"Pine l1 is <20% Times:\t"<<m_pineL1isShort<<std::endl;
        }
        m_log<<"Total Time:\t"<<static_cast<double>(clock()-m_begin)/CLOCKS_PER_SEC<<" seconds"<<std::endl;
    }

    void ResetClock()
    {
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

    void Timeout()
    {
        PrintStatistics();
        exit(0);
    }

    bool IsTimeout()
    {
        clock_t current = clock();
        double seconds = static_cast<double>(current - m_begin) / (CLOCKS_PER_SEC );
        return seconds > m_timelimit;
    }

    void Tick()
    {
        m_tick = clock();
    }

    void StatMainSolver()
    {
        m_mainSolverTime += static_cast<double>(clock() - m_tick)/CLOCKS_PER_SEC;
        m_mainSolverCalls++;
    }

    void StatInvSolver()
    {
        m_invSolverTime += static_cast<double>(clock() - m_tick)/CLOCKS_PER_SEC;
    }

    void StatPine()
    {
        m_pineTime += static_cast<double>(clock() - m_tick)/CLOCKS_PER_SEC;
    }

    void StatGetNewLevel()
    {
        m_getNewLevelTime += static_cast<double>(clock() - m_tick)/CLOCKS_PER_SEC;
    }

    void StatUpdateUc()
    {
        m_updateUcTime += static_cast<double>(clock() - m_tick)/CLOCKS_PER_SEC;
    }

    void CountRestartTimes() {m_restartTimes++;}

    std::shared_ptr<State> lastState;
    std::ofstream m_res;
    std::ofstream m_debug;
private:

    string GetFileName(string filePath)
	{
		auto startIndex = filePath.find_last_of("/");
		if (startIndex == string::npos)
		{
			startIndex = 0;
		}
		else
		{
			startIndex++;
		}
		auto endIndex = filePath.find_last_of(".");
		assert (endIndex != string::npos);
		return filePath.substr(startIndex, endIndex-startIndex);	
	}

    int m_mainSolverCalls = 0;
    int m_invSolverCalls = 0;
    int m_restartTimes = 0;
    double m_mainSolverTime = 0;
    double m_invSolverTime = 0;
    double m_getNewLevelTime = 0;
    double m_updateUcTime = 0;
    double m_timelimit = 0;
    //pine
    double m_pineTime = 0;
    int m_pineCalled = 0;
    int m_pineIsShort = 0;
    int m_pineL1isShort = 0;

    std::shared_ptr<AigerModel> m_model;
    clock_t m_tick;
    clock_t m_begin;
    
    std::ofstream m_log;
    Settings m_settings;
    
};

}//namespace car

#endif