#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <fstream>
#include "State.h"
#include <time.h>
#include <stack>
#include "OverSequence.h"
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
    

    void PrintFramesInfo(IOverSequence* sequence);

    void PrintCounterExample(int badNo, bool isForward = false);

    void PrintSafe(int badNo);

    void PrintUcNums(std::vector<int> &uc, IOverSequence* sequence);

    void PrintSAT(std::vector<int>& vec, int frameLevel);

    void PrintStatistics()
    {
        m_log<<std::endl<<"MainSolverCalls:\t"<<m_mainSolverCalls<<std::endl;
        m_log<<"MainSolver takes:\t"<<m_mainSolverTime<<" seconds"<<std::endl;
        m_log<<"InvSolver takes:\t"<<m_invSolverTime<<" seconds"<<std::endl;
        m_log<<"GetNewLevel Procedure takes:\t"<<m_getNewLevelTime<<" seconds"<<std::endl;
        m_log<<"Update uc takes:\t"<<m_updateUcTime<<" seconds"<<std::endl;
        m_log<<"Restart Times:\t"<<m_restartTimes<<std::endl;
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
    }

    void Timeout()
    {
        PrintStatistics();
        exit(0);
    }

    bool IsTimeout()
    {
        clock_t current = clock();
	    double minutes = static_cast<double>(current - m_begin) / (CLOCKS_PER_SEC * 60);
        return minutes > m_timelimit;
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
    std::shared_ptr<AigerModel> m_model;
    clock_t m_tick;
    clock_t m_begin;
    
    std::ofstream m_log;
    Settings m_settings;
    
};

}//namespace car

#endif