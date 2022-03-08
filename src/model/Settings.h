#ifndef SETTINGS_H
#define SETTINGS_H

#include<iostream>

namespace car
{


enum RestartCondition
{
    UcNums = 0,
    Depth
};

enum RestartBehaviour
{
    BackToInit = 0,
    BackToHalf
};

struct Settings
{
    bool debug = false;
    bool forward = false;
    bool propagation = false;
    bool minimal_uc = false;
    bool restart = false;
    bool end = false;
    bool inter = false;
    bool rotate = false;
    bool inputS = false;
    bool luby = false;
    float growthRate = 1.5;
    int threshold = 64;
    int timelimit = 0;
    RestartCondition condition = RestartCondition::UcNums;
    RestartBehaviour behaviour = RestartBehaviour::BackToInit; 
    std::string aigFilePath;
    std::string outputDir;
};





}

#endif