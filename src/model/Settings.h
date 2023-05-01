#ifndef SETTINGS_H
#define SETTINGS_H

#include <iostream>

namespace car {


enum RestartCondition {
  UcNums = 0,
  Depth
};

enum RestartBehaviour {
  BackToInit = 0,
  BackToHalf
};

struct Settings {
  bool debug = false;
  bool forward = true;
  bool propagation = true;
  bool minimal_uc = false;
  bool ctg = true;
  bool restart = false;
  bool end = false;
  bool inter = false;
  bool rotate = false;
  bool inputS = false;
  bool luby = false;
  bool pine = false;
  bool empi = true;
  bool incr = false;
  bool Visualization = false;
  bool pVisualization = false;
  bool preorder = false;
  float growthRate = 1.5;
  int threshold = 64;
  int timelimit = 0;
  int Branching = 0;
  bool skip_refer = false;
  bool stat = false;
  bool draw = false;
  int seed = 0;
  RestartCondition condition = RestartCondition::UcNums;
  RestartBehaviour behaviour = RestartBehaviour::BackToInit;
  std::string aigFilePath;
  std::string outputDir;
  std::string cexFilePath;
};


} // namespace car

#endif