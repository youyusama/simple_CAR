#include "AigerModel.h"
#include "BackwardChecker.h"
#include "ForwardChecker.h"
#include "Settings.h"
#include "restart.h"
#include <cstdio>
#include <memory>
#include <string.h>

using namespace car;
using namespace std;

void PrintUsage();
Settings GetArgv(int argc, char **argv);

int main(int argc, char **argv) {
  Settings settings = GetArgv(argc, argv);
  shared_ptr<AigerModel> aigerModel(new AigerModel(settings));
  BaseChecker *checker;
  if (settings.forward) {
    checker = new ForwardChecker(settings, aigerModel);
  } else {
    checker = new BackwardChecker(settings, aigerModel);
  }
  checker->Run();
  delete checker;
  return 0;
}

void PrintUsage() {
  printf("Usage: simplecar <-f|-b>  [<-end><-interation><-rotation><-pine>] <aiger file> <output directory> [<-vis> [counter-example file]]\n");
  printf("       -timeout        set timeout\n");
  printf("       -f              forward checking (Default = backward checking)\n");
  printf("       -b              backward checking \n");
  printf("       -inter          active intersection\n");
  printf("       -rotation       active rotation\n");
  printf("       -prop           active propagation\n");
  printf("       -end            state numeration from end of the sequence\n");
  printf("       -h              print help information\n");
  printf("       -debug          print debug info\n");
  printf("       -depth          restart-depth mode\n");
  printf("       -restart        active restart policy\n");
  printf("       -vis            output visualization\n");
  printf("       -pine           active pineapple\n");
  printf("NOTE: -f and -b cannot be used together!\n");
  exit(0);
}

Settings GetArgv(int argc, char **argv) {
  bool hasSetInputDir = false;
  bool hasSetOutputDir = false;
  bool hasSetCexFile = false;
  Settings settings;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-f") == 0) {
      settings.forward = true;
    } else if (strcmp(argv[i], "-b") == 0) {
      settings.forward = false;
    } else if (strcmp(argv[i], "-timeout") == 0) {
      settings.timelimit = stoi(argv[++i]);
    } else if (strcmp(argv[i], "-inter") == 0) {
      settings.inter = true;
    } else if (strcmp(argv[i], "-rotation") == 0) {
      settings.rotate = true;
    } else if (strcmp(argv[i], "-prop") == 0) {
      settings.propagation = true;
    } else if (strcmp(argv[i], "-end") == 0) {
      settings.end = true;
    } else if (strcmp(argv[i], "-debug") == 0) {
      settings.debug = true;
    } else if (strcmp(argv[i], "-restart") == 0) {
      settings.restart = true;
    } else if (strcmp(argv[i], "-luby") == 0) {
      settings.luby = true;
    } else if (strcmp(argv[i], "-pine") == 0) {
      settings.pine = true;
    } else if (strcmp(argv[i], "-empi") == 0) {
      settings.empi = true;
    } else if (strcmp(argv[i], "-stat") == 0) {
      settings.stat = true;
    } else if (strcmp(argv[i], "-sr") == 0) {
      settings.skip_refer = true;
    } else if (strcmp(argv[i], "-incr") == 0) {
      settings.incr = true;
    } else if (strcmp(argv[i], "-depth") == 0) {
      settings.condition = RestartCondition::Depth;
    } else if (strcmp(argv[i], "-br") == 0) {
      settings.Branching = atoi(argv[i + 1]);
      i++;
    } else if (strcmp(argv[i], "-vis") == 0) {
      settings.pVisualization = true;
    } else if (!hasSetInputDir) {
      settings.aigFilePath = string(argv[i]);
      hasSetInputDir = true;
    } else if (!hasSetOutputDir) {
      settings.outputDir = string(argv[i]);
      if (settings.outputDir[settings.outputDir.length() - 1] != '/') {
        settings.outputDir += "/";
      }
      hasSetOutputDir = true;
    } else if (settings.Visualization && !hasSetCexFile) {
      settings.cexFilePath = string(argv[i]);
      hasSetCexFile = true;
    } else {
      PrintUsage();
    }
  }
  return settings;
}