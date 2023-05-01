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
  if (settings.draw) return 0;
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
  printf("Usage: ./simplecar AIG_FILE.aig OUTPUT_PATH/\n");
  printf("Configs:\n");
  printf("       -timeout        set timeout (s)\n");
  printf("       -f              forward searching (Default)\n");
  printf("       -b              backward searching \n");
  printf("       -br             branching (1: sum 2: VSIDS 3: ACIDS 0: static)\n");
  printf("       -rs             refer-skipping\n");
  printf("       -seed           seed (works when > 0) for random var ordering\n");
  printf("       -h              print help information\n");
  printf("       -debug          print debug info\n");
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
    } else if (strcmp(argv[i], "-end") == 0) {
      settings.end = true;
    } else if (strcmp(argv[i], "-debug") == 0) {
      settings.debug = true;
    } else if (strcmp(argv[i], "-stat") == 0) {
      settings.stat = true;
    } else if (strcmp(argv[i], "-rs") == 0) {
      settings.skip_refer = true;
    } else if (strcmp(argv[i], "-draw") == 0) {
      settings.draw = true;
    } else if (strcmp(argv[i], "-br") == 0) {
      settings.Branching = atoi(argv[i + 1]);
      i++;
    } else if (strcmp(argv[i], "-seed") == 0) {
      settings.seed = atoi(argv[i + 1]);
      i++;
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