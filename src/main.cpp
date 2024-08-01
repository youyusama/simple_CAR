#include "AigerModel.h"
#include "BackwardChecker.h"
#include "ForwardChecker.h"
#include "Settings.h"
#include <cstdio>
#include <memory>
#include <string.h>


void PrintUsage() {
    cout << "Usage: ./simplecar AIG_FILE.aig OUTPUT_PATH/" << endl;
    cout << "Configs:" << endl;
    cout << "       -f | -b         forward (default) | backward searching" << endl;
    cout << "       -b               searching" << endl;
    cout << "       -br             branching (1: sum 2: VSIDS 3: ACIDS 0: static)" << endl;
    cout << "       -rs             refer-skipping" << endl;
    cout << "       -seed           seed (works when > 0) for random var ordering" << endl;
    cout << "       -h              print help information" << endl;
    cout << "       -w              output witness" << endl;
    cout << "       -debug          print debug info" << endl;
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
        } else if (strcmp(argv[i], "-end") == 0) {
            settings.end = true;
        } else if (strcmp(argv[i], "-debug") == 0) {
            settings.debug = true;
        } else if (strcmp(argv[i], "-rs") == 0) {
            settings.skip_refer = true;
        } else if (strcmp(argv[i], "-w") == 0) {
            settings.witness = true;
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
        } else {
            PrintUsage();
        }
    }
    return settings;
}


int main(int argc, char **argv) {
    Settings settings = GetArgv(argc, argv);
    shared_ptr<AigerModel> aigerModel(new AigerModel(settings));
    shared_ptr<BaseChecker> checker;
    if (settings.forward) {
        checker = make_shared<ForwardChecker>(settings, aigerModel);
    } else {
        checker = make_shared<BackwardChecker>(settings, aigerModel);
    }
    checker->Run();
    return 0;
}