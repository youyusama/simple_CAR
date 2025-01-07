#include "AigerModel.h"
#include "BMC.h"
#include "BackwardChecker.h"
#include "ForwardChecker.h"
#include "Log.h"
#include "Settings.h"
#include <cstdio>
#include <memory>
#include <string.h>


void PrintUsage() {
    cout << "Usage: ./simplecar AIG_FILE.aig" << endl;
    cout << "Configs:" << endl;
    cout << "   -f | -b             forward (default) | backward searching" << endl;
    cout << "   -br n               branching (1: sum 2: VSIDS 3: ACIDS 0: static)" << endl;
    cout << "   -rs                 refer-skipping" << endl;
    cout << "   -seed n             seed (works when > 0) for random var ordering" << endl;
    cout << "   -w OUTPUT_PATH/     output witness" << endl;
    cout << "   -v n                verbosity" << endl;
    cout << "   -h                  print help information" << endl;
    exit(0);
}


Settings GetArgv(int argc, char **argv) {
    bool hasInputFile = false;
    Settings settings;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0) {
            settings.forward = true;
        } else if (strcmp(argv[i], "-b") == 0) {
            settings.backward = true;
        } else if (strcmp(argv[i], "-bmc") == 0) {
            settings.bmc = true;
        } else if (strcmp(argv[i], "-cnf") == 0) {
            settings.cnf = true;
            settings.outputDir = string(argv[i + 1]);
            if (settings.outputDir.back() != '/') settings.outputDir += "/";
            i++;
        } else if (strcmp(argv[i], "-bmc_lec") == 0) {
            settings.bmc_lec = true;
        } else if (strcmp(argv[i], "-bmc_lec_aag") == 0) {
            settings.bmc_lec_aag = true;
            settings.outputDir = string(argv[i + 1]);
            if (settings.outputDir.back() != '/') settings.outputDir += "/";
            i++;
        } else if (strcmp(argv[i], "-k") == 0) {
            settings.bmc_k = atoi(argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "-br") == 0) {
            settings.Branching = atoi(argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "-rs") == 0) {
            settings.skip_refer = true;
        } else if (strcmp(argv[i], "-seed") == 0) {
            settings.seed = atoi(argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "-w") == 0) {
            settings.witness = true;
            settings.outputDir = string(argv[i + 1]);
            if (settings.outputDir.back() != '/') settings.outputDir += "/";
            i++;
        } else if (strcmp(argv[i], "-v") == 0) {
            settings.verbosity = atoi(argv[i + 1]);
            i++;
        } else if (!hasInputFile) {
            settings.aigFilePath = string(argv[i]);
            hasInputFile = true;
        } else {
            PrintUsage();
        }
    }
    return settings;
}


int main(int argc, char **argv) {
    Settings settings = GetArgv(argc, argv);
    shared_ptr<Log> log(new Log(settings.verbosity));
    shared_ptr<AigerModel> aigerModel(new AigerModel(settings));
    log->StatInit();
    shared_ptr<BaseChecker> checker;
    if (settings.forward) {
        checker = make_shared<ForwardChecker>(settings, aigerModel, log);
    } else if (settings.backward) {
        checker = make_shared<BackwardChecker>(settings, aigerModel, log);
    } else if (settings.bmc) {
        checker = make_shared<BMC>(settings, aigerModel, log);
    } else if (settings.bmc_lec) {
        aigerModel->GenerateBMCLEC();
        return 0;
    }
    checker->Run();
    return 0;
}