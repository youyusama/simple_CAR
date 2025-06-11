#include "BMC.h"
#include "BackwardChecker.h"
#include "ForwardChecker.h"
#include "Log.h"
#include "Model.h"
#include "Settings.h"
#include <cstdio>
#include <memory>
#include <string.h>

using namespace car;

int main(int argc, char **argv) {
    Settings settings;
    if (!ParseSettings(argc, argv, settings)) return EXIT_FAILURE;

    shared_ptr<Log> log(new Log(settings.verbosity));
    shared_ptr<Model> aigerModel(new Model(settings));
    log->StatInit();
    shared_ptr<BaseChecker> checker;
    switch (settings.alg) {
    case MCAlgorithm::FCAR:
        checker = make_shared<ForwardChecker>(settings, aigerModel, log);
        break;
    case MCAlgorithm::BCAR:
        checker = make_shared<BackwardChecker>(settings, aigerModel, log);
        break;
    case MCAlgorithm::BMC:
        checker = make_shared<BMC>(settings, aigerModel, log);
        break;
    default:
        return EXIT_FAILURE;
    }
    CheckResult res = checker->Run();
    switch (res) {
    case CheckResult::Safe:
        std::cout << "Safe" << std::endl;
        break;
    case CheckResult::Unsafe:
        std::cout << "Unsafe" << std::endl;
        break;
    case CheckResult::Unknown:
        std::cout << "Unknown" << std::endl;
        break;
    default:
        break;
    }
    if (settings.witnessOutputDir.size() > 0)
        checker->Witness();
    return 0;
}