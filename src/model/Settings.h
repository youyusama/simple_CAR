#ifndef SETTINGS_H
#define SETTINGS_H

#include "CLI11.hpp"
#include <string>

using namespace std;

namespace car {

enum class MCAlgorithm { FCAR,
                         BCAR,
                         BMC,
                         IC3 };

enum class MCSATSolver { minisat,
                         cadical,
                         minicore,
                         kissat };

struct Settings {
    int verbosity = 0;
    string aigFilePath;
    string witnessOutputDir = "";

    MCSATSolver solver = MCSATSolver::minisat;
    MCAlgorithm alg = MCAlgorithm::FCAR;
    int bmcK = -1;
    bool dt = true;
    int branching = 1;
    int randomSeed = 0;
    bool referSkipping = false;
    bool internalSignals = false;
    bool restart = false;
    int restartThreshold = 128;
    float restartGrowthRate = 1.5;
    bool restartLuby = false;
    bool solveInProperty = false;
    int ctgMaxRecursionDepth = 2;
    int ctgMaxStates = 3;
    bool satSolveInDomain = false;
    int bmc_step = 1;
    bool bad_pred = false;
    double maxObligationAct = 20.0;
};

bool ParseSettings(int argc, char **argv, Settings &settings);

} // namespace car

#endif