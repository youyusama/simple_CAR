#ifndef SETTINGS_H
#define SETTINGS_H

#include "CLI11.hpp"
#include <string>

using namespace std;

namespace car {

enum class MCAlgorithm { FCAR,
                         BCAR,
                         BMC,
                         IC3,
                         L2S,
                         KLIVE,
                         FAIR,
                         KFAIR,
                         RLIVE };

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
    MCAlgorithm safetyBaseAlg = MCAlgorithm::FCAR;
    int shoalUnroll = 1;
    bool rlivePruneDead = false;
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
    int ctgMaxRecursionDepth = 1;
    int ctgMaxStates = 3;
    int ctgMaxBlocks = 1;
    bool satSolveInDomain = false;
    int bmc_step = 1;
    int eq = 2;
    int eqTimeout = 600;
    bool searchFromBadPred = false;
    bool detailedTimers = false;
};

bool ParseSettings(int argc, char **argv, Settings &settings);

} // namespace car

#endif
