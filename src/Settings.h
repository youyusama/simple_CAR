#ifndef SETTINGS_H
#define SETTINGS_H

#include "CLI11.hpp"
#include <string>

using namespace std;

namespace car {

enum class MCAlgorithm { FCAR,
                         BCAR,
                         BMC,
                         KIND,
                         IC3,
                         L2S,
                         KLIVE,
                         FAIR,
                         KFAIR,
                         RLIVE };

enum class MCAlgorithmProperty { Safety,
                                 Liveness };

inline MCAlgorithmProperty GetMCAlgorithmProperty(MCAlgorithm alg) {
    switch (alg) {
    case MCAlgorithm::L2S:
    case MCAlgorithm::KLIVE:
    case MCAlgorithm::FAIR:
    case MCAlgorithm::KFAIR:
    case MCAlgorithm::RLIVE:
        return MCAlgorithmProperty::Liveness;
    default:
        return MCAlgorithmProperty::Safety;
    }
}

enum class MCSATSolver { minisat,
                         cadical,
                         minicore,
                         kissat };

struct Settings {
    int verbosity = 0;
    string aigFilePath;
    string witnessOutputDir = "";
    bool wlDisablePackageResize = false;
    string wlBitblastOutputPath = "";

    MCSATSolver solver = MCSATSolver::minisat;
    MCAlgorithm alg = MCAlgorithm::FCAR;
    MCAlgorithm safetyBaseAlg = MCAlgorithm::FCAR;
    int shoalUnroll = 1;
    bool rlivePruneDead = false;
    int bmcK = -1;
    bool bmcCnf = false;
    string bmcCnfDir = "";
    int bmcCnfK = -1;
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
    int ctgMaxCTG = 3;
    int ctgMaxBlocks = 1;
    int genMaxFail = 0;
    bool activeLemmaLearning = false;
    int allThreshold = 8;
    int allMaxStates = 32;
    bool satSolveInDomain = false;
    int shrink = 0;
    double maxObligationAct = 10.0;
    int bmcStep = 1;
    int eq = 2;
    int eqTimeout = 600;
    bool searchFromBadPred = false;
    bool detailedTimers = false;
};

bool ParseSettings(int argc, char **argv, Settings &settings);

} // namespace car

#endif
