#ifndef SETTINGS_H
#define SETTINGS_H

#include "CLI11.hpp"
#include <string>

using namespace std;

namespace car {

enum class MCAlgorithm { FCAR,
                         BCAR,
                         BMC };

enum class MCSATSolver { minisat,
                         cadical,
                         minicore };

struct Settings {
    int verbosity = 0;
    string aigFilePath;
    string witnessOutputDir = "";

    MCSATSolver solver = MCSATSolver::minisat;
    MCAlgorithm alg = MCAlgorithm::FCAR;
    int bmcK = -1;
    bool end = false;
    int branching = 0;
    int randomSeed = 0;
    bool referSkipping = false;
    bool internalSignals = false;
    bool multipleSolvers = true;
    bool restart = false;
    int restart_threshold = 64;
    float restart_growth_rate = 1.5;
    bool luby = false;
};

bool ParseSettings(int argc, char **argv, Settings &settings);

} // namespace car

#endif