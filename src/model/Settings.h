#ifndef SETTINGS_H
#define SETTINGS_H

#include "CLI11.hpp"
#include <string>

using namespace std;

namespace car {

enum class MCAlgorithm { FCAR,
                         BCAR,
                         BMC };

struct Settings {
    int verbosity = 0;
    string aigFilePath;
    string witnessOutputDir = "";

    int solver = 0;
    MCAlgorithm alg = MCAlgorithm::FCAR;
    int bmcK = -1;
    bool end = false;
    int branching = 0;
    int randomSeed = 0;
    bool referSkipping = false;
    bool internalSignals = false;
};

bool ParseSettings(int argc, char **argv, Settings &settings);

} // namespace car

#endif