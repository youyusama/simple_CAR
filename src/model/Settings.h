#ifndef SETTINGS_H
#define SETTINGS_H

#include <string>

using namespace std;

namespace car {

struct Settings {
    bool debug = false;
    bool forward = false;
    bool backward = false;
    bool bmc = false;
    int bmc_k = -1;
    bool end = false;
    int Branching = 0;
    bool skip_refer = false;
    bool witness = false;
    int seed = 0;
    int verbosity = 0;
    bool internalSignals = false;
    int solver = 0;
    string aigFilePath;
    string outputDir;
};

} // namespace car

#endif