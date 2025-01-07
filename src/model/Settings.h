#ifndef SETTINGS_H
#define SETTINGS_H

#include <iostream>

using namespace std;

namespace car {

struct Settings {
    bool debug = false;
    bool forward = false;
    bool backward = false;
    bool bmc = false;
    bool bmc_lec = false;
    bool bmc_lec_aag = false;
    int bmc_k = -1;
    bool cnf = false;
    bool end = false;
    int Branching = 0;
    bool skip_refer = false;
    bool witness = false;
    int seed = 0;
    int verbosity = 0;
    string aigFilePath;
    string outputDir;
};

} // namespace car

#endif