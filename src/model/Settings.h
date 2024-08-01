#ifndef SETTINGS_H
#define SETTINGS_H

#include <iostream>

using namespace std;

namespace car {

struct Settings {
    bool debug = false;
    bool forward = true;
    bool propagation = true;
    bool minimal_uc = false;
    bool ctg = true;
    bool end = false;
    int Branching = 0;
    bool skip_refer = false;
    bool witness = false;
    int seed = 0;
    string aigFilePath;
    string outputDir;
    string cexFilePath;
};

} // namespace car

#endif