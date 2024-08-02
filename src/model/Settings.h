#ifndef SETTINGS_H
#define SETTINGS_H

#include <iostream>

using namespace std;

namespace car {

struct Settings {
    bool debug = false;
    bool forward = true;
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