#ifndef SETTINGS_H
#define SETTINGS_H

#include <iostream>

namespace car {

struct Settings {
    bool debug = false;
    bool forward = true;
    bool propagation = true;
    bool minimal_uc = false;
    bool ctg = true;
    bool end = false;
    int threshold = 64;
    int timelimit = 0;
    int Branching = 0;
    bool skip_refer = false;
    bool witness = false;
    int seed = 0;
    std::string aigFilePath;
    std::string outputDir;
    std::string cexFilePath;
};

} // namespace car

#endif