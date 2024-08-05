#include "Log.h"

namespace car {

void signalHandler(int signum) {
    GLOBAL_LOG->PrintStatistics();
    GLOBAL_LOG->L(0, "Unknown");
    exit(signum);
}


shared_ptr<Log> GLOBAL_LOG = nullptr;


string CubeToStr(cube c) {
    string s;
    for (int l : c) s.append(to_string(l) + " ");
    return s;
}


void compress_vector(vector<int> &res, vector<int> &v) {
    int count = 0;
    int tempi = 0;
    for (int l : v) {
        if (l > 0)
            tempi = (tempi << 1) + 1;
        else
            tempi <<= 1;
        count++;
        if (count == 32 || l == v.back()) {
            res.emplace_back(tempi);
            tempi = 0;
            count = 0;
        }
    }
}


string CubeToStrShort(cube c) {
    cube s;
    compress_vector(s, c);
    return CubeToStr(s);
}
} // namespace car