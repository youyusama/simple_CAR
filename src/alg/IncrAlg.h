#ifndef INCRALG_H
#define INCRALG_H

#include "BaseAlg.h"
#include "IncrCheckerHelpers.h"
#include <vector>

namespace car {

class IncrAlg : public BaseAlg {
  public:
    virtual void SetInit(const cube &c) = 0;
    virtual void SetSearchFromInitSucc(bool b) = 0;
    virtual void SetLoopRefuting(bool b) = 0;
    virtual void SetDead(const std::vector<cube> &dead) = 0;
    virtual void SetShoals(const std::vector<FrameList> &shoals) = 0;
    virtual void SetWalls(const std::vector<FrameList> &walls) = 0;

    virtual cube GetReachedTarget() = 0;
    virtual std::vector<std::pair<cube, cube>> GetCexTrace() = 0;
    virtual FrameList GetInv() = 0;
    virtual void KLiveIncr() = 0;

    virtual ~IncrAlg() = default;
};

} // namespace car

#endif
