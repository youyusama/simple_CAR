#ifndef MAINSOLVER_H
#define MAINSOLVER_H

#include "CarSolver.h"

namespace car
{

class MainSolver : public CarSolver
{
public:
    MainSolver(std::shared_ptr<AigerModel> model, bool isForward);
private:

};

}

#endif