#ifndef MAINSOLVER_H
#define MAINSOLVER_H

#ifdef CADICAL
#include "CarSolver_cadical.h"
#else
#include "CarSolver.h"
#endif

namespace car
{

#ifdef CADICAL
class MainSolver : public CarSolver_cadical
{
public:
    MainSolver(std::shared_ptr<AigerModel> model, bool isForward);
#else
class MainSolver : public CarSolver
{
public:
    MainSolver(std::shared_ptr<AigerModel> model, bool isForward);
#endif

private:

};

}

#endif