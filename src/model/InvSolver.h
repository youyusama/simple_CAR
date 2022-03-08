#ifndef INVSOLVER_H
#define INVSOLVER_H

#ifdef CADICAL
#include "CarSolver_cadical.h"
#else
#include "CarSolver.h"
#endif

namespace car
{

#ifdef CADICAL
class InvSolver : public CarSolver_cadical
#else
class InvSolver : public CarSolver
#endif

{
public:
    InvSolver(std::shared_ptr<AigerModel> model);
private:
};

}//namespace car

#endif