#ifndef BASECHECKER_H
#define BASECHECKER_H

#include "AigerModel.h"
#include "Settings.h"

namespace car
{

    class BaseChecker
    {
    public:
        virtual bool Run() = 0;
        virtual ~BaseChecker(){};
    };

} //namespace car



#endif
