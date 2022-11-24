#ifndef MAINSOLVER_H
#define MAINSOLVER_H

#include "CarSolver.h"
#include "SimpSolver.h"

namespace car {

class MainSolver : public CarSolver {
public:
  MainSolver(std::shared_ptr<AigerModel> model, bool isForward, bool by_sslv = false);

  void add_negation_bad();

private:
};

} // namespace car

#endif