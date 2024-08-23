#ifndef MAINSOLVER_H
#define MAINSOLVER_H

#include "../sat/minisat/simp/SimpSolver.h"
#include "CarSolver.h"

namespace car {

class MainSolver : public CarSolver {
  public:
    MainSolver(shared_ptr<AigerModel> model, bool by_sslv = false);

    void AddNegationBad();

  private:
};

} // namespace car

#endif