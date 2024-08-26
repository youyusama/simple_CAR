#ifndef MAINSOLVER_H
#define MAINSOLVER_H


#ifdef CADICAL
#include "CadicalSolver.h"
#else
#include "../sat/minisat/simp/SimpSolver.h"
#include "MinisatSolver.h"
#endif

namespace car {

class MainSolver :
#ifdef CADICAL
    public CadicalSolver
#else
    public MinisatSolver
#endif
{
  public:
    MainSolver(shared_ptr<AigerModel> model);

    bool Solve(const shared_ptr<cube> assumption, int frameLevel);

    void AddUC(const cube &uc, int frameLevel);

    void AddNegationBad();

  private:
    inline int GetFrameFlag(int frameLevel);

    vector<int> m_frameFlags;
};

} // namespace car

#endif