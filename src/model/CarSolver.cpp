#include "CarSolver.h"
#include <algorithm>
using namespace Minisat;

namespace car {
CarSolver::CarSolver() {}

CarSolver::~CarSolver() {
  ;
}

int lit_id(Lit l) {
  if (sign(l))
    return -(var(l) + 1);
  else
    return var(l) + 1;
}

bool CarSolver::SolveWithAssumption() {
  lbool result = solveLimited(m_assumptions);
  if (result == l_True) {
    return true;
  } else if (result == l_False) {
    return false;
  } else // result == l_Undef
  {
    // placeholder
    assert(true);
    return false;
  }
}

bool CarSolver::SolveWithAssumption(std::vector<int> &assumption, int frameLevel) {
  m_assumptions.clear();
  m_assumptions.push(GetLit(GetFrameFlag(frameLevel)));
  for (auto it = assumption.begin(); it != assumption.end(); ++it) {
    m_assumptions.push(GetLit(*it));
  }
  lbool result = solveLimited(m_assumptions);
  if (result == l_True) {
    return true;
  } else if (result == l_False) {
    return false;
  } else // result == l_Undef
  {
    // placeholder
    assert(true);
    return false;
  }
}

void CarSolver::AddClause(const std::vector<int> &clause) {
  vec<Lit> literals;
  for (int i = 0; i < clause.size(); ++i) {
    literals.push(GetLit(clause[i]));
  }
  bool result = addClause(literals);
  assert(result != false);
}

void CarSolver::AddUnsatisfiableCore(const std::vector<int> &clause, int frameLevel) {
  int flag = GetFrameFlag(frameLevel);
  vec<Lit> literals;
  literals.push(GetLit(-flag));
  if (m_isForward) {
    for (int i = 0; i < clause.size(); ++i) {
      literals.push(GetLit(-clause[i]));
    }
  } else {
    for (int i = 0; i < clause.size(); ++i) {
      literals.push(GetLit(-m_model->GetPrime(clause[i])));
    }
  }
  bool result = addClause(literals);
  if (!result) {
    // placeholder
  }
}


std::pair<std::shared_ptr<std::vector<int>>, std::shared_ptr<std::vector<int>>> CarSolver::GetAssignment(std::ofstream &out) {
  out << "GetAssignment:" << std::endl;
  assert(m_model->GetNumInputs() < nVars());
  std::shared_ptr<std::vector<int>> inputs(new std::vector<int>());
  std::shared_ptr<std::vector<int>> latches(new std::vector<int>());
  inputs->reserve(m_model->GetNumInputs());
  latches->reserve(m_model->GetNumLatches());
  for (int i = 0; i < m_model->GetNumInputs(); ++i) {
    if (model[i] == l_True) {
      inputs->emplace_back(i + 1);
    } else if (model[i] == l_False) {
      inputs->emplace_back(-i - 1);
    }
  }
  for (int i = m_model->GetNumInputs(), end = m_model->GetNumInputs() + m_model->GetNumLatches(); i < end; ++i) {
    if (m_isForward) {
      if (model[i] == l_True) {
        latches->emplace_back(i + 1);
      } else if (model[i] == l_False) {
        latches->emplace_back(-i - 1);
      }
    } else {
      int p = m_model->GetPrime(i + 1);
      lbool val = model[abs(p) - 1];
      if ((val == l_True && p > 0) || (val == l_False && p < 0)) {
        latches->emplace_back(i + 1);
      } else {
        latches->emplace_back(-i - 1);
      }
    }
  }
  //
  for (auto it = inputs->begin(); it != inputs->end(); ++it) {
    out << *it << " ";
  }
  for (auto it = latches->begin(); it != latches->end(); ++it) {
    out << *it << " ";
  }
  out << std::endl;

  //
  return std::pair<std::shared_ptr<std::vector<int>>, std::shared_ptr<std::vector<int>>>(inputs, latches);
}

std::pair<std::shared_ptr<std::vector<int>>, std::shared_ptr<std::vector<int>>> CarSolver::GetAssignment() {
  assert(m_model->GetNumInputs() < nVars());
  std::shared_ptr<std::vector<int>> inputs(new std::vector<int>());
  std::shared_ptr<std::vector<int>> latches(new std::vector<int>());
  inputs->reserve(m_model->GetNumInputs());
  latches->reserve(m_model->GetNumLatches());
  for (int i = 0; i < m_model->GetNumInputs(); ++i) {
    if (model[i] == l_True) {
      inputs->emplace_back(i + 1);
    } else if (model[i] == l_False) {
      inputs->emplace_back(-i - 1);
    }
  }
  for (int i = m_model->GetNumInputs(), end = m_model->GetNumInputs() + m_model->GetNumLatches(); i < end; ++i) {
    if (m_isForward) {
      if (model[i] == l_True) {
        latches->emplace_back(i + 1);
      } else if (model[i] == l_False) {
        latches->emplace_back(-i - 1);
      }
    } else {
      int p = m_model->GetPrime(i + 1);
      lbool val = model[abs(p) - 1];
      if ((val == l_True && p > 0) || (val == l_False && p < 0)) {
        latches->emplace_back(i + 1);
      } else {
        latches->emplace_back(-i - 1);
      }
    }
  }
  return std::pair<std::shared_ptr<std::vector<int>>, std::shared_ptr<std::vector<int>>>(inputs, latches);
}


std::shared_ptr<std::vector<int>> CarSolver::GetUnsatisfiableCoreFromBad(int badId) {
  std::shared_ptr<std::vector<int>> uc(new std::vector<int>());
  uc->reserve(conflict.size());
  int val;

  for (int i = 0; i < conflict.size(); ++i) {
    val = -GetLiteralId(conflict[i]);
    if (m_model->IsLatch(val) && val != badId) {
      uc->emplace_back(val);
    }
  }
  std::sort(uc->begin(), uc->end(), cmp);
  return uc;
}

std::shared_ptr<std::vector<int>> CarSolver::GetUnsatisfiableCore() {
  std::shared_ptr<std::vector<int>> uc(new std::vector<int>());
  uc->reserve(conflict.size());
  int val;
  if (m_isForward) {
    for (int i = 0; i < conflict.size(); ++i) {
      val = -GetLiteralId(conflict[i]);
      std::vector<int> ids = m_model->GetPrevious(val);
      if (val > 0) {
        for (auto x : ids) {
          uc->push_back(x);
        }
      } else {
        for (auto x : ids) {
          uc->push_back(-x);
        }
      }
    }
  } else {
    for (int i = 0; i < conflict.size(); ++i) {
      val = -GetLiteralId(conflict[i]);
      if (m_model->IsLatch(val)) {
        uc->emplace_back(val);
      }
    }
  }

  std::sort(uc->begin(), uc->end(), cmp);
  return uc;
}


// ================================================================================
// @brief: after sat solve, get uc
// @input:
// @output:
// ================================================================================
std::shared_ptr<cube> CarSolver::Getuc(bool minimal) {
  // get conflict as assumption
  LSet ass;
  for (int i = 0; i < conflict.size(); i++)
    ass.insert(~conflict[i]);

  // compute muc
  if (minimal) Getmuc(ass);

  // get <int> form muc
  int val;
  std::shared_ptr<cube> muc(new cube());
  if (m_isForward) {
    for (int i = 0; i < ass.size(); ++i) {
      val = GetLiteralId(ass[i]);
      std::vector<int> ids = m_model->GetPrevious(val);
      if (val > 0) {
        for (auto x : ids) {
          muc->push_back(x);
        }
      } else {
        for (auto x : ids) {
          muc->push_back(-x);
        }
      }
    }
  } else {
    for (int i = 0; i < ass.size(); ++i) {
      val = GetLiteralId(ass[i]);
      if (m_model->IsLatch(val)) {
        muc->emplace_back(val);
      }
    }
  }
  std::sort(muc->begin(), muc->end(), cmp);
  return muc;
}


// ================================================================================
// @brief: get muc by uc
// @input: assumption
// @output:
// ================================================================================
void CarSolver::Getmuc(LSet &ass) {
  if (ass.size() > 215) return;
  vec<Lit> tass;
  LSet mass;

  while (ass.size() > 0) {
    int sz = ass.size();
    Lit t = ass[sz - 1];
    tass.clear();
    for (int i = 0; i < sz - 1; i++) tass.push(ass[i]);
    for (int i = 0; i < mass.size(); i++) tass.push(mass[i]);
    // tass.push(GetLit(GetFrameFlag(level)));
    lbool result = solveLimited(tass);
    if (result == l_True) {
      mass.insert(t);
      ass.clear();
      for (int i = 0; i < sz - 1; i++) ass.insert(tass[i]);
    } else if (result == l_False) {
      ass.clear();
      for (int i = 0; i < conflict.size(); i++) {
        if (!mass.has(~conflict[i]))
          ass.insert(~conflict[i]);
      }
    } else {
      assert(false);
    }
  }
  for (int i = 0; i < mass.size(); i++) ass.insert(mass[i]);
}


std::shared_ptr<std::vector<int>> CarSolver::justGetUC() {
  // get conflict as assumption
  LSet ass;
  for (int i = 0; i < conflict.size(); i++)
    ass.insert(~conflict[i]);

  // compute muc
  // Getmuc(ass);

  std::shared_ptr<std::vector<int>> uc(new std::vector<int>());
  uc->reserve(ass.size());
  int val;
  for (int i = 0; i < ass.size(); ++i) {
    val = GetLiteralId(ass[i]);
    if (m_model->IsLatch(val)) {
      uc->emplace_back(val);
    }
  }
  std::sort(uc->begin(), uc->end(), cmp);
  return uc;
}


void CarSolver::clean_assumptions() {
  m_assumptions.clear();
}


void CarSolver::AddNewFrame(const std::vector<std::shared_ptr<std::vector<int>>> &frame, int frameLevel) {
  for (int i = 0; i < frame.size(); ++i) {
    AddUnsatisfiableCore(*frame[i], frameLevel);
  }
  // std::cout << "clauses in SAT solver: \n";
  // for (int i = clauses.size ()-1; i > clauses.size ()-5; i --)
  // {
  // 	Clause& c = ca[clauses[i]];
  // 	for (int j = 0; j < c.size (); j ++)
  // 		std::cout << lit_id (c[j]) << " ";
  // 	std::cout << "0 " << std::endl;
  // }
}

std::string CarSolver::ShowLatest5Clause() {
  std::string res = "clauses in SAT solver: \n";
  for (int i = clauses.size() - 1; i > clauses.size() - 100; i--) {
    Clause &c = ca[clauses[i]];
    for (int j = 0; j < c.size(); j++)
      res += std::to_string(lit_id(c[j])) + " ";
    res += "0 \n";
  }
  return res;
}

bool CarSolver::SolveWithAssumptionAndBad(std::vector<int> &assumption, int badId) {
  m_assumptions.clear();
  m_assumptions.push(GetLit(badId));
  for (auto it = assumption.begin(); it != assumption.end(); ++it) {
    m_assumptions.push(GetLit(*it));
  }
  lbool result = solveLimited(m_assumptions);
  if (result == l_True) {
    return true;
  } else if (result == l_False) {
    return false;
  } else // result == l_Undef
  {
    // placeholder
    assert(true);
    return false;
  }
}

int CarSolver::get_temp_flag() {
  return GetNewVar();
}


void CarSolver::add_temp_clause(std::vector<int> *cls, int temp_flag, bool is_primed) {
  std::vector<int> *temp_cls = new std::vector<int>();
  temp_cls->emplace_back(-temp_flag);
  for (int l : *cls) {
    if (is_primed)
      temp_cls->emplace_back(m_model->GetPrime(l));
    else
      temp_cls->emplace_back(l);
  }
  // for (auto j : *temp_cls) {
  //   std::cout << j << " | ";
  // }
  // std::cout << std::endl;
  AddClause(*temp_cls);
}


void CarSolver::release_temp_cls(int temp_flag) {
  releaseVar(~GetLit(temp_flag));
}


#pragma region private


inline int CarSolver::GetFrameFlag(int frameLevel) {
  if (frameLevel < 0) {
    // placeholder
  }
  while (m_frameFlags.size() <= frameLevel) {
    m_frameFlags.emplace_back(m_maxFlag++);
  }
  return m_frameFlags[frameLevel];
}

inline int CarSolver::GetLiteralId(const Minisat::Lit &l) {
  return sign(l) ? -(var(l) + 1) : var(l) + 1;
}


#pragma endregion


} // namespace car