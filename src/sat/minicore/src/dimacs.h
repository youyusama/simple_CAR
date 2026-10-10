#pragma once

#include "solver.h"
#include <iosfwd>
#include <string>

namespace minicore {

struct DimacsStats {
    int declared_vars = 0;
    int declared_clauses = 0;
    int parsed_clauses = 0;
};

bool parse_DIMACS(std::istream &in,
                  const std::string &name,
                  Solver &solver,
                  DimacsStats &stats,
                  std::ostream &err);

} // namespace minicore
