#include "dimacs.h"
#include <cstdlib>
#include <sstream>

namespace minicore {

namespace {

bool parseClauseLine(const std::string &line,
                     const std::string &name,
                     int line_num,
                     Solver &solver,
                     int max_var,
                     std::vector<Lit> &clause,
                     DimacsStats &stats,
                     std::ostream &err) {
    std::istringstream iss(line);
    std::string token;
    while (iss >> token) {
        if (token == "c") {
            break;
        }

        char *end = nullptr;
        long lit = std::strtol(token.c_str(), &end, 10);
        if (end == token.c_str() || *end != '\0') {
            err << name << ":" << line_num
                << ": parse error: invalid token '" << token << "'\n";
            return false;
        }

        if (lit == 0) {
            solver.addClause_(clause);
            clause.clear();
            stats.parsed_clauses++;
            continue;
        }

        int var = std::abs(static_cast<int>(lit));
        if (var > max_var) {
            err << name << ":" << line_num
                << ": parse error: literal " << lit
                << " exceeds declared variable bound " << max_var << "\n";
            return false;
        }

        clause.push_back(lit > 0 ? mkLit(var) : ~mkLit(var));
    }

    return true;
}

} // namespace

bool parse_DIMACS(std::istream &in,
                  const std::string &name,
                  Solver &solver,
                  DimacsStats &stats,
                  std::ostream &err) {
    std::string line;
    std::vector<Lit> clause;
    bool header_seen = false;
    int line_num = 0;

    while (std::getline(in, line)) {
        line_num++;

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        const size_t pos = line.find_first_not_of(" \t");
        if (pos == std::string::npos) {
            continue;
        }

        if (line[pos] == 'c') {
            continue;
        }

        if (!header_seen) {
            std::istringstream iss(line.substr(pos));
            std::string prefix;
            std::string format;
            if (!(iss >> prefix >> format >> stats.declared_vars >> stats.declared_clauses) ||
                prefix != "p" || format != "cnf") {
                err << name << ":" << line_num
                    << ": parse error: expected 'p cnf <vars> <clauses>' header\n";
                return false;
            }

            std::string extra;
            if (iss >> extra) {
                err << name << ":" << line_num
                    << ": parse error: unexpected token '" << extra << "' after header\n";
                return false;
            }

            for (int i = 0; i < stats.declared_vars; ++i) {
                solver.newVar();
            }

            header_seen = true;
            continue;
        }

        if (!parseClauseLine(line.substr(pos),
                             name,
                             line_num,
                             solver,
                             stats.declared_vars,
                             clause,
                             stats,
                             err)) {
            return false;
        }
    }

    if (!header_seen) {
        err << name << ": parse error: missing DIMACS header\n";
        return false;
    }

    if (!clause.empty()) {
        err << name << ":" << line_num
            << ": parse error: last clause without terminating 0\n";
        return false;
    }

    if (stats.declared_clauses != stats.parsed_clauses) {
        err << name << ": warning: parsed " << stats.parsed_clauses
            << " clauses but header declares " << stats.declared_clauses << "\n";
    }

    return true;
}

} // namespace minicore
