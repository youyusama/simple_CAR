#include "solver.h"
#include "solver_types.h"
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace minicore;

static Solver *solver;


static void SIGINT_exit(int) {
    if (solver->verbosity > 0) {
        solver->printStats();
    }
    std::cout << std::endl
              << "*** INTERRUPTED ***\n";
    exit(0);
}


bool parse_DIMACS(const std::string &filename, Solver &S) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        std::cerr << "Error! Cannot open file: " << filename << std::endl;
        return false;
    }

    std::string line;
    int nvars = 0;
    int nclauses = 0;
    int parsed_clauses = 0;
    int line_num = 0;

    while (std::getline(in, line)) {
        line_num++;

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) continue;
        if (line[0] == 'c') continue;
        if (line[0] == 'p') {
            std::istringstream iss(line);
            std::string token;

            if (!(iss >> token) || token != "p") {
                std::cerr << "Error! Wrong head, line " << line_num << std::endl;
                return false;
            }

            if (!(iss >> token) || token != "cnf") {
                std::cerr << "Error! Only support cnf, line " << line_num << std::endl;
                return false;
            }

            if (!(iss >> nvars)) {
                std::cerr << "Error! Cannot read var num , line " << line_num << std::endl;
                return false;
            }

            if (!(iss >> nclauses)) {
                std::cerr << "Error! Cannot read clause num , line " << line_num << std::endl;
                return false;
            }

            for (int i = 0; i < nvars; i++) {
                S.newVar();
            }

            continue;
        }

        // handle clauses
        std::vector<Lit> clause;
        std::istringstream iss(line);
        int literal;

        while (iss >> literal) {
            if (literal == 0) {
                if (!clause.empty()) {
                    if (!S.addClause_(clause)) {
                        std::cerr << "Warning! Clause conflict, line " << line_num << std::endl;
                    }
                    clause.clear();
                    parsed_clauses++;
                }
                break;
            }

            if (abs(literal) > nvars) {
                std::cerr << "Error! Over indexed var " << abs(literal)
                          << " , line " << line_num << std::endl;
                return false;
            }

            int var;
            var = abs(literal) - 1;
            clause.push_back(literal > 0 ? mkLit(var) : ~mkLit(var));
        }
    }

    if (nclauses > 0 && parsed_clauses != nclauses) {
        std::cerr << "Warning! Wrong number of clauses " << parsed_clauses
                  << " different with declared" << nclauses << std::endl;
    }

    return true;
}


int main(int argc, char **argv) {
    std::string filename = "";
    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) {
            verbose = true;
        } else {
            if (filename.empty()) {
                filename = argv[i];
            } else {
                std::cerr << "Error! Too much args '" << argv[i] << "'\n";
                return 0;
            }
        }
    }

    Solver S;
    double initial_time = cpuTime();
    S.verbosity = verbose ? 1 : 0;
    solver = &S;

    sigTerm(SIGINT_exit);


    if (S.verbosity > 0) {
        std::cout << "============================[ Problem Statistics ]=============================" << std::endl;
        std::cout << "|                                                                             |" << std::endl;
    }

    if (!parse_DIMACS(filename, S)) {
        std::cerr << "Error! Parse file failed." << std::endl;
        return 0;
    }

    if (S.verbosity > 0) {
        std::cout << "|  Number of variables:  "
                  << std::setw(12) << S.nVars()
                  << "                                         |\n";

        std::cout << "|  Number of clauses:    "
                  << std::setw(12) << S.nClauses()
                  << "                                         |\n";
    }

    double parsed_time = cpuTime();
    if (S.verbosity > 0) {
        std::cout << "|  Parse time:           "
                  << std::setw(12) << std::fixed << std::setprecision(3)
                  << (parsed_time - initial_time)
                  << " s                                       |\n";

        std::cout << "|                                                                             |\n";
    }

    if (!S.simplify()) {
        if (S.verbosity > 0) {
            std::cout << "===============================================================================\n";
            std::cout << "Solved by unit propagation\n";
            S.printStats();
            std::cout << std::endl;
        }
        std::cout << "UNSATISFIABLE\n";
        exit(20);
    }

    lbool ret = S.solve_main();
    if (S.verbosity > 0) {
        S.printStats();
        std::cout << std::endl;
    }
    std::cout << (ret == l_True ? "SATISFIABLE\n" : ret == l_False ? "UNSATISFIABLE\n"
                                                                   : "INDETERMINATE\n");

    exit(ret == l_True ? 10 : ret == l_False ? 20
                                             : 0);
}