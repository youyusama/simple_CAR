#include "dimacs.h"
#include "solver.h"
#include "solver_types.h"
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace minicore;

static Solver *solver = nullptr;


static void SIGINT_exit(int) {
    if (solver != nullptr && solver->verbosity > 0) {
        solver->printStats();
    }
    std::cout << std::endl
              << "*** INTERRUPTED ***\n";
    exit(0);
}

static void printUsage(const char *argv0) {
    std::cout << "Usage: " << argv0 << " [-v] [--model] [input.cnf|-]\n";
}

static void printModel(const Solver &solver, int max_var) {
    std::cout << "v";
    for (int i = 1; i <= max_var; ++i) {
        const lbool value = solver.value(i);
        if (value == l_True) {
            std::cout << " " << i;
        } else if (value == l_False) {
            std::cout << " -" << i;
        }
    }
    std::cout << " 0\n";
}

int main(int argc, char **argv) {
    std::string filename = "";
    bool verbose = false;
    bool print_model = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "--model") == 0) {
            print_model = true;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
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
    DimacsStats stats;
    double initial_time = cpuTime();
    S.verbosity = verbose ? 1 : 0;
    solver = &S;

    sigTerm(SIGINT_exit);


    if (S.verbosity > 0) {
        std::cout << "============================[ Problem Statistics ]=============================" << std::endl;
        std::cout << "|                                                                             |" << std::endl;
    }

    bool parsed = false;
    std::string input_name = filename.empty() || filename == "-" ? "<stdin>" : filename;
    if (filename.empty() || filename == "-") {
        parsed = parse_DIMACS(std::cin, input_name, S, stats, std::cerr);
    } else {
        std::ifstream in(filename);
        if (!in.is_open()) {
            std::cerr << "Error! Cannot open file: " << filename << std::endl;
            return 1;
        }
        parsed = parse_DIMACS(in, input_name, S, stats, std::cerr);
    }

    if (!parsed) {
        std::cerr << "Error! Parse file failed." << std::endl;
        return 1;
    }

    if (S.verbosity > 0) {
        std::cout << "|  Number of variables:  "
                  << std::setw(12) << stats.declared_vars
                  << "                                         |\n";

        std::cout << "|  Number of clauses:    "
                  << std::setw(12) << stats.parsed_clauses
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

    lbool ret = S.solve();
    if (ret == l_True && print_model) {
        printModel(S, stats.declared_vars);
    }
    std::cout << (ret == l_True ? "SATISFIABLE\n" : ret == l_False ? "UNSATISFIABLE\n"
                                                                   : "INDETERMINATE\n");

    exit(ret == l_True ? 10 : ret == l_False ? 20
                                             : 0);
}
