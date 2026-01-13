#include "solver.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using minicore::Lit;
using minicore::Solver;
using minicore::lbool;
using minicore::mkLit;
using minicore::var;
using minicore::Var;

struct Cnf {
    std::vector<std::vector<int>> clauses;
    int max_var = 0;
};

static void usage() {
    std::cerr << "usage: minicore_bench <cnf> [--repeat N] [--assumptions N] [--seed S] [--assumptions-file PATH] [--domain-file PATH]\n";
}

static Cnf parse_dimacs(const std::string &path) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "cannot open " << path << "\n";
        std::exit(1);
    }
    Cnf cnf;
    std::string line;
    std::vector<int> clause;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        if (line[0] == 'c' || line[0] == 'p') {
            continue;
        }
        std::istringstream iss(line);
        int lit = 0;
        while (iss >> lit) {
            if (lit == 0) {
                if (!clause.empty()) {
                    cnf.clauses.emplace_back(std::move(clause));
                    clause.clear();
                }
            } else {
                int v = std::abs(lit);
                if (v > cnf.max_var) {
                    cnf.max_var = v;
                }
                clause.push_back(lit);
            }
        }
    }
    if (!clause.empty()) {
        std::cerr << "dimacs missing trailing 0 for a clause\n";
        std::exit(1);
    }
    if (cnf.clauses.empty()) {
        std::cerr << "empty cnf: " << path << "\n";
        std::exit(1);
    }
    return cnf;
}

struct RunStats {
    double solve_ms = 0.0;
    uint64_t decisions = 0;
    uint64_t conflicts = 0;
    uint64_t propagations = 0;
    uint64_t restarts = 0;
    uint64_t search_ns = 0;
    uint64_t analyze_ns = 0;
    uint64_t cancel_ns = 0;
    uint64_t reduce_ns = 0;
    uint64_t decide_ns = 0;
    uint64_t prop_sample_ns = 0;
    uint64_t prop_sample_hits = 0;
    uint64_t prop_calls = 0;
    bool sat = false;
};

static uint64_t lcg_next(uint64_t &state) {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return state;
}

static std::vector<Lit> make_assumptions(int max_var,
                                         int count,
                                         uint64_t seed,
                                         int run_idx) {
    if (count > max_var) {
        count = max_var;
    }
    std::vector<Lit> assumps;
    assumps.reserve(count);
    std::vector<char> used(static_cast<size_t>(max_var) + 1, 0);
    uint64_t state =
        seed ^ (static_cast<uint64_t>(run_idx) + 1ULL) * 0x9E3779B97F4A7C15ULL;
    while (static_cast<int>(assumps.size()) < count) {
        uint64_t vraw = lcg_next(state);
        int v = static_cast<int>(vraw % static_cast<uint64_t>(max_var)) + 1;
        if (used[static_cast<size_t>(v)]) {
            continue;
        }
        used[static_cast<size_t>(v)] = 1;
        bool neg = (lcg_next(state) >> 63) != 0;
        assumps.emplace_back(mkLit(v, neg));
    }
    return assumps;
}

static std::vector<std::vector<Lit>> load_assumptions(const std::string &path, int max_var) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "cannot open assumptions file " << path << "\n";
        std::exit(1);
    }
    std::vector<std::vector<Lit>> runs;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        if (line[0] == 'c' || line[0] == 'p') {
            continue;
        }
        std::istringstream iss(line);
        int lit = 0;
        std::vector<Lit> assumps;
        while (iss >> lit) {
            if (lit == 0) {
                break;
            }
            int v = std::abs(lit);
            if (v > max_var) {
                std::cerr << "assumption var out of range: " << v << "\n";
                std::exit(1);
            }
            assumps.emplace_back(mkLit(v, lit < 0));
        }
        if (!assumps.empty()) {
            runs.emplace_back(std::move(assumps));
        }
    }
    if (runs.empty()) {
        std::cerr << "assumptions file is empty: " << path << "\n";
        std::exit(1);
    }
    return runs;
}

static std::vector<std::vector<Var>> load_domains(const std::string &path, int max_var) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "cannot open domain file " << path << "\n";
        std::exit(1);
    }
    std::vector<std::vector<Var>> runs;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        if (line[0] == 'c' || line[0] == 'p') {
            continue;
        }
        std::istringstream iss(line);
        int lit = 0;
        std::vector<Var> domain;
        while (iss >> lit) {
            if (lit == 0) {
                break;
            }
            int v = std::abs(lit);
            if (v > max_var) {
                std::cerr << "domain var out of range: " << v << "\n";
                std::exit(1);
            }
            domain.emplace_back(static_cast<Var>(v));
        }
        if (!domain.empty()) {
            runs.emplace_back(std::move(domain));
        }
    }
    if (runs.empty()) {
        std::cerr << "domain file is empty: " << path << "\n";
        std::exit(1);
    }
    return runs;
}
int main(int argc, char **argv) {
    std::string cnf_path;
    int repeat = 1000;
    int assumption_count = -1;
    uint64_t seed = 0;
    std::string assumptions_path;
    std::string domain_path;
    bool repeat_set = false;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--repeat" && i + 1 < argc) {
            repeat = std::atoi(argv[++i]);
            repeat_set = true;
        } else if (arg == "--assumptions" && i + 1 < argc) {
            assumption_count = std::atoi(argv[++i]);
        } else if (arg == "--seed" && i + 1 < argc) {
            seed = static_cast<uint64_t>(std::strtoull(argv[++i], nullptr, 10));
        } else if (arg == "--assumptions-file" && i + 1 < argc) {
            assumptions_path = argv[++i];
        } else if (arg == "--domain-file" && i + 1 < argc) {
            domain_path = argv[++i];
        } else if (cnf_path.empty()) {
            cnf_path = arg;
        } else {
            usage();
            return 1;
        }
    }
    if (cnf_path.empty()) {
        usage();
        return 1;
    }

    auto parse_start = std::chrono::steady_clock::now();
    Cnf cnf = parse_dimacs(cnf_path);
    auto parse_end = std::chrono::steady_clock::now();
    double parse_ms =
        std::chrono::duration<double, std::milli>(parse_end - parse_start)
            .count();

    Solver solver;
    solver.verbosity = 0;
    for (const auto &cls : cnf.clauses) {
        std::vector<Lit> lits;
        lits.reserve(cls.size());
        for (int l : cls) {
            Var v = std::abs(l);
            while (v >= solver.nVars()) {
                solver.newVar();
            }
            lits.emplace_back(mkLit(v, l < 0));
        }
        solver.addClause(lits);
    }

    if (assumption_count < 0) {
        assumption_count = std::min(10, cnf.max_var);
    }

    std::vector<std::vector<Lit>> assumptions;
    if (!assumptions_path.empty()) {
        assumptions = load_assumptions(assumptions_path, cnf.max_var);
        assumption_count = static_cast<int>(assumptions[0].size());
        if (!repeat_set) {
            repeat = static_cast<int>(assumptions.size());
        } else if (repeat > static_cast<int>(assumptions.size())) {
            std::cerr << "repeat exceeds assumptions count\n";
            return 1;
        }
    }

    std::vector<std::vector<Var>> domains;
    if (!domain_path.empty()) {
        if (assumptions_path.empty()) {
            std::cerr << "domain-file requires assumptions-file to keep runs aligned\n";
            return 1;
        }
        domains = load_domains(domain_path, cnf.max_var);
        if (domains.size() != assumptions.size()) {
            std::cerr << "domain count does not match assumptions count\n";
            return 1;
        }
        if (!repeat_set) {
            repeat = static_cast<int>(domains.size());
        } else if (repeat > static_cast<int>(domains.size())) {
            std::cerr << "repeat exceeds domains count\n";
            return 1;
        }
    }

    bool use_domain = !domains.empty();
    if (use_domain) {
        solver.solve_in_domain = true;
    }

    RunStats sum;
    uint64_t sat_count = 0;
    uint64_t unsat_count = 0;
    uint64_t domain_vars_sum = 0;
    for (int i = 0; i < repeat; i++) {
        std::vector<Lit> assumps;
        if (!assumptions.empty()) {
            assumps = assumptions[static_cast<size_t>(i)];
        } else {
            assumps = make_assumptions(cnf.max_var, assumption_count, seed, i);
        }
        if (use_domain) {
            const auto &domain = domains[static_cast<size_t>(i)];
            domain_vars_sum += domain.size();
            solver.resetTempDomain();
            solver.setTempDomain(domain);
        }
        uint64_t d0 = solver.decisions;
        uint64_t c0 = solver.conflicts;
        uint64_t p0 = solver.propagations;
        uint64_t r0 = solver.starts;
        auto start = std::chrono::steady_clock::now();
        lbool res = solver.solve(assumps);
        auto end = std::chrono::steady_clock::now();
        const auto &prof = solver.profileStats();

        sum.solve_ms +=
            std::chrono::duration<double, std::milli>(end - start).count();
        sum.decisions += solver.decisions - d0;
        sum.conflicts += solver.conflicts - c0;
        sum.propagations += solver.propagations - p0;
        if (solver.starts > r0) {
            sum.restarts += (solver.starts - r0) - 1;
        }
        sum.search_ns += prof.search_ns;
        sum.analyze_ns += prof.analyze_ns;
        sum.cancel_ns += prof.cancel_ns;
        sum.reduce_ns += prof.reduce_ns;
        sum.decide_ns += prof.decide_ns;
        sum.prop_sample_ns += prof.prop_sample_ns;
        sum.prop_sample_hits += prof.prop_sample_hits;
        sum.prop_calls += prof.prop_calls;
        if (res == minicore::l_True) {
            sat_count++;
        } else if (res == minicore::l_False) {
            unsat_count++;
        }
    }
    double avg_search_ms = sum.search_ns / static_cast<double>(repeat) / 1e6;
    double avg_analyze_ms = sum.analyze_ns / static_cast<double>(repeat) / 1e6;
    double avg_cancel_ms = sum.cancel_ns / static_cast<double>(repeat) / 1e6;
    double avg_reduce_ms = sum.reduce_ns / static_cast<double>(repeat) / 1e6;
    double avg_decide_ms = sum.decide_ns / static_cast<double>(repeat) / 1e6;
    double prop_sample_rate =
        sum.prop_calls == 0 ? 0.0 : static_cast<double>(sum.prop_sample_hits) / static_cast<double>(sum.prop_calls);
    double avg_prop_sample_est_ms = 0.0;
    if (sum.prop_sample_hits > 0 && sum.prop_calls > 0) {
        double avg_sample_ns = static_cast<double>(sum.prop_sample_ns) / static_cast<double>(sum.prop_sample_hits);
        avg_prop_sample_est_ms = (avg_sample_ns * static_cast<double>(sum.prop_calls)) /
                                 static_cast<double>(repeat) / 1e6;
    }
    uint64_t residual_ns = 0;
    if (sum.search_ns > sum.analyze_ns + sum.cancel_ns + sum.reduce_ns + sum.decide_ns) {
        residual_ns = sum.search_ns - sum.analyze_ns - sum.cancel_ns - sum.reduce_ns - sum.decide_ns;
    }
    double avg_residual_ms = residual_ns / static_cast<double>(repeat) / 1e6;
    double avg_props = sum.propagations / static_cast<double>(repeat);
    double avg_prop_ms = avg_props == 0.0 ? 0.0 : (avg_residual_ms / avg_props);
    std::cout << "solver=minicore\n"
              << "runs=" << repeat << "\n"
              << "parse_ms=" << parse_ms << "\n"
              << "avg_time_ms=" << (sum.solve_ms / repeat) << "\n"
              << "avg_decisions=" << (sum.decisions / repeat) << "\n"
              << "avg_conflicts=" << (sum.conflicts / repeat) << "\n"
              << "avg_propagations=" << (sum.propagations / repeat) << "\n"
              << "avg_restarts=" << (sum.restarts / repeat) << "\n"
              << "avg_prop_time_ms=" << avg_prop_ms << "\n"
              << "avg_search_ms=" << avg_search_ms << "\n"
              << "avg_analyze_ms=" << avg_analyze_ms << "\n"
              << "avg_cancel_ms=" << avg_cancel_ms << "\n"
              << "avg_reduce_ms=" << avg_reduce_ms << "\n"
              << "avg_decide_ms=" << avg_decide_ms << "\n"
              << "avg_prop_sample_est_ms=" << avg_prop_sample_est_ms << "\n"
              << "avg_residual_ms=" << avg_residual_ms << "\n"
              << "prop_sample_rate=" << prop_sample_rate << "\n"
              << "avg_prop_calls=" << (sum.prop_calls / repeat) << "\n"
              << "sat=" << sat_count << "\n"
              << "unsat=" << unsat_count << "\n"
              << "assumptions=" << assumption_count << "\n"
              << "assumptions_file=" << (assumptions_path.empty() ? "-" : assumptions_path) << "\n"
              << "domain_file=" << (domain_path.empty() ? "-" : domain_path) << "\n"
              << "avg_domain_vars=" << (use_domain ? (domain_vars_sum / repeat) : 0) << "\n"
              << "vars=" << cnf.max_var << "\n"
              << "clauses=" << cnf.clauses.size() << "\n\n";
    return 0;
}
