#include "Settings.h"

namespace car {

bool ParseSettings(int argc, char **argv, Settings &settings) {
    CLI::App app{"simpleCAR: a Bit-Level CAR Model Checker"};

    app.add_option("-v", settings.verbosity, "Verbosity")
        ->default_val(0);

    app.add_option("aiger_file", settings.aigFilePath, "Input .aig File Path")
        ->required()
        ->check(CLI::ExistingFile);

    app.add_option("-w", settings.witnessOutputDir, "Witness Output Dir")
        ->check(CLI::ExistingDirectory);

    app.add_option("-a", settings.alg, "Model Checking Algorithm")
        ->transform(CLI::CheckedTransformer(
            std::map<std::string, MCAlgorithm>{
                {"fcar", MCAlgorithm::FCAR},
                {"bcar", MCAlgorithm::BCAR},
                {"bmc", MCAlgorithm::BMC},
                {"ic3", MCAlgorithm::IC3}}))
        ->default_val("fcar");

    app.add_option("-s", settings.solver, "Main SAT Solver")
        ->transform(CLI::CheckedTransformer(
            std::map<std::string, MCSATSolver>{
                {"minisat", MCSATSolver::minisat},
                {"cadical", MCSATSolver::cadical},
                {"minicore", MCSATSolver::minicore},
                {"kissat", MCSATSolver::kissat},
            }))
        ->default_val("minisat");

    app.add_option("-k", settings.bmcK, "BMC bound")
        ->default_val(-1);

    app.add_option("--step", settings.bmc_step, "Performs BMC by unrolling k steps in a single batch")
        ->default_val(1);

    app.add_option("--br", settings.branching, "branching # i-good lemma")
        ->default_val(1)
        ->check(CLI::Range(1, 3));

    app.add_option("--seed", settings.randomSeed, "random seed # i-good lemma")
        ->default_val(0)
        ->excludes("--br");

    app.add_flag("--rs", settings.referSkipping, "refer-skipping # i-good lemma")
        ->default_val(false);

    app.add_flag("--is", settings.internalSignals, "internal signals")
        ->default_val(false);

    app.add_flag("--restart", settings.restart, "enable restart mechanism")
        ->default_val(false);

    app.add_option("--restart_threshold", settings.restartThreshold, "restart threshold")
        ->default_val(128);

    app.add_option("--restart_growth_rate", settings.restartGrowthRate, "restart growth rate")
        ->default_val(1.5);

    app.add_flag("--luby", settings.restartLuby, "enable Luby's restart strategy")
        ->default_val(false);

    app.add_flag("--solve_in_property", settings.solveInProperty, "solve in property")
        ->default_val(false);

    app.add_option("--ctg_max_rec_lvl", settings.ctgMaxRecursionDepth, "CTG max recursion depth")
        ->default_val(2);

    app.add_option("--ctg_max_states", settings.ctgMaxStates, "CTG max states")
        ->default_val(3);

    app.add_flag("--sd", settings.satSolveInDomain, "solve SAT in domain")
        ->default_val(false)
        ->excludes("--is");

    app.add_flag("--bad_pred", settings.bad_pred, "enumerate bad predecessors")
        ->default_val(false);

    app.add_option("--max_obligation_act", settings.maxObligationAct, "maximum activity for obligations")
        ->default_val(20.0);

    app.add_option("--mic_rand_rate", settings.micRandRate, "random removal rate for MIC")
        ->default_val(0.0)
        ->check(CLI::Range(0.0, 1.0));
    
    app.add_flag("--lift_rand", settings.liftRand, "randomly lift literals in MIC")
        ->default_val(false);

    app.add_flag("--cadical_options_pre", settings.cadicalOptionsPre, "enable CaDiCaL preprocessing options")
        ->default_val(false);

    app.add_flag("--cadical_options_sol", settings.cadicalOptionsSol, "enable CaDiCaL solving options")
        ->default_val(false);
    
    app.add_flag("--cadical_simplify", settings.cadicalSimplify, "enable CaDiCaL simplify")
        ->default_val(false);

    try {
        app.parse(argc, argv);
        return true;
    } catch (const CLI::ParseError &e) {
        app.exit(e);
        return false;
    }
}

} // namespace car