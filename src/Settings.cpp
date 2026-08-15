#include "Settings.h"

#include <filesystem>

namespace car {

bool ParseSettings(int argc, char **argv, Settings &settings) {
    CLI::App app{"simpleCAR: a Bit-Level CAR Model Checker"};

    app.add_option("-v", settings.verbosity, "Verbosity")
        ->default_val(0);

    app.add_option("input_file", settings.aigFilePath, "Input .aig, .aag, or .btor2 file")
        ->required()
        ->check(CLI::ExistingFile);

    app.add_option("-w", settings.witnessOutputDir, "Witness Output Dir")
        ->check(CLI::ExistingDirectory);

    app.add_flag("--wl-no-coi", settings.wlDisableCoi,
                 "Disable word-level property COI reduction")
        ->default_val(false);

    app.add_flag("--wl-no-package-resize", settings.wlDisablePackageResize,
                 "Disable word-level package resizing")
        ->default_val(false);

    app.add_option("--wl-bitblast-only", settings.wlBitblastOutputPath,
                   "Preprocess an array-free BTOR2 model and write the resulting .aig")
        ->type_name("OUTPUT.aig");

    app.add_option("-a", settings.alg, "Model Checking Algorithm")
        ->transform(CLI::CheckedTransformer(
            std::map<std::string, MCAlgorithm>{
                {"fcar", MCAlgorithm::FCAR},
                {"bcar", MCAlgorithm::BCAR},
                {"bmc", MCAlgorithm::BMC},
                {"wlbmc", MCAlgorithm::WLBMC},
                {"kind", MCAlgorithm::KIND},
                {"ic3", MCAlgorithm::IC3},
                {"l2s", MCAlgorithm::L2S},
                {"klive", MCAlgorithm::KLIVE},
                {"fair", MCAlgorithm::FAIR},
                {"kfair", MCAlgorithm::KFAIR},
                {"rlive", MCAlgorithm::RLIVE}}))
        ->default_val("ic3");

    app.add_option("--sa", settings.safetyBaseAlg, "Safety base algorithm")
        ->transform(CLI::CheckedTransformer(
            std::map<std::string, MCAlgorithm>{
                {"fcar", MCAlgorithm::FCAR},
                {"bcar", MCAlgorithm::BCAR},
                {"bmc", MCAlgorithm::BMC},
                {"kind", MCAlgorithm::KIND},
                {"ic3", MCAlgorithm::IC3}}))
        ->default_val("ic3");

    // app.add_option("--su,--shoal-unroll", settings.shoalUnroll, "unroll shoals for K cycles in rlive")
    //     ->default_val(1);

    app.add_flag("--pd,--rlive-prune-dead", settings.rlivePruneDead, "prune dead states in rlive")
        ->default_val(false);

    app.add_option("-s", settings.solver, "Main SAT Solver")
        ->transform(CLI::CheckedTransformer(
            std::map<std::string, MCSATSolver>{
                {"minisat", MCSATSolver::minisat},
                {"cadical", MCSATSolver::cadical},
                {"minicore", MCSATSolver::minicore},
                {"kissat", MCSATSolver::kissat},
            }))
        ->default_val("minicore");

    app.add_option("-k", settings.bmcK, "BMC bound")
        ->default_val(-1);

    app.add_flag("--bmc_cnf", settings.bmcCnf,
                 "Only generate the DIMACS CNF for one BMC instance")
        ->default_val(false);

    app.add_option("--bmc_cnf_dir", settings.bmcCnfDir,
                   "Output directory for generated BMC DIMACS CNF files");

    app.add_option("--bmc_cnf_k", settings.bmcCnfK,
                   "Target unrolling depth k for generated BMC DIMACS CNF")
        ->default_val(-1);

    app.add_option("--step", settings.bmcStep, "Performs BMC by unrolling k steps in a single batch")
        ->default_val(1);

    app.add_option("--br", settings.branching, "branching # i-good lemma")
        ->default_val(1)
        ->check(CLI::Range(1, 3));

    app.add_option("--seed", settings.randomSeed, "random seed # i-good lemma")
        ->default_val(0)
        ->excludes("--br");

    app.add_flag("--rs", settings.referSkipping, "refer-skipping # i-good lemma")
        ->default_val(false);

    // app.add_flag("--is", settings.internalSignals, "internal signals")
    //     ->default_val(false);

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

    app.add_option("--ctg_max_lvl", settings.ctgMaxRecursionDepth, "CTG max recursion depth")
        ->default_val(2);

    app.add_option("--ctg_max_ctg", settings.ctgMaxCTG, "CTG max counterexamples to generalization")
        ->default_val(3);

    app.add_option("--ctg_max_blocks", settings.ctgMaxBlocks, "CTG max blocks")
        ->default_val(1);

    app.add_option("--gen_max_fail", settings.genMaxFail, "Generalization max failed literal-drop attempts (0 means unlimited)")
        ->default_val(0);

    app.add_flag("--all,--active_lemma_learning", settings.activeLemmaLearning,
                 "enable Active Lemma Learning (ALL)")
        ->default_val(false);

    app.add_option("--all_threshold", settings.allThreshold, "ALL hotspot threshold")
        ->default_val(8)
        ->check(CLI::Range(1, INT32_MAX));

    app.add_option("--all_max_states", settings.allMaxStates, "ALL prover budget (max CTP states)")
        ->default_val(32)
        ->check(CLI::Range(1, INT32_MAX));

    app.add_option("--max_ob_act", settings.maxObligationAct, "max obligation activity")
        ->default_val(10.0);

    bool disableSatSolveInDomain = false;
    app.add_flag("--nosd", disableSatSolveInDomain,
                 "disable SAT solving in domain")
        ->default_val(false);

    app.add_option("--shrink", settings.shrink,
                   "shrink unsat core strategy: 0 disabled, 1-3 enabled")
        ->default_val(0)
        ->check(CLI::Range(0, 3));

    app.add_option("--eq", settings.eq,
                   "equivalent variable checking ( 0: none,\n\
                    1 : combination of 2 and 3,\n\
                    2 : ternary simulation,\n\
                    3 : random simulation,\n\
                    4 : SAT-based latch simulation) ")
        ->default_val(2);

    app.add_option("--eq_timeout", settings.eqTimeout, "equivalent variable checking timeout for SAT-backed equivalence checks (in seconds)")
        ->default_val(600);

    app.add_flag("--bp", settings.searchFromBadPred, "search from bad predecessor")
        ->default_val(false);

    app.add_flag("-t", settings.detailedTimers, "detailed timers")
        ->default_val(false);

    try {
        app.parse(argc, argv);
        settings.satSolveInDomain = !disableSatSolveInDomain;

        if (!settings.wlBitblastOutputPath.empty()) {
            if (std::filesystem::path(settings.aigFilePath).extension() !=
                ".btor2") {
                throw CLI::ValidationError(
                    "--wl-bitblast-only", "requires a .btor2 input file");
            }
            if (std::filesystem::path(settings.wlBitblastOutputPath)
                    .extension() != ".aig") {
                throw CLI::ValidationError(
                    "--wl-bitblast-only", "output path must end in .aig");
            }
        }

        if (settings.alg == MCAlgorithm::WLBMC) {
            if (std::filesystem::path(settings.aigFilePath).extension() !=
                ".btor2") {
                throw CLI::ValidationError(
                    "-a", "wlbmc requires a .btor2 input file");
            }
            if (settings.bmcK < 0) {
                throw CLI::ValidationError(
                    "-k", "wlbmc requires a non-negative bound");
            }
        }

        bool requires_incremental_safety_checker =
            settings.alg == MCAlgorithm::KLIVE ||
            settings.alg == MCAlgorithm::FAIR ||
            settings.alg == MCAlgorithm::KFAIR ||
            settings.alg == MCAlgorithm::RLIVE;
        bool is_incremental_safety_checker =
            settings.safetyBaseAlg == MCAlgorithm::FCAR ||
            settings.safetyBaseAlg == MCAlgorithm::IC3;
        if (requires_incremental_safety_checker && !is_incremental_safety_checker) {
            throw CLI::ValidationError(
                "--sa",
                "rlive, klive, fair, and kfair require 'fcar' or 'ic3'");
        }

        if (settings.bmcCnf) {
            if (settings.alg != MCAlgorithm::BMC) {
                throw CLI::ValidationError("--bmc_cnf", "requires '-a bmc'");
            }
            if (settings.bmcCnfDir.empty()) {
                throw CLI::ValidationError("--bmc_cnf_dir",
                                           "must be set when '--bmc_cnf' is enabled");
            }
            if (settings.bmcCnfK < 0) {
                throw CLI::ValidationError("--bmc_cnf_k",
                                           "must be a non-negative integer when '--bmc_cnf' is enabled");
            }
        } else {
            if (!settings.bmcCnfDir.empty()) {
                throw CLI::ValidationError("--bmc_cnf_dir", "requires '--bmc_cnf'");
            }
            if (settings.bmcCnfK != -1) {
                throw CLI::ValidationError("--bmc_cnf_k", "requires '--bmc_cnf'");
            }
        }

        return true;
    } catch (const CLI::ParseError &e) {
        app.exit(e);
        return false;
    }
}

} // namespace car
