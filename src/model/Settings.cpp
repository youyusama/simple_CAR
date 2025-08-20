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
            }))
        ->default_val("fcar");

    app.add_option("-s", settings.solver, "Main SAT Solver")
        ->transform(CLI::CheckedTransformer(
            std::map<std::string, MCSATSolver>{
                {"minisat", MCSATSolver::minisat},
                {"cadical", MCSATSolver::cadical},
                {"minicore", MCSATSolver::minicore},
            }))
        ->default_val("minisat");

    app.add_option("-k", settings.bmcK, "BMC bound")
        ->default_val(-1);

    app.add_option("--br", settings.branching, "branching # i-good lemma")
        ->default_val(1)
        ->check(CLI::Range(1, 3));

    app.add_option("--seed", settings.randomSeed, "random seed # i-good lemma")
        ->default_val(0)
        ->excludes("--br");

    app.add_flag("--rs", settings.referSkipping, "refer-skipping # i-good lemma");

    app.add_flag("--is", settings.internalSignals, "internal signals");

    app.add_flag("--restart", settings.restart, "enable restart mechanism");

    app.add_option("--restart_threshold", settings.restart_threshold, "restart threshold")
        ->default_val(128);

    app.add_option("--restart_growth_rate", settings.restart_growth_rate, "restart growth rate")
        ->default_val(1.5);

    app.add_flag("--luby", settings.luby, "enable Luby's restart strategy");

    try {
        app.parse(argc, argv);
        return true;
    } catch (const CLI::ParseError &e) {
        app.exit(e);
        return false;
    }
}

} // namespace car