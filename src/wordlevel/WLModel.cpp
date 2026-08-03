#include "WLModel.h"

#include "Btor2Frontend.h"
#include "Model.h"
#include "WLArrayAbstraction.h"
#include "WLBoolectorBitblast.h"
#include "WLPackageResize.h"

#include <stdexcept>

namespace car {

WLModel::WLModel(const Settings &settings, Log &log)
    : m_settings(settings), m_log(log), m_inputPath(settings.aigFilePath) {
    // The initial abstraction tracks no memory/address pairs.
    Build({});
}

void WLModel::WriteBitblastAig() const {
    if (!m_aig || m_settings.wlBitblastOutputPath.empty()) {
        throw std::runtime_error("missing AIGER bitblast output");
    }
    if (!aiger_open_and_write_to_file(
            m_aig.get(), m_settings.wlBitblastOutputPath.c_str())) {
        throw std::runtime_error(
            "failed to write AIGER output: " +
            m_settings.wlBitblastOutputPath);
    }
}

void WLModel::Build(const std::vector<WLMemoryPair> &memoryPairs) {
    // Re-run the complete WL pipeline after each CEGAR refinement.
    WLModelBuildResult result = BuildFromBtor2(memoryPairs);
    m_sourceHasArrays = result.sourceHasArrays;
    m_arrayReads = std::move(result.arrayReads);
    m_traceMap = std::move(result.traceMap);
    m_aig = std::move(result.aig);

    // AIG export stops at the final bitblast, before Model simplification.
    if (!m_settings.wlBitblastOutputPath.empty()) {
        m_model.reset();
        return;
    }
    m_model = std::make_unique<Model>(m_settings, m_log, m_aig);
}

WLModelBuildResult
WLModel::BuildFromBtor2(const std::vector<WLMemoryPair> &memoryPairs) {
    // Stage 1: parse and validate the format-specific source model.
    Btor2IR ir = Btor2Frontend::LoadIR(m_inputPath);
    const bool sourceHasArrays = ir.HasArrays();
    const bool bitblastOnly = !m_settings.wlBitblastOutputPath.empty();

    Btor2IR processedIr;
    std::vector<WLMemoryPair> tracePairs;
    std::vector<WLArrayRead> reads;
    WLIRTraceMap traceSources;

    // Stage 2: normal checking abstracts arrays; export mode rejects them.
    if (bitblastOnly) {
        if (sourceHasArrays) {
            throw std::runtime_error(
                "--wl-bitblast-only accepts only array-free BTOR2 input");
        }
        processedIr = std::move(ir);
    } else {
        WLArrayAbstractionResult abstraction =
            WLArrayAbstraction::Run(ir, memoryPairs);
        if (abstraction.ir.HasArrays()) {
            throw std::runtime_error(
                "array abstraction produced an array-valued word-level model");
        }
        processedIr = std::move(abstraction.ir);
        tracePairs = std::move(abstraction.tracePairs);
        reads = std::move(abstraction.reads);
        traceSources = std::move(abstraction.traceSources);
    }

    // Stage 3: compact safe finite-domain packages in the array-free IR.
    if (!m_settings.wlDisablePackageResize)
        WLPackageResize::Run(processedIr, traceSources);

    // Stage 4: standard bitblast produces the bit-level model and trace mapping.
    WLTraceMap traceMap;
    traceMap.memoryPairs = std::move(tracePairs);
    std::shared_ptr<aiger> aig =
        GenerateWLAig(processedIr, traceSources, traceMap);

    return {std::move(aig),
            sourceHasArrays,
            std::move(reads),
            std::move(traceMap)};
}

} // namespace car
