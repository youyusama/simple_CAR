#include "WLModel.h"

#include "Btor2Frontend.h"
#include "Model.h"
#include "WLArrayAbstraction.h"
#include "WLBoolectorBitblast.h"
#include "WLPackageResize.h"

namespace car {

WLModel::WLModel(const Settings &settings, Log &log)
    : m_settings(settings), m_log(log), m_inputPath(settings.aigFilePath) {
    // The initial abstraction tracks no memory/address pairs.
    Rebuild({});
}

void WLModel::Rebuild(const std::vector<WLMemoryPair> &memoryPairs) {
    // Re-run the complete WL pipeline after each CEGAR refinement.
    WLModelLoadResult result = LoadBtor2(m_inputPath, memoryPairs);
    m_hasArrays = result.hasArrays;
    m_arrayReads = std::move(result.arrayReads);
    m_traceMap = std::move(result.traceMap);
    m_model = std::make_unique<Model>(m_settings, m_log, std::move(result.aig));
}

WLModelLoadResult
WLModel::LoadBtor2(const std::string &path,
                   const std::vector<WLMemoryPair> &memoryPairs) {
    // Stage 1: parse and validate the format-specific source model.
    Btor2IR ir = Btor2Frontend::LoadIR(path);
    const bool hasArrays = ir.HasArrays();

    // Stage 2: eliminate arrays through selected-slot abstraction.
    WLArrayAbstractionResult abstraction =
        WLArrayAbstraction::Run(ir, memoryPairs);
    if (abstraction.ir.HasArrays()) {
        throw std::runtime_error(
            "array abstraction produced an array-valued word-level model");
    }

    // Stage 3: compact safe finite-domain packages in the array-free IR.
    WLPackageResize::Run(abstraction.ir);

    // Stage 4: standard bitblast produces the bit-level model and trace mapping.
    WLTraceMap traceMap;
    traceMap.memoryPairs = std::move(abstraction.tracePairs);
    std::shared_ptr<aiger> aig = GenerateWLAig(
        abstraction.ir, abstraction.traceSources, traceMap);

    return {std::move(aig),
            hasArrays,
            std::move(abstraction.reads),
            std::move(traceMap)};
}

} // namespace car
