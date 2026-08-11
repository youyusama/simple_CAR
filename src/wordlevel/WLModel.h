#ifndef WL_MODEL_H
#define WL_MODEL_H

extern "C" {
#include "aiger.h"
}

#include "Settings.h"
#include "CarTypes.h"
#include "WLTypes.h"

#include <memory>
#include <string>
#include <vector>

namespace car {

class Log;
class Model;
class Btor2IR;

struct WLModelBuildResult {
    // Completed WL pipeline result before constructing the bit-level Model.
    std::shared_ptr<aiger> aig;
    bool sourceHasArrays{false};
    WLTraceMap traceMap;
};

// Entry point for word-level model processing.  Format-specific frontends
// provide IRs; this module applies WL abstraction/optimization/lowering.
class WLModel {
  public:
    WLModel(const Settings &settings, Log &log);
    ~WLModel();

    Model &BitModel() { return *m_model; }
    const Model &BitModel() const { return *m_model; }

    bool SourceHasArrays() const { return m_sourceHasArrays; }
    const Btor2IR &SourceIR() const { return *m_sourceIr; }

    // Decode the bit-level checker interface into a word-level replay seed.
    WLReplayTrace DecodeBitTrace(
        const std::vector<std::pair<Cube, Cube>> &trace) const;

    void WriteBitblastAig() const;

    // Run the complete WL pipeline using the current CEGAR memory pairs.
    void Build(const std::vector<WLMemoryPair> &memoryPairs);

  private:
    WLModelBuildResult
    BuildFromBtor2(const std::vector<WLMemoryPair> &memoryPairs);

    const Settings &m_settings;
    Log &m_log;
    std::string m_inputPath;
    std::unique_ptr<Btor2IR> m_sourceIr;
    std::shared_ptr<aiger> m_aig;
    std::unique_ptr<Model> m_model;
    bool m_sourceHasArrays{false};
    WLTraceMap m_traceMap;
};

} // namespace car

#endif
