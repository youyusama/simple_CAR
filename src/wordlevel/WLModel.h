#ifndef WL_MODEL_H
#define WL_MODEL_H

extern "C" {
#include "aiger.h"
}

#include "Settings.h"
#include "WLTypes.h"

#include <memory>
#include <string>
#include <vector>

namespace car {

class Log;
class Model;

struct WLModelLoadResult {
    // Completed WL pipeline result before constructing the bit-level Model.
    std::shared_ptr<aiger> aig;
    bool hasArrays{false};
    std::vector<WLArrayRead> arrayReads;
    WLTraceMap traceMap;
};

// Entry point for word-level model processing.  Format-specific frontends
// provide IRs; this module applies WL abstraction/optimization/lowering.
class WLModel {
  public:
    WLModel(const Settings &settings, Log &log);

    Model &BitModel() { return *m_model; }
    const Model &BitModel() const { return *m_model; }

    bool HasArrays() const { return m_hasArrays; }
    const std::vector<WLArrayRead> &ArrayReads() const { return m_arrayReads; }
    const WLTraceMap &TraceMap() const { return m_traceMap; }
    const std::shared_ptr<aiger> &GetAiger() const { return m_aig; }
    const std::string &InputPath() const { return m_inputPath; }

    void WriteBitblastOutput() const;

    // Rebuild the abstraction using the current CEGAR memory pairs.
    void Rebuild(const std::vector<WLMemoryPair> &memoryPairs);

  private:
    WLModelLoadResult
    LoadBtor2(const std::vector<WLMemoryPair> &memoryPairs);
    void AdoptLoadResult(WLModelLoadResult result);

    const Settings &m_settings;
    Log &m_log;
    std::string m_inputPath;
    std::shared_ptr<aiger> m_aig;
    std::unique_ptr<Model> m_model;
    bool m_hasArrays{false};
    std::vector<WLArrayRead> m_arrayReads;
    WLTraceMap m_traceMap;
};

} // namespace car

#endif
