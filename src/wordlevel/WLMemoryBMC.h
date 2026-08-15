#ifndef WL_MEMORY_BMC_H
#define WL_MEMORY_BMC_H

#include "BaseAlg.h"
#include "WLTypes.h"

#include <memory>

namespace car {

class Log;
class WLModel;
struct Settings;

// Exact bounded checker for BTOR2 memories.  It keeps memories word-level and
// encodes only the read/write forwarding relations needed by a finite trace.
class WLMemoryBMC {
  public:
    WLMemoryBMC(const Settings &settings, WLModel &model, Log &log);
    ~WLMemoryBMC();

    // BMC finds a bounded counterexample or returns Unknown after exhausting k.
    CheckResult Run(unsigned bound);
    const WLWitnessTrace &GetWitnessTrace() const { return m_witnessTrace; }

  private:
    class Impl;

    const Settings &m_settings;
    WLModel &m_model;
    Log &m_log;
    std::unique_ptr<Impl> m_impl;
    WLWitnessTrace m_witnessTrace;
};

} // namespace car

#endif
