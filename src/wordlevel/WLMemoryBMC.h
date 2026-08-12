#ifndef WL_MEMORY_BMC_H
#define WL_MEMORY_BMC_H

#include "BaseAlg.h"
#include "WLTypes.h"

#include <memory>

namespace car {

class Log;
class WLModel;

// Exact bounded checker for BTOR2 memories.  It keeps memories word-level and
// encodes only the read/write forwarding relations needed by a finite trace.
class WLMemoryBMC {
  public:
    WLMemoryBMC(const Settings &settings, WLModel &model, Log &log);
    ~WLMemoryBMC();

    // Standalone semantics are BMC semantics: a bounded proof returns Unknown.
    CheckResult Run(unsigned bound);
    bool CompletedBound() const { return m_completedBound; }
    unsigned CheckedBound() const { return m_checkedBound; }
    const WLWitnessTrace &GetWitnessTrace() const { return m_witnessTrace; }

  private:
    class Impl;

    const Settings &m_settings;
    WLModel &m_model;
    Log &m_log;
    std::unique_ptr<Impl> m_impl;
    bool m_completedBound{false};
    unsigned m_checkedBound{0};
    WLWitnessTrace m_witnessTrace;
};

} // namespace car

#endif
