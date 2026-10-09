#pragma once

#include "Btor2IR.h"
#include "Settings.h"
#include "WLTrace.h"
#include <memory>
#include <string>

namespace car {
class Log;
class WLArraySimplifier;

// Fixed input semantics. Checking sessions own their precision and encodings.
class WLModel {
  public:
    WLModel(const Settings &settings, Log &log);
    ~WLModel();
    bool SourceHasArrays() const { return m_sourceIr.HasArrays(); }
    const Btor2IR &SourceIR() const { return m_sourceIr; }
    const Btor2IR &PropertyIR() const;
    void RestoreSourceTrace(WLTrace &trace) const;
    // Export a transformed copy; fixed source/property semantics stay intact.
    void WriteScalarAig(const std::string &path, bool resize) const;

  private:
    Btor2IR m_sourceIr;
    bool m_disableCoi;
    Log &m_log;
    mutable std::unique_ptr<WLArraySimplifier> m_arraySimplifier;
    mutable std::unique_ptr<Btor2IR> m_propertyIr;
};
} // namespace car
