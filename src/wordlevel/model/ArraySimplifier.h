#pragma once

#include "Btor2IR.h"
#include "WLTrace.h"
#include <map>
#include <string>
#include <vector>

namespace car {

// One-time, property-preserving array definition elimination. All reconstruction
// expressions and private fallback inputs belong to this pass, not to WLTrace.
class ArraySimplifier {
  public:
    struct Statistics {
        size_t comparisonsBefore{0}, comparisonsAfter{0}, definedInputs{0};
        std::string context;
        std::vector<std::string> skippedRules;
    };

    explicit ArraySimplifier(const Btor2IR &source);
    const Btor2IR &IR() const { return m_ir; }
    const Statistics &Stats() const { return m_stats; }
    // Input choices refer to IR(); output choices refer to the constructor IR.
    // The caller still completes source COI choices and verifies SourceIR.
    void RestoreTrace(WLTrace &trace, const Btor2IR &property) const;

  private:
    Btor2IR m_ir, m_replayIr;
    std::map<int64_t, int64_t> m_definitions;
    std::vector<int64_t> m_privateInputs;
    Statistics m_stats;
};

} // namespace car
