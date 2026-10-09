#pragma once

#include "Btor2IR.h"
#include "WLTrace.h"

namespace car {

// One segment-level finite-domain resizing pass. Owns its output IR and the
// private correspondence needed to restore execution choices to its input IR.
class PackageResize {
  public:
    explicit PackageResize(const Btor2IR &ir);

    const Btor2IR &IR() const { return m_ir; }
    // Lift choices for IR() back to the input IR. An omitted port stays omitted;
    // a supplied split port must include all its segments. Derived states and
    // intermediate expressions are left to simulation in the input IR.
    WLTrace RestoreTrace(const WLTrace &trace) const;

  private:
    class Rewriter;
    struct Segment {
        int64_t nodeId;
        uint32_t offset;
        uint32_t originalWidth;
    };
    struct Port {
        int64_t nodeId;
        uint32_t width;
        std::vector<Segment> segments;
    };
    void RestorePorts(const std::vector<Port> &ports,
                      const std::unordered_map<int64_t, BitVector> &values,
                      std::unordered_map<int64_t, BitVector> &restored) const;

    Btor2IR m_ir;
    std::vector<Port> m_inputs;
    std::vector<Port> m_states;
};

} // namespace car
