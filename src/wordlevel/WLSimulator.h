#ifndef WL_SIMULATOR_H
#define WL_SIMULATOR_H

#include "model/Btor2IR.h"
#include "WLTrace.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace car {

// Deterministic IR execution and concrete verification; no abstraction or solver.
class WLSimulator {
  public:
    enum class VerificationKind { Confirmed, Invalid, Incomplete, Unsupported, InternalError };

    struct VerificationResult {
        VerificationKind kind{VerificationKind::Incomplete};
        unsigned time{0};
        int64_t nodeId{0};
        std::string reason;
    };

    explicit WLSimulator(const Btor2IR &ir);
    ~WLSimulator();

    enum class MissingChoices { Reject, Zero };
    struct Execution {
        // Only requested BV expressions are retained, keyed by their signed IR ID.
        std::vector<std::unordered_map<int64_t, WLBitVector>> observations;
        std::vector<bool> constraintsHold;
        std::vector<bool> bad;
    };

    // A view of the current execution frame, valid only during onFrame. Queries
    // share the executor's caches and see the same overrides as property/next
    // evaluation. Returned values are owned copies.
    class Frame {
      public:
        virtual ~Frame() = default;
        virtual WLBitVector Scalar(int64_t id) const = 0;
        virtual WLArrayValue Array(int64_t id) const = 0;
        virtual WLBitVector ReadArray(int64_t id, const WLBitVector &address) const = 0;
        virtual std::optional<WLBitVector> FindArrayDifference(
            int64_t lhs, int64_t rhs) const = 0;
    };

    struct SimulationOptions {
        // Optional per-frame replacements of positive scalar expression IDs.
        // States and init/next statements cannot be overridden. Values remain
        // fixed while all dependent expressions and successor states recompute.
        std::vector<std::unordered_map<int64_t, WLBitVector>> overrides;
        std::function<void(size_t, const Frame &)> onFrame;
    };

    // Execute any supported IR. Property outcomes are reported, not assumed.
    // Reject malformed ports/init/next; throw with frame/node diagnostics.
    // Zero fills free choices only, including unspecified array cells. Derived
    // states always follow init/next. Repeated executions start from fresh caches.
    Execution Simulate(const WLTrace &choices, const std::vector<int64_t> &observe,
                       MissingChoices missing = MissingChoices::Reject,
                       const SimulationOptions &options = {});

    // Check a concrete candidate against this IR, without abstract observations.
    // All inputs and free state choices must be total. Initialized/successor
    // states may be omitted; if supplied, they are checked, not trusted.
    // Requires constraints through the last frame and bad in that last frame.
    // Incomplete/failed candidates do NOT prove an abstract trace spurious.
    VerificationResult Verify(const WLTrace &trace);

    // Extend a verified property-COI candidate with omitted free choices.
    // Derived states remain computed by Verify on full SourceIR. This extension
    // must itself be verified; failure is Unknown, never an array conflict.
    static void CompleteCoiChoices(const Btor2IR &source, const Btor2IR &property,
                                   WLTrace &trace);

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace car

#endif
