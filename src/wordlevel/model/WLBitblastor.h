#ifndef WL_BITBLASTOR_H
#define WL_BITBLASTOR_H

#include "Btor2IR.h"
#include "CarTypes.h"
#include "WLTrace.h"

extern "C" {
#include "aiger.h"
}

#include <boolector/boolector.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace car {

// A complete input/state word in the IR passed to GenerateWLAig. Bits are
// contiguous in LSB-first order after AIGER reencoding.
struct WLWordSpan {
    int64_t nodeId{0};
    uint32_t firstAigVar{0};
    uint32_t width{0};
};

struct WLWordLayout {
    // Physical AIG ports: a no-next IR state is represented by AIG inputs.
    std::vector<WLWordSpan> inputSpans;
    std::vector<WLWordSpan> latchSpans;
};

struct WLAigGate {
    uint64_t node{0};
    uint64_t child0{0};
    uint64_t child1{0};
};

// Shared Boolector lowering and AIG bitblasting service for word-level clients.
class WLBitblastor {
  public:
    using LeafResolver =
        std::function<BoolectorNode *(const Btor2IRNode &)>;

    class ScalarContext {
      public:
        ~ScalarContext();

        BoolectorNode *Lower(int64_t signedId);

      private:
        friend class WLBitblastor;
        class Impl;

        ScalarContext(WLBitblastor &bitblastor,
                      const Btor2IR &ir,
                      LeafResolver leafResolver);

        std::unique_ptr<Impl> m_impl;
    };

    explicit WLBitblastor(const Btor2IR &ir);
    ~WLBitblastor();

    WLBitblastor(const WLBitblastor &) = delete;
    WLBitblastor &operator=(const WLBitblastor &) = delete;

    Btor *BtorInstance() const;
    BoolectorSort Sort(int64_t sortId);
    BoolectorNode *Variable(int64_t sortId, const char *symbol);
    std::unique_ptr<ScalarContext>
    CreateScalarContext(LeafResolver leafResolver);

    // Return AIG literals in logical LSB-to-MSB order and collect their gates.
    std::vector<uint64_t> Bitblast(BoolectorNode *node);
    const std::vector<WLAigGate> &Gates() const;
    const char *Symbol(uint64_t literal) const;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

// Standard lowering of an optimized, array-free word-level IR to AIGER.
std::shared_ptr<aiger> GenerateWLAig(const Btor2IR &ir,
                                   WLWordLayout &traceMap);

// Reverse the port mapping of GenerateWLAig into this IR's execution choices.
// The IR, original AIG and layout must belong to the same bitblast build.
// Equivalences and trueId describe the checker's preprocessing of that AIG.
// Restore initial latches, fill absent free choices with zero, and omit derived
// successor states. No-next IR states remain per-frame choices, encoded as inputs.
// Optional AIG replay checks constraints/bad/pins; without it callers validate
// the recovered choices in the corresponding word-level IR. No SAT completion,
// resize restoration or live WLBitblastor/Boolector instance is needed here.
WLTrace RecoverWLCheckerChoices(
    const Btor2IR &ir, const aiger &aig,
    const std::unordered_map<Var, Lit> &equivalences, Var trueId,
    const WLWordLayout &layout,
    const std::vector<std::pair<Cube, Cube>> &partial, bool validateAig = true);

} // namespace car

#endif
