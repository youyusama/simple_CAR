#ifndef WL_BITBLASTOR_H
#define WL_BITBLASTOR_H

#include "Btor2Frontend.h"
#include "WLTypes.h"

extern "C" {
#include "aiger.h"
}

#include <boolector/boolector.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace car {

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
                                     const WLIRTraceMap &traceSources,
                                     WLTraceMap &traceMap);

} // namespace car

#endif
