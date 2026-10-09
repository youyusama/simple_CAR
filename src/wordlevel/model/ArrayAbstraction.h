#ifndef ARRAY_ABSTRACTION_H
#define ARRAY_ABSTRACTION_H

#include "Btor2IR.h"
#include "WLTrace.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace car {

// Array abstraction and its refinement rules for one fixed PropertyIR, which
// must outlive this object and its BuildResults. Each BuildResult owns the
// abstract IR together with its matching precision and expression bindings;
// Builder and Analyzer keep temporary execution data within a single call.
class ArrayAbstraction {
    friend struct ArrayAbstractionTestAccess;
    class Builder;
    class Analyzer;
    struct Slot {
        int64_t arrayNodeId{0};
        size_t selectorIndex{0};
    };

  public:
    // Stable within this abstraction's append-only address registry; zero is invalid.
    using Address = size_t;
    struct TrackingTarget {
        int64_t group{0}; // canonical dependency-group representative
        Address address{0};
        unsigned delay{0};
        bool operator==(const TrackingTarget &other) const {
            return group == other.group && address == other.address && delay == other.delay;
        }
    };

    // Position is the stable selector identity. Slots and guards are derived
    // from these targets and the fixed model, never stored as parallel tables.
    struct Precision {
        std::vector<TrackingTarget> targets;
        unsigned MaxDelay() const;
    };
    struct AnalysisResult {
        enum class Kind { ConcreteCandidate, Refined, Unknown };
        Kind kind{Kind::Unknown};
        WLTrace trace;
        std::vector<TrackingTarget> targets;
        std::string reason;
        size_t readCorrections{0}, comparisonCorrections{0}, trials{0};
    };
    class BuildResult;

    explicit ArrayAbstraction(const Btor2IR &ir);
    ArrayAbstraction(Btor2IR &&) = delete;
    const Btor2IR &IR() const { return m_ir; }

    // Original scalar input/state ports retain their IDs in the abstract IR.
    BuildResult Build(const Precision &precision) const;
    // Corrections use fixed reference values and execute the actual abstract IR.
    // Refined supplies new legal targets; it does not prove trace exclusion or
    // commit precision. WLCEGAR publishes a replacement only after a full build.
    AnalysisResult AnalyzeCounterexample(const BuildResult &build,
        const WLTrace &abstractChoices);
    // Stage a validated, deduplicated batch without modifying the input value.
    // No additions returns nullopt. Existing duplicate targets retain identities.
    std::optional<Precision> ExtendPrecision(const BuildResult &build,
        const std::vector<TrackingTarget> &targets) const;
    unsigned MaxDelay(const BuildResult &build) const;
    size_t SelectorCount(const BuildResult &build) const;
    size_t SlotCount(const BuildResult &build) const;

    int64_t ArrayGroup(int64_t arrayNodeId) const;
    const std::vector<int64_t> &ArrayRoots(int64_t arrayNodeId) const;
    bool SameArrayGroup(int64_t lhs, int64_t rhs) const;
    // Declarations are append-only, so existing address and selector identities
    // remain valid when refinement adds an address in a later round.
    Address RegisterOriginalAddress(int64_t signedNodeId);
    Address RegisterConstantAddress(int64_t indexSort, const BitVector &value);
    Address OriginalAddress(int64_t signedNodeId) const;
    Address WitnessAddress(int64_t comparison) const;
    std::optional<int64_t> WitnessComparison(Address address) const;
    std::optional<BitVector> ConstantAddressValue(Address address) const;
    int64_t OriginalNode(Address address) const;
    int64_t AddressSort(Address address) const;
    std::pair<int, int64_t> AddressOrder(Address address) const;
    // Normalize array expressions to dependency groups and check address sorts.
    TrackingTarget MakeTarget(int64_t arrayNodeId, Address address, unsigned delay) const;
    void ValidateTarget(const TrackingTarget &target) const;

  private:
    struct ComparisonBinding {
        int64_t equal{0}, address{0}, left{0}, right{0};
    };
    const Precision &GetPrecision(const BuildResult &build) const;
    // Generated BV expression IDs, including real choice ports. Recording an
    // expression adds neither an input nor a constraint to the circuit.
    std::vector<int64_t> ObservationExpressions(const BuildResult &build) const;

    // Queries return BV references in this build's pre-resize abstract IR.
    // Reads and witness sides may be expressions or aliases, not primary ports.
    int64_t ReadExpression(const BuildResult &build, int64_t read) const;
    int64_t SelectorWord(const BuildResult &build, size_t selector) const;
    int64_t SlotWord(const BuildResult &build, int64_t arrayNodeId, size_t selector) const;
    ComparisonBinding ComparisonExpressions(const BuildResult &build, int64_t comparison) const;

    void ValidatePrecision(const Precision &precision) const;
    std::vector<Slot> Slots(const Precision &precision) const;
    struct OriginalSource { int64_t signedNodeId; };
    struct WitnessSource { int64_t comparisonNodeId; };
    struct ConstantSource { int64_t indexSort; BitVector value; };
    using AddressSource = std::variant<OriginalSource, WitnessSource, ConstantSource>;
    const AddressSource &Source(Address address) const;
    void CheckBuild(const BuildResult &build) const;
    const Btor2IR &m_ir;
    std::vector<AddressSource> m_addressSources{OriginalSource{0}};
    std::unordered_map<int64_t, Address> m_originalAddresses;
    std::unordered_map<int64_t, Address> m_witnessAddresses;
    std::map<std::pair<int64_t, std::string>, Address> m_constantAddresses;
    std::unordered_map<int64_t, int64_t> m_arrayGroups;
    std::unordered_map<int64_t, std::vector<int64_t>> m_arrayRoots;
};

// A complete build snapshot. Callers can inspect the IR but cannot separate or
// replace its bindings. Copies/moves retain the matching IR and metadata.
class ArrayAbstraction::BuildResult {
  public:
    BuildResult(const BuildResult &) = default;
    BuildResult(BuildResult &&) = default;
    BuildResult &operator=(const BuildResult &) = default;
    BuildResult &operator=(BuildResult &&) = default;
    const Btor2IR &IR() const { return ir; }

  private:
    friend class ArrayAbstraction;
    friend struct ArrayAbstractionTestAccess;
    BuildResult(const Btor2IR &source, Precision precision)
        : source(&source), precision(std::move(precision)) {}
    Btor2IR ir;
    const Btor2IR *source; // Non-owning identity of the fixed PropertyIR.
    Precision precision;
    // Semantic objects map to this build's pre-resize BV references, which may
    // be signed for inversion. Original scalar ports use their unchanged IDs.
    std::unordered_map<int64_t, int64_t> semanticReads;
    std::vector<int64_t> selectorBindings;
    std::map<std::pair<int64_t, size_t>, int64_t> slotValueBindings;
    std::unordered_map<int64_t, ComparisonBinding> comparisons;
};

} // namespace car
#endif
