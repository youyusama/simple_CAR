#include "WLPackageResize.h"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace car {
namespace {

using boost::multiprecision::cpp_int;

bool IsConstant(Btor2Tag tag) {
    return tag == BTOR2_TAG_const || tag == BTOR2_TAG_constd ||
           tag == BTOR2_TAG_consth || tag == BTOR2_TAG_zero ||
           tag == BTOR2_TAG_one || tag == BTOR2_TAG_ones;
}

bool IsValueNode(Btor2Tag tag) {
    return tag != BTOR2_TAG_bad && tag != BTOR2_TAG_constraint &&
           tag != BTOR2_TAG_init && tag != BTOR2_TAG_next &&
           tag != BTOR2_TAG_output && tag != BTOR2_TAG_fair &&
           tag != BTOR2_TAG_justice;
}

bool IsUnaryBitblast(Btor2Tag tag) {
    switch (tag) {
    case BTOR2_TAG_dec:
    case BTOR2_TAG_inc:
    case BTOR2_TAG_neg:
    case BTOR2_TAG_not:
    case BTOR2_TAG_redand:
    case BTOR2_TAG_redor:
    case BTOR2_TAG_redxor: return true;
    default: return false;
    }
}

bool IsBinaryBitblast(Btor2Tag tag) {
    switch (tag) {
    case BTOR2_TAG_add:
    case BTOR2_TAG_and:
    case BTOR2_TAG_iff:
    case BTOR2_TAG_implies:
    case BTOR2_TAG_mul:
    case BTOR2_TAG_nand:
    case BTOR2_TAG_nor:
    case BTOR2_TAG_or:
    case BTOR2_TAG_rol:
    case BTOR2_TAG_ror:
    case BTOR2_TAG_saddo:
    case BTOR2_TAG_sdiv:
    case BTOR2_TAG_sdivo:
    case BTOR2_TAG_sgt:
    case BTOR2_TAG_sgte:
    case BTOR2_TAG_sll:
    case BTOR2_TAG_slt:
    case BTOR2_TAG_slte:
    case BTOR2_TAG_smod:
    case BTOR2_TAG_smulo:
    case BTOR2_TAG_sra:
    case BTOR2_TAG_srem:
    case BTOR2_TAG_srl:
    case BTOR2_TAG_ssubo:
    case BTOR2_TAG_sub:
    case BTOR2_TAG_uaddo:
    case BTOR2_TAG_udiv:
    case BTOR2_TAG_ugt:
    case BTOR2_TAG_ugte:
    case BTOR2_TAG_ult:
    case BTOR2_TAG_ulte:
    case BTOR2_TAG_umulo:
    case BTOR2_TAG_urem:
    case BTOR2_TAG_usubo:
    case BTOR2_TAG_xnor:
    case BTOR2_TAG_xor: return true;
    default: return false;
    }
}

bool IsUnsignedOrder(Btor2Tag tag) {
    return tag == BTOR2_TAG_ult || tag == BTOR2_TAG_ulte ||
           tag == BTOR2_TAG_ugt || tag == BTOR2_TAG_ugte;
}

uint32_t DataOperandCount(Btor2Tag tag) {
    switch (tag) {
    case BTOR2_TAG_const:
    case BTOR2_TAG_constd:
    case BTOR2_TAG_consth:
    case BTOR2_TAG_zero:
    case BTOR2_TAG_one:
    case BTOR2_TAG_ones:
    case BTOR2_TAG_input:
    case BTOR2_TAG_state: return 0;
    case BTOR2_TAG_slice:
    case BTOR2_TAG_uext:
    case BTOR2_TAG_sext:
    case BTOR2_TAG_bad:
    case BTOR2_TAG_constraint:
    case BTOR2_TAG_output:
    case BTOR2_TAG_fair: return 1;
    case BTOR2_TAG_ite: return 3;
    case BTOR2_TAG_init:
    case BTOR2_TAG_next: return 2;
    case BTOR2_TAG_justice:
        throw std::runtime_error(
            "word-level resizing does not support justice metadata");
    default: return IsUnaryBitblast(tag) ? 1 : 2;
    }
}

uint32_t CeilLog2(uint64_t value) {
    uint32_t result = 0;
    uint64_t capacity = 1;
    while (capacity < value) {
        if (capacity > std::numeric_limits<uint64_t>::max() / 2)
            return 64;
        capacity <<= 1;
        ++result;
    }
    return std::max<uint32_t>(1, result);
}

std::vector<bool> ConstantBits(const Btor2IR &ir,
                               const Btor2IRNode &node) {
    const uint32_t width = ir.Sort(node.sortId).width;
    cpp_int value = 0;
    if (node.tag == BTOR2_TAG_one) {
        value = 1;
    } else if (node.tag == BTOR2_TAG_ones) {
        value = (cpp_int(1) << width) - 1;
    } else if (node.tag == BTOR2_TAG_const) {
        for (char digit : node.constant) {
            value <<= 1;
            if (digit == '1') value += 1;
        }
    } else if (node.tag == BTOR2_TAG_consth) {
        for (char digit : node.constant) {
            value <<= 4;
            if (digit >= '0' && digit <= '9')
                value += digit - '0';
            else if (digit >= 'a' && digit <= 'f')
                value += digit - 'a' + 10;
            else if (digit >= 'A' && digit <= 'F')
                value += digit - 'A' + 10;
        }
    } else if (node.tag == BTOR2_TAG_constd) {
        value = cpp_int(node.constant);
    }
    const cpp_int mask = (cpp_int(1) << width) - 1;
    value &= mask;
    std::vector<bool> bits(width);
    for (uint32_t bit = 0; bit < width; ++bit)
        bits[bit] = static_cast<bool>((value >> bit) & 1);
    return bits;
}

// Shared constants must not merge otherwise independent word-level packages.
Btor2IR CloneConstantsPerUse(const Btor2IR &input) {
    Btor2IR output;
    for (const auto &[id, sort] : input.Sorts()) {
        (void)id;
        output.AddSort(sort);
    }
    int64_t nextId = INT64_C(1) << 55;
    for (const Btor2IRNode &node : input.Nodes()) {
        Btor2IRNode copy = node;
        const uint32_t operands = DataOperandCount(node.tag);
        for (uint32_t i = 0; i < operands; ++i) {
            const int64_t signedArg = copy.args[i];
            const Btor2IRNode &argument = input.Node(signedArg);
            int64_t argumentId = std::abs(signedArg);
            if (IsConstant(argument.tag) &&
                input.Sort(argument.sortId).width > 1) {
                Btor2IRNode proxy = argument;
                proxy.id = nextId++;
                proxy.symbol.clear();
                output.AddNode(proxy);
                argumentId = proxy.id;
            }
            // Turn BTOR2's signed-ID inversion into an ordinary bitwise node.
            if (signedArg < 0) {
                Btor2IRNode inversion;
                inversion.id = nextId++;
                inversion.tag = BTOR2_TAG_not;
                inversion.sortId = argument.sortId;
                inversion.nargs = 1;
                inversion.args[0] = argumentId;
                output.AddNode(inversion);
                argumentId = inversion.id;
            }
            copy.args[i] = argumentId;
        }
        output.AddNode(copy);
    }
    output.SetHasArrays(false);
    return output;
}

struct SegmentView {
    uint32_t lo{0};
    uint32_t hi{0};
    size_t classId{0};
};

class SegmentAnalyzer {
  public:
    static constexpr size_t kBitLevelClass =
        std::numeric_limits<size_t>::max();
    static constexpr size_t kInvalidMember =
        std::numeric_limits<size_t>::max();

    explicit SegmentAnalyzer(const Btor2IR &ir) : m_ir(ir) {
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            if (!node.sortId || !IsValueNode(node.tag)) continue;
            const Btor2IRSort &sort = m_ir.Sort(node.sortId);
            if (sort.tag != BTOR2_TAG_SORT_bitvec) continue;
            // Width-one nodes already are the bit-level partition and need no
            // interval/equivalence-class allocation.
            if (sort.width > 1) RegisterNode(node.id, sort.width);
        }
    }

    void Run() {
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            if (node.sortId && IsValueNode(node.tag) &&
                m_ir.Sort(node.sortId).tag == BTOR2_TAG_SORT_bitvec)
                Process(node.id);
        }

        // Negative references are implicit bitwise inversion and therefore bit-level.
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            const uint32_t operands = DataOperandCount(node.tag);
            for (uint32_t i = 0; i < operands; ++i) {
                if (node.args[i] < 0 && m_segments.count(-node.args[i]))
                    Bitblast(-node.args[i]);
            }
        }

        // Sequential boundaries use exactly the same segmentation and package encoding.
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            if (node.tag != BTOR2_TAG_init && node.tag != BTOR2_TAG_next)
                continue;
            if (!m_segments.count(std::abs(node.args[0]))) continue;
            Process(std::abs(node.args[0]));
            Process(std::abs(node.args[1]));
            MakeCompatible({std::abs(node.args[0]), std::abs(node.args[1])});
            UnionNodes({std::abs(node.args[0]), std::abs(node.args[1])});
            if (node.tag == BTOR2_TAG_next)
                m_next[node.args[0]] = node.args[1];
            else
                m_init[node.args[0]] = node.args[1];
        }

        // Freeze the union-find classes into the member vectors consumed by
        // package sizing; no class operation occurs after this point.
        FinalizeClasses();
    }

    std::vector<SegmentView> Segments(int64_t signedId) const {
        std::vector<SegmentView> result;
        auto found = m_segments.find(std::abs(signedId));
        if (found == m_segments.end()) {
            if (Width(signedId) != 1)
                throw std::runtime_error("missing word-level segmentation");
            return {{0, 1, kBitLevelClass}};
        }
        const auto &segments = found->second;
        for (const auto &[lo, segment] : segments) {
            (void)lo;
            SegmentView canonical = segment;
            canonical.classId = FindClass(canonical.classId);
            result.push_back(canonical);
        }
        return result;
    }

    SegmentView SegmentAt(int64_t signedId, uint32_t bit) const {
        auto found = m_segments.find(std::abs(signedId));
        if (found == m_segments.end()) {
            if (Width(signedId) == 1 && bit == 0)
                return {0, 1, kBitLevelClass};
            throw std::runtime_error("missing word-level segment lookup");
        }
        const auto &segments = found->second;
        auto it = segments.upper_bound(bit);
        if (it == segments.begin())
            throw std::runtime_error("invalid segment lookup");
        --it;
        if (bit >= it->second.hi)
            throw std::runtime_error("invalid segment boundary lookup");
        SegmentView canonical = it->second;
        canonical.classId = FindClass(canonical.classId);
        return canonical;
    }

    struct Member {
        int64_t nodeId{0};
        uint32_t lo{0};
        bool operator<(const Member &other) const {
            return std::tie(nodeId, lo) < std::tie(other.nodeId, other.lo);
        }
    };

    struct Class {
        bool active{true};
        size_t parent{0};
        size_t memberCount{0};
        uint32_t width{0};
        size_t memberHead{kInvalidMember};
        size_t memberTail{kInvalidMember};
        std::vector<Member> members;
    };

    const std::vector<Class> &Classes() const { return m_classes; }
    const std::vector<std::pair<int64_t, int64_t>> &Comparisons() const {
        return m_comparisons;
    }
    const std::unordered_map<int64_t, int64_t> &Next() const { return m_next; }
    const std::unordered_map<int64_t, int64_t> &Init() const { return m_init; }

  private:
    struct MemberRecord {
        Member member;
        size_t next{kInvalidMember};
    };

    size_t NewClass(uint32_t width) {
        const size_t id = m_classes.size();
        Class cls;
        cls.parent = id;
        cls.width = width;
        m_classes.push_back(std::move(cls));
        return id;
    }

    size_t NewMember(Member member) {
        const size_t id = m_memberRecords.size();
        m_memberRecords.push_back({member, kInvalidMember});
        return id;
    }

    void AppendMember(size_t classId, size_t memberId) {
        Class &cls = m_classes.at(classId);
        if (cls.memberTail == kInvalidMember) {
            cls.memberHead = memberId;
        } else {
            m_memberRecords.at(cls.memberTail).next = memberId;
        }
        cls.memberTail = memberId;
        m_memberRecords.at(memberId).next = kInvalidMember;
        ++cls.memberCount;
    }

    size_t FindClass(size_t id) {
        Class &cls = m_classes.at(id);
        if (cls.parent != id) cls.parent = FindClass(cls.parent);
        return cls.parent;
    }

    size_t FindClass(size_t id) const {
        while (m_classes.at(id).parent != id) id = m_classes.at(id).parent;
        return id;
    }

    void RegisterNode(int64_t id, uint32_t width) {
        const size_t classId = NewClass(width);
        AppendMember(classId, NewMember({id, 0}));
        m_segments[id].emplace(0, SegmentView{0, width, classId});
    }

    void Process(int64_t signedId) {
        const int64_t id = std::abs(signedId);
        if (!m_processed.insert(id).second) return;
        const Btor2IRNode &node = m_ir.Node(id);
        const uint32_t operands = DataOperandCount(node.tag);
        for (uint32_t i = 0; i < operands; ++i) {
            const Btor2IRNode &argument = m_ir.Node(node.args[i]);
            if (argument.sortId && IsValueNode(argument.tag))
                Process(node.args[i]);
        }

        if (IsConstant(node.tag)) {
            const std::vector<bool> bits = ConstantBits(m_ir, node);
            for (uint32_t bit = 1; bit < bits.size(); ++bit) {
                if (bits[bit] != bits[bit - 1]) Split(id, bit);
            }
            return;
        }
        switch (node.tag) {
        case BTOR2_TAG_input:
        case BTOR2_TAG_state: return;
        case BTOR2_TAG_eq:
        case BTOR2_TAG_neq:
            if (!m_segments.count(std::abs(node.args[0]))) return;
            MakeCompatible({std::abs(node.args[0]), std::abs(node.args[1])});
            UnionNodes({std::abs(node.args[0]), std::abs(node.args[1])});
            m_comparisons.emplace_back(node.args[0], node.args[1]);
            return;
        case BTOR2_TAG_ite:
            Bitblast(std::abs(node.args[0]));
            if (!m_segments.count(id)) return;
            MakeCompatible(
                {id, std::abs(node.args[1]), std::abs(node.args[2])});
            UnionNodes({id, std::abs(node.args[1]), std::abs(node.args[2])});
            return;
        case BTOR2_TAG_slice: ProcessSlice(node); return;
        case BTOR2_TAG_concat: ProcessConcat(node); return;
        case BTOR2_TAG_uext:
        case BTOR2_TAG_sext:
            // BTOR2 extension operators are lowered to exact bit-level constructs.
            Bitblast(id);
            Bitblast(std::abs(node.args[0]));
            return;
        default: break;
        }

        // Section 5 permits unsigned ordering against zero/one-run constants:
        // each segment comparison can be rewritten using equality only.
        if (IsUnsignedOrder(node.tag) &&
            (IsConstant(m_ir.Node(node.args[0]).tag) ||
             IsConstant(m_ir.Node(node.args[1]).tag))) {
            if (!m_segments.count(std::abs(node.args[0]))) return;
            MakeCompatible({std::abs(node.args[0]), std::abs(node.args[1])});
            UnionNodes({std::abs(node.args[0]), std::abs(node.args[1])});
            m_comparisons.emplace_back(node.args[0], node.args[1]);
            return;
        }

        if (IsUnaryBitblast(node.tag) || IsBinaryBitblast(node.tag)) {
            Bitblast(id);
            for (uint32_t i = 0; i < operands; ++i)
                Bitblast(std::abs(node.args[i]));
            // Preserve the paper's corresponding-bit equivalence where widths agree.
            std::vector<int64_t> compatible;
            if (m_segments.count(id)) compatible.push_back(id);
            const uint32_t width = Width(id);
            for (uint32_t i = 0; i < operands; ++i) {
                if (m_segments.count(std::abs(node.args[i])) &&
                    Width(node.args[i]) == width)
                    compatible.push_back(std::abs(node.args[i]));
            }
            if (compatible.size() > 1) UnionNodes(compatible);
            return;
        }
        throw std::runtime_error("unclassified word-level BTOR2 operator at id " +
                                 std::to_string(node.id));
    }

    uint32_t Width(int64_t id) const {
        return m_ir.Sort(m_ir.Node(id).sortId).width;
    }

    void ProcessSlice(const Btor2IRNode &node) {
        const int64_t source = std::abs(node.args[0]);
        const uint32_t upper = static_cast<uint32_t>(node.args[1]);
        const uint32_t lower = static_cast<uint32_t>(node.args[2]);
        Split(source, lower);
        Split(source, upper + 1);
        for (const SegmentView &segment : Segments(source)) {
            if (segment.lo > lower && segment.lo <= upper)
                Split(node.id, segment.lo - lower);
        }
        if (!m_segments.count(node.id)) return;
        for (const SegmentView &result : Segments(node.id)) {
            MergeClasses(result.classId,
                         SegmentAt(source, lower + result.lo).classId);
        }
    }

    void ProcessConcat(const Btor2IRNode &node) {
        const int64_t high = std::abs(node.args[0]);
        const int64_t low = std::abs(node.args[1]);
        const uint32_t lowWidth = Width(low);
        Split(node.id, lowWidth);
        for (const SegmentView &segment : Segments(low)) {
            if (segment.lo) Split(node.id, segment.lo);
        }
        for (const SegmentView &segment : Segments(high)) {
            if (segment.lo) Split(node.id, lowWidth + segment.lo);
        }
        for (const SegmentView &result : Segments(node.id)) {
            const SegmentView source = result.lo < lowWidth
                                           ? SegmentAt(low, result.lo)
                                           : SegmentAt(high, result.lo - lowWidth);
            if (source.classId != kBitLevelClass)
                MergeClasses(result.classId, source.classId);
        }
    }

    void MakeCompatible(const std::vector<int64_t> &nodes) {
        if (nodes.empty()) return;
        const uint32_t width = Width(nodes.front());
        for (int64_t id : nodes) {
            if (Width(id) != width)
                throw std::runtime_error("incompatible package widths");
        }
        // A split propagates by relative offset through its equivalence class and
        // can therefore introduce a new global cut in another operand. Iterate
        // until all operands have exactly the same global boundaries.
        while (true) {
            std::set<uint32_t> cuts;
            std::vector<std::set<uint32_t>> nodeCuts;
            for (int64_t id : nodes) {
                std::set<uint32_t> current;
                for (const SegmentView &segment : Segments(id)) {
                    if (segment.lo) {
                        cuts.insert(segment.lo);
                        current.insert(segment.lo);
                    }
                }
                nodeCuts.push_back(std::move(current));
            }
            const bool compatible = std::all_of(
                nodeCuts.begin(), nodeCuts.end(),
                [&](const std::set<uint32_t> &current) {
                    return current == cuts;
                });
            if (compatible) return;
            for (int64_t id : nodes)
                for (uint32_t cut : cuts) Split(id, cut);
        }
    }

    void UnionNodes(const std::vector<int64_t> &nodes) {
        if (nodes.size() < 2) return;
        const std::vector<SegmentView> base = Segments(nodes.front());
        for (size_t index = 1; index < nodes.size(); ++index) {
            const std::vector<SegmentView> other = Segments(nodes[index]);
            if (base.size() != other.size())
                throw std::runtime_error("segment union requires compatible nodes");
            for (size_t segment = 0; segment < base.size(); ++segment) {
                // Earlier merges may retain the other class and invalidate snapshots.
                MergeClasses(SegmentAt(nodes.front(), base[segment].lo).classId,
                             SegmentAt(nodes[index], other[segment].lo).classId);
            }
        }
    }

    void Bitblast(int64_t id) {
        if (!m_segments.count(std::abs(id))) return;
        const uint32_t width = Width(id);
        for (uint32_t bit = 1; bit < width; ++bit) Split(id, bit);
    }

    void Split(int64_t signedId, uint32_t cut) {
        const int64_t id = std::abs(signedId);
        if (cut == 0 || cut >= Width(id)) return;
        auto &nodeSegments = m_segments.at(id);
        auto it = nodeSegments.upper_bound(cut);
        if (it == nodeSegments.begin()) return;
        --it;
        if (it->second.lo == cut || cut >= it->second.hi) return;

        const size_t oldId = FindClass(it->second.classId);
        Class &old = m_classes.at(oldId);
        if (!old.active) throw std::runtime_error("split inactive segment class");
        const uint32_t offset = cut - it->second.lo;
        const uint32_t oldWidth = old.width;
        size_t memberId = old.memberHead;
        old.active = false;
        old.memberHead = kInvalidMember;
        old.memberTail = kInvalidMember;
        old.memberCount = 0;

        const size_t lowId = NewClass(offset);
        const size_t highId = NewClass(oldWidth - offset);

        // A union-find set cannot be split directly. Rebind every segment in
        // the old root while reusing its member record for the low half.
        while (memberId != kInvalidMember) {
            const Member member = m_memberRecords.at(memberId).member;
            const size_t nextMember = m_memberRecords.at(memberId).next;
            auto &segments = m_segments.at(member.nodeId);
            auto current = segments.find(member.lo);
            if (current == segments.end() ||
                FindClass(current->second.classId) != oldId)
                throw std::runtime_error("corrupt segment equivalence class");
            const uint32_t hi = current->second.hi;
            segments.erase(current);
            segments.emplace(member.lo,
                             SegmentView{member.lo,
                                         member.lo + offset,
                                         lowId});
            segments.emplace(member.lo + offset,
                             SegmentView{member.lo + offset, hi, highId});
            AppendMember(lowId, memberId);
            AppendMember(
                highId,
                NewMember({member.nodeId, member.lo + offset}));
            memberId = nextMember;
        }
    }

    void MergeClasses(size_t lhsId, size_t rhsId) {
        lhsId = FindClass(lhsId);
        rhsId = FindClass(rhsId);
        if (lhsId == rhsId) return;
        if (m_classes[lhsId].width != m_classes[rhsId].width)
            throw std::runtime_error("cannot merge different segment widths");
        if (!m_classes[lhsId].active || !m_classes[rhsId].active)
            throw std::runtime_error("cannot merge inactive segment class");
        if (m_classes[lhsId].memberCount < m_classes[rhsId].memberCount)
            std::swap(lhsId, rhsId);
        Class &lhs = m_classes[lhsId];
        Class &rhs = m_classes[rhsId];

        // Union by size bounds tree depth; the intrusive member chains make
        // the corresponding equivalence-class merge constant time.
        rhs.parent = lhsId;
        rhs.active = false;
        if (rhs.memberHead != kInvalidMember) {
            if (lhs.memberTail == kInvalidMember)
                lhs.memberHead = rhs.memberHead;
            else
                m_memberRecords.at(lhs.memberTail).next = rhs.memberHead;
            lhs.memberTail = rhs.memberTail;
        }
        lhs.memberCount += rhs.memberCount;
        rhs.memberHead = kInvalidMember;
        rhs.memberTail = kInvalidMember;
        rhs.memberCount = 0;
    }

    void FinalizeClasses() {
        for (size_t id = 0; id < m_classes.size(); ++id) {
            Class &cls = m_classes[id];
            cls.members.clear();
            if (!cls.active) continue;
            if (FindClass(id) != id)
                throw std::runtime_error("active segment class is not a root");
            cls.members.reserve(cls.memberCount);

            size_t memberId = cls.memberHead;
            while (memberId != kInvalidMember) {
                const MemberRecord &record = m_memberRecords.at(memberId);
                const auto node = m_segments.find(record.member.nodeId);
                if (node == m_segments.end())
                    throw std::runtime_error("missing segment class member");
                const auto segment = node->second.find(record.member.lo);
                if (segment == node->second.end() ||
                    FindClass(segment->second.classId) != id ||
                    segment->second.hi - segment->second.lo != cls.width)
                    throw std::runtime_error("corrupt finalized segment class");
                cls.members.push_back(record.member);
                memberId = record.next;
            }
            if (cls.members.size() != cls.memberCount)
                throw std::runtime_error("segment class member count mismatch");
            cls.memberHead = kInvalidMember;
            cls.memberTail = kInvalidMember;
        }

        // The analysis is immutable now, so retain only the compact vectors
        // used by PackageSizer and release the transient linked-member pool.
        std::vector<MemberRecord>().swap(m_memberRecords);
    }

    const Btor2IR &m_ir;
    std::unordered_map<int64_t, std::map<uint32_t, SegmentView>> m_segments;
    std::vector<Class> m_classes;
    std::vector<MemberRecord> m_memberRecords;
    std::unordered_set<int64_t> m_processed;
    std::vector<std::pair<int64_t, int64_t>> m_comparisons;
    std::unordered_map<int64_t, int64_t> m_next;
    std::unordered_map<int64_t, int64_t> m_init;
};

struct SourceVertex {
    enum class Kind { Input, State, Constant } kind{Kind::Input};
    int64_t nodeId{0};
    uint32_t lo{0};
    int constantValue{-1};

    bool operator<(const SourceVertex &other) const {
        return std::tie(kind, nodeId, lo, constantValue) <
               std::tie(other.kind, other.nodeId, other.lo,
                        other.constantValue);
    }
    bool operator==(const SourceVertex &other) const {
        return kind == other.kind && nodeId == other.nodeId &&
               lo == other.lo && constantValue == other.constantValue;
    }
};

struct SegmentKey {
    int64_t nodeId{0};
    uint32_t lo{0};
    bool operator<(const SegmentKey &other) const {
        return std::tie(nodeId, lo) < std::tie(other.nodeId, other.lo);
    }
};

class PackageSizer {
  public:
    PackageSizer(const Btor2IR &ir, const SegmentAnalyzer &analysis)
        : m_ir(ir), m_analysis(analysis) {}

    std::unordered_map<size_t, uint32_t> Run() {
        BuildFeederGraph();
        ComputeFeeders();
        BuildComparisonGraphs();

        std::unordered_map<size_t, uint32_t> widths;
        const auto &classes = m_analysis.Classes();
        for (size_t id = 0; id < classes.size(); ++id) {
            const auto &cls = classes[id];
            if (!cls.active) continue;
            if (cls.width == 1) {
                widths[id] = 1;
                continue;
            }

            uint64_t variables = 0;
            for (const auto &member : cls.members) {
                const Btor2Tag tag = m_ir.Node(member.nodeId).tag;
                if (tag == BTOR2_TAG_input || tag == BTOR2_TAG_state)
                    ++variables;
            }
            const uint32_t general =
                std::min(cls.width, CeilLog2(variables + 2));
            uint32_t target = general;
            auto graph = m_graphs.find(id);
            if (graph != m_graphs.end() && graph->second.simple &&
                !graph->second.edges.empty()) {
                target = std::min(target,
                                  std::min(cls.width,
                                           CeilLog2(Color(graph->second))));
            }
            widths[id] = target;
        }
        return widths;
    }

  private:
    struct ComparisonGraph {
        bool simple{true};
        std::set<SourceVertex> vertices;
        std::set<std::pair<SourceVertex, SourceVertex>> edges;
    };

    static SegmentKey Key(const SegmentView &segment, int64_t nodeId) {
        return {std::abs(nodeId), segment.lo};
    }

    void AddEdge(const SegmentKey &from, const SegmentKey &to) {
        m_edges[from].insert(to);
    }

    void BuildFeederGraph() {
        for (const Btor2IRNode &node : m_ir.Nodes()) {
            if (!node.sortId || !IsValueNode(node.tag) ||
                m_ir.Sort(node.sortId).tag != BTOR2_TAG_SORT_bitvec)
                continue;
            for (const SegmentView &segment : m_analysis.Segments(node.id)) {
                if (segment.hi - segment.lo == 1) continue;
                SegmentKey from{node.id, segment.lo};
                switch (node.tag) {
                case BTOR2_TAG_input:
                    m_seeds[from].insert(
                        {SourceVertex::Kind::Input, node.id, segment.lo, -1});
                    break;
                case BTOR2_TAG_state: {
                    m_seeds[from].insert(
                        {SourceVertex::Kind::State, node.id, segment.lo, -1});
                    auto next = m_analysis.Next().find(node.id);
                    if (next != m_analysis.Next().end()) {
                        SegmentView feeder =
                            m_analysis.SegmentAt(next->second, segment.lo);
                        AddEdge(from, Key(feeder, next->second));
                    }
                    break;
                }
                default:
                    if (IsConstant(node.tag)) {
                        const std::vector<bool> bits = ConstantBits(m_ir, node);
                        const int value = bits[segment.lo] ? 1 : 0;
                        // Equal constants share one comparison-graph vertex.
                        m_seeds[from].insert(
                            {SourceVertex::Kind::Constant, 0, 0, value});
                    } else if (node.tag == BTOR2_TAG_ite) {
                        AddEdge(from,
                                Key(m_analysis.SegmentAt(node.args[1], segment.lo),
                                    node.args[1]));
                        AddEdge(from,
                                Key(m_analysis.SegmentAt(node.args[2], segment.lo),
                                    node.args[2]));
                    } else if (node.tag == BTOR2_TAG_slice) {
                        const uint32_t lower =
                            static_cast<uint32_t>(node.args[2]);
                        AddEdge(from,
                                Key(m_analysis.SegmentAt(
                                        node.args[0], lower + segment.lo),
                                    node.args[0]));
                    } else if (node.tag == BTOR2_TAG_concat) {
                        const uint32_t lowWidth =
                            m_ir.Sort(m_ir.Node(node.args[1]).sortId).width;
                        const int64_t operand = segment.lo < lowWidth
                                                    ? node.args[1]
                                                    : node.args[0];
                        const uint32_t bit = segment.lo < lowWidth
                                                 ? segment.lo
                                                 : segment.lo - lowWidth;
                        AddEdge(from,
                                Key(m_analysis.SegmentAt(operand, bit), operand));
                    }
                    break;
                }
            }
        }
    }

    void ComputeFeeders() {
        for (const auto &[key, seed] : m_seeds) m_feeders[key] = seed;
        bool changed = true;
        while (changed) {
            changed = false;
            for (const auto &[from, targets] : m_edges) {
                auto &feeders = m_feeders[from];
                for (const SegmentKey &target : targets) {
                    const auto &source = m_feeders[target];
                    const size_t before = feeders.size();
                    feeders.insert(source.begin(), source.end());
                    changed = changed || feeders.size() != before;
                }
            }
        }
    }

    std::set<SourceVertex> ReplaceInitializedStates(
        const std::set<SourceVertex> &sources) const {
        std::set<SourceVertex> result;
        for (const SourceVertex &source : sources) {
            if (source.kind != SourceVertex::Kind::State) {
                result.insert(source);
                continue;
            }
            auto init = m_analysis.Init().find(source.nodeId);
            if (init == m_analysis.Init().end()) {
                result.insert(source);
                continue;
            }
            const SegmentView initSegment =
                m_analysis.SegmentAt(init->second, source.lo);
            auto feeders = m_feeders.find(Key(initSegment, init->second));
            if (feeders != m_feeders.end() && feeders->second.size() == 1 &&
                feeders->second.begin()->kind == SourceVertex::Kind::Constant)
                result.insert(*feeders->second.begin());
            else
                result.insert(source);
        }
        return result;
    }

    void BuildComparisonGraphs() {
        for (const auto &[lhsId, rhsId] : m_analysis.Comparisons()) {
            const auto lhsSegments = m_analysis.Segments(lhsId);
            for (const SegmentView &lhs : lhsSegments) {
                if (lhs.hi - lhs.lo == 1) continue;
                const SegmentView rhs = m_analysis.SegmentAt(rhsId, lhs.lo);
                auto lhsRaw = m_feeders[Key(lhs, lhsId)];
                auto rhsRaw = m_feeders[Key(rhs, rhsId)];
                auto onlyStateOrConstant = [](const auto &set) {
                    return std::all_of(
                        set.begin(), set.end(), [](const SourceVertex &source) {
                            return source.kind != SourceVertex::Kind::Input;
                        });
                };
                ComparisonGraph &graph = m_graphs[lhs.classId];
                graph.simple = graph.simple &&
                               (onlyStateOrConstant(lhsRaw) ||
                                onlyStateOrConstant(rhsRaw));
                const auto left = ReplaceInitializedStates(lhsRaw);
                const auto right = ReplaceInitializedStates(rhsRaw);
                graph.vertices.insert(left.begin(), left.end());
                graph.vertices.insert(right.begin(), right.end());
                for (const SourceVertex &a : left) {
                    for (const SourceVertex &b : right) {
                        if (a == b) continue;
                        graph.edges.insert(a < b ? std::make_pair(a, b)
                                                 : std::make_pair(b, a));
                    }
                }
            }
        }
    }

    static uint64_t Color(const ComparisonGraph &graph) {
        std::vector<SourceVertex> vertices(graph.vertices.begin(),
                                           graph.vertices.end());
        std::map<SourceVertex, size_t> index;
        for (size_t i = 0; i < vertices.size(); ++i) index[vertices[i]] = i;
        std::vector<std::set<size_t>> adjacent(vertices.size());
        for (const auto &[lhs, rhs] : graph.edges) {
            const size_t a = index.at(lhs);
            const size_t b = index.at(rhs);
            adjacent[a].insert(b);
            adjacent[b].insert(a);
        }

        const int uncolored = -1;
        std::vector<int> colors(vertices.size(), uncolored);
        std::map<int, int> constantColors;
        int nextConstantColor = 0;
        for (size_t i = 0; i < vertices.size(); ++i) {
            if (vertices[i].kind != SourceVertex::Kind::Constant) continue;
            auto [it, inserted] = constantColors.emplace(
                vertices[i].constantValue, nextConstantColor);
            if (inserted) ++nextConstantColor;
            colors[i] = it->second;
        }

        while (std::find(colors.begin(), colors.end(), uncolored) !=
               colors.end()) {
            size_t selected = vertices.size();
            size_t bestSaturation = 0;
            size_t bestDegree = 0;
            for (size_t i = 0; i < vertices.size(); ++i) {
                if (colors[i] != uncolored) continue;
                std::set<int> neighborColors;
                for (size_t neighbor : adjacent[i]) {
                    if (colors[neighbor] != uncolored)
                        neighborColors.insert(colors[neighbor]);
                }
                if (selected == vertices.size() ||
                    neighborColors.size() > bestSaturation ||
                    (neighborColors.size() == bestSaturation &&
                     adjacent[i].size() > bestDegree)) {
                    selected = i;
                    bestSaturation = neighborColors.size();
                    bestDegree = adjacent[i].size();
                }
            }
            std::set<int> forbidden;
            for (size_t neighbor : adjacent[selected]) {
                if (colors[neighbor] != uncolored)
                    forbidden.insert(colors[neighbor]);
            }
            int color = 0;
            while (forbidden.count(color)) ++color;
            colors[selected] = color;
        }
        int maxColor = colors.empty() ? 0 :
            *std::max_element(colors.begin(), colors.end());
        return static_cast<uint64_t>(maxColor + 1);
    }

    const Btor2IR &m_ir;
    const SegmentAnalyzer &m_analysis;
    std::map<SegmentKey, std::set<SegmentKey>> m_edges;
    std::map<SegmentKey, std::set<SourceVertex>> m_seeds;
    std::map<SegmentKey, std::set<SourceVertex>> m_feeders;
    std::unordered_map<size_t, ComparisonGraph> m_graphs;
};

class SegmentIRRewriter {
  public:
    SegmentIRRewriter(const Btor2IR &input,
                      const SegmentAnalyzer &analysis,
                      const std::unordered_map<size_t, uint32_t> &widths,
                      const WLIRTraceMap &traceSources)
        : m_input(input),
          m_analysis(analysis),
          m_widths(widths),
          m_inputTraceSources(traceSources) {}

    std::pair<Btor2IR, WLIRTraceMap> Run() {
        // Rewrite in source topological order so very deep bit-level cones do not
        // consume the C++ call stack. Constant proxies are inserted before use.
        for (const Btor2IRNode &node : m_input.Nodes()) {
            if (node.sortId && IsValueNode(node.tag) &&
                m_input.Sort(node.sortId).tag == BTOR2_TAG_SORT_bitvec)
                Build(node.id);
        }
        for (const Btor2IRNode &node : m_input.Nodes()) {
            switch (node.tag) {
            case BTOR2_TAG_init:
            case BTOR2_TAG_next: RewriteTransition(node); break;
            case BTOR2_TAG_bad:
            case BTOR2_TAG_constraint:
            case BTOR2_TAG_fair: RewriteMetadata(node); break;
            case BTOR2_TAG_output:
                // BTOR2 outputs are observability metadata and are not consumed
                // by the safety transition system emitted by this backend.
                break;
            case BTOR2_TAG_justice:
                throw std::runtime_error(
                    "word-level resizing does not support justice metadata");
            default: break;
            }
        }
        m_output.SetHasArrays(false);
        return {std::move(m_output), std::move(m_outputTraceSources)};
    }

  private:
    struct Piece {
        uint32_t lo{0};
        uint32_t hi{0};
        uint32_t width{0};
        int64_t nodeId{0};
        size_t classId{0};
    };

    int64_t EnsureSort(uint32_t width) {
        auto found = m_sorts.find(width);
        if (found != m_sorts.end()) return found->second;
        const int64_t id = m_nextSortId++;
        m_output.AddSort({id, BTOR2_TAG_SORT_bitvec, width, 0, 0});
        m_sorts[width] = id;
        return id;
    }

    int64_t AddNode(Btor2Tag tag,
                    uint32_t width,
                    std::array<int64_t, 3> args = {},
                    uint32_t nargs = 0,
                    std::string symbol = {},
                    std::string constant = {}) {
        Btor2IRNode node;
        node.id = m_nextNodeId++;
        node.tag = tag;
        node.sortId = EnsureSort(width);
        node.nargs = nargs;
        node.args = args;
        node.symbol = std::move(symbol);
        node.constant = std::move(constant);
        m_output.AddNode(node);
        return node.id;
    }

    uint32_t TargetWidth(const SegmentView &segment) const {
        if (segment.classId == SegmentAnalyzer::kBitLevelClass) return 1;
        return m_widths.at(segment.classId);
    }

    const std::vector<Piece> &Build(int64_t signedId) {
        const int64_t id = std::abs(signedId);
        auto cached = m_pieces.find(id);
        if (cached != m_pieces.end()) return cached->second;
        const Btor2IRNode &node = m_input.Node(id);
        std::vector<Piece> pieces;
        if (node.tag == BTOR2_TAG_input || node.tag == BTOR2_TAG_state) {
            pieces = BuildVariable(node);
        } else if (IsConstant(node.tag)) {
            pieces = BuildConstant(node);
        } else if (node.tag == BTOR2_TAG_eq || node.tag == BTOR2_TAG_neq) {
            pieces = BuildEquality(node);
        } else if (IsUnsignedOrder(node.tag) &&
                   (IsConstant(m_input.Node(node.args[0]).tag) ||
                    IsConstant(m_input.Node(node.args[1]).tag))) {
            pieces = BuildConstantOrder(node);
        } else if (node.tag == BTOR2_TAG_ite) {
            pieces = BuildIte(node);
        } else if (node.tag == BTOR2_TAG_slice) {
            pieces = BuildSlice(node);
        } else if (node.tag == BTOR2_TAG_concat) {
            pieces = BuildConcat(node);
        } else {
            pieces = BuildBitblasted(node);
        }
        auto [it, inserted] = m_pieces.emplace(id, std::move(pieces));
        if (!inserted) throw std::runtime_error("duplicate segment rewrite");
        return it->second;
    }

    std::vector<Piece> BuildVariable(const Btor2IRNode &node) {
        std::vector<Piece> result;
        const auto trace = m_inputTraceSources.find(node.id);
        for (const SegmentView &segment : m_analysis.Segments(node.id)) {
            const uint32_t originalWidth = segment.hi - segment.lo;
            const uint32_t width = TargetWidth(segment);
            const std::string symbol =
                node.symbol.empty()
                    ? std::string{}
                    : node.symbol + ".seg." + std::to_string(segment.lo) +
                          "." + std::to_string(segment.hi - 1);
            const int64_t id = AddNode(
                node.tag, width, {}, 0, symbol);
            result.push_back(
                {segment.lo, segment.hi, width, id, segment.classId});
            if (trace != m_inputTraceSources.end()) {
                WLIRTraceSource source = trace->second;
                source.originalBitOffset += segment.lo;
                source.originalSegmentWidth = originalWidth;
                source.resized = width < originalWidth;
                m_outputTraceSources[id] = source;
            }
        }
        return result;
    }

    std::vector<Piece> BuildConstant(const Btor2IRNode &node) {
        const std::vector<bool> bits = ConstantBits(m_input, node);
        std::vector<Piece> result;
        for (const SegmentView &segment : m_analysis.Segments(node.id)) {
            const uint32_t width = TargetWidth(segment);
            const Btor2Tag tag = bits[segment.lo] ? BTOR2_TAG_ones
                                                  : BTOR2_TAG_zero;
            const int64_t id = AddNode(tag, width);
            result.push_back(
                {segment.lo, segment.hi, width, id, segment.classId});
        }
        return result;
    }

    const Piece &PieceAt(const std::vector<Piece> &pieces,
                         uint32_t bit) const {
        auto it = std::find_if(pieces.begin(), pieces.end(), [&](const Piece &p) {
            return p.lo <= bit && bit < p.hi;
        });
        if (it == pieces.end()) throw std::runtime_error("missing rewritten segment");
        return *it;
    }

    std::vector<Piece> BuildEquality(const Btor2IRNode &node) {
        const auto &lhs = Build(node.args[0]);
        const auto &rhs = Build(node.args[1]);
        std::vector<int64_t> comparisons;
        for (const Piece &left : lhs) {
            const Piece &right = PieceAt(rhs, left.lo);
            if (left.lo != right.lo || left.hi != right.hi ||
                left.width != right.width)
                throw std::runtime_error("incompatible equality segments");
            comparisons.push_back(AddNode(
                node.tag, 1, {left.nodeId, right.nodeId, 0}, 2));
        }
        int64_t result = comparisons.front();
        const Btor2Tag combine = node.tag == BTOR2_TAG_eq ? BTOR2_TAG_and
                                                          : BTOR2_TAG_or;
        for (size_t i = 1; i < comparisons.size(); ++i)
            result = AddNode(combine, 1, {result, comparisons[i], 0}, 2);
        const SegmentView segment = m_analysis.Segments(node.id).front();
        return {{0, 1, 1, result, segment.classId}};
    }

    int64_t BoolConstant(bool value) {
        return AddNode(value ? BTOR2_TAG_one : BTOR2_TAG_zero, 1);
    }

    int64_t BoolAnd(int64_t lhs, int64_t rhs) {
        return AddNode(BTOR2_TAG_and, 1, {lhs, rhs, 0}, 2);
    }

    int64_t BoolOr(int64_t lhs, int64_t rhs) {
        return AddNode(BTOR2_TAG_or, 1, {lhs, rhs, 0}, 2);
    }

    std::vector<Piece> BuildConstantOrder(const Btor2IRNode &node) {
        bool constantOnLeft = IsConstant(m_input.Node(node.args[0]).tag);
        const int64_t valueId = constantOnLeft ? node.args[1] : node.args[0];
        const int64_t constantId = constantOnLeft ? node.args[0] : node.args[1];
        const auto &values = Build(valueId);
        const auto &constants = Build(constantId);
        const std::vector<bool> constantBits =
            ConstantBits(m_input, m_input.Node(constantId));

        bool less = node.tag == BTOR2_TAG_ult || node.tag == BTOR2_TAG_ulte;
        const bool inclusive =
            node.tag == BTOR2_TAG_ulte || node.tag == BTOR2_TAG_ugte;
        if (constantOnLeft) less = !less;

        int64_t prefixEqual = BoolConstant(true);
        int64_t result = BoolConstant(false);
        for (size_t index = values.size(); index-- > 0;) {
            const Piece &value = values[index];
            const Piece &constant = PieceAt(constants, value.lo);
            if (value.lo != constant.lo || value.hi != constant.hi ||
                value.width != constant.width)
                throw std::runtime_error(
                    "incompatible constant-order segments");
            const int64_t equal = AddNode(
                BTOR2_TAG_eq, 1, {value.nodeId, constant.nodeId, 0}, 2);
            const bool constantOne = constantBits[value.lo];
            int64_t strict = BoolConstant(false);
            if ((less && constantOne) || (!less && !constantOne)) {
                strict = AddNode(BTOR2_TAG_neq,
                                 1,
                                 {value.nodeId, constant.nodeId, 0},
                                 2);
            }
            result = BoolOr(result, BoolAnd(prefixEqual, strict));
            prefixEqual = BoolAnd(prefixEqual, equal);
        }
        if (inclusive) result = BoolOr(result, prefixEqual);
        const SegmentView segment = m_analysis.Segments(node.id).front();
        return {{0, 1, 1, result, segment.classId}};
    }

    std::vector<Piece> BuildIte(const Btor2IRNode &node) {
        const int64_t selector = Whole(node.args[0]);
        const auto &onTrue = Build(node.args[1]);
        const auto &onFalse = Build(node.args[2]);
        std::vector<Piece> result;
        for (const SegmentView &segment : m_analysis.Segments(node.id)) {
            const Piece &lhs = PieceAt(onTrue, segment.lo);
            const Piece &rhs = PieceAt(onFalse, segment.lo);
            const uint32_t width = TargetWidth(segment);
            if (lhs.lo != segment.lo || rhs.lo != segment.lo ||
                lhs.width != width || rhs.width != width)
                throw std::runtime_error("incompatible ITE segments");
            const int64_t id = AddNode(
                BTOR2_TAG_ite, width, {selector, lhs.nodeId, rhs.nodeId}, 3);
            result.push_back(
                {segment.lo, segment.hi, width, id, segment.classId});
        }
        return result;
    }

    std::vector<Piece> BuildSlice(const Btor2IRNode &node) {
        const auto &source = Build(node.args[0]);
        const uint32_t lower = static_cast<uint32_t>(node.args[2]);
        std::vector<Piece> result;
        for (const SegmentView &segment : m_analysis.Segments(node.id)) {
            const Piece &input = PieceAt(source, lower + segment.lo);
            result.push_back({segment.lo,
                              segment.hi,
                              input.width,
                              input.nodeId,
                              segment.classId});
        }
        return result;
    }

    std::vector<Piece> BuildConcat(const Btor2IRNode &node) {
        const auto &high = Build(node.args[0]);
        const auto &low = Build(node.args[1]);
        const uint32_t lowWidth =
            m_input.Sort(m_input.Node(node.args[1]).sortId).width;
        std::vector<Piece> result;
        for (const SegmentView &segment : m_analysis.Segments(node.id)) {
            const Piece &input = segment.lo < lowWidth
                                     ? PieceAt(low, segment.lo)
                                     : PieceAt(high, segment.lo - lowWidth);
            result.push_back({segment.lo,
                              segment.hi,
                              input.width,
                              input.nodeId,
                              segment.classId});
        }
        return result;
    }

    std::vector<Piece> BuildBitblasted(const Btor2IRNode &node) {
        const uint32_t resultWidth = m_input.Sort(node.sortId).width;
        std::array<int64_t, 3> args = node.args;
        const uint32_t operands = DataOperandCount(node.tag);
        for (uint32_t i = 0; i < operands; ++i) args[i] = Whole(node.args[i]);
        const int64_t full = AddNode(node.tag, resultWidth, args, node.nargs);
        std::vector<Piece> result;
        for (const SegmentView &segment : m_analysis.Segments(node.id)) {
            if (segment.hi - segment.lo != 1 ||
                TargetWidth(segment) != 1)
                throw std::runtime_error(
                    "bitblasted operator retained a word-level segment");
            int64_t bit = full;
            if (resultWidth != 1) {
                bit = AddNode(BTOR2_TAG_slice,
                              1,
                              {full,
                               static_cast<int64_t>(segment.lo),
                               static_cast<int64_t>(segment.lo)},
                              3);
            }
            result.push_back(
                {segment.lo, segment.hi, 1, bit, segment.classId});
        }
        return result;
    }

    int64_t Whole(int64_t signedId) {
        auto cached = m_whole.find(signedId);
        if (cached != m_whole.end()) return cached->second;
        const auto &pieces = Build(std::abs(signedId));
        for (const Piece &piece : pieces) {
            if (piece.width != piece.hi - piece.lo)
                throw std::runtime_error(
                    "width-sensitive operator consumes a resized package");
        }
        int64_t result = pieces.back().nodeId;
        uint32_t width = pieces.back().width;
        for (size_t index = pieces.size() - 1; index-- > 0;) {
            result = AddNode(BTOR2_TAG_concat,
                             width + pieces[index].width,
                             {result, pieces[index].nodeId, 0},
                             2);
            width += pieces[index].width;
        }
        if (signedId < 0)
            result = AddNode(BTOR2_TAG_not, width, {result, 0, 0}, 1);
        m_whole[signedId] = result;
        return result;
    }

    void RewriteTransition(const Btor2IRNode &node) {
        const auto &states = Build(node.args[0]);
        const auto &values = Build(node.args[1]);
        for (const Piece &state : states) {
            const Piece &value = PieceAt(values, state.lo);
            if (state.lo != value.lo || state.hi != value.hi ||
                state.width != value.width)
                throw std::runtime_error("incompatible transition segments");
            AddNode(node.tag,
                    state.width,
                    {state.nodeId, value.nodeId, 0},
                    2);
        }
    }

    void RewriteMetadata(const Btor2IRNode &node) {
        const int64_t expression = Whole(node.args[0]);
        const uint32_t width = m_input.Sort(m_input.Node(node.args[0]).sortId).width;
        AddNode(node.tag, width, {expression, 0, 0}, 1, node.symbol);
    }

    const Btor2IR &m_input;
    const SegmentAnalyzer &m_analysis;
    const std::unordered_map<size_t, uint32_t> &m_widths;
    const WLIRTraceMap &m_inputTraceSources;
    Btor2IR m_output;
    WLIRTraceMap m_outputTraceSources;
    int64_t m_nextSortId{INT64_C(1) << 53};
    int64_t m_nextNodeId{INT64_C(1) << 54};
    std::unordered_map<uint32_t, int64_t> m_sorts;
    std::unordered_map<int64_t, std::vector<Piece>> m_pieces;
    std::unordered_map<int64_t, int64_t> m_whole;
};

} // namespace

void WLPackageResize::Run(Btor2IR &ir, WLIRTraceMap &traceSources) {
    if (ir.HasArrays()) {
        throw std::runtime_error(
            "segment-level word reduction requires array-free IR");
    }

    // Per-use constants avoid accidental package merging through DAG sharing.
    ir = CloneConstantsPerUse(ir);
    SegmentAnalyzer analyzer(ir);
    analyzer.Run();

    PackageSizer sizer(ir, analyzer);
    const auto widths = sizer.Run();

    SegmentIRRewriter rewriter(ir, analyzer, widths, traceSources);
    auto rewritten = rewriter.Run();
    ir = std::move(rewritten.first);
    traceSources = std::move(rewritten.second);
}

} // namespace car
