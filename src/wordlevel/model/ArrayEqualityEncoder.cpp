#include "ArrayEqualityEncoder.h"

#include <algorithm>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <unordered_set>

namespace car {
namespace {
// Only compute 2^w if representable. Used with counts, never to expand a large
// domain merely because an address is wide (e.g. BV80).
bool CoversDomain(size_t count, uint32_t width) {
    return width < std::numeric_limits<size_t>::digits &&
           count >= (size_t{1} << width);
}

struct Components {
    explicit Components(size_t n) : parent(n) {
        std::iota(parent.begin(), parent.end(), 0);
    }
    size_t Find(size_t a) {
        while (parent[a] != a) {
            parent[a] = parent[parent[a]];
            a = parent[a];
        }
        return a;
    }
    void Join(size_t a, size_t b) { parent[Find(a)] = Find(b); }
    std::vector<size_t> parent;
};

// Literal folding only: no assumptions about symbolic indices or SAT values.
std::optional<BitVector> LiteralValue(const Btor2IR &ir, int64_t id) {
    const auto &node = ir.Node(id);
    const auto width = ir.Sort(node.sortId).width;
    BitVector value;
    switch (node.tag) {
    case BTOR2_TAG_zero: value = BitVector::Zero(width); break;
    case BTOR2_TAG_one: value = BitVector::One(width); break;
    case BTOR2_TAG_ones: value = BitVector::Ones(width); break;
    case BTOR2_TAG_const: value = BitVector::FromBinary(width, node.constant); break;
    case BTOR2_TAG_constd: value = BitVector::FromDecimal(width, node.constant); break;
    case BTOR2_TAG_consth: value = BitVector::FromHex(width, node.constant); break;
    default: return std::nullopt;
    }
    if (id < 0)
        for (uint32_t bit = 0; bit < width; ++bit)
            value.SetBit(bit, !value.GetBit(bit));
    return value;
}
} // namespace

bool ArrayEqualityEncoder::HasArrayComparisons(const Btor2IR &ir) {
    for (const auto &node : ir.Nodes())
        if ((node.tag == BTOR2_TAG_eq || node.tag == BTOR2_TAG_neq) &&
            ir.Sort(ir.Node(node.args[0]).sortId).tag == BTOR2_TAG_SORT_array)
            return true;
    return false;
}

ArrayEqualityEncoder::ArrayEqualityEncoder(const Btor2IR &bounded)
    : m_ir(bounded) {
    bounded.ValidateSupportedArrays();
    m_boolSort = BitVectorSort(1);
    m_true = Add(BTOR2_TAG_one, m_boolSort);
    m_false = Add(BTOR2_TAG_zero, m_boolSort);
    std::vector<Btor2IRNode> comparisons;
    for (const auto &node : bounded.Nodes()) {
        if (node.tag == BTOR2_TAG_next)
            throw std::runtime_error("array equality elimination requires bounded IR without next");
        if (node.tag == BTOR2_TAG_init) m_init[node.args[0]] = node.args[1];
        if (node.tag == BTOR2_TAG_read) m_reads[{node.args[0], node.args[1]}] = node.id;
        if ((node.tag == BTOR2_TAG_eq || node.tag == BTOR2_TAG_neq) &&
            bounded.Sort(bounded.Node(node.args[0]).sortId).tag == BTOR2_TAG_SORT_array)
            comparisons.push_back(node);
    }
    // Replace all comparisons before constructing constraints. Original scalar
    // consumers retain their IDs; no special lowering callback is necessary.
    std::map<int64_t, int64_t> equalBits;
    for (const auto &q : comparisons) {
        int64_t e = q.tag == BTOR2_TAG_eq ? q.id : Input(q.sortId);
        auto &replacement = m_ir.MutableNode(q.id);
        replacement.tag = q.tag == BTOR2_TAG_eq ? BTOR2_TAG_input : BTOR2_TAG_not;
        replacement.nargs = q.tag == BTOR2_TAG_eq ? 0 : 1;
        replacement.args = {q.tag == BTOR2_TAG_eq ? 0 : e, 0, 0};
        replacement.symbol = "wl.eq." + std::to_string(q.id);
        replacement.constant.clear();
        equalBits.emplace(q.id, e);
    }
    for (const auto &node : bounded.Nodes()) {
        if (node.sortId && bounded.Sort(node.sortId).tag == BTOR2_TAG_SORT_array &&
            node.tag != BTOR2_TAG_init) Array(node.id);
        if (node.tag == BTOR2_TAG_read)
            m_points.push_back({Array(node.args[0]), node.args[1], node.id});
    }
    for (const auto &q : comparisons) {
        const Ref left = Array(q.args[0]), right = Array(q.args[1]);
        if (m_nodes[left].sortId != m_nodes[right].sortId)
            throw std::runtime_error("array equality elimination sort mismatch");
        const auto sort = m_ir.Sort(m_ir.Node(q.args[0]).sortId);
        const int64_t omega = Input(sort.indexSort);
        const int64_t lhs = Read(q.args[0], omega), rhs = Read(q.args[1], omega);
        const int64_t e = equalBits.at(q.id);
        Require(Or(e, Ne(lhs, rhs))); // not e -> lhs != rhs
        m_comparisons.push_back({left, right, e});
        m_points.push_back({left, omega, lhs});
        m_points.push_back({right, omega, rhs});
    }
    for (Ref i = 0; i < m_nodes.size(); ++i)
        if (m_nodes[i].kind == Node::Kind::Store)
            m_points.push_back({i, m_nodes[i].address, m_nodes[i].data});
    m_stats.nodes = m_nodes.size();
    m_stats.points = m_points.size();
    m_stats.comparisons = m_comparisons.size();
    Encode();
    // Construction caches can name discarded intermediate gates. They are not
    // part of the model-recovery interface and are no longer needed.
    m_gates.clear();
    PruneGeneratedNodes(bounded);
    if (HasArrayComparisons(m_ir))
        throw std::runtime_error("array equality elimination left an array comparison");
}

ArrayEqualityEncoder::Ref ArrayEqualityEncoder::Array(int64_t id) {
    auto cached = m_arrayIds.find(id);
    if (cached != m_arrayIds.end()) return cached->second;
    const auto &source = m_ir.Node(id);
    Node node{Node::Kind::FreeRoot, source.sortId, id};
    switch (source.tag) {
    case BTOR2_TAG_input: break;
    case BTOR2_TAG_state:
        if (!m_init.count(id) ||
            m_ir.Sort(m_ir.Node(m_init.at(id)).sortId).tag != BTOR2_TAG_SORT_bitvec)
            throw std::runtime_error("bounded IR array state must denote a uniform array");
        node.kind = Node::Kind::Uniform;
        node.data = m_init.at(id);
        break;
    case BTOR2_TAG_write:
        node.kind = Node::Kind::Store;
        node.left = Array(source.args[0]);
        node.address = source.args[1];
        node.data = source.args[2];
        break;
    case BTOR2_TAG_ite:
        node.kind = Node::Kind::Choice;
        node.condition = source.args[0];
        node.left = Array(source.args[1]);
        node.right = Array(source.args[2]);
        break;
    default: throw std::runtime_error("unsupported bounded array expression");
    }
    const Ref result = m_nodes.size();
    m_nodes.push_back(node);
    m_arrayIds.emplace(id, result);
    return result;
}

int64_t ArrayEqualityEncoder::Add(Btor2Tag tag, int64_t sort,
                                    std::initializer_list<int64_t> args) {
    Btor2IRNode node;
    node.id = m_ir.FreshId();
    node.tag = tag;
    node.sortId = sort;
    node.nargs = args.size();
    std::copy(args.begin(), args.end(), node.args.begin());
    m_ir.AddNode(node);
    return node.id;
}

int64_t ArrayEqualityEncoder::BitVectorSort(uint32_t width) {
    for (const auto &[id, sort] : m_ir.Sorts())
        if (sort.tag == BTOR2_TAG_SORT_bitvec && sort.width == width) return id;
    return m_ir.AddSort({m_ir.FreshId(), BTOR2_TAG_SORT_bitvec, width, 0, 0});
}

int64_t ArrayEqualityEncoder::Input(int64_t sort) {
    const int64_t id = Add(BTOR2_TAG_input, sort);
    m_ir.MutableNode(id).symbol = "wl.eq.aux." + std::to_string(id);
    return id;
}

int64_t ArrayEqualityEncoder::Constant(uint32_t width, const std::string &bits) {
    const int64_t id = Add(BTOR2_TAG_const, BitVectorSort(width));
    m_ir.MutableNode(id).constant = bits;
    return id;
}

int64_t ArrayEqualityEncoder::Read(int64_t array, int64_t address) {
    auto key = std::make_pair(array, address);
    auto found = m_reads.find(key);
    if (found != m_reads.end()) return found->second;
    const int64_t sort = m_ir.Sort(m_ir.Node(array).sortId).elementSort;
    const int64_t id = Add(BTOR2_TAG_read, sort, {array, address});
    m_reads.emplace(key, id);
    return id;
}

int64_t ArrayEqualityEncoder::Not(int64_t x) const {
    if (x == m_true) return m_false;
    if (x == m_false) return m_true;
    return -x;
}

int64_t ArrayEqualityEncoder::Gate(Btor2Tag tag, int64_t x, int64_t y) {
    if (x > y) std::swap(x, y);
    auto key = std::make_tuple(tag, x, y);
    auto found = m_gates.find(key);
    if (found != m_gates.end()) return found->second;
    const int64_t id = Add(tag, m_boolSort, {x, y});
    m_gates.emplace(key, id);
    return id;
}

int64_t ArrayEqualityEncoder::And(int64_t x, int64_t y) {
    if (x == m_false || y == m_false || x == -y) return m_false;
    if (x == m_true || x == y) return y;
    if (y == m_true) return x;
    return Gate(BTOR2_TAG_and, x, y);
}

int64_t ArrayEqualityEncoder::Or(int64_t x, int64_t y) {
    return Not(And(Not(x), Not(y)));
}

int64_t ArrayEqualityEncoder::Eq(int64_t x, int64_t y) {
    if (m_ir.Sort(m_ir.Node(x).sortId).tag != BTOR2_TAG_SORT_bitvec ||
        m_ir.Sort(m_ir.Node(y).sortId).tag != BTOR2_TAG_SORT_bitvec)
        throw std::runtime_error("equality pass attempted to generate an array comparison");
    if (x == y) return m_true;
    if (x == -y) return m_false;
    const auto lhs = LiteralValue(m_ir, x), rhs = LiteralValue(m_ir, y);
    if (lhs && rhs) return *lhs == *rhs ? m_true : m_false;
    return Gate(BTOR2_TAG_eq, x, y);
}

void ArrayEqualityEncoder::Require(int64_t condition) {
    if (condition != m_true) Add(BTOR2_TAG_constraint, 0, {condition});
}

void ArrayEqualityEncoder::Encode() {
    const int64_t yes = m_true, no = m_false;
    auto require = [&](int64_t c) { Require(c); };
    std::set<int64_t> implications;
    auto implyEqual = [&](int64_t c, int64_t x, int64_t y) {
        if (c == no || x == y) return;
        const int64_t condition = Or(Not(c), Eq(x, y));
        if (condition == yes || !implications.insert(condition).second) return;
        require(condition);
        ++m_stats.implications;
    };

    // Treat every conditional edge as potentially enabled. Distinct components
    // of this supergraph cannot become connected under any scalar assignment.
    Components components(m_nodes.size());
    for (Ref i = 0; i < m_nodes.size(); ++i) {
        const auto &node = m_nodes[i];
        if (node.kind == Node::Kind::Store || node.kind == Node::Kind::Choice)
            components.Join(i, node.left);
        if (node.kind == Node::Kind::Choice) components.Join(i, node.right);
    }
    for (const auto &q : m_comparisons) components.Join(q.left, q.right);
    std::map<Ref, std::vector<Ref>> groups;
    for (Ref i = 0; i < m_nodes.size(); ++i) groups[components.Find(i)].push_back(i);

    // Unrolling may clone the same literal at many frames. Share its query,
    // without merging symbolic inputs from different frames.
    std::map<std::pair<int64_t, std::string>, int64_t> addresses;
    auto addressKey = [&](int64_t id) {
        if (const auto value = LiteralValue(m_ir, id)) {
            const auto key = std::make_pair(m_ir.Node(id).sortId, value->ToBinary());
            return addresses.emplace(key, id).first->second;
        }
        return id;
    };
    for (const auto &group : groups) {
        const int64_t sortId = m_nodes[group.second.front()].sortId;
        const auto &sort = m_ir.Sort(sortId);
        const uint32_t indexWidth = m_ir.Sort(sort.indexSort).width;
        const auto &ids = group.second;
        const size_t n = ids.size();
        std::map<Ref, size_t> local;
        std::vector<Ref> constants;
        std::set<int64_t> writes;
        std::vector<Point> points;
        for (size_t i = 0; i < n; ++i) {
            local.emplace(ids[i], i);
            const auto &node = m_nodes[ids[i]];
            if (node.kind == Node::Kind::Uniform) constants.push_back(ids[i]);
            if (node.kind == Node::Kind::Store) writes.insert(addressKey(node.address));
        }
        std::set<std::tuple<Ref, int64_t, int64_t>> pointKeys;
        for (const auto &p : m_points) {
            if (components.Find(p.array) != group.first) continue;
            const int64_t address = addressKey(p.address);
            if (pointKeys.emplace(p.array, address, p.data).second)
                points.push_back({p.array, address, p.data});
        }
        if (points.empty() && constants.size() < 2) continue;

        // Each closure is a shared combinational DAG, not free Reach variables
        // and not an enumeration of paths. The cache lives for this pass only.
        std::map<int64_t, std::vector<int64_t>> closures;
        auto closure = [&](int64_t address) -> const std::vector<int64_t> & {
            address = addressKey(address);
            auto found = closures.find(address);
            if (found != closures.end()) return found->second;
            if (n && n > std::numeric_limits<size_t>::max() / n)
                throw std::runtime_error("array equality closure size overflow");
            std::vector<int64_t> r(n * n, no);
            for (size_t i = 0; i < n; ++i) r[i * n + i] = yes;
            auto edge = [&](Ref x, Ref y, int64_t c) {
                const size_t i = local.at(x), j = local.at(y);
                r[i * n + j] = r[j * n + i] = Or(r[i * n + j], c);
            };
            for (Ref id : ids) {
                const auto &node = m_nodes[id];
                if (node.kind == Node::Kind::Store)
                    edge(id, node.left, Ne(address, node.address));
                else if (node.kind == Node::Kind::Choice) {
                    edge(id, node.left, node.condition);
                    edge(id, node.right, Not(node.condition));
                }
            }
            for (const auto &q : m_comparisons)
                if (components.Find(q.left) == group.first) edge(q.left, q.right, q.equal);
            for (size_t h = 0; h < n; ++h)
                for (size_t i = 0; i < n; ++i)
                    for (size_t j = i + 1; j < n; ++j)
                        r[i * n + j] = r[j * n + i] =
                            Or(r[i * n + j], And(r[i * n + h], r[h * n + j]));
            ++m_stats.queries;
            return closures.emplace(address, std::move(r)).first->second;
        };
        auto conn = [&](const std::vector<int64_t> &r, Ref x, Ref y) {
            return r[local.at(x) * n + local.at(y)];
        };
        for (size_t i = 0; i < points.size(); ++i) {
            const auto &p = points[i];
            for (size_t j = i + 1; j < points.size(); ++j) {
                const auto &q = points[j];
                if (p.data == q.data) continue;
                const int64_t sameAddress = Eq(p.address, q.address);
                if (sameAddress == no) continue;
                const int64_t connected = p.array == q.array ? yes
                    : conn(closure(p.address), p.array, q.array);
                implyEqual(And(sameAddress, connected), p.data, q.data);
            }
            for (Ref c : constants) {
                if (p.data == m_nodes[c].data) continue;
                implyEqual(conn(closure(p.address), p.array, c), p.data, m_nodes[c].data);
            }
        }
        if (constants.size() < 2) continue;
        std::vector<int64_t> queries;
        if (!CoversDomain(writes.size(), indexWidth)) {
            queries.assign(writes.begin(), writes.end());
            const int64_t lambda = Input(sort.indexSort);
            for (auto address : writes) require(Ne(lambda, address));
            queries.push_back(lambda);
        } else {
            const size_t capacity = size_t{1} << indexWidth;
            for (size_t i = 0; i < capacity; ++i) {
                const auto bits = BitVector::FromUInt64(indexWidth, i).ToBinary();
                queries.push_back(Constant(indexWidth, bits));
            }
        }
        for (auto a : queries) {
            const auto &r = closure(a);
            for (size_t i = 0; i < constants.size(); ++i)
                for (size_t j = i + 1; j < constants.size(); ++j)
                    implyEqual(conn(r, constants[i], constants[j]),
                               m_nodes[constants[i]].data, m_nodes[constants[j]].data);
        }
    }
}

void ArrayEqualityEncoder::PruneGeneratedNodes(const Btor2IR &source) {
    // Preserve source declarations/IDs, all asserted constraints, and every
    // observation needed for total-array recovery. Only generated garbage is
    // removed; walking nargs leaves slice/extension immediates untouched.
    auto work = ModelTerms();
    for (const auto &node : source.Nodes()) work.push_back(node.id);
    for (const auto &node : m_ir.Nodes())
        if (node.tag == BTOR2_TAG_constraint) work.push_back(node.id);
    std::unordered_set<int64_t> live;
    while (!work.empty()) {
        const int64_t id = std::abs(work.back());
        work.pop_back();
        if (!live.insert(id).second) continue;
        const auto &node = m_ir.Node(id);
        for (uint32_t i = 0; i < node.nargs; ++i) work.push_back(node.args[i]);
    }
    Btor2IR output;
    output.CopySortsFrom(m_ir);
    for (const auto &node : m_ir.Nodes())
        if (live.count(node.id)) output.AddNode(node);
    m_ir = std::move(output);
}

std::vector<int64_t> ArrayEqualityEncoder::ModelTerms() const {
    std::set<int64_t> terms;
    for (const auto &n : m_nodes) {
        if (n.condition) terms.insert(n.condition);
        if (n.address) terms.insert(n.address);
        if (n.data) terms.insert(n.data);
    }
    for (const auto &p : m_points) { terms.insert(p.address); terms.insert(p.data); }
    for (const auto &q : m_comparisons) terms.insert(q.equal);
    return {terms.begin(), terms.end()};
}

std::map<int64_t, ArrayValue> ArrayEqualityEncoder::Complete(const Value &value) const {
    std::vector<ArrayValue> result(m_nodes.size());
    std::map<int64_t, std::vector<Ref>> groups;
    for (Ref i = 0; i < m_nodes.size(); ++i) groups[m_nodes[i].sortId].push_back(i);
    for (const auto &group : groups) {
        const int64_t sortId = group.first;
        const auto &sort = m_ir.Sort(sortId);
        const uint32_t indexWidth = m_ir.Sort(sort.indexSort).width;
        const uint32_t elementWidth = m_ir.Sort(sort.elementSort).width;
        const auto &ids = group.second;
        std::map<std::string, BitVector> addresses;
        for (const auto &p : m_points) if (m_nodes[p.array].sortId == sortId) {
            auto a = value(p.address);
            addresses.emplace(a.ToBinary(), a);
        }
        // Store addresses occur in the write endpoints, so outside this set
        // every store edge is enabled. Never demand a background if it is empty.
        const bool background = !CoversDomain(addresses.size(), indexWidth);
        auto at = [&](const BitVector *address) {
            Components components(m_nodes.size());
            for (Ref id : ids) {
                const auto &n = m_nodes[id];
                if (n.kind == Node::Kind::Store && (!address || *address != value(n.address)))
                    components.Join(id, n.left);
                if (n.kind == Node::Kind::Choice)
                    components.Join(id, value(n.condition).IsZero() ? n.right : n.left);
            }
            for (const auto &q : m_comparisons)
                if (m_nodes[q.left].sortId == sortId && !value(q.equal).IsZero())
                    components.Join(q.left, q.right);
            std::map<Ref, BitVector> labels;
            auto label = [&](Ref id, const BitVector &v) {
                auto [it, inserted] = labels.emplace(components.Find(id), v);
                if (!inserted && it->second != v)
                    throw std::runtime_error("inconsistent array equality SAT model completion");
            };
            for (Ref id : ids) if (m_nodes[id].kind == Node::Kind::Uniform)
                label(id, value(m_nodes[id].data));
            if (address) for (const auto &p : m_points)
                if (m_nodes[p.array].sortId == sortId && *address == value(p.address))
                    label(p.array, value(p.data));
            std::map<Ref, BitVector> values;
            for (Ref id : ids) {
                auto found = labels.find(components.Find(id));
                values.emplace(id, found == labels.end() ? BitVector::Zero(elementWidth) : found->second);
            }
            return values;
        };
        if (background) {
            auto defaults = at(nullptr);
            for (Ref id : ids) result[id].defaultValue = defaults.at(id);
        } else for (Ref id : ids) result[id].defaultValue = BitVector::Zero(elementWidth);
        // Check complete equality, not just the explicit witness points.
        std::vector<bool> equal(m_comparisons.size(), true);
        auto check = [&](const auto &values) {
            for (size_t i = 0; i < m_comparisons.size(); ++i) {
                const auto &q = m_comparisons[i];
                if (m_nodes[q.left].sortId == sortId)
                    equal[i] = equal[i] && values.at(q.left) == values.at(q.right);
            }
        };
        if (background) check(at(nullptr));
        for (const auto &[key, address] : addresses) {
            auto values = at(&address);
            check(values);
            for (Ref id : ids) if (values.at(id) != *result[id].defaultValue)
                result[id].entries.push_back({address, values.at(id)});
        }
        for (size_t i = 0; i < m_comparisons.size(); ++i) {
            const auto &q = m_comparisons[i];
            if (m_nodes[q.left].sortId == sortId && equal[i] == value(q.equal).IsZero())
                throw std::runtime_error("array equality witness does not realize comparison");
        }
    }
    std::map<int64_t, ArrayValue> arrays;
    for (Ref i = 0; i < m_nodes.size(); ++i)
        arrays.emplace(m_nodes[i].source, std::move(result[i]));
    return arrays;
}

} // namespace car
