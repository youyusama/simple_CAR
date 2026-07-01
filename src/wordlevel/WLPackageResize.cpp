#include "WLPackageResize.h"

#include <algorithm>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace car {
namespace {

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

uint32_t CeilLog2(uint64_t value) {
    uint32_t result = 0;
    uint64_t capacity = 1;
    while (capacity < value) {
        capacity <<= 1;
        ++result;
    }
    return std::max<uint32_t>(1, result);
}

} // namespace

void WLPackageResize::Run(Btor2IR &ir) {
    // Candidate nodes are multi-bit scalar values that may share one finite domain.
    std::vector<int64_t> ids;
    std::unordered_map<int64_t, size_t> index;
    for (const Btor2IRNode &node : ir.Nodes()) {
        if (!node.sortId || !IsValueNode(node.tag)) continue;
        const Btor2IRSort &sort = ir.Sort(node.sortId);
        if (sort.tag != BTOR2_TAG_SORT_bitvec || sort.width <= 1) continue;
        index[node.id] = ids.size();
        ids.push_back(node.id);
    }
    if (ids.empty()) return;

    // Union-find groups values that must retain one common encoding and width.
    std::vector<size_t> parent(ids.size());
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&](size_t value) {
        size_t root = value;
        while (parent[root] != root) root = parent[root];
        while (parent[value] != value) {
            size_t next = parent[value];
            parent[value] = root;
            value = next;
        }
        return root;
    };
    auto unite = [&](int64_t lhs, int64_t rhs) {
        auto a = index.find(std::abs(lhs));
        auto b = index.find(std::abs(rhs));
        if (a == index.end() || b == index.end()) return;
        if (ir.Sort(ir.Node(lhs).sortId).width !=
            ir.Sort(ir.Node(rhs).sortId).width)
            return;
        size_t ra = find(a->second);
        size_t rb = find(b->second);
        if (ra != rb) parent[rb] = ra;
    };

    // Equality, ITE, init, and next edges connect nodes in the same package.
    for (const Btor2IRNode &node : ir.Nodes()) {
        switch (node.tag) {
        case BTOR2_TAG_ite:
            unite(node.id, node.args[1]);
            unite(node.id, node.args[2]);
            break;
        case BTOR2_TAG_eq:
        case BTOR2_TAG_neq: unite(node.args[0], node.args[1]); break;
        case BTOR2_TAG_init:
        case BTOR2_TAG_next: unite(node.args[0], node.args[1]); break;
        default: break;
        }
    }

    // Packages touched by arithmetic or width-sensitive operations are not resized.
    std::unordered_set<size_t> unsafe;
    auto markUnsafe = [&](int64_t id) {
        auto it = index.find(std::abs(id));
        if (it != index.end()) unsafe.insert(find(it->second));
    };
    for (const Btor2IRNode &node : ir.Nodes()) {
        switch (node.tag) {
        case BTOR2_TAG_input:
        case BTOR2_TAG_state:
        case BTOR2_TAG_const:
        case BTOR2_TAG_constd:
        case BTOR2_TAG_consth:
        case BTOR2_TAG_zero:
        case BTOR2_TAG_one:
        case BTOR2_TAG_ones:
        case BTOR2_TAG_ite:
        case BTOR2_TAG_eq:
        case BTOR2_TAG_neq:
        case BTOR2_TAG_init:
        case BTOR2_TAG_next:
        case BTOR2_TAG_bad:
        case BTOR2_TAG_constraint:
        case BTOR2_TAG_output:
            break;
        default:
            markUnsafe(node.id);
            for (uint32_t i = 0; i < node.nargs; ++i)
                markUnsafe(node.args[i]);
            break;
        }
        if (node.tag == BTOR2_TAG_read) markUnsafe(node.args[1]);
        if (node.tag == BTOR2_TAG_write) {
            markUnsafe(node.args[1]);
            markUnsafe(node.args[2]);
        }
        if (node.tag == BTOR2_TAG_const ||
            node.tag == BTOR2_TAG_constd ||
            node.tag == BTOR2_TAG_consth ||
            node.tag == BTOR2_TAG_one) {
            markUnsafe(node.id);
        }
    }

    // Normalize unsafe roots after all union operations have completed.
    std::unordered_set<size_t> normalizedUnsafe;
    for (size_t root : unsafe) normalizedUnsafe.insert(find(root));

    struct Stats {
        uint32_t oldWidth{0};
        uint64_t variables{0};
        std::vector<int64_t> constants;
    };
    // Count variables and distinct constants to bound each package's finite domain.
    std::unordered_map<size_t, Stats> stats;
    for (int64_t id : ids) {
        size_t root = find(index.at(id));
        if (normalizedUnsafe.count(root)) continue;
        const Btor2IRNode &node = ir.Node(id);
        Stats &entry = stats[root];
        entry.oldWidth = ir.Sort(node.sortId).width;
        if (node.tag == BTOR2_TAG_input || node.tag == BTOR2_TAG_state)
            ++entry.variables;
        if (IsConstant(node.tag)) entry.constants.push_back(id);
    }

    // Materialize each profitable encoding directly in the IR with a new sort.
    int64_t nextSortId = INT64_C(1) << 48;
    for (auto &[root, entry] : stats) {
        std::unordered_map<std::string, uint64_t> codes;
        // Replace original constants by their compact package codes.
        for (int64_t constantId : entry.constants) {
            const Btor2IRNode &constant = ir.Node(constantId);
            std::string key = std::to_string(constant.tag) + ":" +
                              constant.constant;
            if (!codes.count(key)) codes[key] = codes.size();
        }
        uint64_t domain = entry.variables + codes.size();
        uint32_t width =
            std::min(entry.oldWidth, CeilLog2(std::max<uint64_t>(2, domain)));
        if (width >= entry.oldWidth) continue;

        int64_t sortId = nextSortId++;
        ir.AddSort({sortId, BTOR2_TAG_SORT_bitvec, width, 0, 0});
        for (int64_t id : ids) {
            if (find(index.at(id)) == root)
                ir.MutableNode(id).sortId = sortId;
        }
        for (int64_t constantId : entry.constants) {
            Btor2IRNode &constant = ir.MutableNode(constantId);
            std::string key = std::to_string(constant.tag) + ":" +
                              constant.constant;
            constant.tag = BTOR2_TAG_constd;
            constant.constant = std::to_string(codes.at(key));
        }
    }
}

} // namespace car
