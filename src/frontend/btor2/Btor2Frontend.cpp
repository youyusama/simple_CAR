#include "Btor2Frontend.h"

#include <cstdio>
#include <stdexcept>

namespace car {
namespace {

std::runtime_error Unsupported(const Btor2IRNode &node,
                               const std::string &reason) {
    return std::runtime_error("BTOR2 line " + std::to_string(node.line) +
                              " (id " + std::to_string(node.id) + "): " +
                              reason);
}

} // namespace

Btor2IR Btor2Frontend::LoadIR(const std::string &path) {
    Btor2IR ir = Parse(path);
    Validate(ir);
    return ir;
}

Btor2IR Btor2Frontend::Parse(const std::string &path) {
    // Delegate syntax parsing to btor2tools, then copy data into owned C++ objects.
    FILE *file = fopen(path.c_str(), "r");
    if (!file) throw std::runtime_error("cannot open BTOR2 input: " + path);

    Btor2Parser *parser = btor2parser_new();
    if (!btor2parser_read_lines(parser, file)) {
        std::string error = btor2parser_error(parser);
        btor2parser_delete(parser);
        fclose(file);
        throw std::runtime_error(error);
    }
    fclose(file);

    Btor2IR result;
    Btor2LineIterator iterator = btor2parser_iter_init(parser);
    Btor2Line *line;
    while ((line = btor2parser_iter_next(&iterator))) {
        // Sort declarations live in a separate table and are not value nodes.
        if (line->tag == BTOR2_TAG_sort) {
            Btor2IRSort sort;
            sort.id = line->id;
            sort.tag = line->sort.tag;
            if (sort.tag == BTOR2_TAG_SORT_bitvec) {
                sort.width = line->sort.bitvec.width;
            } else {
                sort.indexSort = line->sort.array.index;
                sort.elementSort = line->sort.array.element;
            }
            result.AddSort(sort);
            continue;
        }

        // Preserve signed argument IDs because BTOR2 uses negative IDs for inversion.
        Btor2IRNode node;
        node.id = line->id;
        node.line = line->lineno;
        node.tag = line->tag;
        node.sortId = line->sort.id;
        node.nargs = line->nargs;
        for (size_t i = 0; i < node.args.size(); ++i) {
            node.args[i] = line->args ? line->args[i] : 0;
        }
        if (line->constant) node.constant = line->constant;
        if (line->symbol) node.symbol = line->symbol;
        result.AddNode(node);
    }

    btor2parser_delete(parser);
    return result;
}

void Btor2Frontend::Validate(const Btor2IR &ir) {
    const Btor2IRNode *badProperty = nullptr;

    // Safety and fairness metadata always consumes a one-bit condition.
    for (const Btor2IRNode &node : ir.Nodes()) {
        if (node.tag == BTOR2_TAG_bad) {
            if (badProperty) {
                throw Unsupported(
                    node,
                    "multiple bad properties are unsupported; the first is "
                    "at line " +
                        std::to_string(badProperty->line));
            }
            badProperty = &node;
        }
        if (node.tag != BTOR2_TAG_bad &&
            node.tag != BTOR2_TAG_constraint &&
            node.tag != BTOR2_TAG_fair) {
            continue;
        }
        const Btor2IRNode &condition = ir.Node(node.args[0]);
        if (!condition.sortId ||
            ir.Sort(condition.sortId).tag != BTOR2_TAG_SORT_bitvec ||
            ir.Sort(condition.sortId).width != 1) {
            throw Unsupported(node,
                              "property condition must be a one-bit bitvector");
        }
    }
    if (!badProperty) {
        throw std::runtime_error(
            "BTOR2 input must contain exactly one bad property");
    }
}

} // namespace car
