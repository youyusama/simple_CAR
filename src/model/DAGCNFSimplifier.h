#ifndef DAG_CNF_SIMPLIFIER_H
#define DAG_CNF_SIMPLIFIER_H

#include "CarTypes.h"
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace car {

class DAGCNFSimplifier {
  public:
    DAGCNFSimplifier() = default;

    void FreezeVar(Var var);

    std::vector<Clause> Simplify(const std::vector<Clause> &dagClauses);

  private:
    struct ClauseEntry {
        Clause lits;
        bool removed = false;
    };

    struct Occur {
        std::vector<int> occur;
        bool dirty = false;
        size_t size = 0;
    };

    Var m_maxVar = 0;
    bool m_occurEnabled = false;
    std::unordered_set<Var> m_frozen;
    std::vector<ClauseEntry> m_clauseDb;
    std::vector<std::vector<int>> m_headClauses;
    std::vector<Occur> m_occurs;
    std::vector<int8_t> m_values;

    void Reset(const std::vector<Clause> &dagClauses);

    void EnableOccur();
    void DisableOccur();

    void AddRel(Clause rel);
    void RemoveRel(int clauseId);
    void RemoveRels(const std::vector<int> &clauseIds);
    void RemoveNode(Var var);
    std::vector<int> VarRels(Var var) const;

    int8_t LitValue(Lit lit) const;
    void SetLitValue(Lit lit);
    bool IsVarAssigned(Var var) const;
    Lit AssignedLit(Var var) const;

    bool TryOrderedSimplify(Clause &cls) const;
    bool TryOrderedResolvent(const Clause &a, const Clause &b, Var pivotVar, Clause &out) const;
    static bool TryOrderedSubsumeExceptOne(const Clause &a,
                                           const Clause &b,
                                           bool &subsume,
                                           bool &diffFound,
                                           Lit &diff);

    void OccurAdd(Lit lit, int clauseId);
    void OccurDel(Lit lit, int clauseId);
    size_t OccurNum(Lit lit) const;
    const std::vector<int> &OccurGet(Lit lit);
    void OccurClean(Lit lit);

    bool TryResolvent(const std::vector<int> &pcnf,
                      const std::vector<int> &ncnf,
                      Var pivotVar,
                      size_t limit,
                      std::vector<Clause> &out);

    void Eliminate(Var var);
    void BveSimplify();
    void ConstSimplify();
    void ConstSimplifyVar(Var var);
    void SubsumeSimplify();
    void ClauseSubsumeCheck(int clauseId);

    static std::vector<Clause> ClauseSubsumeSimplify(std::vector<Clause> clauses);

    std::vector<Clause> Finalize() const;
};

} // namespace car

#endif // DAG_CNF_SIMPLIFIER_H
