#ifndef DAG_CNF_SIMPLIFIER_H
#define DAG_CNF_SIMPLIFIER_H

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace car {

class DAGCNFSimplifier {
  public:
    using clause = std::vector<int>;

    DAGCNFSimplifier() = default;

    void FreezeVar(int var);

    std::vector<clause> Simplify(const std::vector<clause> &dag_clauses);

  private:
    struct ClauseEntry {
        clause lits;
        bool removed = false;
    };

    struct Occur {
        std::vector<int> occur;
        bool dirty = false;
        size_t size = 0;
    };

    int m_maxVar = 0;
    bool m_occurEnabled = false;
    std::unordered_set<int> m_frozen;
    std::vector<ClauseEntry> m_clauseDb;
    std::vector<std::vector<int>> m_headClauses;
    std::vector<Occur> m_occurs;
    std::vector<int8_t> m_values;

    void Reset(const std::vector<clause> &dag_clauses);

    void EnableOccur();
    void DisableOccur();

    void AddRel(clause rel);
    void RemoveRel(int clause_id);
    void RemoveRels(const std::vector<int> &clause_ids);
    void RemoveNode(int var);
    std::vector<int> VarRels(int var) const;

    int8_t LitValue(int lit) const;
    void SetLitValue(int lit);
    bool IsVarAssigned(int var) const;
    int AssignedLit(int var) const;

    static bool LitLess(int a, int b);
    static int LitVar(int lit);
    static int LitIndex(int lit);
    static int LitNeg(int lit);

    bool TryOrderedSimplify(clause &cls) const;
    bool TryOrderedResolvent(const clause &a, const clause &b, int pivot_var, clause &out) const;
    static bool TryOrderedSubsumeExceptOne(const clause &a,
                                           const clause &b,
                                           bool &subsume,
                                           bool &diff_found,
                                           int &diff);

    void OccurAdd(int lit, int clause_id);
    void OccurDel(int lit, int clause_id);
    size_t OccurNum(int lit) const;
    const std::vector<int> &OccurGet(int lit);
    void OccurClean(int lit);

    bool TryResolvent(const std::vector<int> &pcnf,
                      const std::vector<int> &ncnf,
                      int pivot_var,
                      size_t limit,
                      std::vector<clause> &out);

    void Eliminate(int var);
    void BveSimplify();
    void ConstSimplify();
    void ConstSimplifyVar(int var);
    void SubsumeSimplify();
    void ClauseSubsumeCheck(int clause_id);

    static std::vector<clause> ClauseSubsumeSimplify(std::vector<clause> clauses);

    std::vector<clause> Finalize() const;
};

} // namespace car

#endif // DAG_CNF_SIMPLIFIER_H
