#ifndef DAG_CNF_SIMPLIFIER_H
#define DAG_CNF_SIMPLIFIER_H

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace car {

class DAGCNFSimplifier {
  public:
    using Clause = std::vector<int>;
    DAGCNFSimplifier() = default;

    void FreezeVar(int var);

    std::vector<Clause> Simplify(const std::vector<Clause> &dagClauses, int trueId);

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

    int m_maxVar = 0;
    bool m_occurEnabled = false;
    std::unordered_set<int> m_frozen;
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
    void RemoveNode(int var);
    std::vector<int> VarRels(int var) const;

    int8_t LitValue(int lit) const;
    void SetLitValue(int lit);
    bool IsVarAssigned(int var) const;
    int AssignedLit(int var) const;

    static bool LitLess(int a, int b);
    static int LitVar(int lit);
    static int LitNeg(int lit);
    static int VarPosLit(int var);
    static int VarNegLit(int var);

    bool TryOrderedSimplify(Clause &cls) const;
    bool TryOrderedResolvent(const Clause &a, const Clause &b, int pivotVar, Clause &out) const;
    static bool TryOrderedSubsumeExceptOne(const Clause &a,
                                           const Clause &b,
                                           bool &subsume,
                                           bool &diffFound,
                                           int &diff);

    void OccurAdd(int lit, int clauseId);
    void OccurDel(int lit, int clauseId);
    size_t OccurNum(int lit) const;
    const std::vector<int> &OccurGet(int lit);
    void OccurClean(int lit);

    bool TryResolvent(const std::vector<int> &pcnf,
                      const std::vector<int> &ncnf,
                      int pivotVar,
                      size_t limit,
                      std::vector<Clause> &out);

    void Eliminate(int var);
    void BveSimplify();
    void ConstSimplify();
    void ConstSimplifyVar(int var);
    void SubsumeSimplify();
    void ClauseSubsumeCheck(int clauseId);

    static std::vector<Clause> ClauseSubsumeSimplify(std::vector<Clause> clauses);

    std::vector<Clause> Finalize() const;
};

} // namespace car

#endif // DAG_CNF_SIMPLIFIER_H
