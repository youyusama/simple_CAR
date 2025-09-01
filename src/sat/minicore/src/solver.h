#ifndef MINICORE_SOLVER_H
#define MINICORE_SOLVER_H

#include "math.h"
#include "solver_types.h"
#include "utils.hpp"
#include <algorithm>
#include <assert.h>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>

namespace minicore {

inline std::string vec2str(const std::vector<Lit> &lits) {
    std::string res;
    for (Lit l : lits) {
        res += std::to_string(toInt(l)) + " ";
    }
    return res;
}

class Solver {
  public:
    // Constructor/Destructor:
    //
    Solver();
    virtual ~Solver();

    // Problem specification:
    //
    Var newVar(); // Add a new variable with parameters specifying variable mode.

    bool addClause(const std::vector<Lit> &ps); // Add a clause to the solver.
    bool addClause_(std::vector<Lit> &ps);      // Add a clause to the solver without making superflous internal copy. Will
                                                // change the passed vector 'ps'.

    bool addTempClause(const std::vector<Lit> &ps); // Add a temp clause that only effects next solve

    // Solving:
    //
    bool simplify();                             // Removes already satisfied clauses.
    bool solve(const std::vector<Lit> &assumps); // Search for a model that respects a given set of assumptions.
    bool solve();                                // Search without assumptions.
    lbool solve_main();                          // Search invoked from main.cpp
    bool okay() const;                           // FALSE means solver is in a conflicting state

    // Read state:
    //
    lbool value(Var x) const;      // The current value of a variable.
    lbool value(Lit p) const;      // The current value of a literal.
    lbool modelValue(Var x) const; // The value of a variable in the last model. The last call to solve must have been satisfiable.
    lbool modelValue(Lit p) const; // The value of a literal in the last model. The last call to solve must have been satisfiable.
    int nAssigns() const;          // The current number of assigned literals.
    int nClauses() const;          // The current number of original clauses.
    int nLearnts() const;          // The current number of learnt clauses.
    int nVars() const;             // The current number of variables.
    void printStats() const;       // Print some current statistics to standard output.

    // Memory managment:
    //
    virtual void garbageCollect();
    void checkGarbage();

    // Extra results: (read-only member variable)
    //
    std::vector<lbool> model;                  // If problem is satisfiable, this vector contains the model (if any).
    std::unordered_set<Lit, LitHash> conflict; // If problem is unsatisfiable (possibly under assumptions),
                                               // this vector represent the final conflict clause expressed in the assumptions.

    // Mode of operation:
    //
    int verbosity;
    double random_seed;

    int restart_first;        // The initial restart limit.                                                                (default 100)
    double restart_inc;       // The factor with which the restart limit is multiplied in each restart.                    (default 1.5)
    double learntsize_factor; // The intitial limit for learnt clauses is a factor of the original clauses.                (default 1 / 3)
    double learntsize_inc;    // The limit for learnt clauses is multiplied with this factor each restart.                 (default 1.1)

    int learntsize_adjust_start_confl;
    double learntsize_adjust_inc;

    // Statistics: (read-only member variable)
    //
    uint64_t solves, starts, decisions, rnd_decisions, propagations, conflicts;
    uint64_t dec_vars, num_clauses, num_learnts, clauses_literals, learnts_literals, max_literals, tot_literals;

  protected:
    std::shared_ptr<ClauseAllocator> ca;

    // Solver state:
    //
    std::vector<CRef> clauses;      // List of problem clauses.
    std::vector<CRef> learnts;      // List of learnt clauses.
    std::vector<CRef> temp_clauses; // List of clauses learnt from the temp clause.
    std::vector<Lit> trail;         // Assignment stack; stores all assigments made in the order they were made.
    std::vector<int> trail_lim;     // Separator indices for different decision levels in 'trail'.
    std::vector<Lit> assumptions;   // Current set of assumptions provided to solve by the user.

    std::vector<lbool> assigns;   // The current assignments.
    std::vector<char> polarity;   // The preferred polarity of each variable.
    std::vector<VarData> vardata; // Stores reason and level for each variable.
    OccLists watches;             // 'watches[lit]' is a list of constraints watching 'lit' (will go there if literal becomes true).

    bool solveInDomain;                 // Deciside in domain.
    std::vector<char> permanent_domain; // A variable is a decision variable in all queries.
    std::vector<char> temporary_domain; // A variable is a decision variable in the next query.

    DecisionBuckets order_list; // A priority queue of variables ordered with respect to the variable activity.

    // VarOrderLt var_order_lt; // Compare function for var on activity order
    reduceDB_lt reduce_db_lt;

    bool ok;                  // If FALSE, the constraints are already unsatisfiable. No part of the solver state may be used!
    uint32_t cla_inc;         // Amount to bump next clause with.
    size_t qhead;             // Head of queue (as index into the trail -- no more explicit propagation queue in MiniSat).
    int simpDB_assigns;       // Number of top-level assignments since last execution of 'simplify()'.
    int64_t simpDB_props;     // Remaining number of propagations that must be made before next execution of 'simplify()'.
    double progress_estimate; // Set by 'search()'.
    bool remove_satisfied;    // Indicates whether possibly inefficient linear scan for satisfied clauses should be performed in 'simplify'.
    Var next_var;             // Next variable to be created.
    Var alloced_var;          // Variable with structure created.
    Var temp_cls_act_var;     // Variable to activate temp clause.
    bool temp_cls_activated;  // A temp clause is added.

    // Temporaries (to reduce allocation overhead). Each variable is prefixed by the method in which it is
    // used, exept 'seen' wich is used in several places.
    //
    std::vector<char> seen;
    std::vector<Lit> analyze_toclear;

    double max_learnts;
    double learntsize_adjust_confl;
    int learntsize_adjust_cnt;

    // Main internal methods:
    //
    void insertVarOrder(Var x); // Insert a variable in the decision order priority queue.
    Lit pickBranchLit();        // Return the next decision variable.
    void newDecisionLevel();    // Begins a new decision level.
    void uncheckedEnqueue(Lit p,
                          CRef from = CRef_Undef); // Enqueue a literal. Assumes value of literal is undefined.
    CRef propagate();                              // Perform unit propagation. Returns possibly conflicting clause.
    void cancelUntil(size_t level);                // Backtrack until a certain level.
    void analyze(CRef confl,
                 std::vector<Lit> &out_learnt,
                 int &out_btlevel); // (bt = backtrack)
    void analyzeFinal(Lit p,
                      std::unordered_set<Lit, LitHash> &out_conflict);
    bool litRedundant(Lit p);                    // (helper method for 'analyze()')
    lbool search(int nof_conflicts);             // Search for a given number of conflicts.
    lbool solve_();                              // Main solve method (assumptions given in 'assumptions').
    void reduceDB();                             // Reduce the set of learnt clauses.
    void removeSatisfied(std::vector<CRef> &cs); // Shrink 'cs' to contain only non-satisfied clauses.
    void removeTempLearnt();                     // Remove the clauses learnt from the temp clause.

    // Maintaining Variable/Clause activity:
    //
    void varBumpActivity(Var v);     // Increase a variable with the current 'bump' value.
    void claBumpActivity(Clause &c); // Increase a clause with the current 'bump' value.

    // Operations on clauses:
    //
    void attachClause(CRef cr);                            // Attach a clause to watcher lists.
    void detachClause(CRef cr);                            // Detach a clause to watcher lists.
    void removeClause(CRef cr);                            // Detach and free a clause.
    bool isRemoved(CRef cr) const;                         // Test if a clause has been removed.
    bool locked(const Clause &c) const;                    // Returns TRUE if a clause is a reason for some implication in the current state.
    bool satisfied(const Clause &c) const;                 // Returns TRUE if a clause is satisfied in the current state.
    bool deducedByTemp(const std::vector<Lit> &cls) const; // If a clause is deduced by the temp clause.

    // Misc:
    //
    size_t decisionLevel() const;        // Gives the current decisionlevel.
    uint32_t abstractLevel(Var x) const; // Used to represent an abstraction of sets of decision levels.
    CRef reason(Var x) const;
    size_t level(Var x) const;
    double progressEstimate() const;
    void relocAll(std::shared_ptr<ClauseAllocator> new_ca);

    // Incremental modelchecking decision domain:
    //
    bool inDomain(Var x) const;
    void setDomain(const std::vector<Var> dvars);
    void setTempDomain(const std::vector<Var> dvars);
    void resetTempDomain();

    // Static helpers:
    //

    // Returns a random float 0 <= x < 1. Seed must never be 0.
    static inline double drand(double &seed) {
        seed *= 1389796;
        int q = (int)(seed / 2147483647);
        seed -= (double)q * 2147483647;
        return seed / 2147483647;
    }

    // Returns a random integer 0 <= x < size. Seed must never be 0.
    static inline int irand(double &seed, int size) {
        return (int)(drand(seed) * size);
    }
};


//=================================================================================================
// Implementation of inline methods:

inline CRef Solver::reason(Var x) const { return vardata[x].reason; }
inline size_t Solver::level(Var x) const { return vardata[x].level; }

inline void Solver::insertVarOrder(Var v) {
    order_list.insert(v);
}

inline void Solver::varBumpActivity(Var v) {
    order_list.update(v);
}

inline void Solver::claBumpActivity(Clause &c) {
    cla_inc++;
    c.activity() += cla_inc;
    c.activity() = c.activity() >> 1;
}

inline void Solver::checkGarbage(void) {
    if (ca->wasted_memory() > ca->allocated_memory() * 0.5)
        garbageCollect();
}

inline bool Solver::addClause(const std::vector<Lit> &cls) {
    std::vector<Lit> add_tmp = cls;
    return addClause_(add_tmp);
}

inline bool Solver::isRemoved(CRef cr) const { return ca->get_clause(cr).get_mark() == 1; }
inline bool Solver::locked(const Clause &c) const { return value(c[0]) == l_True && reason(var(c[0])) != CRef_Undef && reason(var(c[0])) == &c; }
inline void Solver::newDecisionLevel() { trail_lim.emplace_back(trail.size()); }

inline size_t Solver::decisionLevel() const { return trail_lim.size(); }
inline uint32_t Solver::abstractLevel(Var x) const { return 1 << (level(x) & 31); }
inline lbool Solver::value(Var x) const { return assigns[x]; }
inline lbool Solver::value(Lit p) const { return assigns[var(p)] ^ sign(p); }
inline lbool Solver::modelValue(Var x) const { return model[x]; }
inline lbool Solver::modelValue(Lit p) const { return model[var(p)] ^ sign(p); }
inline int Solver::nAssigns() const { return trail.size(); }
inline int Solver::nClauses() const { return num_clauses; }
inline int Solver::nLearnts() const { return num_learnts; }
inline int Solver::nVars() const { return next_var; }

inline bool Solver::solve() {
    assumptions.clear();
    return solve_() == l_True;
}
inline bool Solver::solve(const std::vector<Lit> &assumps) {
    if (temp_cls_activated) {
        assumptions.clear();
        assumptions.emplace_back(mkLit(temp_cls_act_var));
        assumptions.resize(assumps.size() + 1);
        std::copy(assumps.begin(), assumps.end(), assumptions.begin() + 1);
    } else
        assumptions = assumps;
    return solve_() == l_True;
}
inline lbool Solver::solve_main() {
    assumptions.clear();
    return solve_();
}
inline bool Solver::okay() const { return ok; }

inline bool Solver::inDomain(Var x) const { return !solveInDomain || permanent_domain[x] || temporary_domain[x]; }

inline void Solver::setDomain(const std::vector<Var> dvars) {
    for (Var x : dvars) permanent_domain[x] = 1;
}

inline void Solver::setTempDomain(const std::vector<Var> dvars) {
    for (Var x : dvars) temporary_domain[x] = 1;
}

inline void Solver::resetTempDomain() {
    std::fill(temporary_domain.begin(), temporary_domain.end(), 0);
}

} // namespace minicore

#endif