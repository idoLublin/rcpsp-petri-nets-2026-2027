#pragma once
// Sub-problem bounds for a search node: solve a smaller or relaxed version of the remaining problem exactly (or
// until a budget runs out) with the sub-solver in SubsetSolver.h, and use its lower bound as h. Both are admissible:
// the sub-problem's optimum can only be shorter than the real remaining makespan, and a budget-limited run returns
// a bound below that optimum.
//
// The remaining problem at a node, in the node's own clock (now = 0): the unfinished activities; one in progress has
// its remaining time as its duration, the others their full duration; precedences among the unfinished ones. (The
// relaxation may also start an in-progress activity later than now, which can only shorten it.)
//
//   RCPSP_TT2_SINGLERES  single-resource relaxation: for each resource k, keep k's capacity and make every other
//                        resource unlimited, solve; h = the largest of these optima.
//   RCPSP_TT2_LOOKAHEAD  look-ahead: the remaining problem itself, with every real capacity, under an expansion cap;
//                        h = the sub-solver's bound when it stops (the optimum if it finishes within the cap).
// _EXPAND sets each one's expansion cap per sub-solve (default 2000; 0 = no cap, bounded only by the time limit).

#include <chrono>
#include <vector>
#include "NodeDeadlineBound.h"
#include "SubsetSolver.h"

namespace subb {

struct Stats {
    long srCalls = 0, srBetter = 0, srCapped = 0, srExpands = 0;
    long laCalls = 0, laBetter = 0, laCapped = 0, laExpands = 0;
    double srSec = 0.0, laSec = 0.0;
};
inline Stats& st() { static Stats s; return s; }
inline void reset() { st() = Stats{}; }

// The remaining problem at a node as a SubsetInstance (real capacities). unfinished: 1-based ids; active: (1-based
// id, remaining time). The dummy source and sink (duration 0) are left out.
inline SubsetInstance remainingProblem(const deadline::Inst& I, const std::vector<short>& unfinished,
                                       const std::vector<std::pair<short, short>>& active) {
    std::vector<int> local(I.n, -1), rem(I.n, -1), resid;
    for (const auto& a : active) if (a.first >= 1 && a.first <= I.n) rem[a.first - 1] = a.second;
    for (short id : unfinished) {
        const int i = id - 1;
        if (i < 0 || i >= I.n || I.p[i] <= 0) continue;
        local[i] = (int)resid.size();
        resid.push_back(i);
    }
    SubsetInstance P;
    P.n = (int)resid.size(); P.R = I.R;
    P.dur.resize(P.n); P.release.assign(P.n, 0);
    P.demand.assign(P.n, std::vector<int>(I.R, 0));
    P.preds.assign(P.n, {});
    P.cap = I.cap;
    for (int k = 0; k < P.n; ++k) {
        const int i = resid[k];
        P.dur[k] = rem[i] >= 0 ? rem[i] : I.p[i];
        for (int r = 0; r < I.R; ++r) P.demand[k][r] = I.dem[r][i];
        for (int q : I.preds[i]) if (local[q] >= 0) P.preds[k].push_back(local[q]);
    }
    return P;
}

// RCPSP_TT2_SINGLERES: the largest single-resource relaxation optimum (remaining makespan, node clock).
inline int singleResource(const std::vector<short>& unfinished, const std::vector<std::pair<short, short>>& active,
                          long expandCap, std::chrono::steady_clock::time_point until) {
    const deadline::Inst& I = tt2nb::inst();
    SubsetInstance P = remainingProblem(I, unfinished, active);
    if (P.n == 0) return 0;
    const auto t0 = std::chrono::steady_clock::now();
    ++st().srCalls;
    const int BIG = 1 << 24;
    long best = 0;
    for (int k = 0; k < I.R; ++k) {
        if (I.cap[k] <= 0) continue;
        for (int r = 0; r < I.R; ++r) P.cap[r] = (r == k) ? I.cap[k] : BIG;
        const SubsetResult rr = subsetRcpspLB(P, expandCap, until);
        st().srExpands += rr.expands;
        if (rr.capped) ++st().srCapped;
        best = std::max(best, rr.lb);
    }
    st().srSec += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return (int)best;
}

// RCPSP_TT2_LOOKAHEAD: the sub-solver's bound on the remaining problem itself.
inline int lookAhead(const std::vector<short>& unfinished, const std::vector<std::pair<short, short>>& active,
                     long expandCap, std::chrono::steady_clock::time_point until) {
    const deadline::Inst& I = tt2nb::inst();
    const SubsetInstance P = remainingProblem(I, unfinished, active);
    if (P.n == 0) return 0;
    const auto t0 = std::chrono::steady_clock::now();
    ++st().laCalls;
    const SubsetResult rr = subsetRcpspLB(P, expandCap, until);
    st().laExpands += rr.expands;
    if (rr.capped) ++st().laCapped;
    st().laSec += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return (int)rr.lb;
}

}  // namespace subb
