#pragma once
// RCPSP_TT2_THETA=1: Vilím's Θ-tree earliest-completion-time bound, max'ed into the heuristic. Archived: measured as
// a small node cut that costs more time than it saves (see HEURISTICS.md).
//
// Per resource of capacity C, over the unfinished activities that use it, each with an earliest start est (now = 0)
// and an energy e = remaining duration x demand:
//     ECT = ceil( max over subsets O of ( C * min_{i in O} est_i + sum_{i in O} e_i ) / C )
// is a lower bound on when all of them can be done. It is stronger than the energy bound (which is the case
// "all activities, est = 0"). The Θ-tree computes the maximum in O(n log n): sort by est, then combine bottom-up
//     energy(parent)   = energy(left) + energy(right)
//     envelope(parent) = max( envelope(right), envelope(left) + energy(right) )
// (Vilím 2008). h = the largest ECT over the resources.

#include <algorithm>
#include <climits>
#include <utility>
#include <vector>
#include "../NodeDeadlineBound.h"

namespace theta {

inline long& better() { static long v = 0; return v; }   // times it beat the bound it was max'ed with
inline double& sec() { static double v = 0.0; return v; }
inline void reset() { better() = 0; sec() = 0.0; }

// leaves = (est, energy) pairs, C > 0
inline long thetaTreeECT(const std::vector<std::pair<long, long>>& leaves, long C) {
    if (leaves.empty() || C <= 0) return 0;
    std::vector<std::pair<long, long>> s(leaves);
    std::sort(s.begin(), s.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    const int n = (int)s.size();
    int m = 1;
    while (m < n) m <<= 1;                                   // pad to a power of two with empty leaves
    const long NEG = LONG_MIN / 4;
    std::vector<long> energy(2 * m, 0), env(2 * m, NEG);
    for (int i = 0; i < n; ++i) { energy[m + i] = s[i].second; env[m + i] = s[i].first * C + s[i].second; }
    for (int i = m - 1; i >= 1; --i) {
        energy[i] = energy[2 * i] + energy[2 * i + 1];
        const long envLplus = (env[2 * i] <= NEG / 2) ? NEG : env[2 * i] + energy[2 * i + 1];
        env[i] = std::max(env[2 * i + 1], envLplus);
    }
    if (env[1] <= NEG / 2) return 0;
    return (env[1] + C - 1) / C;
}

// The bound at a node (remaining makespan, node clock). unfinished: 1-based ids; active: (1-based id, remaining).
inline int bound(const std::vector<short>& unfinished, const std::vector<std::pair<short, short>>& active) {
    const deadline::Inst& I = tt2nb::inst();
    std::vector<char> live(I.n, 0);
    std::vector<int> d(I.n, 0), est(I.n, 0);
    for (short id : unfinished) if (id >= 1 && id <= I.n) { live[id - 1] = 1; d[id - 1] = I.p[id - 1]; }
    for (const auto& a : active) if (a.first >= 1 && a.first <= I.n && live[a.first - 1]) d[a.first - 1] = a.second;
    for (int i : I.topo) {                                   // earliest starts over the unfinished sub-DAG
        if (!live[i]) continue;
        for (int q : I.preds[i]) if (live[q]) est[i] = std::max(est[i], est[q] + d[q]);
    }
    long best = 0;
    std::vector<std::pair<long, long>> leaves;
    for (int k = 0; k < I.R; ++k) {
        if (I.cap[k] <= 0) continue;
        leaves.clear();
        for (int i = 0; i < I.n; ++i)
            if (live[i] && d[i] > 0 && I.dem[k][i] > 0) leaves.emplace_back((long)est[i], (long)d[i] * I.dem[k][i]);
        best = std::max(best, thetaTreeECT(leaves, I.cap[k]));
    }
    return (int)best;
}

}  // namespace theta
