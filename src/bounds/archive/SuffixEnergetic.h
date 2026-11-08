#pragma once
// RCPSP_TT2_TT_SER=1 (with RCPSP_TT2_TT=1): suffix energetic reasoning, a cheap form of NodeEnergetic.h. Archived:
// measured as adding nothing (see HEURISTICS.md).
//
// It checks only the intervals [t1, D) that end at the deadline, with t1 = each live activity's earliest start: a
// live activity must still do min(D - t1, d, es + d - t1) units of work inside [t1, D) wherever it starts, so more
// than capacity x (D - t1) of it on a resource refutes D. O(n^2 R) instead of O(n^3 R).

#include <algorithm>
#include <vector>
#include "../Propagation.h"

namespace tt2nb {

// true = D is refuted. live / d / es are indexed by 0-based activity; pts is scratch.
inline bool suffixEnergeticRefutes(const deadline::Inst& I, int D, const std::vector<char>& live, const std::vector<int>& d,
                                   const std::vector<int>& es, std::vector<int>& pts) {
    pts.clear();
    for (int i = 0; i < I.n; ++i) if (live[i] && d[i] > 0) pts.push_back(es[i]);
    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    for (int t1 : pts) {
        const int L = D - t1;
        if (L <= 0) continue;
        for (int k = 0; k < I.R; ++k) {
            long W = 0; const long capL = (long)I.cap[k] * L;
            for (int i = 0; i < I.n; ++i) {
                if (!live[i] || d[i] <= 0) continue;
                const int dem = I.dem[k][i];
                if (dem == 0) continue;
                const int ov = std::min(std::min(L, d[i]), es[i] + d[i] - t1);
                if (ov > 0) { W += (long)dem * ov; if (W > capL) return true; }
            }
        }
    }
    return false;
}

}  // namespace tt2nb
