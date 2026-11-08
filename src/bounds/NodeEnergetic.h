#pragma once
// RCPSP_TT2_TT_ENERGY=1 (with RCPSP_TT2_TT=1): energetic reasoning as an extra proof step of the per-node deadline
// bound (NodeDeadlineBound.h).
//
// For a time interval [t1, t2) and a live activity with window [es, ls] (earliest / latest start) and duration d,
// part of its work falls inside the interval wherever it starts:
//     min(t2 - t1, d, es + d - t1, t2 - ls)        (if positive)
// (the smaller of its overlap when started as early as possible and when started as late as possible). If, on some
// resource, the demand-weighted sum of these forced parts exceeds capacity x (t2 - t1), the deadline is impossible.
// Unlike the compulsory-part test this needs no tight window, so it can raise h further above the critical path.
// The intervals: every pair of window ends (es and ls of the live activities, plus 0 and D) - O(n^2) intervals at
// O(n R) each, which is what makes it expensive.

#include <algorithm>
#include <vector>
#include "Propagation.h"

namespace tt2nb {

// true = D is refuted. live / d / es / ls are indexed by 0-based activity; pts is scratch.
inline bool energeticRefutes(const deadline::Inst& I, int D, const std::vector<char>& live, const std::vector<int>& d,
                             const std::vector<int>& es, const std::vector<int>& ls, std::vector<int>& pts) {
    pts.clear();
    for (int i = 0; i < I.n; ++i) if (live[i] && d[i] > 0) { pts.push_back(es[i]); pts.push_back(ls[i]); }
    pts.push_back(0); pts.push_back(D);
    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    const int M = (int)pts.size();
    for (int x = 0; x < M; ++x)
        for (int y = x + 1; y < M; ++y) {
            const int t1 = pts[x], t2 = pts[y], L = t2 - t1;
            if (L <= 0) continue;
            for (int k = 0; k < I.R; ++k) {
                long W = 0; const long capL = (long)I.cap[k] * L;
                for (int i = 0; i < I.n; ++i) {
                    if (!live[i] || d[i] <= 0) continue;
                    const int dem = I.dem[k][i];
                    if (dem == 0) continue;
                    const int ov = std::min(std::min(L, d[i]), std::min(es[i] + d[i] - t1, t2 - ls[i]));
                    if (ov > 0) { W += (long)dem * ov; if (W > capL) return true; }
                }
            }
        }
    return false;
}

}  // namespace tt2nb
