#pragma once
// Last year's lower bounds (project 2025-2026), ported to this solver. All off by default; each one is max'ed into
// the heuristic (RCPSP_TT2::HCost in petri/Environment.h), so h stays admissible. Every flag and counter is in the
// CSV (extra::csvColumns), and HEURISTICS.md has the same table.
//
//   flag                 bound                                                       form at a node
//   RCPSP_TT2_LBCS=1     LBCS, critical sequence (Stinson, Davis & Khumawala 1978)   per node
//   RCPSP_TT2_LBCC=1     LBcc, critical capacity ("tank" model of work arrival)      per node, uses g
//   RCPSP_TT2_LBIP0=1    LBip0, incompatible pairs (Klein & Scholl 1999, LB10)       root value - g
//   RCPSP_TT2_LBRC=1     LBrc, resource capacity (Klein & Scholl 1999, LB2)          root value - g
//   RCPSP_TT2_LBER=D     LBER, enhanced energetic reasoning (Haouari, Kooli &        root value - g
//                        Neron 2012), destructive bound at depth D = 1..4
//   RCPSP_TT2_LBER_NODE=D  the same energetic test on the residual problem          per node
//                        at every node, D = 1..4
//
// LBER depth: 1 = classical energetic reasoning (CER), 2 = + dual feasible functions (DFF, Fekete & Schepers),
// 3 = + shaving at the root (SHV), 4 = + relaxed revisited ER (RER, two knapsacks per interval). Last year's depths
// 5 and 6 (exact IPs per interval, EER / GER) needed OR-Tools and are not ported. Shaving is a root step only, so
// RCPSP_TT2_LBER_NODE=3 is the same as 2.
//
// HOW THEY DID LAST YEAR (TT2 forward search, no dominance, PSPLIB, 300 s, one core; the students' summary,
// analysis/RESULTS_SUMMARY.md of 2026-08-05). Proven optimal of 480:
//                 j30   j60   j90
//   CP (base)     476   241   182      CP = max(critical path, energy) = this solver's base heuristic
//   LBCC          476   236   179
//   LBIP0         476   241   183
//   LBMAX (LBrc)  476   241   182
//   LBCS          477   240   181
//   LBER d3       477   242   -        (root form; j30 mean 892 k expansions vs CP 643 k on solved instances)
// With the cutset dominance (DR5): LBCS 480 / 299 / 208, LBMAX 480 / 301 / 209. So none of them moved coverage by
// more than one or two instances: the root forms (LBip0, LBrc, LBER - g) decay with g and are dead after the first
// few levels, and the per-node ones rarely lift f across C*. This solver's base heuristic already contains LBrc's
// per-node form (remaining work / capacity), which dominates LBrc - g up to rounding.
//
// Consistency: "root value - g" is consistent (h falls by exactly the edge cost). The per-node bounds (LBCS, LBcc,
// LBER_NODE) are admissible but not always consistent, so the forward / backward search reopens closed nodes
// when one of them is on (RCPSP_TT2_REOPEN, see Globals.h).
//
// Orientation: the bounds use the topological order of the loaded net, so they work on the reversed net (backward
// search, HBAE) unchanged; there g counts from the end of the schedule, and the root values are the same.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ostream>
#include <vector>
#include "../common/Globals.h"

namespace ly {

// ── Per-net data, computed once per loaded net ───────────────────────────────
// Two slots, like resDemandMatrix() in petri/Heuristic.h: HBAE keeps the forward and the reversed net alive.
struct Net {
    uint64_t gen = ~0ULL; const P_RCPSP::Activity* ptr = nullptr;
    int n = 0, K = 0;
    std::vector<std::vector<int>> dem;   // dem[k][j], j 1-based
    std::vector<int> cap;                // capacity per resource
    std::vector<int> dur;                // duration, 1-based
    std::vector<int> topo;               // activity ids in a topological order of the loaded net
    std::vector<int> es;                 // root earliest start (critical-path forward pass)
    std::vector<int> tail;               // longest path from the finish of j to the end (durations of successors)
    int lb1 = 0;                         // critical-path length
    int lbrc = -1, lbip0 = -1;           // root values, computed on first use
    int lber[5] = {-1, -1, -1, -1, -1};  // LBER root value per depth, computed on first use
};

inline Net& net() {
    static Net slot[2]; static int next = 0;
    const P_RCPSP::Activity* ptr = RCPSPex.activities.data();
    for (auto& s : slot) if (s.gen == g_rcpsp_load_gen && s.ptr == ptr) return s;
    Net& s = slot[next]; next ^= 1;
    s = Net{};
    s.gen = g_rcpsp_load_gen; s.ptr = ptr;
    const int n = (int)RCPSPex.activities.size(), K = (int)RCPSPex.resources.size();
    s.n = n; s.K = K;
    s.cap.assign(K, 0); s.dem.assign(K, std::vector<int>(n + 1, 0)); s.dur.assign(n + 1, 0);
    for (int k = 0; k < K; ++k) s.cap[k] = RCPSPex.resources[k].second;
    for (int j = 1; j <= n; ++j) {
        s.dur[j] = RCPSPex.activities[j - 1].duration;
        for (const auto& kv : RCPSPex.activities[j - 1].resource_demands)
            for (int k = 0; k < K; ++k) if (RCPSPex.resources[k].first == kv.first) { s.dem[k][j] = kv.second; break; }
    }
    // topological order (Kahn) from the predecessor lists
    std::vector<int> indeg(n + 1, 0);
    for (int j = 1; j <= n; ++j) indeg[j] = (int)RCPSPex.backword_dependencies[j - 1].size();
    std::vector<int> q; for (int j = 1; j <= n; ++j) if (!indeg[j]) q.push_back(j);
    for (size_t h = 0; h < q.size(); ++h)
        for (int s2 : RCPSPex.dependencies[q[h] - 1]) if (--indeg[s2] == 0) q.push_back(s2);
    s.topo = q;
    s.es.assign(n + 1, 0); s.tail.assign(n + 1, 0);
    for (int j : s.topo) for (int p : RCPSPex.backword_dependencies[j - 1]) s.es[j] = std::max(s.es[j], s.es[p] + s.dur[p]);
    for (auto it = s.topo.rbegin(); it != s.topo.rend(); ++it)
        for (int c : RCPSPex.dependencies[*it - 1]) s.tail[*it] = std::max(s.tail[*it], s.dur[c] + s.tail[c]);
    for (int j = 1; j <= n; ++j) s.lb1 = std::max(s.lb1, s.es[j] + s.dur[j]);
    return s;
}

// ── Counters (per instance, written to the CSV) ─────────────────────────────
struct Stats { long calls = 0, lbcsBetter = 0, lbccBetter = 0, ip0Better = 0, rcBetter = 0, lberBetter = 0, lberNodeBetter = 0;
               double sec = 0.0; int rootIp0 = -1, rootRc = -1, rootLber = -1; };
inline Stats& st() { static Stats s; return s; }
inline long long& timeoutSec() { static long long t = 300; return t; }
inline void reset(long long timeout) { st() = Stats{}; timeoutSec() = timeout; }

inline bool anyOn() { return g_tt2_lbcs || g_tt2_lbcc || g_tt2_lbip0 || g_tt2_lbrc || g_tt2_lber > 0 || g_tt2_lber_node > 0; }
// the per-node ones: admissible, not always consistent (the search reopens closed nodes when one is on)
inline bool inconsistentOn() { return g_tt2_lbcs || g_tt2_lbcc || g_tt2_lber_node > 0; }

// Residual view of a node: which activities are unfinished / active and the active ones' remaining time.
struct Residual {
    std::vector<char> live; std::vector<int> rem;   // rem[j] = remaining time if active, -1 otherwise
    void build(int n, const std::vector<short>& unfinished, const std::vector<std::pair<short, short>>& active) {
        live.assign(n + 1, 0); rem.assign(n + 1, -1);
        for (short j : unfinished) live[j] = 1;
        for (const auto& [j, r] : active) if (j > 0 && j <= n) rem[j] = r;
    }
};

// ── LBrc (Klein & Scholl LB2) ────────────────────────────────────────────────
// max over resources of ceil(total work / capacity), once per instance; at a node: LBrc - g.
// Last year (TT2, j30 / j60 / j90): 476 / 241 / 182, the same as the base heuristic: the base already has the
// per-node form of this bound, which is never weaker except for the rounding up.
inline int lbrcRoot(Net& N) {
    if (N.lbrc >= 0) return N.lbrc;
    int best = 0;
    for (int k = 0; k < N.K; ++k) {
        if (N.cap[k] <= 0) continue;
        long long w = 0;
        for (int j = 1; j <= N.n; ++j) w += (long long)N.dem[k][j] * N.dur[j];
        best = std::max(best, (int)((w + N.cap[k] - 1) / N.cap[k]));
    }
    return N.lbrc = best;
}

// ── LBip0 (Klein & Scholl LB10, incompatible pairs) ─────────────────────────
// Two activities i, j that are not ordered by precedence but cannot run together (some resource k with
// r_ik + r_jk > a_k) run one after the other in every schedule. The longest path through the arc i -> j is
// ES[i] + d_i + d_j + tail[j]; the pair forces at least min(path i -> j, path j -> i). LBip0 = the largest
// such pair bound (and at least the critical path), once per instance; at a node: LBip0 - g.
// Last year (TT2): 476 / 241 / 183 - one more j90 instance than the base, otherwise the same; the root value
// falls with g like LBrc.
inline int lbip0Root(Net& N) {
    if (N.lbip0 >= 0) return N.lbip0;
    const int n = N.n, W = n + 1;
    std::vector<unsigned char> reach((size_t)W * W, 0);   // reach[i][j]: j is a (transitive) successor of i
    for (auto it = N.topo.rbegin(); it != N.topo.rend(); ++it) {
        const int i = *it;
        for (int c : RCPSPex.dependencies[i - 1]) {
            reach[(size_t)i * W + c] = 1;
            for (int j = 1; j <= n; ++j) if (reach[(size_t)c * W + j]) reach[(size_t)i * W + j] = 1;
        }
    }
    int best = N.lb1;
    for (int i = 1; i < n; ++i) {
        if (!N.dur[i]) continue;
        for (int j = i + 1; j <= n; ++j) {
            if (!N.dur[j] || reach[(size_t)i * W + j] || reach[(size_t)j * W + i]) continue;
            bool incompatible = false;
            for (int k = 0; k < N.K && !incompatible; ++k) incompatible = N.dem[k][i] + N.dem[k][j] > N.cap[k];
            if (!incompatible) continue;
            const int ij = N.es[i] + N.dur[i] + N.dur[j] + N.tail[j];
            const int ji = N.es[j] + N.dur[j] + N.dur[i] + N.tail[i];
            best = std::max(best, std::min(std::max(N.lb1, ij), std::max(N.lb1, ji)));
        }
    }
    return N.lbip0 = best;
}

// ── LBCS (critical sequence) ─────────────────────────────────────────────────
// Take one critical chain of the residual problem (a longest precedence path, length CPM; its links run one after
// the other). An unstarted activity i off the chain can run in parallel only with the links it is resource-
// compatible with; the rest of i must lie outside the chain, so T >= CPM + d_i - (time of the compatible links).
// LBCS = CPM + the largest such extension. Note: last year's first version (a time-profile form) was NOT
// admissible (j30 13:5 gave 68 instead of 67); this is their fixed chain form.
// Last year (TT2): 477 / 240 / 181 vs the base 476 / 241 / 182: +1 on j30, -1 on j60 and j90 (the time per node).
inline int lbcs(Net& N, const Residual& R) {
    std::vector<int> ES(N.n + 1, -1), EF(N.n + 1, -1);
    int cpm = 0;
    for (int j : N.topo) {
        if (!R.live[j]) continue;
        if (R.rem[j] >= 0) { ES[j] = 0; EF[j] = R.rem[j]; }
        else {
            int s = 0;
            for (int p : RCPSPex.backword_dependencies[j - 1]) if (EF[p] > s) s = EF[p];
            ES[j] = s; EF[j] = s + N.dur[j];
        }
        cpm = std::max(cpm, EF[j]);
    }
    if (cpm == 0) return 0;
    std::vector<int> chain;
    int cur = -1;
    for (int j : N.topo) if (R.live[j] && EF[j] == cpm) { cur = j; break; }
    while (cur != -1) {
        chain.push_back(cur);
        if (ES[cur] == 0) break;
        int nxt = -1;
        for (int p : RCPSPex.backword_dependencies[cur - 1]) if (EF[p] == ES[cur]) { nxt = p; break; }
        cur = nxt;
    }
    std::vector<char> onChain(N.n + 1, 0);
    for (int c : chain) onChain[c] = 1;
    int ext = 0;
    for (int i : N.topo) {
        if (!R.live[i] || onChain[i] || R.rem[i] >= 0) continue;
        const int d = N.dur[i];
        if (d <= ext) continue;
        int overlap = 0;
        for (int c : chain) {
            bool fits = true;
            for (int k = 0; k < N.K && fits; ++k) fits = N.dem[k][i] + N.dem[k][c] <= N.cap[k];
            if (fits) overlap += (R.rem[c] >= 0) ? R.rem[c] : N.dur[c];
        }
        ext = std::max(ext, d - overlap);
    }
    return cpm + ext;
}

// ── LBcc (critical capacity, the tank model) ─────────────────────────────────
// Per resource: work arrives at each activity's earliest start and drains at the capacity per time unit; the tank
// never goes below 0. When the last work has arrived at time t, the remaining makespan is at least
// t + ceil(tank / capacity). At a node, active activities arrive at 0 with their remaining work; an unstarted one at
// max(0, root ES - g) (its root earliest start is a valid lower bound on its absolute start).
// Last year (TT2): 476 / 236 / 179, below the base on j60 / j90 (slower per node, rarely higher).
// Idea for a student: the residual earliest start (forward pass from this node) is never smaller than root ES - g.
inline int lbcc(Net& N, const Residual& R, int g) {
    std::vector<std::pair<int, int>> ev;   // (release time, activity)
    for (int j = 1; j <= N.n; ++j) {
        if (!R.live[j]) continue;
        ev.push_back({R.rem[j] >= 0 ? 0 : std::max(0, N.es[j] - g), j});
    }
    if (ev.empty()) return 0;
    std::sort(ev.begin(), ev.end());
    int best = 0;
    for (int k = 0; k < N.K; ++k) {
        if (N.cap[k] <= 0) continue;
        long long tank = 0; int t = 0;
        for (const auto& [rt, j] : ev) {
            tank -= (long long)(rt - t) * N.cap[k];
            if (tank < 0) tank = 0;
            t = rt;
            tank += (long long)N.dem[k][j] * (R.rem[j] >= 0 ? R.rem[j] : N.dur[j]);
        }
        best = std::max(best, t + (int)((tank + N.cap[k] - 1) / N.cap[k]));
    }
    return best;
}

// ── LBER (enhanced energetic reasoning) ──────────────────────────────────────
// Destructive bound: for a trial makespan C every activity j gets a window [r_j, d_j] (d_j = C - tail[j]); if some
// resource needs more work inside some interval [t1, t2) than capacity x (t2 - t1), C is impossible and the bound
// is at least C + 1. The intervals are the O(n^2) pairs from O1 = {r_j, d_j - p_j} x O2 = {d_j, r_j + p_j}.
// Last year (TT2, root form, depth 3): 477 / 242 / not run vs the base 476 / 241, with MORE expansions on j30
// (892 k vs 643 k mean on solved instances); on TT the per-node form at depth >= 3 was inadmissible until the
// shaving windows stopped being carried out of the root (their fix 94b2967, kept here).
struct ErScratch { std::vector<int> P, Rr, D; std::vector<int> o1, o2; std::vector<long long> knap; };
inline ErScratch& er() { static ErScratch s; return s; }

// mandatory work of j inside [t1, t2): min of its left-shifted and right-shifted overlap
inline long long erWork(int b, int p, int r, int d, int t1, int t2) {
    if (!b) return 0;
    const int span = t2 - t1;
    const int w = std::min(std::min(std::min(span, p), std::max(0, r + p - t1)), std::max(0, t2 - d + p));
    return w > 0 ? (long long)b * w : 0;
}
// Fekete-Schepers dual feasible function f2_s (s = 1, 2): f(B) = B, never above the exact form, so the test stays valid
inline long long dff2(long long x, long long B, int s) {
    if (x <= 0 || B <= 0) return 0;
    const long long num = x * (s + 1);
    return num % B == 0 ? x : (num / B) * (B / s);
}
inline long long knapMax(const std::vector<std::pair<int, long long>>& items, int cap) {
    if (cap <= 0) return 0;
    auto& dp = er().knap; dp.assign(cap + 1, 0);
    for (const auto& [w, v] : items) {
        if (w <= 0 || w > cap || v <= 0) continue;
        for (int c = cap; c >= w; --c) dp[c] = std::max(dp[c], dp[c - w] + v);
    }
    return dp[cap];
}
// RER (depth 4): relaxed revisited energetic reasoning for resource k on [t1, t2) - activities that can sit either
// left, right or inside are counted inside and two knapsacks take back the most work the left / right sides can hold.
inline bool rerOverload(const Net& N, const std::vector<int>& ids, int t1, int t2, int k) {
    auto& S = er();
    const long long span = t2 - t1;
    long long a0 = 0;
    for (int j : ids) {
        const int b = N.dem[k][j]; if (!b) continue;
        const int r = S.Rr[j], d = S.D[j], p = S.P[j];
        if (!((r + p > t1) && (d - p < t2))) continue;
        if ((r + p >= t2) && (d - p <= t1)) a0 += b;
    }
    const long long mk = (long long)N.cap[k] - a0;
    if (mk < 0) return true;
    long long base = 0, sumL = 0, sumR = 0;
    std::vector<std::pair<int, long long>> L, Rt;
    for (int j : ids) {
        const int b = N.dem[k][j]; if (!b) continue;
        const int r = S.Rr[j], d = S.D[j], p = S.P[j];
        if (!((r + p > t1) && (d - p < t2))) continue;
        if ((r + p >= t2) && (d - p <= t1)) continue;
        const long long Wi = (long long)b * p;
        const long long Wl = (long long)b * std::min(std::min((int)span, p), std::max(0, r + p - t1));
        const long long Wr = (long long)b * std::min(std::min((int)span, p), std::max(0, t2 - d + p));
        const bool Lonly = (r + p < t2) && (d - p <= t1), Ronly = (r + p >= t2) && (d - p > t1), Ionly = (t1 < r) && (d < t2);
        if (Lonly) base += Wl;
        else if (Ronly) base += Wr;
        else if (Ionly) base += Wi;
        else {
            base += Wi;
            const bool aLI = (r <= t1) && (d < t2) && (d - p > t1);
            const bool aRI = (r > t1) && (d >= t2) && (r + p < t2);
            const bool both = (r <= t1) && (d >= t2) && (r + p < t2) && (d - p > t1);
            if (aLI || both) { L.push_back({b, Wi - Wl}); sumL += b; }
            if (aRI || both) { Rt.push_back({b, Wi - Wr}); sumR += b; }
        }
    }
    const long long E = knapMax(L, (int)std::min(sumL, mk)) + knapMax(Rt, (int)std::min(sumR, mk));
    return base - E > mk * span;
}
// The overload test on the current windows (S.Rr, S.D, S.P): true = some resource is overloaded = C impossible.
inline bool erOverload(const Net& N, const std::vector<int>& ids, int depth) {
    auto& S = er();
    S.o1.clear(); S.o2.clear();
    for (int j : ids) { S.o1.push_back(S.Rr[j]); S.o1.push_back(S.D[j] - S.P[j]); S.o2.push_back(S.D[j]); S.o2.push_back(S.Rr[j] + S.P[j]); }
    std::sort(S.o1.begin(), S.o1.end()); S.o1.erase(std::unique(S.o1.begin(), S.o1.end()), S.o1.end());
    std::sort(S.o2.begin(), S.o2.end()); S.o2.erase(std::unique(S.o2.begin(), S.o2.end()), S.o2.end());
    for (int t1 : S.o1) for (int t2 : S.o2) {
        if (t2 <= t1) continue;
        const long long span = t2 - t1;
        for (int k = 0; k < N.K; ++k) {
            const long long avail = (long long)N.cap[k] * span;
            long long w = 0;
            for (int j : ids) {
                w += erWork(N.dem[k][j], S.P[j], S.Rr[j], S.D[j], t1, t2);
                if (w > avail) return true;                                   // depth 1: CER
            }
            if (depth >= 2)                                                   // depth 2: DFF on the reduced instance
                for (int s = 1; s <= 2; ++s) {
                    long long fw = 0;
                    for (int j : ids) {
                        const int b = N.dem[k][j]; if (!b) continue;
                        const int pr = (int)(erWork(1, S.P[j], S.Rr[j], S.D[j], t1, t2));
                        if (pr) fw += dff2(b, N.cap[k], s) * pr;
                    }
                    if (fw > avail) return true;
                }
            if (depth >= 4 && rerOverload(N, ids, t1, t2, k)) return true;  // depth 4: RER
        }
    }
    return false;
}

// Root value at depth 1..4: start at the critical path, raise C while the test (and at depth >= 3 the shaving)
// refutes it. Shaving windows are conditional on the trial C, so they are reset for every C and never kept.
inline int lberRoot(Net& N, int depth) {
    depth = std::max(1, std::min(4, depth));
    if (N.lber[depth] >= 0) return N.lber[depth];
    auto& S = er(); S.P.assign(N.n + 1, 0); S.Rr.assign(N.n + 1, 0); S.D.assign(N.n + 1, 0);
    std::vector<int> ids; for (int j = 1; j <= N.n; ++j) ids.push_back(j);
    long long sumP = 0; int C = 0;
    for (int j : ids) { S.P[j] = N.dur[j]; sumP += N.dur[j]; C = std::max(C, N.es[j] + N.dur[j] + N.tail[j]); }
    const int cMax = std::max((int)sumP, C);
    const auto out = std::chrono::steady_clock::now() + std::chrono::seconds(std::max(1LL, timeoutSec() / 10));
    const int shaveDepth = std::min(depth, 4);
    while (C < cMax && std::chrono::steady_clock::now() < out) {
        for (int j : ids) { S.Rr[j] = N.es[j]; S.D[j] = C - N.tail[j]; }
        if (erOverload(N, ids, depth)) { ++C; continue; }
        if (depth < 3) break;
        bool changed = true, refuted = false;                          // dichotomous shaving to a fixpoint
        while (changed && !refuted) {
            changed = false;
            for (int j : ids) {
                const int lo = S.Rr[j], hi = S.D[j] - S.P[j];
                if (hi <= lo) continue;
                const int M = (lo + hi + 1) / 2, sr = S.Rr[j], sd = S.D[j];
                S.D[j] = M + S.P[j] - 1; const bool leftBad = erOverload(N, ids, shaveDepth); S.D[j] = sd;
                S.Rr[j] = M;             const bool rightBad = erOverload(N, ids, shaveDepth); S.Rr[j] = sr;
                if (leftBad && rightBad) { refuted = true; break; }
                if (leftBad) { S.Rr[j] = M; changed = true; }
                else if (rightBad) { S.D[j] = M + S.P[j] - 1; changed = true; }
            }
        }
        if (refuted) { ++C; continue; }
        break;
    }
    return N.lber[depth] = C;   // every C below it was refuted (stopping on the time cap only makes it weaker)
}

// Per-node form: the same destructive test on the residual problem (origin = this node), depth 1, 2 or 4.
inline int lberNode(Net& N, const Residual& R, int g, int depth) {
    depth = std::max(1, std::min(4, depth));
    auto& S = er(); S.P.resize(N.n + 1); S.Rr.resize(N.n + 1); S.D.resize(N.n + 1);
    std::vector<int> ids, EF(N.n + 1, -1);
    long long sumP = 0; int cInit = 0;
    for (int j : N.topo) {
        if (!R.live[j]) continue;
        ids.push_back(j);
        int s = 0;
        for (int p : RCPSPex.backword_dependencies[j - 1]) if (EF[p] > s) s = EF[p];
        const int p = R.rem[j] >= 0 ? R.rem[j] : N.dur[j];
        if (R.rem[j] >= 0) s = 0;
        EF[j] = s + p; S.P[j] = p;
        S.Rr[j] = R.rem[j] >= 0 ? 0 : std::max(s, N.es[j] - g);
        sumP += p;
        cInit = std::max(cInit, EF[j] + N.tail[j]);
    }
    if (ids.size() < 2) return cInit;
    auto refuted = [&](int C) { for (int j : ids) S.D[j] = C - N.tail[j]; return erOverload(N, ids, depth); };
    if (!refuted(cInit)) return cInit;
    int lo = cInit + 1, hi = std::max((int)sumP, cInit);
    while (lo < hi) { const int mid = lo + (hi - lo) / 2; if (refuted(mid)) lo = mid + 1; else hi = mid; }
    return lo;   // lo - 1 was refuted, so the bound holds even if the test is not monotone in C
}

// ── Entry point from RCPSP_TT2::HCost ────────────────────────────────────────
inline double raise(double base, const std::vector<short>& unfinished, const std::vector<std::pair<short, short>>& active, int g) {
    const auto t0 = std::chrono::steady_clock::now();
    auto& s = st(); ++s.calls;
    Net& N = net();
    auto lift = [&](int v, long& counter) { if (v > base) { base = v; ++counter; } };
    if (g_tt2_lbrc)      { const int r = lbrcRoot(N);  s.rootRc = r;  lift(r - g, s.rcBetter); }
    if (g_tt2_lbip0)     { const int r = lbip0Root(N); s.rootIp0 = r; lift(r - g, s.ip0Better); }
    if (g_tt2_lber > 0)  { const int r = lberRoot(N, (int)g_tt2_lber); s.rootLber = r; lift(r - g, s.lberBetter); }
    if (g_tt2_lbcs || g_tt2_lbcc || g_tt2_lber_node > 0) {
        Residual R; R.build(N.n, unfinished, active);
        if (g_tt2_lbcs) lift(lbcs(N, R), s.lbcsBetter);
        if (g_tt2_lbcc) lift(lbcc(N, R, g), s.lbccBetter);
        if (g_tt2_lber_node > 0) lift(lberNode(N, R, g, (int)g_tt2_lber_node), s.lberNodeBetter);
    }
    s.sec += secs_since(t0);
    return base;
}

inline const char* csvColumns() {
    return "tt2LBCS,tt2LBCC,tt2LBIP0,tt2LBRC,tt2LBER,tt2LBERNode,tt2Reopen,lyCalls,lbcsBetter,lbccBetter,lbip0Better,"
           "lbrcBetter,lberBetter,lberNodeBetter,lySec,lbip0Root,lbrcRoot,lberRoot,";
}
inline void writeCsv(std::ostream& o) {
    const auto& s = st();
    o << g_tt2_lbcs << "," << g_tt2_lbcc << "," << g_tt2_lbip0 << "," << g_tt2_lbrc << "," << g_tt2_lber << ","
      << g_tt2_lber_node << "," << tt2ReopenOn() << "," << s.calls << "," << s.lbcsBetter << "," << s.lbccBetter << ","
      << s.ip0Better << "," << s.rcBetter << "," << s.lberBetter << "," << s.lberNodeBetter << "," << s.sec << ","
      << s.rootIp0 << "," << s.rootRc << "," << s.rootLber << ",";
}

}  // namespace ly
