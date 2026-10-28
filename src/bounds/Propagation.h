#pragma once
// Deadline reasoning, the CP side of the solver: given a deadline D, every activity gets a window
// [earliest start, latest start = D - tail], and propagation shrinks the windows until a fixpoint or until one
// empties, which proves that no schedule ends by D. Used by:
//   bounds/NodeDeadlineBound.h   raise h at a search node (RCPSP_TT2_TT)
//   bidirectional/Prover.h       prove an HBAE incumbent optimal (RCPSP_BI_REFUTE)

#include <algorithm>
#include <chrono>
#include <vector>
#include "../common/Globals.h"

namespace deadline {

// The loaded instance in array form (0-based activities).
struct Inst {
    int n = 0, R = 0, cpm = 0, energy = 0, horizon = 0, maxp = 0;
    std::vector<int> p, cap, topo, es0, tail;              // es0 = earliest starts, tail = longest path to the sink incl. p
    std::vector<std::vector<int>> dem, preds, succ, resOf; // dem[k][i]; resOf[i] = resources with demand
};

inline Inst build() {
    Inst I;
    I.n = (int)RCPSPex.activities.size();
    I.R = (int)RCPSPex.resources.size();
    I.p.resize(I.n); I.cap.resize(I.R);
    I.dem.assign(I.R, std::vector<int>(I.n, 0));
    I.resOf.assign(I.n, {});
    for (int k = 0; k < I.R; ++k) I.cap[k] = RCPSPex.resources[k].second;
    for (int i = 0; i < I.n; ++i) {
        I.p[i] = RCPSPex.activities[i].duration;
        I.maxp = std::max(I.maxp, I.p[i]);
        for (const auto& d : RCPSPex.activities[i].resource_demands)
            for (int k = 0; k < I.R; ++k)
                if (RCPSPex.resources[k].first == d.first) I.dem[k][i] += d.second;
        for (int k = 0; k < I.R; ++k) if (I.dem[k][i] > 0) I.resOf[i].push_back(k);
    }
    I.preds.assign(I.n, {}); I.succ.assign(I.n, {});
    for (int i = 0; i < I.n; ++i)
        for (short q : RCPSPex.backword_dependencies[i]) { I.preds[i].push_back(q - 1); I.succ[q - 1].push_back(i); }
    // topological order (works for the reversed instance too, where ids are not topological)
    std::vector<int> indeg(I.n, 0);
    for (int i = 0; i < I.n; ++i) indeg[i] = (int)I.preds[i].size();
    for (int i = 0; i < I.n; ++i) if (indeg[i] == 0) I.topo.push_back(i);
    for (size_t h = 0; h < I.topo.size(); ++h)
        for (int s : I.succ[I.topo[h]]) if (--indeg[s] == 0) I.topo.push_back(s);
    I.es0.assign(I.n, 0); I.tail.assign(I.n, 0);
    for (int i : I.topo) for (int q : I.preds[i]) I.es0[i] = std::max(I.es0[i], I.es0[q] + I.p[q]);
    for (int h = I.n - 1; h >= 0; --h) {
        const int i = I.topo[h]; int m = 0;
        for (int s : I.succ[i]) m = std::max(m, I.tail[s]);
        I.tail[i] = I.p[i] + m;
    }
    for (int i = 0; i < I.n; ++i) { I.cpm = std::max(I.cpm, I.es0[i] + I.p[i]); I.horizon += std::max(1, I.p[i]); }
    for (int k = 0; k < I.R; ++k) {
        long e = 0; for (int i = 0; i < I.n; ++i) e += (long)I.p[i] * I.dem[k][i];
        if (I.cap[k] > 0) I.energy = std::max(I.energy, (int)((e + I.cap[k] - 1) / I.cap[k]));
    }
    return I;
}

// Fingerprint of the instance currently in RCPSPex. The bidirectional search swaps in the reversed instance,
// so a cached Inst must be checked against it before use.
inline unsigned long long instFp() {
    unsigned long long h = 1469598103934665603ULL;
    auto mix = [&h](unsigned long long v) { h = (h ^ v) * 1099511628211ULL; };
    mix(RCPSPex.activities.size()); mix(RCPSPex.resources.size());
    for (size_t i = 0; i < RCPSPex.activities.size(); ++i) {
        mix((unsigned long long)RCPSPex.activities[i].duration);
        mix((unsigned long long)RCPSPex.backword_dependencies[i].size());
    }
    for (const auto& r : RCPSPex.resources) mix((unsigned long long)r.second);
    return h;
}

// Precedence + time-table (compulsory-part) propagation on the windows [es, ls] for deadline D, to a fixpoint
// (at most 60 rounds; stopping early leaves valid, only looser windows). false = some window emptied: D refuted.
inline bool propagate(const Inst& I, int D, std::vector<int>& es, std::vector<int>& ls) {
    static std::vector<std::vector<int>> prof;
    static std::vector<char> bad;
    const int H = D + 2;
    if ((int)prof.size() < I.R) prof.resize(I.R);
    for (int k = 0; k < I.R; ++k) if ((int)prof[k].size() < H) prof[k].resize(H);
    if ((int)bad.size() < H) bad.resize(H);
    for (int iter = 0; iter < 60; ++iter) {
        bool changed = false;
        for (int i : I.topo) {                                   // precedence, forward
            int v = es[i];
            for (int q : I.preds[i]) v = std::max(v, es[q] + I.p[q]);
            if (v > es[i]) { es[i] = v; changed = true; }
        }
        for (int h = I.n - 1; h >= 0; --h) {                     // precedence, backward
            const int i = I.topo[h];
            int v = std::min(ls[i], D - I.p[i]);
            for (int s : I.succ[i]) v = std::min(v, ls[s] - I.p[i]);
            if (v < ls[i]) { ls[i] = v; changed = true; }
        }
        for (int i = 0; i < I.n; ++i) if (es[i] > ls[i] || es[i] < 0) return false;
        for (int k = 0; k < I.R; ++k) std::fill(prof[k].begin(), prof[k].begin() + H, 0);
        for (int i = 0; i < I.n; ++i) {                          // resource profile of the compulsory parts [ls, es + p)
            if (I.p[i] == 0) continue;
            const int a = ls[i], b = es[i] + I.p[i];
            if (a < b) for (int k : I.resOf[i]) for (int t = a; t < b; ++t) prof[k][t] += I.dem[k][i];
        }
        for (int k = 0; k < I.R; ++k) for (int t = 0; t <= D; ++t) if (prof[k][t] > I.cap[k]) return false;
        for (int i = 0; i < I.n; ++i) {                          // time-table filtering of every window
            if (I.p[i] == 0 || I.resOf[i].empty()) continue;
            const int a = ls[i], b = es[i] + I.p[i];
            const int lo = es[i], hi = ls[i] + I.p[i];
            // bad[t]: placing i over t would overload a resource, counting the others' compulsory parts only
            for (int t = lo; t < hi; ++t) {
                bool bd = false;
                for (int k : I.resOf[i]) {
                    int other = prof[k][t] - ((a < b && t >= a && t < b) ? I.dem[k][i] : 0);
                    if (other + I.dem[k][i] > I.cap[k]) { bd = true; break; }
                }
                bad[t] = bd;
            }
            int t = es[i];
            while (t <= ls[i]) {                                 // push es past any placement covering a bad point
                int last = -1;
                for (int u = t; u < t + I.p[i]; ++u) if (bad[u]) last = u;
                if (last >= 0) t = last + 1; else break;
            }
            if (t > ls[i]) return false;
            if (t > es[i]) { es[i] = t; changed = true; }
            t = ls[i];
            while (t >= es[i]) {                                 // and pull ls back the same way
                int first = -1;
                for (int u = t; u < t + I.p[i]; ++u) if (bad[u]) { first = u; break; }
                if (first >= 0) t = first - I.p[i]; else break;
            }
            if (t < es[i]) return false;
            if (t < ls[i]) { ls[i] = t; changed = true; }
        }
        if (!changed) break;
    }
    return true;
}

// Energetic-reasoning check on the intervals [t1, t2) between window ends; false = overload (D refuted).
inline bool energeticOk(const Inst& I, const std::vector<int>& es, const std::vector<int>& ls) {
    std::vector<int> T1, T2;
    for (int i = 0; i < I.n; ++i) if (I.p[i] > 0) { T1.push_back(es[i]); T1.push_back(ls[i]); T2.push_back(es[i] + I.p[i]); T2.push_back(ls[i] + I.p[i]); }
    std::sort(T1.begin(), T1.end()); T1.erase(std::unique(T1.begin(), T1.end()), T1.end());
    std::sort(T2.begin(), T2.end()); T2.erase(std::unique(T2.begin(), T2.end()), T2.end());
    for (int t1 : T1)
        for (int t2 : T2) {
            if (t2 <= t1) continue;
            const int L = t2 - t1;
            for (int k = 0; k < I.R; ++k) {
                long W = 0; const long capL = (long)I.cap[k] * L;
                for (int i = 0; i < I.n; ++i) {
                    const int d = I.dem[k][i];
                    if (d == 0 || I.p[i] == 0) continue;
                    int ov = std::min({L, I.p[i], es[i] + I.p[i] - t1, t2 - ls[i]});
                    if (ov > 0) { W += (long)d * ov; if (W > capL) return false; }
                }
            }
        }
    return true;
}

// The root of a refutation: propagation, the energetic check, and one level of shaving (an activity whose
// earliest / latest start makes propagation fail cannot start there, so its window shrinks).
// 1 = D refuted, 0 = not refuted (es / ls = the windows after shaving), -1 = time budget hit.
inline int refutedWin(const Inst& I, int D, std::chrono::steady_clock::time_point until, std::vector<int>& es, std::vector<int>& ls) {
    es = I.es0; ls.assign(I.n, 0);
    for (int i = 0; i < I.n; ++i) ls[i] = D - I.tail[i];
    if (!propagate(I, D, es, ls)) return 1;
    if (!energeticOk(I, es, ls)) return 1;
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < I.n; ++i) {
            if (I.p[i] == 0) continue;
            if (std::chrono::steady_clock::now() > until) return -1;
            while (es[i] < ls[i]) {                      // can i start at its earliest start?
                std::vector<int> e2 = es, l2 = ls; l2[i] = es[i];
                if (!propagate(I, D, e2, l2)) { ++es[i]; changed = true; } else break;
            }
            while (ls[i] > es[i]) {                      // can i start at its latest start?
                std::vector<int> e2 = es, l2 = ls; e2[i] = ls[i];
                if (!propagate(I, D, e2, l2)) { --ls[i]; changed = true; } else break;
            }
            if (changed && !propagate(I, D, es, ls)) return 1;
        }
    }
    return 0;
}

// Is `start` (0-based, per activity) a precedence- and resource-feasible schedule ending by mk?
inline bool feasible(const Inst& I, const std::vector<int>& start, int mk) {
    for (int i = 0; i < I.n; ++i) {
        if (start[i] < 0 || start[i] + I.p[i] > mk) return false;
        for (int q : I.preds[i]) if (start[q] + I.p[q] > start[i]) return false;
    }
    for (int k = 0; k < I.R; ++k) {
        std::vector<int> u(mk + 1, 0);
        for (int i = 0; i < I.n; ++i) if (I.dem[k][i]) for (int t = start[i]; t < start[i] + I.p[i]; ++t) { u[t] += I.dem[k][i]; if (u[t] > I.cap[k]) return false; }
    }
    return true;
}

}  // namespace deadline
