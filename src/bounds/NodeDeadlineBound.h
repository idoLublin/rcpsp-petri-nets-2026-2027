#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// RCPSP_TT2_TT=1 — per-node deadline reasoning for the Petri-net search (forward, backward and HBAE).
// At a node we know the unfinished activities, the ones in progress with their REMAINING time, and the elapsed
// time g. The remaining problem is therefore a smaller RCPSP starting now:
//
//   duration   d[i] = remaining time            (in progress)      |  p[i]  (not started)
//   earliest   es[i] = 0                        (in progress)      |  max over unfinished predecessors q of es[q] + d[q]
//   latest     ls[i] = D - (tail[i] - p[i] + d[i])
//
// Ask: can the remaining work finish within D? Compulsory parts [ls, es+d) that overload a resource, or a window
// that empties, prove it cannot — so the remaining makespan is > D and h can be raised to D+1. Every raise is a
// proof, so h stays admissible; failure to refute leaves h untouched.
//
// Cost control: at most RCPSP_TT2_TT_STEPS deadlines per call (the loop stops at the first deadline it cannot
// refute), the cheap compulsory-part test before the window tightening (RCPSP_TT2_TT_FULL), and only nodes with
// at least RCPSP_TT2_TT_MINACT unfinished activities.
//
// Optional extra proof steps, tried after the compulsory-part test: energetic reasoning (RCPSP_TT2_TT_ENERGY,
// NodeEnergetic.h) and its cheap suffix form (RCPSP_TT2_TT_SER, archive/SuffixEnergetic.h).
// ─────────────────────────────────────────────────────────────────────────────
#include <algorithm>
#include <chrono>
#include <vector>
#include "NodeEnergetic.h"
#include "Propagation.h"
#include "archive/SuffixEnergetic.h"

namespace tt2nb {

struct Stats { long calls = 0, raised = 0, steps = 0, tt0Fail = 0, propFail = 0, energyFail = 0, serFail = 0; double sec = 0.0; long raiseSum = 0; };
inline Stats& st() { static Stats s; return s; }

// HBAE alternates between the forward and the reversed net within one solve, so keep a small cache of built
// instances keyed by fingerprint instead of rebuilding on every side switch. A cached Inst of a DIFFERENT
// problem would make the bound inadmissible, hence the fingerprint check.
struct Ctx { deadline::Inst I; bool built = false; uint64_t fp = 0; };
inline Ctx& ctx() { static Ctx c; return c; }
inline std::vector<std::pair<uint64_t, deadline::Inst>>& cache() { static std::vector<std::pair<uint64_t, deadline::Inst>> c; return c; }
inline void reset() { ctx().built = false; ctx().fp = 0; cache().clear(); st() = Stats{}; }
inline void build() {                       // fingerprint -> Inst, at most 4 kept (forward + reversed + slack)
    const uint64_t fp = deadline::instFp();
    for (auto& e : cache()) if (e.first == fp) { ctx().I = e.second; ctx().built = true; ctx().fp = fp; return; }
    deadline::Inst I = deadline::build();
    if (cache().size() >= 4) cache().erase(cache().begin());
    cache().emplace_back(fp, I);
    ctx().I = std::move(I); ctx().built = true; ctx().fp = fp;
}
// The loaded instance in array form, rebuilt when the loaded instance changed (also used by the other node bounds).
inline const deadline::Inst& inst() {
    if (!ctx().built || ctx().fp != deadline::instFp()) build();
    return ctx().I;
}

// scratch, reused across calls
struct Buf {
    std::vector<int> d, es, ls, order, pts;
    std::vector<char> live;
    std::vector<std::vector<int>> prof;
};
inline Buf& buf() { static Buf b; return b; }

// true = D is refuted: the remaining work cannot finish within D
inline bool refuted(int D) {
    const deadline::Inst& I = ctx().I;
    Buf& b = buf();
    if (D < 0) return true;
    const int H = D + 2;
    // ── windows ────────────────────────────────────────────────────────────
    for (int i = 0; i < I.n; ++i) {
        if (!b.live[i]) continue;
        b.ls[i] = D - (I.tail[i] - I.p[i] + b.d[i]);
        if (b.es[i] > b.ls[i]) return true;
    }
    // ── compulsory parts (cheap test) ──────────────────────────────────────
    auto& prof = b.prof;
    if ((int)prof.size() < I.R) prof.resize(I.R);
    for (int k = 0; k < I.R; ++k) { if ((int)prof[k].size() < H) prof[k].resize(H); std::fill(prof[k].begin(), prof[k].begin() + H, 0); }
    for (int i = 0; i < I.n; ++i) {
        if (!b.live[i] || b.d[i] <= 0) continue;
        const int a = b.ls[i], e = b.es[i] + b.d[i];
        if (a < e) for (int k : I.resOf[i]) for (int t = a; t < e && t <= D; ++t) prof[k][t] += I.dem[k][i];
    }
    for (int k = 0; k < I.R; ++k)
        for (int t = 0; t <= D; ++t)
            if (prof[k][t] > I.cap[k]) { ++st().tt0Fail; return true; }
    // ── optional proof steps ───────────────────────────────────────────────
    if (g_tt2_node_tt_ser && suffixEnergeticRefutes(I, D, b.live, b.d, b.es, b.pts)) { ++st().serFail; return true; }
    if (g_tt2_node_tt_energy && energeticRefutes(I, D, b.live, b.d, b.es, b.ls, b.pts)) { ++st().energyFail; return true; }
    if (!g_tt2_node_tt_full) return false;
    // ── window tightening against the forced demand ────────────────────────
    for (int pass = 0; pass < 3; ++pass) {
        bool changed = false;
        for (int i = 0; i < I.n; ++i) {
            if (!b.live[i] || b.d[i] <= 0 || I.resOf[i].empty()) continue;
            const int a = b.ls[i], e = b.es[i] + b.d[i];
            int t = b.es[i];
            while (t <= b.ls[i]) {                       // push es past any placement covering a blocked point
                int last = -1;
                for (int u = t; u < t + b.d[i] && u <= D; ++u) {
                    bool bad = false;
                    for (int k : I.resOf[i]) {
                        const int other = prof[k][u] - ((a < e && u >= a && u < e) ? I.dem[k][i] : 0);
                        if (other + I.dem[k][i] > I.cap[k]) { bad = true; break; }
                    }
                    if (bad) last = u;
                }
                if (last >= 0) t = last + 1; else break;
            }
            if (t > b.ls[i]) { ++st().propFail; return true; }
            if (t > b.es[i]) { b.es[i] = t; changed = true; }
        }
        if (!changed) break;
        // successors inherit the pushed earliest starts
        for (int idx = 0; idx < (int)b.order.size(); ++idx) {
            const int i = b.order[idx];
            for (int s : I.succ[i]) if (b.live[s]) b.es[s] = std::max(b.es[s], b.es[i] + b.d[i]);
        }
        for (int i = 0; i < I.n; ++i) if (b.live[i] && b.es[i] > b.ls[i]) { ++st().propFail; return true; }
    }
    return false;
}

// h0 = the admissible heuristic we already have (remaining makespan). Returns a value >= h0, still admissible.
// unfinished: 1-based activity ids; active: (1-based id, remaining time) for the ones in progress.
inline int raise(const std::vector<short>& unfinished, const std::vector<std::pair<short, short>>& active, int h0) {
    if (!g_tt2_node_tt) return h0;
    if (!ctx().built || ctx().fp != deadline::instFp()) build();
    const deadline::Inst& I = ctx().I;
    if (I.n <= 0) return h0;
    if ((long)unfinished.size() < g_tt2_node_tt_minact) return h0;
    const auto t0 = std::chrono::steady_clock::now();
    ++st().calls;
    Buf& b = buf();
    b.d.assign(I.n, 0); b.es.assign(I.n, 0); b.ls.assign(I.n, 0); b.live.assign(I.n, 0);
    for (const auto& u : unfinished) {
        const int i = (int)u - 1;
        if (i < 0 || i >= I.n) continue;
        b.live[i] = 1; b.d[i] = I.p[i];
    }
    for (const auto& a : active) {
        const int i = (int)a.first - 1;
        if (i < 0 || i >= I.n || !b.live[i]) continue;
        b.d[i] = std::max(0, (int)a.second);            // only the remaining time is still to be done
    }
    // earliest starts: topological forward pass over the live sub-DAG (in-progress ones start now)
    b.order.clear();
    for (int idx = 0; idx < I.n; ++idx) {
        const int i = I.topo[idx];
        if (!b.live[i]) continue;
        b.order.push_back(i);
        for (int q : I.preds[i]) if (b.live[q]) b.es[i] = std::max(b.es[i], b.es[q] + b.d[q]);
    }
    for (const auto& a : active) {                       // in progress => already running, cannot be pushed
        const int i = (int)a.first - 1;
        if (i >= 0 && i < I.n && b.live[i]) b.es[i] = 0;
    }
    int h = h0;
    for (long s = 0; s < g_tt2_node_tt_steps; ++s) {
        const std::vector<int> es0 = b.es;               // refuted() tightens es; restore for the next deadline
        ++st().steps;
        if (refuted(h)) { ++h; b.es = es0; } else { b.es = es0; break; }
    }
    st().sec += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (h > h0) { ++st().raised; st().raiseSum += (h - h0); }
    return h;
}

}  // namespace tt2nb
