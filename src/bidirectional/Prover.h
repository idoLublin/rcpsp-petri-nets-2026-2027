#pragma once
// ── Optimality prover for HBAE (RCPSP_BI_REFUTE=1) ──────────────────────────────────────────────────────────
// When the bidirectional search finds a new incumbent U, try to prove that no schedule ends by D = U - 1. A
// complete depth-first search over start-time splits, failure-directed in the sense of Vilim, Laborie & Shaw
// (CPAIOR 2015): a choice (j, v) splits activity j's window into "s_j <= v" and "s_j > v"; each choice keeps a
// rating per branch, a moving average of 0 (propagation failed) or 1 + the fraction of the search space the branch
// kept, and a node branches on the open choice with the lowest rating sum (the one most likely to fail on both
// sides), the lower-rated branch first. Choices come from a fixed grid over each activity's root window (so
// ratings carry across nodes and restarts) and, where no grid pivot splits a window any more, from its midpoint.
// Node propagation: precedence + time-table (bounds/Propagation.h). The root of the search is the shaved fixpoint
// of refutedWin. Restarts with a growing fail limit keep the ratings, and at each restart the explored part of the
// tree is recorded as nogoods (Lecoutre et al. 2007). A run that ends without reaching its limit has explored the
// whole tree, so "refuted" is a proof. A leaf where every window is a single value passed propagation, so it is a
// schedule ending by D (checked with feasible()).

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include "../bounds/Propagation.h"

namespace deadline {

struct Refuter {
    static constexpr int kGrid = 7;            // pivots per window
    static constexpr double kAlpha = 0.15;     // rating step
    static constexpr double kInit = 1.0;       // initial rating
    static constexpr long kLim0 = 128;         // first fail limit
    static constexpr double kGrow = 1.5;       // fail-limit growth per restart

    struct Choice { int j, v; double rl, rr; };
    struct Lit { int j, v; bool le; };   // le: s_j <= v, else s_j >= v + 1
    const Inst& I; const int D; const std::chrono::steady_clock::time_point until;
    std::vector<Choice> ch;
    std::vector<std::vector<Lit>> ng;     // nogoods
    std::vector<Lit> path; std::vector<char> pathFirst;
    std::vector<std::vector<int>> chOf;   // choices of each activity
    std::mt19937 rng;
    long nodes = 0, fails = 0, failLimit = 0, restarts = 0;
    bool stopped = false, timeUp = false;
    std::vector<int> sol;
    Refuter(const Inst& i, int d, std::chrono::steady_clock::time_point u) : I(i), D(d), until(u), chOf(i.n), rng(4243u + (unsigned)d) {}

    void makeGrid(const std::vector<int>& es, const std::vector<int>& ls) {
        for (int j = 0; j < I.n; ++j) {
            const int w = ls[j] - es[j];
            if (w <= 0) continue;
            const int k = std::min(w, kGrid);
            int last = INT_MIN;
            for (int q = 1; q <= k; ++q) {
                const int v = es[j] + (int)((long)q * w / (k + 1));   // es <= v < ls
                if (v == last || v >= ls[j]) continue;
                last = v; chOf[j].push_back((int)ch.size()); ch.push_back({j, v, kInit, kInit});
            }
        }
    }
    // move one branch rating toward the local value (0 = the branch failed, else 1 + the kept search-space fraction)
    static void rate(Choice& x, bool left, double local) {
        double& rt = left ? x.rl : x.rr;
        rt = (1.0 - kAlpha) * rt + kAlpha * local;
    }
    static double keptFraction(const std::vector<int>& es, const std::vector<int>& ls, const std::vector<int>& e2, const std::vector<int>& l2) {
        double lg = 0.0;
        for (size_t j = 0; j < es.size(); ++j)
            if (e2[j] != es[j] || l2[j] != ls[j]) lg += std::log((double)(l2[j] - e2[j] + 1) / (double)(ls[j] - es[j] + 1));
        return std::exp(lg);
    }
    // nogood unit rule to a fixpoint with propagate(); false = a nogood holds or propagation fails
    bool propNogoods(std::vector<int>& es, std::vector<int>& ls) {
        for (bool changed = true; changed;) {
            changed = false;
            for (const auto& g : ng) {
                int open = -1, nOpen = 0; bool sat = false;
                for (int q = 0; q < (int)g.size() && !sat && nOpen < 2; ++q) {
                    const Lit& L = g[q];
                    const bool tru = L.le ? ls[L.j] <= L.v : es[L.j] > L.v;
                    const bool fls = L.le ? es[L.j] > L.v : ls[L.j] <= L.v;
                    if (fls) sat = true; else if (!tru) { open = q; ++nOpen; }
                }
                if (sat || nOpen >= 2) continue;
                if (nOpen == 0) return false;                          // every literal holds
                const Lit& L = g[open];                                // the last one must be false
                if (L.le) es[L.j] = std::max(es[L.j], L.v + 1); else ls[L.j] = std::min(ls[L.j], L.v);
                if (es[L.j] > ls[L.j]) return false;
                changed = true;
            }
            if (changed && !propagate(I, D, es, ls)) return false;
        }
        return true;
    }
    bool check(std::vector<int>& es, std::vector<int>& ls) {
        if (!propagate(I, D, es, ls)) return false;
        if (!ng.empty() && !propNogoods(es, ls)) return false;
        return true;
    }
    // reduced nld-nogoods of the current path (called when the fail limit stops the search)
    void recordNogoods() {
        std::vector<Lit> pos;
        for (size_t i = 0; i < path.size(); ++i) {
            const Lit& L = path[i];
            if (pathFirst[i]) { pos.push_back(L); continue; }
            std::vector<Lit> g = pos; g.push_back({L.j, L.v, !L.le});   // the refuted first branch at depth i
            ng.push_back(std::move(g));
        }
    }
    // 1 = this subtree holds no schedule, 0 = schedule found (sol), -1 = stopped (fail limit or time)
    int dfs(const std::vector<int>& es, const std::vector<int>& ls) {
        if (stopped) return -1;
        if ((++nodes & 255) == 0 && std::chrono::steady_clock::now() > until) { stopped = timeUp = true; return -1; }
        int best = -1; double br = 1e300; int openJ = -1, openW = INT_MAX;
        std::uniform_real_distribution<double> jit(0.0, 1e-6);
        for (int j = 0; j < I.n; ++j) {
            if (es[j] >= ls[j]) continue;
            if (ls[j] - es[j] < openW) { openW = ls[j] - es[j]; openJ = j; }
            for (int c : chOf[j]) {
                const Choice& x = ch[c];
                if (x.v < es[j] || x.v >= ls[j]) continue;          // decided here
                const double r = x.rl + x.rr + jit(rng);
                if (r < br) { br = r; best = c; }
            }
        }
        if (openJ < 0) {                                           // every window is one value: a schedule
            if (!feasible(I, es, D)) { std::cerr << "[refute] propagation passed an infeasible leaf at D=" << D << std::endl; std::abort(); }
            sol = es; return 0;
        }
        if (best < 0) {                                            // no grid pivot left: split the narrowest window
            const int v = es[openJ] + (ls[openJ] - es[openJ] - 1) / 2;
            best = (int)ch.size(); ch.push_back({openJ, v, kInit, kInit}); chOf[openJ].push_back(best);
        }
        const int j = ch[best].j, v = ch[best].v;
        const bool leftFirst = ch[best].rl <= ch[best].rr;
        for (int side = 0; side < 2; ++side) {
            const bool left = (side == 0) == leftFirst;
            std::vector<int> e2 = es, l2 = ls;
            if (left) l2[j] = v; else e2[j] = v + 1;
            const bool ok = check(e2, l2);
            rate(ch[best], left, ok ? 1.0 + keptFraction(es, ls, e2, l2) : 0.0);
            if (!ok) {
                if (++fails > failLimit && failLimit > 0) { stopped = true; recordNogoods(); return -1; }
                continue;
            }
            path.push_back({j, v, left}); pathFirst.push_back(side == 0 ? 1 : 0);
            const int r = dfs(e2, l2);
            path.pop_back(); pathFirst.pop_back();
            if (r != 1) return r;
        }
        return 1;
    }
    // 1 refuted, 0 schedule found (sol), -1 time budget
    int run(const std::vector<int>& es0, const std::vector<int>& ls0) {
        makeGrid(es0, ls0);
        std::vector<int> es = es0, ls = ls0;
        for (long lim = kLim0;; lim = std::max(lim + 1, (long)(lim * kGrow))) {
            stopped = false; fails = 0; failLimit = lim;
            if (!ng.empty() && !propNogoods(es, ls)) return 1;         // the nogoods alone refute the root
            const int r = dfs(es, ls);
            if (r >= 0) return r;
            if (timeUp) return -1;
            ++restarts;
        }
    }
};

// One deadline: 1 refuted (no schedule ends by D), 0 a schedule ending by D (sol, mk), -1 budget.
inline int refuteDeadline(const Inst& I, int D, std::chrono::steady_clock::time_point until,
                          std::vector<int>& sol, int& mk, long& nodes) {
    std::vector<int> es, ls;
    const int r0 = refutedWin(I, D, until, es, ls);              // propagation + energetic + shaving at the root
    if (r0 == 1) return 1;
    if (r0 < 0) return -1;
    Refuter R(I, D, until);
    const int r = R.run(es, ls);
    nodes += R.nodes;
    if (r == 0) { sol = R.sol; mk = 0; for (int i = 0; i < I.n; ++i) mk = std::max(mk, sol[i] + I.p[i]); }
    return r;
}

}  // namespace deadline
