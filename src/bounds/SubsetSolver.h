#pragma once
// A small exact RCPSP solver for a sub-problem, returning a PROVEN lower bound on its minimum makespan. Used by the
// sub-problem bounds (SubProblemBounds.h).
//
// Search: A* over activity orderings; each activity is placed at its earliest precedence- and resource-feasible start.
// That enumerates exactly the active schedules, and an optimal schedule is always active, so A* with the admissible
// bound max(critical path, energy) is optimal. Inside, DR5 cutset dominance: of two partial schedules with the same
// scheduled set, one whose every start is no later dominates.
//
// Budget: `expandCap` expansions (<= 0 = no cap) and the time point `until`. A* expands in non-decreasing f, so when
// the budget runs out the f of the node in hand is the smallest f still open — a valid lower bound, just weaker than
// the optimum. The result says which happened (`capped`).
//
// Activities are LOCAL indices 0..n-1; preds[i] = i's predecessors inside the sub-problem.

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

struct SubsetInstance {
    int n = 0;                              // activities
    int R = 0;                              // resources
    std::vector<int> dur;                   // [n]
    std::vector<int> release;               // [n] earliest start
    std::vector<int> cap;                   // [R]
    std::vector<std::vector<int>> demand;   // [n][R]
    std::vector<std::vector<int>> preds;    // [n] -> local predecessor indices
};

struct SubsetResult {
    long lb      = 0;       // proven lower bound on the sub-problem's minimum makespan
    long expands = 0;       // expansions performed
    bool capped  = false;   // true = stopped on the budget (lb is a partial bound, not the optimum)
};

inline SubsetResult subsetRcpspLB(const SubsetInstance& P, long expandCap, std::chrono::steady_clock::time_point until) {
    const int n = P.n, R = P.R;
    if (n <= 0) return {0, 0, false};

    // successors and a topological order (Kahn)
    std::vector<std::vector<int>> succ(n);
    std::vector<int> indeg(n, 0);
    for (int i = 0; i < n; ++i)
        for (int p : P.preds[i]) { succ[p].push_back(i); indeg[i]++; }
    std::vector<int> topo;
    {
        std::vector<int> d = indeg;
        for (int i = 0; i < n; ++i) if (d[i] == 0) topo.push_back(i);
        for (size_t h = 0; h < topo.size(); ++h)
            for (int v : succ[topo[h]]) if (--d[v] == 0) topo.push_back(v);
    }

    // tail[i] = longest path of durations from i's start to the end
    std::vector<int> tail(n, 0);
    for (int idx = (int)topo.size() - 1; idx >= 0; --idx) {
        const int i = topo[idx]; int m = 0;
        for (int s : succ[i]) m = std::max(m, tail[s]);
        tail[i] = P.dur[i] + m;
    }

    // energy bound: per resource, ceil(total work / capacity); the same for every partial schedule
    long rcLB = 0;
    for (int r = 0; r < R; ++r) {
        if (P.cap[r] <= 0) continue;
        long e = 0; for (int i = 0; i < n; ++i) e += (long)P.demand[i][r] * (long)P.dur[i];
        rcLB = std::max(rcLB, (e + P.cap[r] - 1) / P.cap[r]);
    }

    // admissible makespan bound of a partial schedule: max(earliest starts + tails, energy)
    auto makespanLB = [&](const std::vector<short>& start) -> int {
        std::vector<int> est(n, 0);
        for (int i : topo) {
            int e = P.release[i];
            for (int p : P.preds[i]) e = std::max(e, est[p] + P.dur[p]);
            if (start[i] >= 0) e = start[i];
            est[i] = e;
        }
        long lb = rcLB;
        for (int i = 0; i < n; ++i) lb = std::max(lb, (long)est[i] + (long)tail[i]);
        return (int)lb;
    };

    // earliest precedence- and resource-feasible start of activity a, given the placed ones
    auto earliestFeasible = [&](int a, const std::vector<short>& start) -> int {
        int t = P.release[a];
        for (int p : P.preds[a]) t = std::max(t, (int)start[p] + P.dur[p]);
        while (true) {
            const int end = t + P.dur[a];
            bool ok = true;
            int nextDrop = INT_MAX;
            std::vector<int> checkT; checkT.push_back(t);   // the usage peak is at t or at a placed start
            for (int j = 0; j < n; ++j) if (start[j] >= 0) {
                const int sj = start[j], fj = start[j] + P.dur[j];
                if (sj > t && sj < end) checkT.push_back(sj);
                if (fj > t) nextDrop = std::min(nextDrop, fj);
            }
            for (int r = 0; r < R && ok; ++r) {
                if (P.demand[a][r] == 0) continue;
                for (int tau : checkT) {
                    int use = 0;
                    for (int j = 0; j < n; ++j)
                        if (start[j] >= 0 && start[j] <= tau && tau < start[j] + P.dur[j]) use += P.demand[j][r];
                    if (use + P.demand[a][r] > P.cap[r]) { ok = false; break; }
                }
            }
            if (ok) return t;
            t = (nextDrop == INT_MAX) ? t + 1 : nextDrop;
        }
    };

    struct Node { std::vector<short> start; int f; int depth; };
    struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };
    std::priority_queue<Node, std::vector<Node>, Cmp> open;

    // DR5 cutset dominance per scheduled set (a skyline of start vectors)
    std::unordered_map<uint64_t, std::vector<std::vector<short>>> domStore;
    auto setKey = [&](const std::vector<short>& s) -> uint64_t {
        uint64_t h = 1469598103934665603ULL;
        for (int i = 0; i < n; ++i) if (s[i] >= 0) { h ^= (uint64_t)(unsigned)(i + 1); h *= 1099511628211ULL; }
        return h;
    };
    auto admit = [&](const std::vector<short>& ns) -> bool {     // false = dominated, skip it
        auto& lst = domStore[setKey(ns)];
        for (size_t i = 0; i < lst.size(); ) {
            const auto& v = lst[i];
            bool sameset = true, vLEns = true, nsLEv = true;
            for (int k = 0; k < n; ++k) {
                const bool vi = v[k] >= 0, ni = ns[k] >= 0;
                if (vi != ni) { sameset = false; break; }
                if (vi) { if (v[k] > ns[k]) vLEns = false; if (ns[k] > v[k]) nsLEv = false; }
            }
            if (sameset && vLEns) return false;                                       // v dominates ns
            if (sameset && nsLEv) { lst[i] = lst.back(); lst.pop_back(); continue; }  // ns dominates v
            ++i;
        }
        lst.push_back(ns);
        return true;
    };

    open.push({std::vector<short>(n, -1), makespanLB(std::vector<short>(n, -1)), 0});
    long expands = 0;
    while (!open.empty()) {
        Node cur = open.top(); open.pop();
        if (cur.depth == n) return {cur.f, expands, false};                           // first goal = optimum
        if (expandCap > 0 && expands >= expandCap) return {cur.f, expands, true};
        if ((expands & 1023) == 0 && std::chrono::steady_clock::now() > until) return {cur.f, expands, true};
        ++expands;
        for (int a = 0; a < n; ++a) {
            if (cur.start[a] >= 0) continue;
            bool ready = true;
            for (int p : P.preds[a]) if (cur.start[p] < 0) { ready = false; break; }
            if (!ready) continue;
            std::vector<short> ns = cur.start;
            ns[a] = (short)earliestFeasible(a, cur.start);
            if (!admit(ns)) continue;
            const int f = makespanLB(ns);
            open.push({std::move(ns), f, cur.depth + 1});
        }
    }
    return {0, expands, false};   // not reached for a feasible sub-problem
}
