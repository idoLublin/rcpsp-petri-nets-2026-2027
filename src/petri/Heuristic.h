#pragma once
// The two admissible estimates of the remaining makespan that the TT2 heuristic takes the max of
// (RCPSP_TT2::HCost in Environment.h):
//   getForwardHcost     critical path of what is left (precedence only, resources ignored)
//   getforwardResource  energy bound: per resource, remaining work (duration x demand) / capacity
// Both read the loaded instance (RCPSPex), so they work unchanged on the reversed net.

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>
#include "../common/Globals.h"

// ── Critical path ────────────────────────────────────────────────────────────
// One pass over the unfinished activities in topological order, computing each one's early FINISH: max over its
// already-processed predecessors (a finished predecessor contributes 0) plus its own duration, or its remaining
// time if it is active. h = the largest early finish.
// Orientation: the single pass is a correct longest-path computation only while the activity-id order is
// topological. PSPLIB numbers the forward net that way (1 = source, n = sink); on the reversed net the order is
// descending. The net tells which one it is: only in the reversed net does activity 1 have predecessors.
// (Without this the reversed net got 31-63% of its true critical path.)
inline double getForwardHcost(const std::vector<short>& unstartedTransitions,
                              const std::vector<std::pair<short, short>>& activeTransitionIndices) {
    const bool reversedNet = !RCPSPex.backword_dependencies.empty() && !RCPSPex.backword_dependencies[0].empty();
    std::array<short, 130> activeRem; activeRem.fill(-1);
    for (const auto& p : activeTransitionIndices) if (p.first > 0 && p.first < 130) activeRem[p.first] = p.second;
    std::array<int, 130> ef; ef.fill(-1);
    int h = 0;
    const int n = (int)unstartedTransitions.size();
    for (int k = 0; k < n; ++k) {
        const int a = unstartedTransitions[reversedNet ? n - 1 - k : k];
        int es = 0;
        for (int dep : RCPSPex.backword_dependencies[a - 1]) if (ef[dep] > es) es = ef[dep];
        const int v = es + (activeRem[a] != -1 ? (int)activeRem[a] : (int)RCPSPex.activities[a - 1].duration);
        ef[a] = v;
        if (v > h) h = v;
    }
    return (double)h;
}

// ── Resource (energy) bound ──────────────────────────────────────────────────
// Per resource: remaining energy of the active activities (remaining time x demand) plus the unfinished,
// not-yet-started ones (duration x demand), divided by the capacity; h = max over resources. The dummy source
// and sink are skipped. demand[activity][resource] is cached per instance (g_rcpsp_load_gen) and per activity
// buffer: two slots, because the bidirectional search keeps the forward and the reversed net alive and swaps them.
inline const std::vector<std::array<int, 4>>& resDemandMatrix() {
    struct Slot { uint64_t gen = ~0ULL; const P_RCPSP::Activity* ptr = nullptr; int n = -1, nres = -1;
                  std::vector<std::array<int, 4>> dem; };
    static Slot slot[2]; static int next = 0;
    const P_RCPSP::Activity* ptr = RCPSPex.activities.data();
    const int n = (int)RCPSPex.activities.size(), nres = (int)RCPSPex.resources.size();
    for (auto& s : slot)
        if (s.gen == g_rcpsp_load_gen && s.ptr == ptr && s.n == n && s.nres == nres) return s.dem;
    Slot& s = slot[next]; next ^= 1;
    s.gen = g_rcpsp_load_gen; s.ptr = ptr; s.n = n; s.nres = nres;
    s.dem.assign(n + 1, {0, 0, 0, 0});
    for (int a = 1; a <= n; ++a)
        for (const auto& kv : RCPSPex.activities[a - 1].resource_demands)
            for (int k = 0; k < nres && k < 4; ++k)
                if (RCPSPex.resources[k].first == kv.first) { s.dem[a][k] = kv.second; break; }
    return s.dem;
}

inline double getforwardResource(const std::vector<short>& tempUnfinished,
                                 const std::vector<std::pair<short, short>>& activeTransitionIndices) {
    const int nres = (int)RCPSPex.resources.size();   // PSPLIB: 4 (the state keeps 4 resource places)
    const auto& dem = resDemandMatrix();
    const int n = (int)RCPSPex.activities.size();
    std::array<bool, 130> executing{};
    double tot[4] = {0.0, 0.0, 0.0, 0.0};
    for (const auto& [id, rem] : activeTransitionIndices) {
        if (id > 0 && id < 130) executing[id] = true;
        if (id == 1 || id == n) continue;
        for (int k = 0; k < nres; ++k) tot[k] += rem * dem[id][k];
    }
    for (short id : tempUnfinished) {
        if (id == 1 || id == n) continue;
        if (id > 0 && id < 130 && executing[id]) continue;
        const int dur = RCPSPex.activities[id - 1].duration;
        for (int k = 0; k < nres; ++k) tot[k] += dur * dem[id][k];
    }
    double best = 0.0;
    for (int k = 0; k < nres; ++k) best = std::max(best, tot[k] / RCPSPex.resources[k].second);
    return best;
}
