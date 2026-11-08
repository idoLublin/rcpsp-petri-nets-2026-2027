#pragma once
// RCPSP_TT2 — the search environment HOG2's A* runs on (TemplateAStar), and the successor function the
// bidirectional search reuses on both nets (bidirectional/HBAE.h).
//
// Successors of a state: for each transition that can fire (State.h, getAvailableTransitionIndices_TT2 gives
// its delay d), fire it after d. Three pruning / shaping rules, all sound:
//   DR4    (RCPSP_TT2_DR4)   delayed-start dominance (Liu, Jin, Zhou & Hu 2023, rule 4)
//   batch  (RCPSP_TT2_BATCH) the activities ready NOW may start together in one edge (any feasible subset)
//   DR5    (RCPSP_TT2_DR5)   cutset dominance against every state generated so far (DominanceTT2.h)
// Heuristic: max(critical path, resource energy) of what is left (Heuristic.h), optionally raised by the extra
// bounds (bounds/ExtraBounds.h) and the per-node deadline bound (RCPSP_TT2_TT, bounds/NodeDeadlineBound.h).

#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <vector>
#include "../../HOG2/search/SearchEnvironment.h"
#include "../bounds/ExtraBounds.h"
#include "../bounds/NodeDeadlineBound.h"
#include "DominanceTT2.h"
#include "State.h"

class RCPSP_TT2 : public SearchEnvironment<RCPSPState_TT2, int> {
public:
    void GetSuccessors(const RCPSPState_TT2& nodeID, std::vector<RCPSPState_TT2>& neighbors) const override;
    bool GoalTest(const RCPSPState_TT2& node, const RCPSPState_TT2& goal) const override;
    double HCost(const RCPSPState_TT2& state1, const RCPSPState_TT2& state2) const override;
    double GCost(const RCPSPState_TT2& state1, const RCPSPState_TT2& state2) const override { return state2.g - state1.g; }
    uint64_t GetStateHash(const RCPSPState_TT2& node) const override;
    // Required by SearchEnvironment, not used by the searches here.
    void GetActions(const RCPSPState_TT2&, std::vector<int>&) const override {}
    void ApplyAction(RCPSPState_TT2&, int) const override {}
    bool InvertAction(int&) const override { return true; }
    double GCost(const RCPSPState_TT2& node, const int&) const override { return node.g; }
    uint64_t GetActionHash(int act) const override { return std::hash<int>()(act); }
};

inline void RCPSP_TT2::GetSuccessors(const RCPSPState_TT2& nodeStored, std::vector<RCPSPState_TT2>& neighbors) const {
    // RCPSP_TT2_COMPACT: a stored node does not keep its token lists. Rebuild them on a copy for this expansion;
    // the children are released again at the end. Value 2 keeps the lists and compares them with a rebuild.
    std::optional<RCPSPState_TT2> rebuilt;
    if (g_tt2_compact_on) {
        if (tt2CompactStripped(nodeStored)) { rebuilt.emplace(nodeStored); tt2CompactRebuild(*rebuilt); ++g_tt2_compact_rebuilds; }
        else if (g_tt2_compact_on == 2) tt2CompactCheck(nodeStored, "expanded node");
    }
    const RCPSPState_TT2& nodeID = rebuilt ? *rebuilt : nodeStored;

    // ── profiling (RCPSP_TT2_PROF) and the lower-bound checkpoints ──
    const bool prof = g_tt2_prof;
    std::chrono::steady_clock::time_point pt0, pg0, pd0;
    if (prof) pt0 = std::chrono::steady_clock::now();
    ++g_prof_succ_calls;
    { const int f = (int)nodeID.g + (int)nodeID.h; if (f > g_prof_maxf) g_prof_maxf = f; }
    if ((g_prof_succ_calls & 1023) == 0) {
        const double el = secs_since(g_prof_t0);
        static const double cp[5] = {1.0, 10.0, 60.0, 120.0, 240.0};
        for (int i = 0; i < 5; i++) if (el >= cp[i] && g_prof_f_at[i] < 0) g_prof_f_at[i] = g_prof_maxf;
        if (g_prof_trace && el >= g_prof_next_trace) {
            std::fprintf(g_prof_trace, "%s,%d,%d,%.1f,%ld,%d,%ld,%ld,%.3f,%.3f,%.3f\n", g_prof_size, g_prof_group, g_prof_exam, el,
                         g_prof_succ_calls, g_prof_maxf, g_prof_children_raw, g_prof_children_kept, g_prof_succ_sec, g_heur_time_sec, g_prof_dr5_sec);
            std::fflush(g_prof_trace); g_prof_next_trace = el + 5.0;
        }
    }

    // ── which transitions can fire, and after which delay ──
    std::chrono::steady_clock::time_point pa0; if (prof) pa0 = std::chrono::steady_clock::now();
    std::vector<short> tempUnstarted;
    tempUnstarted.reserve(petri.Transitions.size());
    for (int i = 0; i < (int)petri.Transitions.size(); i++)
        if (nodeID.finishedActivitiys[i + 1] == 0) tempUnstarted.push_back((short)(i + 1));
    const std::vector<std::pair<short, short>> available =
        getAvailableTransitionIndices_TT2(tempUnstarted, nodeID.finishedActivitiys, nodeID.resource_nodes, nodeID.activeTransitionIndices);
    if (prof) { pg0 = std::chrono::steady_clock::now(); g_prof_avail_sec += std::chrono::duration<double>(pg0 - pa0).count(); }

    // ── DR5 before construction ──
    // A time-advancing child is tested against DR5 on its DR5 view, so a pruned child is never built; a batch
    // subset child is built anyway (its extensions start from it). The view of "fire t after delay d": g' = g + d;
    // the scheduled set gains t; an active i stays active iff rem_i > d, with release g + rem_i; t becomes active
    // with release g' + p_t if p_t > 0. The tests run in generation order, so the decisions equal testing after
    // construction.
    const bool preDR5 = g_tt2_dr5 && neighbors.empty();
    TT2DominanceTable* const preTab = preDR5 ? &get_tt2_dominance_table() : nullptr;
    const TT2DominanceTable::Key preKey = preDR5 ? TT2DominanceTable::MakeKey(nodeID) : TT2DominanceTable::Key{0, 0};
    long preCut = 0;
    auto prePrune = [&](short t, short d) -> bool {          // true: DR5 prunes "fire t after d"; do not build it
        short ids[129], rels[129];
        const int g2 = nodeID.g + d, p = RCPSPex.activities[t - 1].duration;
        int n = 0; bool placed = (p <= 0);
        for (const auto& [id, rem] : nodeID.activeTransitionIndices) {   // ascending ids, t inserted in place
            if (!placed && t < id) { ids[n] = t; rels[n] = (short)(g2 + p); ++n; placed = true; }
            if (rem > d && n < 128) { ids[n] = id; rels[n] = (short)(nodeID.g + rem); ++n; }
        }
        if (!placed && n < 128) { ids[n] = t; rels[n] = (short)(g2 + p); ++n; }
        TT2DominanceTable::Key k = preKey;
        if (t < 64) k.lo |= 1ULL << t; else k.hi |= 1ULL << (t - 64);
        if (!preTab->check_and_insert_raw(k, (short)g2, n, ids, rels)) return false;
        ++preCut; ++g_tt2_precheck_cut;
        return true;
    };

    // ── DR4 delayed-start dominance (Liu, Jin, Zhou & Hu 2023, rule 4) ──
    // Prune firing `transId` at delay `delay` iff some other ready transition i has d_i < delay and
    // d_i + p_i <= delay: i can be left-shifted ahead of it and return its resources by `delay`, so this branch is
    // dominated by the (i, d_i) sibling. Strict d_i < delay keeps the minimum-delay child, so a dominating branch
    // always survives.
    auto dr4Dominated = [&](short transId, short delay) -> bool {
        for (const auto& [oId, oDelta] : available) {
            if (oId == transId || oDelta >= delay) continue;
            if (oDelta + RCPSPex.activities[oId - 1].duration <= delay) return true;
        }
        return false;
    };

    if (!g_tt2_batch) {
        // ── serial: one successor per ready transition ──
        for (const auto& [transId, delay] : available) {
            if (g_tt2_dr4 && dr4Dominated(transId, delay)) { ++g_tt2_dr4_pruned; continue; }
            if (preDR5 && prePrune(transId, delay)) continue;
            neighbors.emplace_back(RCPSPState_TT2(nodeID, transId, delay));
        }
    } else {
        // ── batch ──
        // A successor is either (a) a single time-advancing firing (delay > 0), or (b) a resource-feasible
        // non-empty subset of the activities ready now (delay 0), all started at the current time in one edge.
        // Singleton subsets reproduce the delay-0 single firings, so the batch graph is a superset of the serial
        // one (the optimal serial path is still there), while larger subsets are shortcut edges that trade depth
        // for branching. Resource feasibility (combined demand <= tokens ready now) bounds the enumeration.
        std::vector<short> A0;
        for (const auto& [transId, delay] : available) {
            if (delay > 0) {
                if (g_tt2_dr4 && dr4Dominated(transId, delay)) { ++g_tt2_dr4_pruned; continue; }
                if (preDR5 && prePrune(transId, delay)) continue;
                neighbors.emplace_back(RCPSPState_TT2(nodeID, transId, delay));   // (a)
            } else
                A0.push_back(transId);
        }
        std::array<int, 4> avail0 = {0, 0, 0, 0};             // resource tokens ready now, per resource place
        for (int r = 0; r < 4; r++)
            for (const auto& [amt, delay] : nodeID.resource_nodes[r])
                if (delay == 0) avail0[r] += amt;
        auto demandOf = [&](short id, std::array<int, 4>& d) {
            const auto& sl = actDemandSlot();
            if (sl.placeOK) { d = sl.byPlace[id]; return; }
            d = {0, 0, 0, 0};
            for (const auto& [resName, dem] : RCPSPex.activities[id - 1].resource_demands)
                if (dem > 0) d[petri.place_name_to_id.at(resName)] += dem;
        };
        // (b) Enumerate the feasible non-empty subsets by incremental firing: one successor per recursion node
        // yields every feasible subset exactly once. The subset-so-far state is neighbors[curIdx] (nodeID when
        // curIdx < 0), or `local` when DR5 pruned it (then it is not in neighbors, but its extensions still are
        // generated from it). zd = the subset holds a duration-0 activity (its child may start more now).
        long emitted = 0;
        auto rec = [&](auto& self, int start, int curIdx, const RCPSPState_TT2* local, std::array<int, 4> used, bool zd) -> void {
            for (int i = start; i < (int)A0.size(); i++) {
                if (emitted >= g_tt2_batch_cap) return;   // safety valve (singletons already emitted)
                std::array<int, 4> d; demandOf(A0[i], d);
                bool feasible = true;
                for (int r = 0; r < 4; r++) if (used[r] + d[r] > avail0[r]) { feasible = false; break; }
                if (!feasible) continue;
                RCPSPState_TT2 child(local ? *local : curIdx < 0 ? nodeID : neighbors[curIdx], A0[i], 0);
                const bool zd2 = zd || RCPSPex.activities[A0[i] - 1].duration == 0;
                child.batchClosed = !zd2;
                std::array<int, 4> nu = {used[0] + d[0], used[1] + d[1], used[2] + d[2], used[3] + d[3]};
                ++emitted;
                if (preDR5 && preTab->check_and_insert(child)) {
                    ++preCut; ++g_tt2_precheck_cut;
                    self(self, i + 1, -1, &child, nu, zd2);
                } else {
                    neighbors.push_back(std::move(child));
                    self(self, i + 1, (int)neighbors.size() - 1, nullptr, nu, zd2);
                }
            }
        };
        // Batch close: a node reached by a zero-delay batch edge starts nothing more at that time. Its parent
        // emitted every resource-feasible subset of its zero-delay activities, so "S now, then T now" is the
        // parent's child S+T: the same state, which DR5 would drop anyway. Not after a duration-0 activity (it can
        // enable new zero-delay starts) and not when the parent's enumeration hit RCPSP_TT2_BATCH_CAP.
        if (nodeID.batchClosed) {
            ++g_tt2_batch_close_nodes;
            for (short a : A0) {
                std::array<int, 4> d; demandOf(a, d);
                bool fits = true;
                for (int r = 0; r < 4; r++) if (d[r] > avail0[r]) { fits = false; break; }
                if (fits) ++g_tt2_batch_close_skipped;
            }
        } else {
            rec(rec, 0, -1, nullptr, {0, 0, 0, 0}, false);
            if (emitted >= g_tt2_batch_cap)                    // incomplete enumeration: its children may not skip theirs
                for (auto& c : neighbors) c.batchClosed = false;
        }
    }
    if (prof) g_prof_gen_sec += secs_since(pg0);
    g_prof_children_raw += (long)neighbors.size() + preCut;

    // ── DR5 on the built children (only when they were not tested before construction) ──
    // A child dominated by an earlier state (same scheduled set, no-later releases) is dropped and never enqueued.
    if (prof && g_tt2_dr5) pd0 = std::chrono::steady_clock::now();
    if (g_tt2_dr5 && !preDR5) {
        auto& tab = get_tt2_dominance_table();
        size_t w = 0;
        for (size_t i = 0; i < neighbors.size(); i++) {
            if (!tab.check_and_insert(neighbors[i])) {
                if (w != i) neighbors[w] = std::move(neighbors[i]);
                w++;
            }
        }
        neighbors.resize(w);
    }
    if (prof && g_tt2_dr5) g_prof_dr5_sec += secs_since(pd0);
    g_prof_children_kept += (long)neighbors.size();
    // RCPSP_TT2_COMPACT: the children are stored without their token lists (value 2: kept, and compared with a rebuild)
    if (g_tt2_compact_on == 1) { for (auto& c : neighbors) tt2CompactStrip(c); }
    else if (g_tt2_compact_on == 2) { for (const auto& c : neighbors) tt2CompactCheck(c, "child"); }
    if (prof) g_prof_succ_sec += secs_since(pt0);
}

inline bool RCPSP_TT2::GoalTest(const RCPSPState_TT2& node, const RCPSPState_TT2&) const {
    int count = 0;
    for (int i = 1; i <= (int)petri.Transitions.size(); i++)
        if (node.finishedActivitiys[i] == 1) count++;
    return count == (int)petri.Transitions.size();     // every activity finished
}

inline double RCPSP_TT2::HCost(const RCPSPState_TT2& state1, const RCPSPState_TT2&) const {
    ++g_prof_h_calls_all;
    // A zero-delay child started an activity at its parent's time: nothing has finished and no time passed, so the
    // parent's h is still a valid (admissible) estimate - reuse it instead of recomputing. HBAE turns this off
    // (g_tt2_hfresh): its stored states keep h = 0, so the shortcut would score those children h = 0.
    if (state1.isDeltaZero && !g_tt2_hfresh) {
        ++g_prof_h_deltazero;
        state1.h = state1.predessesor_h;
        return state1.h;
    }
    const auto ht0 = std::chrono::steady_clock::now();
    std::vector<short> tempUnfinished;
    tempUnfinished.reserve(petri.Transitions.size());
    for (int i = 0; i < (int)petri.Transitions.size(); i++)
        if (!state1.finishedActivitiys.test(i + 1)) tempUnfinished.push_back((short)(i + 1));

    double base;
    if (g_tt2_prof) {   // the same two calls, timed separately
        const auto c0 = std::chrono::steady_clock::now();
        const double cpv = getForwardHcost(tempUnfinished, state1.activeTransitionIndices);
        const auto c1 = std::chrono::steady_clock::now();
        const double rsv = getforwardResource(tempUnfinished, state1.activeTransitionIndices);
        g_prof_hcp_sec  += std::chrono::duration<double>(c1 - c0).count();
        g_prof_hres_sec += secs_since(c1);
        base = std::max(cpv, rsv);
    } else
        base = std::max(getForwardHcost(tempUnfinished, state1.activeTransitionIndices),
                        getforwardResource(tempUnfinished, state1.activeTransitionIndices));

    // optional extra bounds (Θ-tree, single-resource relaxation, look-ahead), each admissible, so the max is too
    if (extra::anyOn()) base = extra::raise(base, tempUnfinished, state1.activeTransitionIndices);
    // last year's bounds (LBCS, LBcc, LBip0, LBrc, LBER), each admissible; g = this state's clock
    if (ly::anyOn()) base = ly::raise(base, tempUnfinished, state1.activeTransitionIndices, state1.g);

    // RCPSP_TT2_TT: try to REFUTE the remaining makespan `base` on the residual problem; each refutation proves no
    // completion that short exists, so h rises by one and stays admissible.
    if (g_tt2_node_tt) {
        const int rb = tt2nb::raise(tempUnfinished, state1.activeTransitionIndices, (int)std::ceil(base));
        if ((double)rb > base) base = (double)rb;
    }
    g_heur_time_sec += secs_since(ht0);
    ++g_heur_calls;
    if (g_root_h < 0.0) g_root_h = base;
    state1.h = (short)base;
    return state1.h;
}

// FNV-1a over the state identity: the finished set and the active list (id, remaining time), which is sorted.
inline uint64_t RCPSP_TT2::GetStateHash(const RCPSPState_TT2& node) const {
    ++g_prof_hash_calls;
    std::chrono::steady_clock::time_point ht0; if (g_tt2_prof) ht0 = std::chrono::steady_clock::now();
    constexpr uint64_t FNV_OFFSET = 14695981039346656037ULL;
    constexpr uint64_t FNV_PRIME  = 1099511628211ULL;
    uint64_t h = FNV_OFFSET;
    auto mix_int = [&](int v) {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
        for (size_t i = 0; i < sizeof(int); ++i) { h ^= static_cast<uint64_t>(p[i]); h *= FNV_PRIME; }
    };
    for (int i = 1; i <= (int)petri.Transitions.size(); ++i) mix_int(node.finishedActivitiys[i]);
    for (const auto& active : node.activeTransitionIndices) { mix_int(active.first); mix_int(active.second); }
    if (g_tt2_prof) g_prof_hash_sec += secs_since(ht0);
    return h;
}
