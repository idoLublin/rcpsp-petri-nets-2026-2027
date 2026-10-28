#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// HBAE — bidirectional search over the forward net and the reversed net (mode tt2hbae_<set>).
// The algorithm is BAE_RCPSP.h (HOG2's BAE* with the meet lookup separated from the node identity). Everything
// here is glue so that one search can drive TWO different nets:
//   * the state carries which side it belongs to; successors inherit it;
//   * GetSuccessors swaps the globals to that side's net before delegating to RCPSP_TT2, so DR5 / DR4 / batch and
//     the heuristic behave exactly as in the one-directional search (each side has its own DR5 table);
//   * GetStateHash = the FULL marking (+ side bit)  -> node identity within a queue;
//   * GetMeetHash  = the NORMALISED CUT key         -> the two frontiers collide on the same cut, although forward
//     states sit on START events and reversed states on FINISH events.
// The meet: at a time T both sides describe the same cut "what started before T, and what is still running".
// Forward records a running activity's remaining time, the reversed search its elapsed time on the mirrored
// clock; translated, finished/unstarted swap and the running set is shared. Normalisation fixes the one boundary
// each side gets wrong at T: a forward activity started exactly at T counts as not started, a reversed activity
// that ends exactly at T (forward time) counts as finished.
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "../backward/ReverseInstance.h"
#include "../petri/ForwardSearch.h"
#include "BAE_RCPSP.h"
#include "Prover.h"

struct RCPSPState_BI {
    RCPSPState_TT2 s;
    bool rev = false;                 // false = forward net, true = reversed net
    RCPSPState_BI() {}
    RCPSPState_BI(const RCPSPState_TT2& st, bool r) : s(st), rev(r) {}
    RCPSPState_BI(RCPSPState_TT2&& st, bool r) : s(std::move(st)), rev(r) {}
};
inline int g_bi_NT = 0;               // activity count, set once per instance

// ── identity and meet hashes ─────────────────────────────────────────────────
inline const std::bitset<128>& biFinMask() {          // bits 1..NT
    static std::bitset<128> m; static int nt = -1;
    if (nt != g_bi_NT) { m.reset(); for (int i = 1; i <= g_bi_NT && i < 128; ++i) m.set(i); nt = g_bi_NT; }
    return m;
}
inline uint64_t biMix(uint64_t h, uint64_t v) {
    uint64_t x = h ^ (v + 0x9e3779b97f4a7c15ULL);
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}
inline uint64_t biPair(const std::pair<short, short>& p) { return ((uint64_t)(uint16_t)p.first << 16) | (uint16_t)p.second; }
// finished set + (id, value) pairs, sorted by id
inline uint64_t biHashCut(const std::bitset<128>& fin, std::pair<short, short>* av, int n) {
    static const std::bitset<128> LOW(~0ULL);
    std::sort(av, av + n);
    uint64_t h = biMix(0x243f6a8885a308d3ULL, (fin & LOW).to_ullong());
    h = biMix(h, (fin >> 64).to_ullong());
    h = biMix(h, (uint64_t)n);
    for (int i = 0; i < n; ++i) h = biMix(h, biPair(av[i]));
    return h;
}
// cut of a forward state: an activity that started exactly now (remaining == duration) counts as not started
inline uint64_t biCutHashFwd(const RCPSPState_TT2& s) {
    std::pair<short, short> av[128]; int n = 0;
    for (const auto& p : s.activeTransitionIndices) {
        if ((int)p.second == (int)RCPSPex.activities[p.first - 1].duration) continue;
        if (n < 128) av[n++] = p;
    }
    return biHashCut(s.finishedActivitiys & biFinMask(), av, n);
}
// cut of a reversed state, translated to forward time: reversed-finished = forward-unstarted and vice versa;
// a running activity's forward remaining time = duration - its reversed remaining time; 0 = finished
inline uint64_t biCutHashRevTranslated(const RCPSPState_TT2& b) {
    std::bitset<128> fin = biFinMask() & ~b.finishedActivitiys;
    std::pair<short, short> av[128]; int n = 0;
    for (const auto& p : b.activeTransitionIndices) {
        const int tf = (int)RCPSPex.activities[p.first - 1].duration - (int)p.second;
        if (tf == 0) { fin.set(p.first); continue; }
        fin.reset(p.first);
        if (n < 128) av[n++] = {p.first, (short)tf};
    }
    return biHashCut(fin, av, n);
}
// full identity: the finished set, the active list and both token lists
inline uint64_t biIdHash(const RCPSPState_TT2& s) {
    std::pair<short, short> av[128]; int n = 0;
    for (const auto& p : s.activeTransitionIndices) if (n < 128) av[n++] = p;
    uint64_t h = biHashCut(s.finishedActivitiys & biFinMask(), av, n);
    h = biMix(h, 0xA11CE000ULL + (uint64_t)s.activity_nodes.size());
    for (const auto& q : s.activity_nodes) h = biMix(h, biPair(q));
    for (int r = 0; r < 4; ++r) {
        h = biMix(h, 0x5E500000ULL + (uint64_t)r * 131ULL + ((uint64_t)s.resource_nodes[r].size() << 24));
        for (const auto& q : s.resource_nodes[r]) h = biMix(h, biPair(q));
    }
    return h;
}
inline bool operator==(const RCPSPState_BI& a, const RCPSPState_BI& b) {
    return a.rev == b.rev && (a.s.finishedActivitiys & biFinMask()) == (b.s.finishedActivitiys & biFinMask())
        && a.s.activeTransitionIndices == b.s.activeTransitionIndices
        && a.s.activity_nodes == b.s.activity_nodes && a.s.resource_nodes == b.s.resource_nodes;
}

// Holds the net that is not current and swaps the globals. petri / RCPSPex are globals, so the swap is how
// "which net am I on" is expressed; std::swap on them is O(1).
struct BiSides {
    PetriExample petriStash; RCPSP_example rcpspStash; bool curFwd = true;
    // initialstatename / finalstatename are per net (the state constructor reads them), so they swap too
    std::string initF, finalF, initB, finalB;
    void use(bool wantFwd) {
        if (wantFwd != curFwd) { std::swap(petri, petriStash); std::swap(RCPSPex, rcpspStash); curFwd = wantFwd; }
        initialstatename = wantFwd ? initF : initB;
        finalstatename   = wantFwd ? finalF : finalB;
        g_tt2_dom_side = wantFwd ? 0 : 1;      // per-side DR5 table
    }
};

class RCPSP_BiEnv {
public:
    explicit RCPSP_BiEnv(BiSides* sd) : sides(sd) {}
    void GetSuccessors(const RCPSPState_BI& st, std::vector<RCPSPState_BI>& nb) {
        sides->use(!st.rev);
        rawBuf.clear();
        env.GetSuccessors(st.s, rawBuf);
        nb.clear(); nb.reserve(rawBuf.size());
        for (auto& c : rawBuf) nb.emplace_back(std::move(c), st.rev);   // successors inherit the side
    }
    uint64_t GetStateHash(const RCPSPState_BI& st) { return biIdHash(st.s) ^ (st.rev ? 0x9e3779b97f4a7c15ULL : 0ULL); }
    uint64_t GetMeetHash(const RCPSPState_BI& st) {   // frame-independent: no side bit
        sides->use(!st.rev);                          // durations are read from the current net
        return st.rev ? biCutHashRevTranslated(st.s) : biCutHashFwd(st.s);
    }
    // The search relaxes the NODE's g, but the stored STATE keeps the g it was generated with, and
    // RCPSP_TT2::GetSuccessors and DR5 work on the state's g. Keep them in sync.
    void SetStateG(RCPSPState_BI& st, double g) { if ((short)g != st.s.g) st.s.g = (short)g; }
    double GCost(const RCPSPState_BI& a, const RCPSPState_BI& b) {
        const double d = (double)b.s.g - (double)a.s.g;
        return d < 0 ? 0 : d;                         // firings at delay 0 are legitimate zero-cost edges
    }
    // RCPSP_TT2::HCost writes h into the state it scores; here the stored states keep h = 0, so it is restored
    // (and g_tt2_hfresh makes every state get a fresh h, see Environment.h).
    double HCost(const RCPSPState_BI& a) {
        sides->use(!a.rev);
        const short h0 = a.s.h;
        const double r = env.HCost(a.s, a.s);
        a.s.h = h0;
        return r;
    }
    bool GoalTest(const RCPSPState_BI& a, const RCPSPState_BI&) {
        int fin = 0; for (int i = 1; i <= g_bi_NT; i++) if (a.s.finishedActivitiys.test(i)) ++fin;
        return fin == g_bi_NT;
    }
    BiSides* sides;
    RCPSP_TT2 env;
    std::vector<RCPSPState_TT2> rawBuf;
};

// The cost-to-go of a state on its own net (the TT2 heuristic). With the priority f = g + h the search never
// asks for an estimate of the distance already travelled, so one object serves both sides.
class BiHeuristic : public Heuristic<RCPSPState_BI> {
public:
    explicit BiHeuristic(RCPSP_BiEnv* e) : env(e) {}
    double HCost(const RCPSPState_BI& a, const RCPSPState_BI&) const override { return env->HCost(a); }
    RCPSP_BiEnv* env;
};

// Result of one HBAE solve, written to the CSV by runBenchmarkTT2HBAE.
struct HbaeResult {
    double time = 0.0;
    int mk = -1, ub = -1, pathLen = 0;
    bool proven = false, trivial = false;
    const char* stop = "none";
    uint64_t exp = 0, expF = 0, expB = 0, unique = 0, touched = 0, reopened = 0, solUpd = 0, meetHits = 0;
    int64_t firstMeetExp = -1, meetExp = -1;
    double firstU = -1, meetGF = -1, meetGB = -1, meetDepth = -1;
    const char* meetSide = "none";
    const char* meetAt = "none";
    double timeF = 0, timeB = 0, minFF = -1, minFB = -1, rssMB = 0;
    uint64_t openF = 0, openB = 0, dr5PrunedF = 0, dr5PrunedB = 0;
    long refCalls = 0, refProved = 0, refNodes = 0, refAdopted = 0; double refSec = 0;
};

inline HbaeResult solveRCPSP_TT2_HBAE(int group, int exam, const std::string& set) {
    HbaeResult res;
    std::cout << "=== HBAE " << group << ":" << exam << " ===" << std::endl;
    getPetri(petri, group, exam, set);
    getRCPSP(RCPSPex, group, exam, set);
    g_bi_NT = (int)petri.Transitions.size();
    g_instance_rs = instanceRS(group);
    resetTT2Counters(set, group, exam);
    {   // trivial root: the EST schedule is resource-feasible => optimal without search
        std::vector<int> te; int tm = 0;
        if (tryTrivialRootSchedule_TT2(te, tm)) {
            std::cout << "HBAE " << group << ":" << exam << " makespan=" << tm << " expanded=0 trivialAtRoot=1" << std::endl;
            res.mk = res.ub = tm; res.proven = true; res.trivial = true; res.stop = "trivial";
            return res;
        }
    }
    // the side settings for this solve (restored at the end)
    struct Saved { bool dr4 = g_tt2_dr4, batch = g_tt2_batch, hfresh = g_tt2_hfresh; } saved;
    g_tt2_dr4 = g_bi_dr4;
    g_tt2_batch = g_bi_batch;
    g_tt2_hfresh = true;   // stored states keep h = 0: the zero-delay shortcut would score their children h = 0
    g_tt2_dom_side = 0; get_tt2_dominance_table().clear();
    g_tt2_dom_side = 1; get_tt2_dominance_table().clear();

    BiSides sides;
    sides.petriStash = petri; sides.rcpspStash = RCPSPex;
    deriveEndpointNames(sides.initF, sides.finalF);      // the globals hold the FORWARD net
    reverseLoadedInstance();
    deriveEndpointNames(sides.initB, sides.finalB);      // ... and now the REVERSED one
    std::swap(petri, sides.petriStash); std::swap(RCPSPex, sides.rcpspStash);   // global = forward again
    sides.curFwd = true;

    RCPSP_BiEnv benv(&sides);
    sides.use(true);  RCPSPState_TT2 rf; rf.g = 0; RCPSPState_BI start(rf, false);
    sides.use(false); RCPSPState_TT2 rb; rb.g = 0; RCPSPState_BI goal(rb, true);
    BiHeuristic heur(&benv);
    BAE_RCPSP<RCPSPState_BI, int, RCPSP_BiEnv> bae(0.0, 1.0);   // epsilon 0: zero-cost edges; gcd 1: integer costs

    // RCPSP_BI_REFUTE: on each new incumbent U, try to prove that no schedule ends by U - 1 (Prover.h), within
    // RCPSP_BI_REFUTE_MS per incumbent; a proof stops the search with U optimal.
    // Built on the reversed instance, as in the measured runs (it has the same optimum; schedules are mirrored).
    sides.use(false);
    const deadline::Inst refI = deadline::build();
    if (g_bi_refute) {
        bae.SetRefShare(g_bi_refute_share);
        bae.SetOptimalityProver([&refI, &res](double U, std::chrono::steady_clock::time_point until, double& newU) {
            std::vector<int> sol; int mk = 0; long nd = 0;
            const auto lim = std::min(until, std::chrono::steady_clock::now() + std::chrono::milliseconds(g_bi_refute_ms));
            const int r = deadline::refuteDeadline(refI, (int)std::lround(U) - 1, lim, sol, mk, nd);
            res.refNodes += nd;
            if (r == 0 && mk < U && deadline::feasible(refI, sol, mk)) newU = mk;   // a checked, shorter schedule
            return r;
        });
    }

    std::vector<RCPSPState_BI> path;
    const auto t0 = std::chrono::high_resolution_clock::now();
    bae.GetPath(&benv, start, goal, &heur, &heur, path);
    const std::chrono::duration<double> el = std::chrono::high_resolution_clock::now() - t0;
    sides.use(true);                                      // leave the forward net loaded

    const double cost = bae.GetSolutionCost();
    const int ub = (cost >= DBL_MAX / 2) ? -1 : (int)cost;
    // Optimality is proven when the stop rule fired (or the prover proved it), or when one side's open list
    // drained: that side then exhausted its own search, which contains an optimal path (DR5 / DR4 / batch are
    // sound for a side's own search), and every node of that path met the opposite side's index on the way.
    const int drained = bae.GetDrainedSide();
    const bool proven = (ub >= 0) && (bae.StoppedByBound() || drained != 0);
    res.mk = proven ? ub : -1;
    res.ub = ub;
    res.proven = proven;
    if (ub >= 0 && bae.GetMeetFromFwd() >= 0) {   // where the frontiers met: depth = started / NT on the forward clock
        const RCPSPState_BI& m = bae.GetMiddleNode();
        int started = 0;
        if (!m.rev) { started = (int)m.s.activeTransitionIndices.size(); for (int q = 1; q <= g_bi_NT; q++) if (m.s.finishedActivitiys.test(q)) ++started; }
        else        { int rfin = 0; for (int q = 1; q <= g_bi_NT; q++) if (m.s.finishedActivitiys.test(q)) ++rfin; started = g_bi_NT - rfin; }
        res.meetDepth = (double)started / (double)g_bi_NT;
        res.meetAt = (started <= 1) ? "AT-SOURCE" : (started >= g_bi_NT - 1) ? "AT-GOAL" : "MIDDLE";
    } else if (ub >= 0) res.meetAt = "prover";
    res.stop = bae.StoppedByFloor() ? "prover" : bae.StoppedByBound() ? "bound"
             : drained == 1 ? "fwd-drained" : drained == 2 ? "bwd-drained" : "deadline";
    res.time = el.count();
    res.exp = bae.GetNodesExpanded(); res.expF = bae.GetExpanded(true); res.expB = bae.GetExpanded(false);
    res.unique = bae.GetUniqueNodesExpanded(); res.touched = bae.GetNodesTouched();
    res.reopened = bae.GetReopenedNodes(); res.solUpd = bae.GetSolutionUpdates(); res.meetHits = bae.GetMeetHits();
    res.firstMeetExp = bae.GetFirstMeetExp(); res.meetExp = bae.GetMeetExp(); res.firstU = bae.GetFirstU();
    res.meetSide = (ub < 0 || bae.GetMeetFromFwd() < 0) ? "none" : (bae.GetMeetFromFwd() ? "fwd" : "bwd");
    res.meetGF = bae.GetMeetGF(); res.meetGB = bae.GetMeetGB();
    res.pathLen = (int)path.size();
    res.timeF = bae.GetSideSec(true); res.timeB = bae.GetSideSec(false);
    res.openF = bae.GetOpenCount(true); res.openB = bae.GetOpenCount(false);
    res.minFF = bae.GetMinFOpenNow(true); res.minFB = bae.GetMinFOpenNow(false);
    res.rssMB = (double)std::max(bae.GetPeakRSSKB(), bae_rcpsp_rss_kb()) / 1024.0;
    g_tt2_dom_side = 0; res.dr5PrunedF = get_tt2_dominance_table().pruned;
    g_tt2_dom_side = 1; res.dr5PrunedB = get_tt2_dominance_table().pruned;
    g_tt2_dom_side = 0;
    res.refCalls = bae.GetRefCalls(); res.refProved = bae.GetRefProved(); res.refSec = bae.GetRefSec(); res.refAdopted = bae.GetRefAdopted();
    std::cout << "HBAE " << group << ":" << exam << " makespan=" << res.mk << " ub=" << ub << " proven=" << (proven ? 1 : 0)
              << " stop=" << res.stop << " expanded=" << res.exp << " (fwd " << res.expF << ", bwd " << res.expB << ")"
              << " meetAt=" << res.meetAt << " meetDepth=" << res.meetDepth << " time=" << res.time << "s" << std::endl;
    g_tt2_dr4 = saved.dr4; g_tt2_batch = saved.batch; g_tt2_hfresh = saved.hfresh;
    return res;
}

inline const char* hbaeCsvHeader() {
    return "group,exam,time,solved,makespan,ub,proven,stopReason,expandNumber,expandedF,expandedB,uniqueExpanded,touched,reopened,"
           "solUpdates,meetHits,firstMeetExp,meetExp,firstU,meetSide,meetGF,meetGB,meetDepth,meetAt,pathLen,trivialAtRoot,"
           "model,problemType,timeoutS,instanceRS,useTT2DR5,dr4,batch,tt2NodeTT,timeFSec,timeBSec,openFEnd,openBEnd,minFFEnd,minFBEnd,"
           "peakRSSMB,dr5PrunedF,dr5PrunedB,dr4Pruned,heurTimeSec,heurCalls,useTT2Prof,profSuccSec,profAvailSec,profGenSec,profDR5Sec,"
           "profHeurCPSec,profHeurResSec,childrenRaw,childrenKept,nodeTtCalls,nodeTtRaised,nodeTtSec,"
           "biRefute,biRefCalls,biRefProved,biRefSec,biRefNodes,biRefAdopted,biRefMs,biRefShare,knownOptimum";
}

// Solve one instance, append its row, and check it against the known optimum.
inline HbaeResult solveAndWriteHBAE(int g, int e, const std::string& set, const std::string& csv) {
    const HbaeResult r = solveRCPSP_TT2_HBAE(g, e, set);
    {
        std::ofstream f(csv, std::ios::app);
        f << g << "," << e << "," << r.time << "," << (r.proven ? "True" : "False") << "," << r.mk << "," << r.ub << ","
              << (r.proven ? 1 : 0) << "," << r.stop << "," << r.exp << "," << r.expF << "," << r.expB << "," << r.unique << ","
              << r.touched << "," << r.reopened << "," << r.solUpd << "," << r.meetHits << "," << r.firstMeetExp << "," << r.meetExp << ","
              << r.firstU << "," << r.meetSide << "," << r.meetGF << "," << r.meetGB << "," << r.meetDepth << "," << r.meetAt << ","
              << r.pathLen << "," << (r.trivial ? 1 : 0) << ",HBAE," << set << "," << astar_timeout_seconds << "," << g_instance_rs << ","
              << g_tt2_dr5 << "," << g_bi_dr4 << "," << g_bi_batch << "," << g_tt2_node_tt << ","
              << r.timeF << "," << r.timeB << "," << r.openF << "," << r.openB << "," << r.minFF << "," << r.minFB << ","
              << r.rssMB << "," << r.dr5PrunedF << "," << r.dr5PrunedB << "," << g_tt2_dr4_pruned << ","
              << g_heur_time_sec << "," << g_heur_calls << "," << g_tt2_prof << ","
              << g_prof_succ_sec << "," << g_prof_avail_sec << "," << g_prof_gen_sec << "," << g_prof_dr5_sec << ","
              << g_prof_hcp_sec << "," << g_prof_hres_sec << "," << g_prof_children_raw << "," << g_prof_children_kept << ","
              << tt2nb::st().calls << "," << tt2nb::st().raised << "," << tt2nb::st().sec << ","
              << g_bi_refute << "," << r.refCalls << "," << r.refProved << "," << r.refSec << "," << r.refNodes << "," << r.refAdopted << ","
              << (g_bi_refute ? g_bi_refute_ms : 0) << "," << (g_bi_refute ? g_bi_refute_share : 0.0) << ","
              << knownOptimum(set, g, e) << std::endl;
    }
    checkAgainstOptimum(set, g, e, r.proven, r.mk, r.ub, csv);
    return r;
}

// All instances of one set (48 groups x 10), or the RCPSP_ONLY list.
inline void runBenchmarkTT2HBAE(const std::string& set) {
    const std::string csv = resultFile("HBAE", set, hbaeTag(), hbaeCsvHeader());
    std::cout << "\n=== HBAE | " << set << " ===  -> " << csv << std::endl;
    int unproven = 0;
    for (int g = 1; g <= 48; g++)
        for (int e = 1; e <= 10; e++)
            if (onlyInstance(set, g, e) && !solveAndWriteHBAE(g, e, set, csv).proven) ++unproven;
    std::cout << "\nHBAE benchmark done (" << set << ") -> " << csv << "   [" << unproven << " unproven]" << std::endl;
}
