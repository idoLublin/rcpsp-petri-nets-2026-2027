#pragma once
// One TT2 search (forward, or backward on the reversed instance): load the instance, try the trivial-root
// shortcut, run HOG2's A* (TemplateAStar) on RCPSP_TT2, and append one CSV row (columns: tt2CsvHeader).

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "../../HOG2/generic/TemplateAStar.h"
#include "../backward/ReverseInstance.h"
#include "../io/ReadInstance.h"
#include "../io/Results.h"
#include "Environment.h"

// A* order among nodes of equal f: higher g first, then more finished activities, then more active ones.
// (Any order among equal-f nodes keeps the search optimal; it only changes which optimal schedule is found first
// and how many equal-f nodes are expanded before it.)
template <>
struct AStarCompareWithF<RCPSPState_TT2> {
    bool operator()(const AStarOpenClosedDataWithF<RCPSPState_TT2>& i1, const AStarOpenClosedDataWithF<RCPSPState_TT2>& i2) const {
        if (i1.f != i2.f) return i1.f > i2.f;
        if (i1.g != i2.g) return i1.g < i2.g;
        if (i1.data.finishedActivitiys.count() != i2.data.finishedActivitiys.count())
            return i1.data.finishedActivitiys.count() < i2.data.finishedActivitiys.count();
        if (i1.data.activeTransitionIndices.size() != i2.data.activeTransitionIndices.size())
            return i1.data.activeTransitionIndices.size() < i2.data.activeTransitionIndices.size();
        return false;
    }
};

// ── Trivial root ─────────────────────────────────────────────────────────────
// If the CPM/EST schedule (every activity at its earliest precedence-feasible start, resources ignored) is also
// resource-feasible, it attains the critical-path lower bound while satisfying every constraint, so it is optimal:
// return it without running A*. Fills estStart (index = activity id) and outMakespan.
inline bool tryTrivialRootSchedule_TT2(std::vector<int>& estStart, int& outMakespan) {
    const int N = (int)RCPSPex.activities.size();
    if (N == 0) return false;
    // earliest starts by fixpoint iteration, so the ids need not be topologically ordered
    estStart.assign(N + 1, 0);
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 1; i <= N; ++i) {
            int est = 0;
            for (short pred : RCPSPex.backword_dependencies[i - 1])
                est = std::max(est, estStart[pred] + (int)RCPSPex.activities[pred - 1].duration);
            if (est != estStart[i]) { estStart[i] = est; changed = true; }
        }
    }
    int H = 0;
    for (int i = 1; i <= N; ++i) H = std::max(H, estStart[i] + (int)RCPSPex.activities[i - 1].duration);
    outMakespan = H;
    // whole-profile resource check: accumulate the demand over every time unit
    for (const auto& [resName, capacity] : RCPSPex.resources) {
        std::vector<int> usage(H + 1, 0);
        for (int i = 1; i <= N; ++i) {
            const auto& act = RCPSPex.activities[i - 1];
            if (act.duration <= 0) continue;
            auto it = act.resource_demands.find(resName);
            if (it == act.resource_demands.end() || it->second <= 0) continue;
            for (int t = estStart[i]; t < estStart[i] + act.duration; ++t) usage[t] += it->second;
        }
        for (int t = 0; t < H; ++t)
            if (usage[t] > capacity) return false;
    }
    return true;
}

inline std::string tt2CsvHeader() {
    return std::string("group,exam,time,solved,makespan,expandNumber,generatedNumber,depth,model,problemType,maxMem,timeoutS,instanceRS,"
           "trivialAtRoot,rootH,heurTimeSec,heurCalls,useTT2DR5,useTT2DR4,useTT2Batch,tt2BatchCap,tt2Compact,tt2NodeTT,useTT2Prof,"
           "dr4Pruned,domPruned,domThinned,domChecks,domInserts,domMaxBucket,dr5EqualPruned,precheckCut,batchCloseNodes,batchCloseSkipped,"
           "compactRebuilds,compactChecked,nodeTtCalls,nodeTtRaised,nodeTtRaiseSum,nodeTtTt0Fail,nodeTtPropFail,nodeTtSec,"
           "profAstarSec,profSuccSec,profSuccCalls,profAvailSec,profGenSec,profDR5Sec,profHashSec,profHashCalls,profHeurCPSec,profHeurResSec,"
           "hCallsAll,hDeltaZeroHits,childrenRaw,childrenKept,domBuckets,domComparisons,openAtEnd,itemsAtEnd,uniqueExpanded,"
           "maxFExpanded,fAt1s,fAt10s,fAt60s,fAt120s,fAt240s,timedOut,") + extra::allCsvColumns() + "knownOptimum";
}

// Everything after `maxMem`: the flags of this run, then the per-instance counters.
inline void writeTT2Tail(std::ostream& o, const std::string& set, int group, int exam, bool trivial,
                         uint64_t openAtEnd, uint64_t itemsAtEnd, uint64_t uniqueExp, bool timedOut) {
    const auto& dt = get_tt2_dominance_table();
    const auto& nb = tt2nb::st();
    o << "," << astar_timeout_seconds << "," << g_instance_rs << "," << (trivial ? 1 : 0) << "," << g_root_h
      << "," << g_heur_time_sec << "," << g_heur_calls
      << "," << g_tt2_dr5 << "," << g_tt2_dr4 << "," << g_tt2_batch << "," << g_tt2_batch_cap << "," << g_tt2_compact_on
      << "," << g_tt2_node_tt << "," << g_tt2_prof
      << "," << g_tt2_dr4_pruned << "," << dt.pruned << "," << dt.thinned << "," << dt.checks << "," << dt.inserts << "," << dt.maxBucket
      << "," << dt.equalPruned << "," << g_tt2_precheck_cut << "," << g_tt2_batch_close_nodes << "," << g_tt2_batch_close_skipped
      << "," << g_tt2_compact_rebuilds << "," << g_tt2_compact_checked
      << "," << nb.calls << "," << nb.raised << "," << nb.raiseSum << "," << nb.tt0Fail << "," << nb.propFail << "," << nb.sec
      << "," << g_prof_astar_sec << "," << g_prof_succ_sec << "," << g_prof_succ_calls << "," << g_prof_avail_sec << "," << g_prof_gen_sec
      << "," << g_prof_dr5_sec << "," << g_prof_hash_sec << "," << g_prof_hash_calls << "," << g_prof_hcp_sec << "," << g_prof_hres_sec
      << "," << g_prof_h_calls_all << "," << g_prof_h_deltazero << "," << g_prof_children_raw << "," << g_prof_children_kept
      << "," << dt.bucketCount() << "," << dt.comparisons << "," << openAtEnd << "," << itemsAtEnd << "," << uniqueExp
      << "," << g_prof_maxf;
    for (int v : g_prof_f_at) o << "," << v;
    o << "," << (timedOut ? 1 : 0) << ",";
    extra::writeCsv(o);
    o << knownOptimum(set, group, exam) << "\n";
}

inline void resetTT2Counters(const std::string& set, int group, int exam) {
    get_tt2_dominance_table().clear();
    g_tt2_dr4_pruned = g_tt2_precheck_cut = g_tt2_batch_close_nodes = g_tt2_batch_close_skipped = 0;
    g_tt2_compact_rebuilds = g_tt2_compact_checked = 0;
    g_heur_time_sec = 0.0; g_heur_calls = 0; g_root_h = -1.0;
    g_prof_succ_sec = g_prof_avail_sec = g_prof_gen_sec = g_prof_dr5_sec = g_prof_hash_sec = 0.0;
    g_prof_hcp_sec = g_prof_hres_sec = g_prof_astar_sec = 0.0;
    g_prof_succ_calls = g_prof_hash_calls = g_prof_children_raw = g_prof_children_kept = 0;
    g_prof_h_calls_all = g_prof_h_deltazero = 0;
    g_prof_maxf = -1; for (int& v : g_prof_f_at) v = -1;
    g_prof_group = group; g_prof_exam = exam; g_prof_size = set.c_str(); g_prof_next_trace = 5.0;
    g_prof_t0 = std::chrono::steady_clock::now();
    tt2nb::reset();
    extra::reset(astar_timeout_seconds);
}

// Solve one instance; returns the proven makespan, or -1 if the time ran out.
inline int solveRCPSP_TT2(int group, int exam, const std::string& csv, const std::string& set) {
    std::cout << "started solving TT2: " << group << ":" << exam << std::endl;
    getPetri(petri, group, exam, set);
    getRCPSP(RCPSPex, group, exam, set);
    if (g_tt2_reverse) reverseLoadedInstance();     // backward search: the same solver on the reversed instance
    g_instance_rs = instanceRS(group);
    deriveEndpointNames(initialstatename, finalstatename);   // before ANY root construction
    RCPSPState_TT2 first;
    RCPSPState_TT2 goal = first;                    // A* wants a goal state; RCPSP_TT2::GoalTest ignores it
    // RCPSP_TT2_COMPACT: tables for rebuilding the token lists on this net; off for this instance if it is not supported
    g_tt2_compact_on = (g_tt2_compact && tt2CompactPrepare(first)) ? g_tt2_compact : 0;
    resetTT2Counters(set, group, exam);
    const char* model = g_tt2_reverse ? "TT2bwd" : "TT2fwd";

    {   // trivial root: the EST schedule is resource-feasible => optimal without search
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<int> est; int mk = 0;
        if (tryTrivialRootSchedule_TT2(est, mk)) {
            std::cout << "TT2 trivial-at-root: EST schedule is resource-feasible => optimal makespan=" << mk
                      << " (0 expansions, A* skipped)" << std::endl;
            std::ofstream file(csv, std::ios::app);
            file << group << "," << exam << "," << secs_since(t0) << ",True," << mk << ",0,0,0," << model << "," << set
                 << "," << getPeakMemoryKB();
            writeTT2Tail(file, set, group, exam, true, 0, 0, 0, false);
            g_tt2_compact_on = 0;
            checkAgainstOptimum(set, group, exam, true, mk, mk, csv);
            return mk;
        }
    }

    RCPSP_TT2 env;
    TemplateAStar<RCPSPState_TT2, int, RCPSP_TT2> astar;
    astar.SetReopenNodes(tt2ReopenOn());            // needed for an inconsistent h (Globals.h, tt2ReopenOn)
    std::vector<RCPSPState_TT2> path;
    const auto start = std::chrono::high_resolution_clock::now();
    g_prof_t0 = std::chrono::steady_clock::now(); g_prof_next_trace = 5.0;
    astar.GetPath(&env, first, goal, path);         // stops at the time limit (astar_timeout_seconds) with an empty path
    g_prof_astar_sec = secs_since(g_prof_t0);
    const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
    if (g_tt2_compact_on == 1) for (auto& s : path) if (tt2CompactStripped(s)) tt2CompactRebuild(s);   // full states again

    int makespan = 0;
    if (!path.empty()) {
        makespan = path.back().g;                   // the goal's clock is the makespan
        std::cout << "Path found! {'scheduling': {";   // activity: start time (g of the state that fired it)
        bool firstJob = true;
        for (const auto& s : path) {
            if (s.lastTransitionId <= 0) continue;
            std::cout << (firstJob ? "" : ", ") << "'" << s.lastTransitionId << "': " << s.g;
            firstJob = false;
        }
        std::cout << "}, 'makespan': " << makespan << "}" << std::endl;
    } else
        std::cout << "Path not found or timeout occurred." << std::endl;
    std::cout << "Nodes Expanded: " << astar.GetNodesExpanded() << "  Nodes Touched: " << astar.GetNodesTouched() << std::endl;
    if (g_tt2_dr5) {
        const auto& dt = get_tt2_dominance_table();
        std::cout << "TT2 DR5: pruned=" << dt.pruned << " (before construction " << g_tt2_precheck_cut << ") checks=" << dt.checks
                  << " inserts=" << dt.inserts << " thinned=" << dt.thinned << " buckets=" << dt.bucketCount()
                  << " maxBucket=" << dt.maxBucket << std::endl;
    }
    if (g_tt2_dr4) std::cout << "TT2 DR4: successors pruned = " << g_tt2_dr4_pruned << std::endl;
    if (g_tt2_prof)
        std::cout << "TT2 PROF: astar=" << g_prof_astar_sec << "s succ=" << g_prof_succ_sec << " (avail=" << g_prof_avail_sec
                  << " gen=" << g_prof_gen_sec << " dr5=" << g_prof_dr5_sec << ") heur=" << g_heur_time_sec
                  << " (cp=" << g_prof_hcp_sec << " res=" << g_prof_hres_sec << ") hash=" << g_prof_hash_sec
                  << " other=" << (g_prof_astar_sec - g_prof_succ_sec - g_heur_time_sec - g_prof_hash_sec)
                  << "s maxF=" << g_prof_maxf << " children raw/kept=" << g_prof_children_raw << "/" << g_prof_children_kept << std::endl;

    std::ofstream file(csv, std::ios::app);
    file << group << "," << exam << "," << elapsed.count() << "," << (!path.empty() ? "True" : "False") << "," << makespan
         << "," << astar.GetNodesExpanded() << "," << astar.GetNodesTouched() << "," << path.size() << "," << model << "," << set
         << "," << getPeakMemoryKB();
    writeTT2Tail(file, set, group, exam, false, astar.GetNumOpenItems(), (uint64_t)astar.GetNumItems(), astar.GetUniqueNodesExpanded(), path.empty());
    file.close();
    g_tt2_compact_on = 0;
    if (!path.empty()) checkAgainstOptimum(set, group, exam, true, makespan, makespan, csv);
    return path.empty() ? -1 : makespan;
}
