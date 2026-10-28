#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// BAE_RCPSP — HOG2's bidirectional search (generic/BAE.h: Sadhukhan 2013; Alcázar, Riddle & Barley 2020),
// adapted to search the forward net and the reversed net at the same time (bidirectional/HBAE.h).
//
// Changes against HOG2's BAE.h:
//  1. MEET lookup separated from node IDENTITY. Stock BAE* uses env->GetStateHash for both, which is right for a
//     reversible graph. Our two directions are two DIFFERENT nets: identity is the FULL marking, while the meet
//     is on the frame-independent normalised CUT key (forward states sit on START events, reversed states on
//     FINISH events, so their full markings never coincide). So each queue keeps a side index meetHash -> node id
//     and cross-queue lookups go through it. The environment supplies GetMeetHash(state) next to GetStateHash.
//  2. The configuration measured best on the server (the old flags RCPSP_BI_PRIO=1, RCPSP_BI_STOP=max,
//     RCPSP_BI_ACTIVETB=1, RCPSP_BI_NIP=0, RCPSP_BI_ALT=0) is built in:
//       - priority = f = g + h (BAE*'s b = 2g + h - h_opp is in the git history of the ido branch);
//       - side choice: Pohl's cardinality rule (expand the side with the smaller open list);
//       - no lazy nipping, and nodes with f >= U are kept (they are what lets min f rise to U);
//       - stop as soon as U <= max(min f_F, min f_B): while U > C*, each side keeps an open node on an optimal
//         path, so each side's min f is <= C* on its own;
//       - a side whose open list empties ends the search ("drained"; the driver certifies it, see HBAE.h).
//  3. A closed node reached more cheaply is reopened (BAE.h asserted this away with exit(0); zero-cost edges make
//     it legal here). The time limit is astar_timeout_seconds.
//  4. An optional optimality prover (RCPSP_BI_REFUTE, bidirectional/Prover.h) is called on each new incumbent.
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>
#include "../../HOG2/algorithms/AStarOpenClosed.h"
#include "../../HOG2/search/Heuristic.h"
#include "../../HOG2/utils/FPUtil.h"

#ifdef __linux__
#include <cstdio>
#include <unistd.h>
// Resident set size right now, from /proc/self/statm (peakRSSMB column).
inline long bae_rcpsp_rss_kb() {
    long size = 0, res = 0;
    std::FILE* f = std::fopen("/proc/self/statm", "r");
    if (!f) return 0;
    if (std::fscanf(f, "%ld %ld", &size, &res) != 2) res = 0;
    std::fclose(f);
    return res * (sysconf(_SC_PAGESIZE) / 1024);
}
#else
inline long bae_rcpsp_rss_kb() { return 0; }
#endif

// Open-list order: smaller priority (f) first; on ties a zero-delay child first (finish a zero-delay run before
// the rest of its layer), then higher g, more finished activities, more active activities. Reaches into the
// wrapper state (.data.s), so this copy of BAE* is RCPSP-specific by design.
template <class state>
struct BAECompare_RCPSP {
    bool operator()(const AStarOpenClosedData<state>& i1, const AStarOpenClosedData<state>& i2) const {
        if (!fequal(i1.h, i2.h)) return fgreater(i1.h, i2.h);   // the queue is a max-heap: "greater" = later
        if (i1.data.s.isDeltaZero != i2.data.s.isDeltaZero) return !i1.data.s.isDeltaZero;
        if (!fequal(i1.g, i2.g)) return fless(i1.g, i2.g);
        const size_t f1 = i1.data.s.finishedActivitiys.count(), f2 = i2.data.s.finishedActivitiys.count();
        if (f1 != f2) return f1 < f2;
        return i1.data.s.activeTransitionIndices.size() < i2.data.s.activeTransitionIndices.size();
    }
};

template <class state, class action, class environment, class priorityQueue = AStarOpenClosed<state, BAECompare_RCPSP<state>>>
class BAE_RCPSP {
public:
    // gcd: greatest common divisor of the edge costs (1 here, costs are integers); epsilon: cheapest edge (0 here,
    // a firing at delay 0 is a legal zero-cost edge).
    explicit BAE_RCPSP(double epsilon_ = 0.0, double gcd_ = 1.0) : epsilon(epsilon_), gcd(gcd_) {}

    void GetPath(environment* env, const state& from, const state& to,
                 Heuristic<state>* forward, Heuristic<state>* backward, std::vector<state>& thePath) {
        if (!InitializeSearch(env, from, to, forward, backward, thePath)) return;
        while (!DoSingleSearchStep(thePath)) {}
    }

    double GetSolutionCost() const { return currentCost; }
    const state& GetMiddleNode() const { return middleNode; }
    bool StoppedByBound() const { return stoppedByBound; }   // true = U <= max(min f_F, min f_B) (or the prover's proof)
    bool StoppedByFloor() const { return stoppedByFloor; }   // the prover proved U optimal
    int  GetDrainedSide() const { return drainedSide; }      // 0 none, 1 forward open list empty, 2 backward

    // The prover is called on each new incumbent U: 1 = no schedule ends by U - 1 (U optimal), 0 = it found a
    // checked schedule of makespan newU < U (it becomes the incumbent), -1 = neither within its budget.
    void SetOptimalityProver(std::function<int(double, std::chrono::steady_clock::time_point, double&)> f) { proveOptimal = std::move(f); }
    void SetRefShare(double sh) { refShare = sh; }   // the prover may use at most this share of the search's elapsed time
    long GetRefCalls() const { return refCalls; }
    long GetRefProved() const { return refProved; }
    long GetRefAdopted() const { return refAdopted; }
    double GetRefSec() const { return refSec; }

    // ── statistics, read by the driver after GetPath (none of it affects the search) ──
    uint64_t GetNodesExpanded() const { return nodesExpanded; }
    uint64_t GetUniqueNodesExpanded() const { return uniqueNodesExpanded; }
    uint64_t GetNodesTouched() const { return nodesTouched; }
    uint64_t GetReopenedNodes() const { return reopenedNodes; }
    uint64_t GetSolutionUpdates() const { return solutionUpdates; }
    uint64_t GetExpanded(bool fwd) const { return fwd ? expandedFwd : expandedBwd; }
    double   GetSideSec(bool fwd) const { return fwd ? sideSecFwd : sideSecBwd; }
    size_t   GetOpenCount(bool fwd) { return fwd ? forwardQueue.OpenSize() : backwardQueue.OpenSize(); }
    double   GetMinFOpenNow(bool fwd) {           // lowest f among open nodes now (-1 if that side has none)
        priorityQueue& q = fwd ? forwardQueue : backwardQueue;
        if (q.OpenSize() == 0) return -1;
        const double m = minTrueF(fwd);
        return (m >= DBL_MAX) ? -1 : m;
    }
    long     GetPeakRSSKB() const { return peakRSSKB; }
    int64_t  GetMeetExp() const { return meetExp; }            // expansion that produced the final incumbent
    int64_t  GetFirstMeetExp() const { return firstMeetExp; }  // ... and the first incumbent
    double   GetFirstU() const { return firstU; }
    uint64_t GetMeetHits() const { return meetHits; }          // generated nodes whose cut matched the other side
    int      GetMeetFromFwd() const { return meetFromFwd; }    // 1 forward generated it, 0 backward, -1 no meet
    double   GetMeetGF() const { return meetGF; }              // forward half of the incumbent (time from the start)
    double   GetMeetGB() const { return meetGB; }              // backward half (time to the end)

private:
    bool InitializeSearch(environment* env_, const state& from, const state& to,
                          Heuristic<state>* forward, Heuristic<state>* backward, std::vector<state>& thePath);
    bool DoSingleSearchStep(std::vector<state>& thePath);
    void Expand(priorityQueue& current, priorityQueue& opposite,
                std::unordered_map<uint64_t, uint64_t>& currentMeet, std::unordered_map<uint64_t, uint64_t>& oppositeMeet,
                std::vector<double>& currentF, Heuristic<state>* heuristic, const state& target);

    // The solution path: forward queue from the start to the meeting node, then the backward queue to the goal.
    void ExtractPath(std::vector<state>& thePath) {
        std::vector<state> pFor, pBack;
        auto walk = [&](priorityQueue& q, std::unordered_map<uint64_t, uint64_t>& meet, std::vector<state>& out) {
            auto it = meet.find(env->GetMeetHash(middleNode));
            if (it == meet.end()) return;
            uint64_t node = it->second;
            do { out.push_back(q.Lookup(node).data); node = q.Lookup(node).parentID; } while (q.Lookup(node).parentID != node);
            out.push_back(q.Lookup(node).data);
        };
        walk(backwardQueue, backwardMeet, pBack);
        walk(forwardQueue, forwardMeet, pFor);
        std::reverse(pFor.begin(), pFor.end());
        thePath = pFor;
        if (!pBack.empty()) thePath.insert(thePath.end(), pBack.begin() + 1, pBack.end());
    }

    // ── exact min f over OPEN, without scanning OPEN ──
    // A lazy min-heap of (f, id), pushed whenever a node's f is set or lowered. An entry is stale once that node's
    // f changed or the node left OPEN; stale tops are popped on read. (f is tracked next to the queue's priority
    // field so that the stop rule does not depend on what the priority is.)
    typedef std::pair<double, uint64_t> FEntry;
    typedef std::priority_queue<FEntry, std::vector<FEntry>, std::greater<FEntry>> FHeap;
    std::vector<double> trueFF, trueFB;
    FHeap fHeapF, fHeapB;
    void setTrueF(std::vector<double>& v, uint64_t id, double f) {
        if (id >= v.size()) v.resize(id + 1, DBL_MAX);
        v[id] = f;
        (&v == &trueFF ? fHeapF : fHeapB).push(FEntry(f, id));
    }
    void shiftTrueF(std::vector<double>& v, uint64_t id, double delta) {
        if (id < v.size() && v[id] != DBL_MAX) {
            v[id] -= delta;
            (&v == &trueFF ? fHeapF : fHeapB).push(FEntry(v[id], id));
        }
    }
    double minTrueF(bool fwd) {
        priorityQueue& q = fwd ? forwardQueue : backwardQueue;
        std::vector<double>& v = fwd ? trueFF : trueFB;
        FHeap& hp = fwd ? fHeapF : fHeapB;
        while (!hp.empty()) {
            const FEntry& t = hp.top();
            if (t.second < v.size() && v[t.second] == t.first && q.Lookup(t.second).where == kOpenList) return t.first;
            hp.pop();
        }
        return DBL_MAX;
    }
    void recordMeet(bool fromFwd, double gOwn, double gOpp) {   // called whenever the incumbent improves
        meetFromFwd = fromFwd ? 1 : 0;
        meetGF = fromFwd ? gOwn : gOpp;
        meetGB = fromFwd ? gOpp : gOwn;
        meetExp = (int64_t)nodesExpanded;
        if (firstMeetExp < 0) { firstMeetExp = meetExp; firstU = gOwn + gOpp; }
    }

    priorityQueue forwardQueue, backwardQueue;
    std::unordered_map<uint64_t, uint64_t> forwardMeet, backwardMeet;   // meetHash -> node id, one per side
    state goal, start;
    state middleNode;                  // meeting point of the current solution
    double currentCost = DBL_MAX;      // best solution so far (U)
    std::vector<state> neighbors;
    environment* env = nullptr;
    Heuristic<state>* forwardHeuristic = nullptr;
    Heuristic<state>* backwardHeuristic = nullptr;
    double epsilon, gcd;
    std::chrono::steady_clock::time_point timeout, searchStart;

    uint64_t nodesTouched = 0, nodesExpanded = 0, uniqueNodesExpanded = 0, reopenedNodes = 0, solutionUpdates = 0;
    uint64_t expandedFwd = 0, expandedBwd = 0;
    double sideSecFwd = 0.0, sideSecBwd = 0.0;
    long peakRSSKB = 0;
    bool stoppedByBound = false, stoppedByFloor = false;
    int drainedSide = 0;
    int64_t meetExp = -1, firstMeetExp = -1;
    double firstU = -1, meetGF = -1, meetGB = -1;
    uint64_t meetHits = 0;
    int meetFromFwd = -1;
    // prover (RCPSP_BI_REFUTE)
    std::function<int(double, std::chrono::steady_clock::time_point, double&)> proveOptimal;
    double lbFloor = 0.0;              // proven lower bound: the prover sets it to U when it proves U optimal
    double refShare = 0.5, refTriedBudget = 0.0, refTriedU = -1.0, refSec = 0.0;
    long refCalls = 0, refProved = 0, refAdopted = 0;
};

template <class state, class action, class environment, class priorityQueue>
bool BAE_RCPSP<state, action, environment, priorityQueue>::InitializeSearch(environment* env_, const state& from, const state& to,
                                                                            Heuristic<state>* forward, Heuristic<state>* backward,
                                                                            std::vector<state>& thePath) {
    env = env_;
    forwardHeuristic = forward;
    backwardHeuristic = backward;
    currentCost = DBL_MAX;
    lbFloor = 0.0;
    stoppedByBound = stoppedByFloor = false;
    drainedSide = 0;
    forwardQueue.Reset();
    backwardQueue.Reset();
    nodesTouched = nodesExpanded = uniqueNodesExpanded = reopenedNodes = solutionUpdates = 0;
    thePath.resize(0);
    start = from;
    goal = to;
    if (start == goal) return false;
    forwardMeet.clear(); backwardMeet.clear();
    trueFF.clear(); trueFB.clear();
    fHeapF = FHeap(); fHeapB = FHeap();
    expandedFwd = expandedBwd = 0; sideSecFwd = sideSecBwd = 0.0;
    meetExp = firstMeetExp = -1; firstU = meetGF = meetGB = -1; meetHits = 0; meetFromFwd = -1;
    peakRSSKB = bae_rcpsp_rss_kb();
    searchStart = std::chrono::steady_clock::now();
    timeout = searchStart + std::chrono::seconds(astar_timeout_seconds);   // the per-instance budget (TemplateAStar.h)

    const double hF = forwardHeuristic->HCost(start, goal);
    const uint64_t idF = forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, hF);
    forwardMeet[env->GetMeetHash(start)] = idF;
    setTrueF(trueFF, idF, hF);
    const double hB = backwardHeuristic->HCost(goal, start);
    const uint64_t idB = backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, hB);
    backwardMeet[env->GetMeetHash(goal)] = idB;
    setTrueF(trueFB, idB, hB);
    return true;
}

// One step: check whether the search is over, and if not expand one node. Returns true when done.
template <class state, class action, class environment, class priorityQueue>
bool BAE_RCPSP<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector<state>& thePath) {
    // a side exhausted its open list without any solution: nothing found within the budget
    if ((forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0) && currentCost == DBL_MAX) return true;
    if (std::chrono::steady_clock::now() > timeout) return true;

    // Prover: a new incumbent above the proven floor - try to prove it optimal (no schedule ends by U - 1). A
    // schedule the prover finds becomes the incumbent and is tried at once (a few rounds per step). Time: the
    // prover may use at most refShare of the search's elapsed time; an incumbent is retried once the allowance
    // has doubled.
    for (int round = 0; round < 8 && proveOptimal && currentCost < DBL_MAX && currentCost > lbFloor; ++round) {
        if ((nodesExpanded & 63) != 0 && round == 0 && currentCost == refTriedU) break;   // cheap test most steps
        const auto r0 = std::chrono::steady_clock::now();
        const double allow = refShare * std::chrono::duration<double>(r0 - searchStart).count() - refSec;
        if (allow < 0.01) break;
        if (currentCost == refTriedU && allow < 2.0 * refTriedBudget) break;
        refTriedU = currentCost; refTriedBudget = allow; ++refCalls;
        double newU = currentCost;
        const auto lim = std::min(timeout, r0 + std::chrono::microseconds((long long)(allow * 1e6)));
        const int r = proveOptimal(currentCost, lim, newU);
        refSec += std::chrono::duration<double>(std::chrono::steady_clock::now() - r0).count();
        if (r == 1) { lbFloor = currentCost; ++refProved; break; }
        if (r == 0 && newU < currentCost) { currentCost = newU; meetFromFwd = -1; ++refAdopted; ++solutionUpdates; continue; }
        break;
    }
    if (lbFloor > 0 && currentCost < DBL_MAX && currentCost <= lbFloor) {   // the prover proved the incumbent optimal
        stoppedByBound = stoppedByFloor = true; drainedSide = 0;
        if (meetFromFwd >= 0) ExtractPath(thePath);   // an incumbent from the prover has no meeting node
        return true;
    }
    if ((nodesExpanded & 4095) == 0) { const long r = bae_rcpsp_rss_kb(); if (r > peakRSSKB) peakRSSKB = r; }

    // stop rule: U <= max(min f_F, min f_B)
    if (currentCost < DBL_MAX && forwardQueue.OpenSize() > 0 && backwardQueue.OpenSize() > 0) {
        const double mf = minTrueF(true), mb = minTrueF(false);
        if (mf < DBL_MAX && mb < DBL_MAX && currentCost <= std::ceil(std::max(mf, mb) / gcd) * gcd) {
            stoppedByBound = true; drainedSide = 0;
            if (meetFromFwd >= 0) ExtractPath(thePath);
            return true;
        }
    }
    // a side ran out of open nodes while there is an incumbent
    if (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0) {
        stoppedByBound = false;
        drainedSide = forwardQueue.OpenSize() == 0 ? 1 : 2;
        if (meetFromFwd >= 0) ExtractPath(thePath);
        return true;
    }

    // expand one node of the side with the smaller open list (Pohl's cardinality criterion)
    const auto t0 = std::chrono::steady_clock::now();
    const uint64_t e0 = nodesExpanded;
    const bool fwd = !(forwardQueue.OpenSize() > backwardQueue.OpenSize());
    if (fwd) Expand(forwardQueue, backwardQueue, forwardMeet, backwardMeet, trueFF, forwardHeuristic, goal);
    else     Expand(backwardQueue, forwardQueue, backwardMeet, forwardMeet, trueFB, backwardHeuristic, start);
    if (nodesExpanded > e0) {
        const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (fwd) { expandedFwd += nodesExpanded - e0; sideSecFwd += dt; }
        else     { expandedBwd += nodesExpanded - e0; sideSecBwd += dt; }
    }
    return false;
}

// Expand the best open node of `current`: generate its successors, add or relink them, and check each new or
// improved node against the opposite side's meet index (a match is a complete schedule: a new incumbent if cheaper).
template <class state, class action, class environment, class priorityQueue>
void BAE_RCPSP<state, action, environment, priorityQueue>::Expand(priorityQueue& current, priorityQueue& opposite,
                                                                  std::unordered_map<uint64_t, uint64_t>& currentMeet,
                                                                  std::unordered_map<uint64_t, uint64_t>& oppositeMeet,
                                                                  std::vector<double>& currentF, Heuristic<state>* heuristic,
                                                                  const state& target) {
    if (current.OpenSize() == 0) return;
    const uint64_t nextID = current.Close();
    const bool fwdSide = (&current == &forwardQueue);
    nodesExpanded++;
    if (!current.Lookup(nextID).reopened) uniqueNodesExpanded++;
    env->SetStateG(current.Lookup(nextID).data, current.Lookup(nextID).g);
    env->GetSuccessors(current.Lookup(nextID).data, neighbors);

    // a node of this side with cost gOwn and cut hash mh: index it for the opposite side (best g per cut) and
    // look for the opposite side's node with the same cut
    auto meet = [&](uint64_t id, uint64_t mh, double gOwn, const state& succ) {
        auto mit = currentMeet.find(mh);
        if (mit == currentMeet.end() || fless(gOwn, current.Lookup(mit->second).g)) currentMeet[mh] = id;
        auto it = oppositeMeet.find(mh);
        if (it == oppositeMeet.end()) return;
        const dataLocation loc = opposite.Lookup(it->second).where;
        if (loc != kOpenList && loc != kClosedList) return;
        ++meetHits;
        const double gOpp = opposite.Lookup(it->second).g;
        if (fless(gOwn + gOpp, currentCost)) {
            solutionUpdates++;
            currentCost = gOwn + gOpp;
            middleNode = succ;
            recordMeet(fwdSide, gOwn, gOpp);
        }
    };

    for (auto& succ : neighbors) {
        nodesTouched++;
        uint64_t childID = 0;
        const uint64_t hash = env->GetStateHash(succ);
        const dataLocation loc = current.Lookup(hash, childID);
        const double gNew = current.Lookup(nextID).g + env->GCost(current.Lookup(nextID).data, succ);
        switch (loc) {
            case kClosedList: {          // reached more cheaply after it was expanded: relink and reopen
                auto& childData = current.Lookup(childID);
                if (fless(gNew, childData.g)) {
                    reopenedNodes++;
                    const double gDiff = childData.g - gNew;
                    childData.parentID = nextID;
                    childData.g = gNew;
                    childData.h -= gDiff;            // the priority f drops with g (h is static per state)
                    shiftTrueF(currentF, childID, gDiff);
                    env->SetStateG(childData.data, childData.g);
                    current.Reopen(childID);
                }
                break;
            }
            case kOpenList: {            // a cheaper path to an open node: relink, and it may now be the best for its cut
                auto& childData = current.Lookup(childID);
                if (fless(gNew, childData.g)) {
                    const double gDiff = childData.g - gNew;
                    childData.parentID = nextID;
                    childData.g = gNew;
                    childData.h -= gDiff;
                    shiftTrueF(currentF, childID, gDiff);
                    env->SetStateG(childData.data, childData.g);
                    current.KeyChanged(childID);
                    meet(childID, env->GetMeetHash(succ), gNew, succ);
                }
                break;
            }
            case kNotFound: {
                const double h = std::max(heuristic->HCost(succ, target), epsilon);
                const uint64_t newID = current.AddOpenNode(succ, hash, gNew, gNew + h, nextID);
                setTrueF(currentF, newID, gNew + h);
                meet(newID, env->GetMeetHash(succ), gNew, succ);
                break;
            }
        }
    }
}
