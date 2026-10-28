#pragma once
// RCPSPState_TT2 — a marking of the timed Petri net (the TTPNR model of the SoCS paper), in RELATIVE time:
// every token carries the time until it becomes available, measured from the state's own clock g.
//
//   resource_nodes[r]        tokens of resource place r: (amount, time until available), time-ordered, one entry
//                            per time (canonical, so equal markings compare equal)
//   activity_nodes[p - 4]    token of activity place p: (count, time until available)
//   activeTransitionIndices  activities in progress: (id, remaining time), sorted by id
//   finishedActivitiys       finished activities (bit i = activity i)
//   g                        elapsed time (the clock)
//
// A successor fires one transition t (starts activity t) after a delay d (Environment.h computes d): the clock
// advances by d, t takes its resource demand from tokens that are ready, and returns both its resources and its
// output token with delay = its duration. The state identity is (finished set, active list); the token lists
// follow from it (that is what RCPSP_TT2_COMPACT uses).

#include <algorithm>
#include <array>
#include <bitset>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include "../common/Globals.h"
#include "Heuristic.h"

// Per-instance demand tables of the activities: (resource place id, demand) per activity, and the positive
// demand summed per resource place. Rebuilt when another instance (or the other side's net) is loaded.
struct ActDemandSlot {
    uint64_t gen = ~0ULL; const void* ptr = nullptr; int n = -1;
    std::vector<std::vector<std::pair<short, short>>> ent;   // per activity: (resource place id, demand)
    std::vector<std::array<int, 4>> byPlace;                 // per activity: positive demand per resource place id
    bool placeOK = true;                                    // every positive-demand resource place id is < 4
};
inline const ActDemandSlot& actDemandSlot() {
    static ActDemandSlot slot[2]; static int next = 0;
    const void* ptr = RCPSPex.activities.data();
    const int n = (int)RCPSPex.activities.size();
    for (auto& s : slot) if (s.gen == g_rcpsp_load_gen && s.ptr == ptr && s.n == n) return s;
    ActDemandSlot& s = slot[next]; next ^= 1;
    s.gen = g_rcpsp_load_gen; s.ptr = ptr; s.n = n; s.placeOK = true;
    s.ent.assign(n + 1, {});
    s.byPlace.assign(n + 1, {0, 0, 0, 0});
    for (int a = 1; a <= n; ++a)
        for (const auto& kv : RCPSPex.activities[a - 1].resource_demands) {
            const int pid = petri.place_name_to_id.at(kv.first);
            s.ent[a].push_back({(short)pid, (short)kv.second});
            if (kv.second > 0) { if (pid >= 0 && pid < 4) s.byPlace[a][pid] += kv.second; else s.placeOK = false; }
        }
    return s;
}

class RCPSPState_TT2 {
public:
    std::array<std::vector<std::pair<short, short>>, 4> resource_nodes;
    std::vector<std::pair<short, short>> activity_nodes;
    std::bitset<128> finishedActivitiys;
    std::vector<std::pair<short, short>> activeTransitionIndices;
    bool isDeltaZero = false;    // reached by a firing at delay 0 (false at the root)
    bool batchClosed = false;    // reached by a zero-delay batch edge (see Environment.h); not part of the identity
    short g = 0;
    mutable short h = 0;         // last heuristic value computed for this state
    short predessesor_h = 0;     // the parent's h: a zero-delay child reuses it (Environment.h)
    short lastTransitionId = 0;  // the transition fired to create this state

    RCPSPState_TT2();                                                            // the initial marking
    RCPSPState_TT2(const RCPSPState_TT2& prev, short transitionId, short firingTime);   // fire transitionId after firingTime

    bool operator==(const RCPSPState_TT2& other) const {
        if (finishedActivitiys != other.finishedActivitiys) return false;
        // RCPSP_TT2_COMPACT: stored nodes do not keep the token lists, so the identity is the compact part alone.
        if (g_tt2_compact_on) return activeTransitionIndices == other.activeTransitionIndices;
        if (activity_nodes != other.activity_nodes) return false;
        if (resource_nodes != other.resource_nodes) return false;
        if (activeTransitionIndices != other.activeTransitionIndices) {
            std::cerr << "HASH COLLISION: states equal but different activeTransitionIndices!" << std::endl;
            return false;
        }
        return true;
    }
};

// ── Which transitions can fire, and when ─────────────────────────────────────
// For every unstarted activity whose predecessors are all finished or in progress: the delay d until it can
// start = max(latest finish of an in-progress predecessor, time until its resource demand is available).
// Returns (activity id, d) pairs.
inline std::vector<std::pair<short, short>> getAvailableTransitionIndices_TT2(
    const std::vector<short>& unstartedTransitions,
    const std::bitset<128>& finishedActivitiys,
    const std::array<std::vector<std::pair<short, short>>, 4>& resource_nodes,
    const std::vector<std::pair<short, short>>& activeTransitionIndices) {
    std::vector<std::pair<short, short>> available;
    std::array<short, 130> activeRem; activeRem.fill(-1);
    for (const auto& [id, rem] : activeTransitionIndices) if (id >= 0 && id < 130) activeRem[id] = rem;
    const auto byTime = [](const std::pair<short, short>& a, const std::pair<short, short>& b) { return a.second < b.second; };
    // each resource pool is checked for time order once and only copied + sorted if it is not
    std::array<std::vector<std::pair<short, short>>, 4> sortedCopy;
    std::array<const std::vector<std::pair<short, short>>*, 4> pool{};
    const auto poolOf = [&](int r) -> const std::vector<std::pair<short, short>>& {
        if (!pool[r]) {
            if (std::is_sorted(resource_nodes[r].begin(), resource_nodes[r].end(), byTime)) pool[r] = &resource_nodes[r];
            else { sortedCopy[r] = resource_nodes[r]; std::sort(sortedCopy[r].begin(), sortedCopy[r].end(), byTime); pool[r] = &sortedCopy[r]; }
        }
        return *pool[r];
    };
    const auto& ent = actDemandSlot().ent;
    for (short transId : unstartedTransitions) {
        if (transId >= 0 && transId < 130 && activeRem[transId] != -1) continue;      // already active
        bool allPredsFinished = true;
        int maxPredFinishTime = 0;
        for (int predId : RCPSPex.backword_dependencies[transId - 1]) {
            if (finishedActivitiys.test(predId)) continue;
            const short r = (predId >= 0 && predId < 130) ? activeRem[predId] : (short)-1;
            if (r != -1) maxPredFinishTime = std::max(maxPredFinishTime, (int)r);    // active: wait for it
            else { allPredsFinished = false; break; }                                 // not started: blocked
        }
        if (!allPredsFinished) continue;
        bool resourcesOK = true;
        int maxResourceTime = maxPredFinishTime;
        for (const auto& [resID, demand] : ent[transId]) {
            if (resource_nodes[resID].empty()) { resourcesOK = false; break; }
            const auto& tokens = poolOf(resID);
            int totalAvailable = 0, resourceReadyTime = -1;
            for (const auto& [amt, time] : tokens) if (time <= maxPredFinishTime) totalAvailable += amt;
            if (totalAvailable >= demand) resourceReadyTime = maxPredFinishTime;
            else {
                for (const auto& [amt, time] : tokens) {
                    if (time > maxPredFinishTime) {
                        totalAvailable += amt;
                        if (totalAvailable >= demand) { resourceReadyTime = time; break; }
                    }
                }
            }
            if (resourceReadyTime == -1) { resourcesOK = false; break; }
            maxResourceTime = std::max(maxResourceTime, resourceReadyTime);
        }
        if (resourcesOK) available.emplace_back(transId, maxResourceTime);
    }
    return available;
}

// ── The initial marking ──────────────────────────────────────────────────────
inline RCPSPState_TT2::RCPSPState_TT2() {
    std::unordered_map<int, int> place_to_resource_idx;   // resource place id -> resource index (0-3)
    int res_count = 0;
    for (const auto& [resName, cap] : RCPSPex.resources)
        place_to_resource_idx[petri.place_name_to_id.at(resName)] = res_count++;

    activity_nodes.resize(petri.places.size() - res_count);
    finishedActivitiys.reset();
    int activity_counter = 0;
    for (int i = 0; i < (int)petri.places.size(); ++i) {
        const auto& place = petri.places[i];
        if (place.arcs_out.empty()) finalstatename = place.name;
        if (place.arcs_in.empty())  initialstatename = place.name;
        auto it = place_to_resource_idx.find(i);
        if (it != place_to_resource_idx.end()) {                 // resource place
            const int res_idx = it->second;
            if (place.name == initialstatename) resource_nodes[res_idx].push_back({1, 0});
            else if (!place.state.empty() && !place.state[0].empty()) {
                const int val = place.state[0][0];
                if (val > 0) resource_nodes[res_idx].push_back({(short)val, 0});
            }
        } else {                                                 // activity place
            if (place.name == initialstatename) activity_nodes[activity_counter] = {1, 0};
            else if (!place.state.empty() && !place.state[0].empty()) {
                const int val = place.state[0][0];
                activity_nodes[activity_counter] = (val > 0) ? std::make_pair<short, short>((short)val, 0) : std::make_pair<short, short>(0, 0);
            } else activity_nodes[activity_counter] = {0, 0};
            activity_counter++;
        }
    }
    for (const auto& [resName, cap] : RCPSPex.resources) {       // resource capacities
        if (cap <= 0) continue;
        const int res_idx = place_to_resource_idx[petri.place_name_to_id.at(resName)];
        if (resource_nodes[res_idx].empty()) resource_nodes[res_idx].push_back({cap, 0});
    }
    std::vector<short> tempUnstarted;
    tempUnstarted.reserve(petri.Transitions.size());
    for (int i = 0; i < (int)petri.Transitions.size(); i++)
        if (finishedActivitiys[i + 1] == 0) tempUnstarted.push_back((short)(i + 1));
    h = (short)getForwardHcost(tempUnstarted, activeTransitionIndices);
    predessesor_h = h;
    g = 0;
}

// Advance the clock `dt` units: tokens move closer (clamped at 0 = ready now); running activities that reach 0
// finish (their output tokens were produced at firing time, with the delay that has now elapsed).
inline void tt2AdvanceClock(RCPSPState_TT2& s, short dt) {
    for (auto& resVec : s.resource_nodes)
        for (auto& token : resVec)
            token.second = std::max(0, token.second - dt);
    for (auto& token : s.activity_nodes)
        if (token.first > 0) token.second = std::max(0, token.second - dt);
    auto it = s.activeTransitionIndices.begin();
    while (it != s.activeTransitionIndices.end()) {
        it->second -= dt;
        if (it->second <= 0) { s.finishedActivitiys[it->first] = 1; it = s.activeTransitionIndices.erase(it); }
        else ++it;
    }
}
// Canonical resource pools: time-ordered, one entry per release time, so equal markings compare equal.
inline void tt2CanonicalPools(RCPSPState_TT2& s) {
    for (auto& resVec : s.resource_nodes) {
        if (resVec.empty()) continue;
        if (!std::is_sorted(resVec.begin(), resVec.end(),
                            [](const std::pair<short, short>& a, const std::pair<short, short>& b) { return a.second < b.second; }))
            std::sort(resVec.begin(), resVec.end(), [](const auto& a, const auto& b) {
                if (a.second != b.second) return a.second < b.second;
                return a.first > b.first;
            });
        auto it = resVec.begin();
        while (it != resVec.end() - 1) {
            auto next = it + 1;
            if (it->second == next->second) { it->first += next->first; resVec.erase(next); }
            else ++it;
        }
    }
}

// ── Firing: start activity transitionId after firingTime ─────────────────────
inline RCPSPState_TT2::RCPSPState_TT2(const RCPSPState_TT2& prev, short transitionId, short firingTime) {
    isDeltaZero = (firingTime == 0);
    g = prev.g + firingTime;
    predessesor_h = prev.h;
    lastTransitionId = transitionId;
    // Copy the state. The firing appends one token per output arc to a resource place and one entry to the active
    // list when the activity has a duration; copy those containers with exactly that much room (no reallocation).
    finishedActivitiys = prev.finishedActivitiys;
    {
        int extra[4] = {0, 0, 0, 0};
        for (const auto& [placeID, outAmount] : petri.Transitions[transitionId - 1].arcs_out_indices)
            if (placeID >= 0 && placeID < 4) ++extra[placeID];
        for (int r = 0; r < 4; ++r) {
            if (!extra[r]) { resource_nodes[r] = prev.resource_nodes[r]; continue; }
            resource_nodes[r].reserve(prev.resource_nodes[r].size() + extra[r]);
            resource_nodes[r].assign(prev.resource_nodes[r].begin(), prev.resource_nodes[r].end());
        }
    }
    activity_nodes = prev.activity_nodes;
    if (RCPSPex.activities[transitionId - 1].duration > 0) {
        activeTransitionIndices.reserve(prev.activeTransitionIndices.size() + 1);
        activeTransitionIndices.assign(prev.activeTransitionIndices.begin(), prev.activeTransitionIndices.end());
    } else
        activeTransitionIndices = prev.activeTransitionIndices;

    // 1. advance the clock to the firing time
    if (firingTime > 0) tt2AdvanceClock(*this, firingTime);

    // 2. take the resource demand from the earliest tokens (the canonical sort below makes the order among
    //    equal-time tokens irrelevant, so an already time-ordered pool is not re-sorted)
    const Transition& transition = petri.Transitions[transitionId - 1];
    const short duration = RCPSPex.activities[transitionId - 1].duration;
    const auto consume = [](std::vector<std::pair<short, short>>& tokens, int remainingDemand) {
        const auto byTime = [](const std::pair<short, short>& a, const std::pair<short, short>& b) { return a.second < b.second; };
        if (!std::is_sorted(tokens.begin(), tokens.end(), byTime)) std::sort(tokens.begin(), tokens.end(), byTime);
        auto it = tokens.begin();
        while (remainingDemand > 0 && it != tokens.end()) {
            if (it->first > remainingDemand) { it->first -= remainingDemand; remainingDemand = 0; }
            else { remainingDemand -= it->first; it = tokens.erase(it); }
        }
    };
    for (const auto& [resID, demand] : actDemandSlot().ent[transitionId])
        if (demand > 0) consume(resource_nodes[resID], demand);

    // 3. take the input (precedence) tokens
    for (const auto& [placeID, inAmount] : transition.arcs_in_indices)
        if (placeID >= 4) activity_nodes[placeID - 4] = {0, 0};

    // 4. the activity runs: its resources and output tokens come back as future events, delay = duration
    if (duration > 0) {
        activeTransitionIndices.emplace_back(transitionId, duration);
        for (const auto& [placeID, outAmount] : transition.arcs_out_indices) {
            if (placeID < 4) resource_nodes[placeID].emplace_back(outAmount, duration);
            else {
                const short idx = placeID - 4;
                if (idx < (short)activity_nodes.size() && (activity_nodes[idx].first == 0 || activity_nodes[idx].second > duration))
                    activity_nodes[idx] = {outAmount, duration};
            }
        }
        std::sort(activeTransitionIndices.begin(), activeTransitionIndices.end());
    } else {                                                     // duration 0: finishes at once
        finishedActivitiys[transitionId] = 1;
        for (const auto& [placeID, outAmount] : transition.arcs_out_indices) {
            if (placeID < 4) resource_nodes[placeID].emplace_back(outAmount, 0);
            else if (placeID - 4 < (int)activity_nodes.size()) activity_nodes[placeID - 4] = {outAmount, 0};
        }
    }

    // 5. canonical form
    tt2CanonicalPools(*this);
    if (!activeTransitionIndices.empty() && !std::is_sorted(activeTransitionIndices.begin(), activeTransitionIndices.end()))
        std::sort(activeTransitionIndices.begin(), activeTransitionIndices.end());
}

// ── RCPSP_TT2_COMPACT: the token lists as a function of the compact part ─────
// In the forward model the two token lists of a state follow from its finished set and its active list:
//  * resource place r holds, ready now, its root amount minus what the active activities hold, plus one token per
//    active activity that returns to r (amount = the return arc, delay = the activity's remaining time); the pool
//    is time-ordered with one entry per delay (tt2CanonicalPools);
//  * activity place p (at most one producing and one consuming transition) is empty once its consumer has started;
//    before that it holds the producer's token - delay 0 if the producer has finished, the producer's remaining
//    time if it is active - or the root marking if the producer has not started.
// This rests on three facts. A firing takes its demand from tokens that are ready: it is fired at the delay that
// getAvailableTransitionIndices_TT2 computed. Every transition returns to a resource exactly what it takes, and
// the producer of a place is a predecessor of its consumer: tt2CompactPrepare checks the loaded net for these two
// and refuses it otherwise.
struct TT2CompactTab {
    std::array<std::vector<std::pair<short, short>>, 4> rootRes;
    std::vector<std::pair<short, short>> rootAct;
    std::array<int, 4> total{};                                   // amount in each resource place at the root
    std::vector<std::vector<std::pair<short, short>>> resOut;     // per transition id: (resource place, amount returned)
    std::vector<short> prod, prodW, cons;                         // per activity place: producer, its arc weight, consumer (0 = none)
};
inline TT2CompactTab g_tt2_ctab;

inline bool tt2CompactStripped(const RCPSPState_TT2& s) { return s.activity_nodes.empty(); }   // a full state has one entry per activity place
inline void tt2CompactStrip(RCPSPState_TT2& s) {                  // release the token lists: size and capacity
    for (auto& v : s.resource_nodes) std::vector<std::pair<short, short>>().swap(v);
    std::vector<std::pair<short, short>>().swap(s.activity_nodes);
}

// per instance: tables from the loaded net; false = the net is not supported (compact is then off for it)
inline bool tt2CompactPrepare(const RCPSPState_TT2& root) {
    TT2CompactTab& T = g_tt2_ctab;
    T = TT2CompactTab{};
    const auto no = [](const std::string& why) {
        std::cout << "RCPSP_TT2_COMPACT: off for this instance (" << why << ")" << std::endl;
        return false;
    };
    const int NT = (int)petri.Transitions.size();
    const int NA = (int)root.activity_nodes.size();
    if (NT < 1 || NT > 127) return no("more than 127 transitions");
    if (NA < 1 || NA + 4 != (int)petri.places.size()) return no("the activity places are not places 4 and up");
    if (root.finishedActivitiys.any() || !root.activeTransitionIndices.empty()) return no("the root has started activities");
    T.rootRes = root.resource_nodes;
    T.rootAct = root.activity_nodes;
    for (int r = 0; r < 4; ++r) {
        const auto& v = root.resource_nodes[r];
        if (v.size() > 1 || (v.size() == 1 && (v[0].first <= 0 || v[0].second != 0)))
            return no("root pool of resource place " + std::to_string(r) + " is not one ready token");
        T.total[r] = v.empty() ? 0 : v[0].first;
    }
    T.resOut.assign(NT + 1, {});
    T.prod.assign(NA, 0); T.prodW.assign(NA, 0); T.cons.assign(NA, 0);
    for (int t = 1; t <= NT; ++t) {
        const Transition& tr = petri.Transitions[t - 1];
        std::array<int, 4> out{}, dem{};
        for (const auto& [pid, w] : tr.arcs_out_indices) {
            if (w <= 0 || pid < 0 || pid - 4 >= NA) return no("output arc of transition " + std::to_string(t));
            if (pid < 4) { out[pid] += w; T.resOut[t].push_back({pid, w}); continue; }
            if (T.prod[pid - 4]) return no("place " + std::to_string(pid) + " has two producers");
            T.prod[pid - 4] = (short)t; T.prodW[pid - 4] = w;
        }
        for (const auto& [pid, w] : tr.arcs_in_indices) {
            if (pid < 4) continue;                                // resources are taken through resource_demands
            if (pid - 4 >= NA) return no("input arc of transition " + std::to_string(t));
            if (T.cons[pid - 4]) return no("place " + std::to_string(pid) + " has two consumers");
            T.cons[pid - 4] = (short)t;
        }
        for (const auto& [resName, d] : RCPSPex.activities[t - 1].resource_demands) {
            if (d <= 0) continue;
            const int pid = petri.place_name_to_id.at(resName);
            if (pid < 0 || pid >= 4) return no("a resource place id above 3");
            dem[pid] += d;
        }
        if (dem != out) return no("transition " + std::to_string(t) + " does not return what it takes");
    }
    for (int i = 0; i < NA; ++i) {
        if (!T.prod[i]) continue;
        if (T.rootAct[i] != std::pair<short, short>(0, 0)) return no("place " + std::to_string(i + 4) + " is produced and marked at the root");
        if (!T.cons[i]) continue;
        const auto& preds = RCPSPex.backword_dependencies[T.cons[i] - 1];
        if (std::find(preds.begin(), preds.end(), T.prod[i]) == preds.end())
            return no("the producer of place " + std::to_string(i + 4) + " is not a predecessor of its consumer");
    }
    return true;
}

// set resource_nodes / activity_nodes from the finished set and the active list
inline void tt2CompactRebuild(RCPSPState_TT2& s) {
    const TT2CompactTab& T = g_tt2_ctab;
    std::array<short, 128> rem; rem.fill(-1);                     // remaining time of the active activities
    for (const auto& [id, r] : s.activeTransitionIndices) rem[id] = r;
    const int NA = (int)T.rootAct.size();
    s.activity_nodes.resize(NA);
    for (int i = 0; i < NA; ++i) {
        const short c = T.cons[i], p = T.prod[i];
        if (c && (s.finishedActivitiys[c] || rem[c] != -1)) s.activity_nodes[i] = {0, 0};                // taken by its consumer
        else if (p && s.finishedActivitiys[p])              s.activity_nodes[i] = {T.prodW[i], 0};       // produced, ready
        else if (p && rem[p] != -1)                         s.activity_nodes[i] = {T.prodW[i], rem[p]};  // arrives when the producer ends
        else                                                s.activity_nodes[i] = T.rootAct[i];
    }
    std::array<int, 4> held{};
    for (auto& v : s.resource_nodes) v.clear();
    for (const auto& [id, r] : s.activeTransitionIndices)
        for (const auto& [pid, w] : T.resOut[id]) { held[pid] += w; s.resource_nodes[pid].emplace_back(w, r); }
    for (int r = 0; r < 4; ++r) {
        auto& v = s.resource_nodes[r];
        const int ready = T.total[r] - held[r];
        if (ready < 0) {
            std::cerr << "[TT2-COMPACT] resource place " << r << ": the active activities hold " << held[r] << " of " << T.total[r] << std::endl;
            std::exit(7);
        }
        std::sort(v.begin(), v.end(), [](const std::pair<short, short>& a, const std::pair<short, short>& b) { return a.second < b.second; });
        size_t n = 0;                                             // one entry per delay
        for (size_t i = 0; i < v.size(); ++i) {
            if (n && v[n - 1].second == v[i].second) v[n - 1].first += v[i].first;
            else v[n++] = v[i];
        }
        v.resize(n);
        if (ready > 0) v.insert(v.begin(), std::pair<short, short>((short)ready, (short)0));
    }
}

// check mode (RCPSP_TT2_COMPACT=2): the rebuilt lists must equal the lists s holds, else exit(7)
inline void tt2CompactCheck(const RCPSPState_TT2& s, const char* where) {
    RCPSPState_TT2 r(s);
    tt2CompactRebuild(r);
    ++g_tt2_compact_checked;
    if (r.activity_nodes == s.activity_nodes && r.resource_nodes == s.resource_nodes) return;
    const auto dump = [](const char* name, const std::vector<std::pair<short, short>>& v) {
        std::cerr << "  " << name << ":";
        for (const auto& p : v) std::cerr << " " << p.first << "^" << p.second;
        std::cerr << std::endl;
    };
    std::cerr << "[TT2-COMPACT] rebuilt token lists differ from the kept ones (" << where << "), g=" << s.g
              << " last=" << s.lastTransitionId << std::endl << "  finished:";
    for (int i = 1; i <= (int)petri.Transitions.size(); ++i) if (s.finishedActivitiys[i]) std::cerr << " " << i;
    std::cerr << std::endl;
    dump("active (id^remaining)", s.activeTransitionIndices);
    dump("activity kept   ", s.activity_nodes);
    dump("activity rebuilt", r.activity_nodes);
    for (int k = 0; k < 4; ++k) if (r.resource_nodes[k] != s.resource_nodes[k]) {
        std::cerr << "  resource place " << k << std::endl;
        dump("kept   ", s.resource_nodes[k]);
        dump("rebuilt", r.resource_nodes[k]);
    }
    std::exit(7);
}
