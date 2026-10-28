#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// DominanceTT2.h — cutset (DR5) dominance for the TT2 / TTPNR A* search
// (Demeulemeester & Herroelen 1992 cutset rule; Liu et al. C&OR 151 (2023) rule 5).
// Checked and inserted inside RCPSP_TT2::GetSuccessors (Environment.h). Switch: RCPSP_TT2_DR5 (default on).
//
// Rule. Two states share a bucket iff they have the same scheduled set
// S = finished ∪ active ids. Define the release of activity i in state X:
//   r_X(i) = g_X + rem_X(i)  if i is ACTIVE in X,   r_X(i) = g_X  if i is finished.
// Then  A ⪰ B  (A dominates B, prune B)  iff  g_A ≤ g_B  and  r_A(i) ≤ r_B(i) ∀ i∈S.
// Since A,B share S, an id active in A is either active in B (bound g_B+rem_B) or
// finished in B (bound g_B). Finished-in-A ids need no check (r_A = g_A ≤ g_B).
// Every completion of B replays from A with componentwise no-later finishes, so
// A's best makespan ≤ B's. Active lists are sorted by id (state constructor invariant)
// => one merge scan suffices.
//
// g is stored at insert time. In A* g can only DECREASE later (better path / reopen), and a
// smaller true g_A only makes A dominate MORE — so a stale (too-high) stored g can only make
// us miss a prune, never make a wrong one.
//
// Skyline thinning: on insert, stored entries dominated by the new state are dropped from the
// table (transitivity keeps the pruning power).
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>
#include "State.h"

class TT2DominanceTable {
public:
    // Scheduled-set bitset (finished ∪ active ids) packed into two 64-bit words.
    struct Key {
        uint64_t lo, hi;
        bool operator==(const Key& o) const { return lo == o.lo && hi == o.hi; }
    };
    struct KeyHash {
        size_t operator()(const Key& k) const {
            uint64_t h = k.lo + 0x9e3779b97f4a7c15ULL;
            h = (h ^ (h >> 30)) * 0xbf58476d1ce4e5b9ULL;
            h ^= k.hi + 0x9e3779b97f4a7c15ULL;
            h = (h ^ (h >> 27)) * 0x94d049bb133111ebULL;
            return (size_t)(h ^ (h >> 31));
        }
    };

    // An entry keeps its active ids and RELEASES r(i) = g + rem(i) inline (no heap block per entry);
    // more than K actives spill to one array. Two necessary conditions reject most pairs before the
    // merge scan:  A dominates B  =>  g_A <= g_B  and  Rmax_A <= Rmax_B,
    // where Rmax_X = max over the scheduled set of r_X(i) (= g_X if nothing is active).
    static constexpr int K = 12;
    struct FEntry {
        short g = 0, side = 0, n = 0, R = 0;
        short id[K]; short rel[K];
        std::unique_ptr<short[]> big;             // n > K: ids in [0,n), releases in [n,2n)
        const short* ids()  const { return n <= K ? id  : big.get(); }
        const short* rels() const { return n <= K ? rel : big.get() + n; }
    };

    static bool sameEntry(int gA, int nA, const short* idA, const short* relA, int gB, int nB, const short* idB, const short* relB) {
        if (gA != gB || nA != nB) return false;
        for (int j = 0; j < nA; ++j) if (idA[j] != idB[j] || relA[j] != relB[j]) return false;
        return true;
    }
    // does A (gA, releases relA over ids idA) dominate B? Same scheduled set assumed.
    static bool domFast(int gA, int nA, int RA, const short* idA, const short* relA,
                        int gB, int nB, int RB, const short* idB, const short* relB) {
        if (gA > gB || RA > RB) return false;
        int ib = 0;
        for (int ia = 0; ia < nA; ++ia) {
            while (ib < nB && idB[ib] < idA[ia]) ++ib;
            const int bound = (ib < nB && idB[ib] == idA[ia]) ? (int)relB[ib] : gB;   // active in both : finished in B
            if (relA[ia] > bound) return false;
        }
        return true;
    }

    // Per-instance counters (reported in the CSV).
    uint64_t crossSide = 0;   // prunes where the entry came from the OTHER search (must stay 0)
    uint64_t checks = 0, comparisons = 0, pruned = 0, inserts = 0, thinned = 0;
    uint64_t equalPruned = 0; // prunes by an entry EQUAL to the state (same g, same running set, same releases): duplicates
    size_t   maxBucket = 0;

    void clear() {
        fbuckets.clear();
        crossSide = checks = comparisons = pruned = inserts = thinned = equalPruned = 0;
        maxBucket = 0;
    }

    static Key MakeKey(const RCPSPState_TT2& s) {
        std::bitset<128> sched = s.finishedActivitiys;
        for (const auto& a : s.activeTransitionIndices) sched.set(a.first);
        static const std::bitset<128> low64(~0ULL);
        Key k;
        k.lo = (sched & low64).to_ullong();
        k.hi = (sched >> 64).to_ullong();
        return k;
    }

    // True if `s` is dominated by a stored entry (=> prune). Otherwise Pareto-thins
    // the bucket by `s` and inserts `s`. One call per generated child, in GetSuccessors.
    bool check_and_insert(const RCPSPState_TT2& s) {
        const int n = (int)std::min<size_t>(s.activeTransitionIndices.size(), 128);
        short sid[128], srel[128];
        for (int j = 0; j < n; ++j) {
            sid[j]  = s.activeTransitionIndices[j].first;
            srel[j] = (short)(s.g + s.activeTransitionIndices[j].second);
        }
        return check_and_insert_raw(MakeKey(s), s.g, n, sid, srel);
    }

    // The same check on the state's DR5 view only: scheduled set k, g, and the active ids (ascending) with their
    // releases g + rem. RCPSP_TT2::GetSuccessors uses it to test a child BEFORE building it.
    bool check_and_insert_raw(const Key& k, short g, int n, const short* sid, const short* srel) {
        ++checks;
        int R = g;
        for (int j = 0; j < n; ++j) if (srel[j] > R) R = srel[j];
        auto& bucket = fbuckets[k];
        for (size_t i = 0; i < bucket.size(); ++i) {
            const FEntry& e = bucket[i];
            ++comparisons;
            if (domFast(e.g, e.n, e.R, e.ids(), e.rels(), g, n, R, sid, srel)) {
                ++pruned;
                if (sameEntry(e.g, e.n, e.ids(), e.rels(), g, n, sid, srel)) ++equalPruned;
                // Forward and reversed states live on DIFFERENT nets, so the bidirectional search keeps one table
                // per side (g_tt2_dom_side); a prune across sides would be unsound. This catches it if they mix.
                if (e.side != (short)g_tt2_dom_side) {
                    ++crossSide;
                    if (crossSide == 1)
                        std::cerr << "[DR5 CROSS-SIDE PRUNE] entry.side=" << e.side
                                  << " current side=" << g_tt2_dom_side << std::endl;
                }
                // Move-to-front: an entry that prunes a state is swapped to the front of its bucket, so repeat
                // dominators are met first. Decisions depend only on the SET of entries, so they do not change.
                if (i > 0) std::swap(bucket[0], bucket[i]);
                return true;
            }
        }
        for (size_t i = 0; i < bucket.size();) {                 // skyline
            ++comparisons;
            const FEntry& e = bucket[i];
            if (domFast(g, n, R, sid, srel, e.g, e.n, e.R, e.ids(), e.rels())) {
                if (i + 1 != bucket.size()) bucket[i] = std::move(bucket.back());
                bucket.pop_back();
                ++thinned;
            } else ++i;
        }
        bucket.emplace_back();
        FEntry& ne = bucket.back();
        ne.g = g; ne.side = (short)g_tt2_dom_side; ne.n = (short)n; ne.R = (short)R;
        if (n <= K) { for (int j = 0; j < n; ++j) { ne.id[j] = sid[j]; ne.rel[j] = srel[j]; } }
        else { ne.big.reset(new short[2 * n]); for (int j = 0; j < n; ++j) { ne.big[j] = sid[j]; ne.big[n + j] = srel[j]; } }
        ++inserts;
        if (bucket.size() > maxBucket) maxBucket = bucket.size();
        return false;
    }

    size_t bucketCount() const { return fbuckets.size(); }

private:
    std::unordered_map<Key, std::vector<FEntry>, KeyHash> fbuckets;
};

// One table per search side, cleared per instance by the driver. The bidirectional search selects the
// side with g_tt2_dom_side (0 = forward, 1 = backward), so the two directions keep separate tables.
inline TT2DominanceTable& get_tt2_dominance_table() {
    static TT2DominanceTable t0, t1;
    return (g_tt2_dom_side == 0) ? t0 : t1;
}
