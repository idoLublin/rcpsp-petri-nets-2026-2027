#pragma once
// ── Backward search = the forward search on the reversed instance ────────────────────────────────────────────
// RCPSP is symmetric under precedence reversal: reversing every precedence (and swapping source and sink) gives
// an instance with the SAME optimal makespan, whose schedules are the original ones mirrored in time. Reversing
// the loaded net in place turns the forward solver into a correct backward search, with all its dominance rules
// and its heuristic (Heuristic.h detects the reversed orientation by itself).
// Used by ./rcpsp bwd and by the backward side of HBAE (bidirectional/HBAE.h).

#include <string>
#include <utility>
#include <vector>
#include "../common/Globals.h"

inline void reverseLoadedInstance() {
    // 1. swap successor and predecessor lists (the forward code reads backword_dependencies)
    std::swap(RCPSPex.dependencies, RCPSPex.backword_dependencies);
    // 2. flip each transition's ACTIVITY arcs (place id >= 4): old outputs -> inputs, old inputs -> outputs.
    //    Resources stay symmetric: keep the resource-return arcs (arcs_out, place id < 4) and drop the old
    //    resource-consume arcs in arcs_in (consumption goes through resource_demands, which is unchanged).
    for (auto& t : petri.Transitions) {
        std::vector<std::pair<short, short>> nin, nout;
        for (auto& pr : t.arcs_out_indices) { if (pr.first >= 4) nin.push_back(pr); else nout.push_back(pr); }
        for (auto& pr : t.arcs_in_indices)  { if (pr.first >= 4) nout.push_back(pr); }
        t.arcs_in_indices  = std::move(nin);
        t.arcs_out_indices = std::move(nout);
    }
    // 3. flip the places' arcs so source and sink swap (the state constructor finds the source by empty arcs_in,
    //    the sink by empty arcs_out), then clear the OLD source's initial token so it does not seed a spurious
    //    token at the new sink; the new source (the old sink) is tokened by name in the constructor.
    std::string oldSource;
    for (auto& p : petri.places) if (p.arcs_in.empty()) { oldSource = p.name; break; }
    for (auto& p : petri.places) std::swap(p.arcs_in, p.arcs_out);
    for (auto& p : petri.places) if (p.name == oldSource) p.state = {{0}};
}

// The source / sink place names of the CURRENTLY loaded net. initialstatename / finalstatename are globals that
// the RCPSPState_TT2 constructor both writes and reads while it scans the places, so a stale value from the other
// net would seed a spurious token. Call this after loading or reversing and before building a root state.
inline void deriveEndpointNames(std::string& ini, std::string& fin) {
    for (const auto& pl : petri.places) {
        if (pl.arcs_in.empty())  ini = pl.name;
        if (pl.arcs_out.empty()) fin = pl.name;
    }
}
