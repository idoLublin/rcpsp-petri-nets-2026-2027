#pragma once
// Optional lower bounds max'ed into the TT2 heuristic (RCPSP_TT2::HCost in petri/Environment.h), before the
// per-node deadline bound. All off by default; HEURISTICS.md says what each does and how it performed.
//   RCPSP_TT2_THETA      Θ-tree earliest-completion bound              archive/ThetaBound.h
//   RCPSP_TT2_SINGLERES  single-resource relaxation, solved exactly    SubProblemBounds.h
//   RCPSP_TT2_LOOKAHEAD  capped exact look-ahead of the remaining work SubProblemBounds.h
// RCPSP_TT2_BOUND_MAXRS (default 1.0 = every instance): run these three only on instances with resource strength
// <= this value. They are expensive, and loose resources (high RS) are where the plain heuristic is already tight.
// The energetic options of the deadline bound (RCPSP_TT2_TT_ENERGY / _SER) are in NodeDeadlineBound.h; their
// columns are written here too.

#include <chrono>
#include <ostream>
#include <string>
#include <vector>
#include "LastYearBounds.h"
#include "NodeDeadlineBound.h"
#include "SubProblemBounds.h"
#include "archive/ThetaBound.h"

namespace extra {

// the instance's time limit, for the sub-solves (set when the instance starts)
inline std::chrono::steady_clock::time_point& until() { static std::chrono::steady_clock::time_point t; return t; }

inline bool anyOn() { return g_tt2_theta || g_tt2_singleres || g_tt2_lookahead; }

inline void reset(long long timeoutSec) {
    until() = std::chrono::steady_clock::now() + std::chrono::seconds(timeoutSec);
    theta::reset();
    subb::reset();
    ly::reset(timeoutSec);
}

// base = the heuristic so far (remaining makespan); returns max(base, the bounds that are on).
inline double raise(double base, const std::vector<short>& unfinished, const std::vector<std::pair<short, short>>& active) {
    if (g_instance_rs > g_tt2_bound_maxrs + 1e-9) return base;
    if (g_tt2_theta) {
        const auto t0 = std::chrono::steady_clock::now();
        const int v = theta::bound(unfinished, active);
        theta::sec() += secs_since(t0);
        if (v > base) { base = v; ++theta::better(); }
    }
    if (g_tt2_singleres) {
        const int v = subb::singleResource(unfinished, active, g_tt2_singleres_expand, until());
        if (v > base) { base = v; ++subb::st().srBetter; }
    }
    if (g_tt2_lookahead) {
        const int v = subb::lookAhead(unfinished, active, g_tt2_lookahead_expand, until());
        if (v > base) { base = v; ++subb::st().laBetter; }
    }
    return base;
}

// CSV columns (inserted before knownOptimum in every result file): the flags, then the counters.
inline const char* csvColumns() {
    return "tt2NodeTTEnergy,tt2NodeTTSer,nodeTtEnergyFail,nodeTtSerFail,tt2Theta,tt2SingleRes,tt2SingleResExpand,"
           "tt2LookAhead,tt2LookAheadExpand,tt2BoundMaxRS,thetaBetter,thetaSec,singleResCalls,singleResBetter,"
           "singleResCapped,singleResExpands,singleResSec,lookAheadCalls,lookAheadBetter,lookAheadCapped,"
           "lookAheadExpands,lookAheadSec,";
}
// the same plus last year's bounds (LastYearBounds.h)
inline std::string allCsvColumns() { return std::string(csvColumns()) + ly::csvColumns();
}
inline void writeCsv(std::ostream& o) {
    const auto& nb = tt2nb::st();
    const auto& s = subb::st();
    o << g_tt2_node_tt_energy << "," << g_tt2_node_tt_ser << "," << nb.energyFail << "," << nb.serFail << ","
      << g_tt2_theta << "," << g_tt2_singleres << "," << g_tt2_singleres_expand << ","
      << g_tt2_lookahead << "," << g_tt2_lookahead_expand << "," << g_tt2_bound_maxrs << ","
      << theta::better() << "," << theta::sec() << ","
      << s.srCalls << "," << s.srBetter << "," << s.srCapped << "," << s.srExpands << "," << s.srSec << ","
      << s.laCalls << "," << s.laBetter << "," << s.laCapped << "," << s.laExpands << "," << s.laSec << ",";
    ly::writeCsv(o);
}

}  // namespace extra
