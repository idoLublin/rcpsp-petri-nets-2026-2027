#pragma once
// Global state shared by the whole solver: the loaded instance, the run options (environment flags) and the
// per-instance counters written to the CSV. The program is one translation unit (main.cpp includes every
// header once), so plain `inline` globals are enough.
//
// Environment flags are read once in main() (readOptions) and always BY VALUE: RCPSP_X=0 is off, RCPSP_X=1 on.

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include "../io/PetriNet.h"

using namespace P_RCPSP;

// ── The loaded instance ──────────────────────────────────────────────────────
inline PetriExample  petri;
inline RCPSP_example RCPSPex;
inline std::string   finalstatename;     // name of the sink place of the loaded net
inline std::string   initialstatename;   // name of the source place of the loaded net
inline uint64_t      g_rcpsp_load_gen = 0;   // bumped on every load: per-instance caches compare against it
inline double        g_instance_rs = 0.0;    // resource strength of the instance (PSPLIB: by group, 0.2/0.5/0.7/1.0)

// ── Options (see README "Flags") ─────────────────────────────────────────────
inline bool envFlag(const char* name, bool dflt) { const char* e = std::getenv(name); return e ? (std::atoi(e) != 0) : dflt; }
inline long envNum(const char* name, long dflt)  { const char* e = std::getenv(name); return e ? std::atol(e) : dflt; }

// Forward / backward Petri search (TT2)
inline bool g_tt2_dr5 = true;            // RCPSP_TT2_DR5: cutset dominance (DominanceTT2.h)
inline bool g_tt2_dr4 = true;            // RCPSP_TT2_DR4: delayed-start dominance (Environment.h)
inline bool g_tt2_batch = true;          // RCPSP_TT2_BATCH: start a feasible subset of the ready activities per edge
inline long g_tt2_batch_cap = 200000;    // RCPSP_TT2_BATCH_CAP: safety cap on subsets per node (singletons always kept)
inline int  g_tt2_compact = 0;           // RCPSP_TT2_COMPACT: 1 = stored nodes without token lists, 2 = check mode
inline bool g_tt2_node_tt = false;       // RCPSP_TT2_TT: per-node deadline bound (bounds/NodeDeadlineBound.h)
inline long g_tt2_node_tt_steps = 16;    // RCPSP_TT2_TT_STEPS
inline bool g_tt2_node_tt_full = true;   // RCPSP_TT2_TT_FULL: window tightening after the compulsory-part test
inline long g_tt2_node_tt_minact = 2;    // RCPSP_TT2_TT_MINACT: only nodes with at least this many unfinished activities
inline bool g_tt2_prof = false;          // RCPSP_TT2_PROF: time per search phase in the CSV
inline std::FILE* g_prof_trace = nullptr;   // RCPSP_TT2_PROF_TRACE=<file>: a progress snapshot every 5 s

// Bidirectional search (HBAE)
inline bool g_bi_dr4 = true;             // RCPSP_BI_DR4: DR4 on both sides
inline bool g_bi_batch = false;          // RCPSP_BI_BATCH: batch expansion on both sides
inline bool g_bi_refute = false;         // RCPSP_BI_REFUTE: prove each new incumbent optimal with the deadline refuter
inline long g_bi_refute_ms = 2000;       // RCPSP_BI_REFUTE_MS: budget per incumbent
inline double g_bi_refute_share = 0.5;   // RCPSP_BI_REFUTE_SHARE: the prover may use at most this share of the search time

// ── Search-wide state ────────────────────────────────────────────────────────
inline bool g_tt2_reverse = false;   // true while the loaded instance is the reversed one (backward search)
inline int  g_tt2_dom_side = 0;      // which DR5 table is active: 0 forward, 1 backward (HBAE switches it)
inline bool g_tt2_hfresh = false;    // always compute h fresh (HBAE: its stored states keep h = 0, see Environment.h)
inline int  g_tt2_compact_on = 0;    // the RCPSP_TT2_COMPACT value in force for this instance (0 if the net is not supported)

// ── Per-instance counters (reset per instance, written to the CSV) ───────────
inline long   g_tt2_dr4_pruned = 0;              // children DR4 removed
inline long   g_tt2_precheck_cut = 0;            // children DR5 removed before they were built
inline long   g_tt2_batch_close_nodes = 0;       // expansions that skipped their zero-delay subsets (batch close)
inline long   g_tt2_batch_close_skipped = 0;     // feasible zero-delay singletons not built there
inline long   g_tt2_compact_rebuilds = 0;        // expanded nodes whose token lists were rebuilt
inline long   g_tt2_compact_checked = 0;         // check mode: states compared with their rebuild
inline double g_heur_time_sec = 0.0;             // time inside the heuristic
inline long   g_heur_calls = 0;                  // heuristic computations (not the zero-delay shortcut)
inline double g_root_h = -1.0;                   // heuristic value of the first state evaluated

// RCPSP_TT2_PROF: timers run only with the flag (a clock read costs ~20-30 ns); counters always.
inline double g_prof_succ_sec = 0.0;      // whole RCPSP_TT2::GetSuccessors
inline double g_prof_avail_sec = 0.0;     // ready-transition scan
inline double g_prof_gen_sec = 0.0;       // child construction (DR4, batch subsets, DR5 pre-check)
inline double g_prof_dr5_sec = 0.0;       // DR5 over the children that were built
inline double g_prof_hash_sec = 0.0;      // RCPSP_TT2::GetStateHash
inline double g_prof_hcp_sec = 0.0;       // heuristic, critical-path part
inline double g_prof_hres_sec = 0.0;      // heuristic, resource part
inline double g_prof_astar_sec = 0.0;     // whole A* (set by the driver)
inline long   g_prof_succ_calls = 0;      // expansions (GetSuccessors calls)
inline long   g_prof_hash_calls = 0;
inline long   g_prof_children_raw = 0;    // children before DR5
inline long   g_prof_children_kept = 0;   // children after DR5
inline long   g_prof_h_calls_all = 0;     // HCost calls, including the zero-delay shortcut
inline long   g_prof_h_deltazero = 0;     // HCost reused the parent's h (zero-time firing)
inline int    g_prof_maxf = -1;           // largest f expanded so far = the proven lower bound (consistent h)
inline int    g_prof_f_at[5] = {-1, -1, -1, -1, -1};   // g_prof_maxf at 1 / 10 / 60 / 120 / 240 s
inline std::chrono::steady_clock::time_point g_prof_t0;    // search start
inline double g_prof_next_trace = 5.0;
inline int    g_prof_group = 0, g_prof_exam = 0;
inline const char* g_prof_size = "";

inline double secs_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}
