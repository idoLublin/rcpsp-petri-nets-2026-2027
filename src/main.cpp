// Exact RCPSP solver on timed Petri nets (TTPNR, SoCS 2026) — entry point.
// Build and run from the repository root (see README.md):
//   ./build.sh
//   ./rcpsp tt2_j30                 forward search, all 480 j30 instances
//   ./rcpsp tt2one j30 5 3          forward search, one instance (group 5, exam 3)
// The whole program is one translation unit: this file includes every header once.

#include <cstdlib>
#include <iostream>
#include <string>
#include "bidirectional/HBAE.h"     // includes the forward search (petri/), the backward search and the bounds

static void readOptions() {
    g_tt2_dr5 = envFlag("RCPSP_TT2_DR5", true);
    g_tt2_dr4 = envFlag("RCPSP_TT2_DR4", true);
    g_tt2_batch = envFlag("RCPSP_TT2_BATCH", true);
    g_tt2_batch_cap = envNum("RCPSP_TT2_BATCH_CAP", 200000);
    g_tt2_compact = (int)std::max(0L, std::min(2L, envNum("RCPSP_TT2_COMPACT", 0)));
    g_tt2_node_tt = envFlag("RCPSP_TT2_TT", false);
    g_tt2_node_tt_steps = envNum("RCPSP_TT2_TT_STEPS", 16);
    g_tt2_node_tt_full = envFlag("RCPSP_TT2_TT_FULL", true);
    g_tt2_node_tt_minact = envNum("RCPSP_TT2_TT_MINACT", 2);
    g_tt2_prof = envFlag("RCPSP_TT2_PROF", false);
    if (const char* e = std::getenv("RCPSP_TT2_PROF_TRACE"); e && *e) {   // a progress snapshot every 5 s, appended
        g_prof_trace = std::fopen(e, "a");
        if (g_prof_trace) {
            std::fseek(g_prof_trace, 0, SEEK_END);
            if (std::ftell(g_prof_trace) == 0)
                std::fprintf(g_prof_trace, "size,group,exam,elapsedSec,expansions,maxF,childrenRaw,childrenKept,succSec,heurSec,dr5Sec\n");
        }
    }
    g_bi_dr4 = envFlag("RCPSP_BI_DR4", true);
    g_bi_batch = envFlag("RCPSP_BI_BATCH", false);
    g_bi_refute = envFlag("RCPSP_BI_REFUTE", false);
    g_bi_refute_ms = envNum("RCPSP_BI_REFUTE_MS", 2000);
    if (const char* e = std::getenv("RCPSP_BI_REFUTE_SHARE")) g_bi_refute_share = std::atof(e);
    if (const long t = envNum("RCPSP_TIMEOUT_S", 300); t > 0) astar_timeout_seconds = t;
}

static void printOptions(const std::string& mode) {
    std::cout << "mode " << mode << " | time limit " << astar_timeout_seconds << " s | data " << dataDir() << "\n"
              << "TT2: DR5=" << g_tt2_dr5 << " DR4=" << g_tt2_dr4 << " batch=" << g_tt2_batch << " (cap " << g_tt2_batch_cap << ")"
              << " compact=" << g_tt2_compact << " nodeTT=" << g_tt2_node_tt;
    if (g_tt2_node_tt) std::cout << " (steps " << g_tt2_node_tt_steps << ", full " << g_tt2_node_tt_full << ", minAct " << g_tt2_node_tt_minact << ")";
    std::cout << " prof=" << g_tt2_prof << "\n";
    if (mode.rfind("tt2hbae", 0) == 0)
        std::cout << "HBAE: DR5=" << g_tt2_dr5 << " DR4=" << g_bi_dr4 << " batch=" << g_bi_batch << " prover=" << g_bi_refute
                  << (g_bi_refute ? " (" + std::to_string(g_bi_refute_ms) + " ms, share " + std::to_string(g_bi_refute_share) + ")" : std::string()) << "\n";
}

static int usage() {
    std::cerr << "usage: rcpsp <mode> [args]\n"
                 "  tt2_<set>                     forward search, all instances of j30 | j60 | j90 (or the RCPSP_ONLY list)\n"
                 "  tt2rev_<set>                  backward search (forward search on the reversed instances)\n"
                 "  tt2hbae_<set>                 bidirectional search (HBAE)\n"
                 "  tt2one  <set> <group> <exam>  forward search, one instance\n"
                 "  tt2rev  <set> <group> <exam>  backward search, one instance\n"
                 "  tt2hbae <set> <group> <exam>  bidirectional search, one instance\n"
                 "Flags are environment variables, see README.md.\n";
    return 1;
}

int main(int argc, char* argv[]) {
    if (argc < 2) return usage();
    const std::string mode = argv[1];
    readOptions();
    printOptions(mode);
    // RCPSP_TT2_COMPACT applies to the one-directional search only (HBAE keeps the token lists).
    if (g_tt2_compact && mode.rfind("tt2hbae", 0) == 0) { std::cout << "RCPSP_TT2_COMPACT: ignored in HBAE\n"; g_tt2_compact = 0; }

    if (argc >= 5 && (mode == "tt2one" || mode == "tt2rev" || mode == "tt2hbae")) {
        const std::string set = argv[2];
        const int g = std::atoi(argv[3]), e = std::atoi(argv[4]);
        if (mode == "tt2hbae") {
            const std::string csv = resultFile("HBAE", set, hbaeTag() + "-one", hbaeCsvHeader());
            solveAndWriteHBAE(g, e, set, csv);
            std::cout << "-> " << csv << std::endl;
            return 0;
        }
        g_tt2_reverse = (mode == "tt2rev");
        const std::string csv = resultFile(g_tt2_reverse ? "TT2bwd" : "TT2fwd", set, forwardTag() + "-one", tt2CsvHeader());
        solveRCPSP_TT2(g, e, csv, set);
        std::cout << "-> " << csv << std::endl;
        return 0;
    }
    if (mode.rfind("tt2hbae_", 0) == 0) { runBenchmarkTT2HBAE(mode.substr(8)); return 0; }
    if (mode.rfind("tt2rev_", 0) == 0)  { g_tt2_reverse = true; runBenchmarkTT2(mode.substr(7)); return 0; }
    if (mode.rfind("tt2_", 0) == 0)     { runBenchmarkTT2(mode.substr(4)); return 0; }
    return usage();
}
