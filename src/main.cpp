// Exact RCPSP solver on timed Petri nets (TTPNR, SoCS 2026) — entry point.
//   ./rcpsp <search> <set> [<group> [<exam>]]      (run ./rcpsp help for the details and the flags)
// The whole program is one translation unit: this file includes every header once.

#include <cstdlib>
#include <iostream>
#include <string>
#include "bidirectional/HBAE.h"     // includes the forward search (petri/), the backward search and the bounds

static const char* kUsage =
    "usage: ./rcpsp <search> <set> [<group> [<exam>]]\n"
    "  search  fwd    A* on the Petri net, forward in time\n"
    "          bwd    the same A* on the reversed instance (backward in time; same optimum)\n"
    "          hbae   bidirectional: forward and backward at once, meeting in the middle\n"
    "  set     j30 | j60 | j90 (PSPLIB, 48 groups x 10 instances)\n"
    "  group   1-48: only that group; with exam 1-10: only that instance\n"
    "Results: results/<search>_<set>_<flags>_vNN.csv, one row per instance; every proven makespan is checked\n"
    "against data/known_bounds/known_optima.csv and a wrong one stops the run (exit code 3).\n"
    "Examples:\n"
    "  ./rcpsp fwd j30                 all 480 j30 instances\n"
    "  ./rcpsp hbae j60 12 3           one instance, prints its schedule\n"
    "  RCPSP_TIMEOUT_S=60 RCPSP_TT2_TT=1 ./rcpsp fwd j30 5\n";

static const char* kFlags =
    "Flags (environment variables, 0 = off, 1 = on):\n"
    "  RCPSP_TIMEOUT_S=300       time limit per instance, seconds\n"
    "  RCPSP_ONLY=<file>         run only the instances listed there, one 'set,group,exam' per line\n"
    "  RCPSP_RESULTS_DIR=results output folder;  RCPSP_DATA_DIR=data  input folder\n"
    "  RCPSP_TT2_DR5=1           cutset dominance (all searches)\n"
    "  RCPSP_TT2_DR4=1           delayed-start dominance (fwd, bwd)\n"
    "  RCPSP_TT2_BATCH=1         start a feasible subset of the ready activities per edge (fwd, bwd)\n"
    "  RCPSP_TT2_COMPACT=0       1 = keep stored nodes compact (same search, less memory), 2 = check mode (fwd, bwd)\n"
    "  RCPSP_TT2_TT=0            per-node deadline bound (all searches); _STEPS=16 _FULL=1 _MINACT=2\n"
    "  RCPSP_TT2_TT_ENERGY=0     with RCPSP_TT2_TT: energetic reasoning as an extra proof step (_SER=0: its suffix form)\n"
    "  RCPSP_TT2_THETA=0         Θ-tree bound in the heuristic (all searches)\n"
    "  RCPSP_TT2_SINGLERES=0     single-resource relaxation in the heuristic; _EXPAND=2000 per sub-solve (0 = no cap)\n"
    "  RCPSP_TT2_LOOKAHEAD=0     capped look-ahead of the remaining work in the heuristic; _EXPAND=2000\n"
    "  RCPSP_TT2_BOUND_MAXRS=1   run THETA / SINGLERES / LOOKAHEAD only on instances with RS <= this\n"
    "  RCPSP_TT2_LBCS=0 _LBCC=0 _LBIP0=0 _LBRC=0   last year's bounds in the heuristic (bounds/LastYearBounds.h)\n"
    "  RCPSP_TT2_LBER=0          last year's energetic bound, root value - g, depth 1-4; _LBER_NODE=0: at every node\n"
    "  RCPSP_TT2_REOPEN=-1       reopen closed nodes (fwd, bwd); -1 = on when an optional bound needs it, 0 off, 1 on\n"
    "  (what each bound does and how it performed: HEURISTICS.md)\n"
    "  RCPSP_TT2_PROF=0          time per search phase in the CSV; RCPSP_TT2_PROF_TRACE=<file> a snapshot every 5 s\n"
    "  RCPSP_BI_DR4=1            hbae: delayed-start dominance on both sides\n"
    "  RCPSP_BI_BATCH=0          hbae: batch expansion on both sides\n"
    "  RCPSP_BI_REFUTE=0         hbae: try to prove each new incumbent optimal; _MS=2000 per incumbent, _SHARE=0.5\n";

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
    g_tt2_node_tt_energy = envFlag("RCPSP_TT2_TT_ENERGY", false);
    g_tt2_node_tt_ser = envFlag("RCPSP_TT2_TT_SER", false);
    g_tt2_theta = envFlag("RCPSP_TT2_THETA", false);
    g_tt2_singleres = envFlag("RCPSP_TT2_SINGLERES", false);
    g_tt2_singleres_expand = envNum("RCPSP_TT2_SINGLERES_EXPAND", 2000);
    g_tt2_lookahead = envFlag("RCPSP_TT2_LOOKAHEAD", false);
    g_tt2_lookahead_expand = envNum("RCPSP_TT2_LOOKAHEAD_EXPAND", 2000);
    if (const char* e = std::getenv("RCPSP_TT2_BOUND_MAXRS")) g_tt2_bound_maxrs = std::atof(e);
    g_tt2_lbcs = envFlag("RCPSP_TT2_LBCS", false);
    g_tt2_lbcc = envFlag("RCPSP_TT2_LBCC", false);
    g_tt2_lbip0 = envFlag("RCPSP_TT2_LBIP0", false);
    g_tt2_lbrc = envFlag("RCPSP_TT2_LBRC", false);
    g_tt2_lber = std::max(0L, std::min(4L, envNum("RCPSP_TT2_LBER", 0)));
    g_tt2_lber_node = std::max(0L, std::min(4L, envNum("RCPSP_TT2_LBER_NODE", 0)));
    g_tt2_reopen = std::max(-1L, std::min(1L, envNum("RCPSP_TT2_REOPEN", -1)));
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

static void printOptions(const std::string& search, const std::string& set) {
    std::cout << search << " " << set << " | time limit " << astar_timeout_seconds << " s | data " << dataDir() << "\n";
    if (search == "hbae")
        std::cout << "flags: DR5=" << g_tt2_dr5 << " DR4=" << g_bi_dr4 << " batch=" << g_bi_batch;
    else
        std::cout << "flags: DR5=" << g_tt2_dr5 << " DR4=" << g_tt2_dr4 << " batch=" << g_tt2_batch << " compact=" << g_tt2_compact;
    std::cout << " nodeTT=" << g_tt2_node_tt;
    if (g_tt2_node_tt) std::cout << " (steps " << g_tt2_node_tt_steps << ", full " << g_tt2_node_tt_full << ", minAct " << g_tt2_node_tt_minact
                                 << ", energy " << g_tt2_node_tt_energy << ", ser " << g_tt2_node_tt_ser << ")";
    std::cout << " theta=" << g_tt2_theta << " singleRes=" << g_tt2_singleres;
    if (g_tt2_singleres) std::cout << " (cap " << g_tt2_singleres_expand << ")";
    std::cout << " lookAhead=" << g_tt2_lookahead;
    if (g_tt2_lookahead) std::cout << " (cap " << g_tt2_lookahead_expand << ")";
    if (g_tt2_theta || g_tt2_singleres || g_tt2_lookahead) std::cout << " boundMaxRS=" << g_tt2_bound_maxrs;
    std::cout << " lbcs=" << g_tt2_lbcs << " lbcc=" << g_tt2_lbcc << " lbip0=" << g_tt2_lbip0 << " lbrc=" << g_tt2_lbrc
              << " lber=" << g_tt2_lber << " lberNode=" << g_tt2_lber_node;
    if (search != "hbae") std::cout << " reopen=" << tt2ReopenOn();
    if (search == "hbae") {
        std::cout << " prover=" << g_bi_refute;
        if (g_bi_refute) std::cout << " (" << g_bi_refute_ms << " ms per incumbent, share " << g_bi_refute_share << ")";
    }
    std::cout << " prof=" << g_tt2_prof << "\n";
}

static int usage(const char* why = nullptr) {
    if (why) std::cerr << why << "\n\n";
    std::cerr << kUsage << "\n" << kFlags;
    return 1;
}

int main(int argc, char* argv[]) {
    if (argc >= 2 && (std::string(argv[1]) == "help" || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        std::cout << kUsage << "\n" << kFlags;
        return 0;
    }
    if (argc < 3 || argc > 5) return usage();
    const std::string search = argv[1], set = argv[2];
    if (search != "fwd" && search != "bwd" && search != "hbae") return usage(("unknown search '" + search + "'").c_str());
    if (set != "j30" && set != "j60" && set != "j90") return usage(("unknown set '" + set + "'").c_str());
    const int group = argc >= 4 ? std::atoi(argv[3]) : 0, exam = argc >= 5 ? std::atoi(argv[4]) : 0;
    if (argc >= 4 && (group < 1 || group > 48)) return usage("group must be 1-48");
    if (argc >= 5 && (exam < 1 || exam > 10)) return usage("exam must be 1-10");

    readOptions();
    if (search == "hbae") g_tt2_compact = 0;   // compact stored nodes apply to the one-directional search only
    // the backward search is the forward search on the reversed instance (backward/ReverseInstance.h)
    g_tt2_reverse = (search == "bwd");
    printOptions(search, set);

    std::string tag = (search == "hbae") ? hbaeTag() : forwardTag();
    if (group) tag += "-g" + std::to_string(group) + (exam ? "e" + std::to_string(exam) : "");
    const std::string csv = resultFile(search, set, tag, search == "hbae" ? hbaeCsvHeader() : tt2CsvHeader());
    std::cout << "results -> " << csv << std::endl;

    // all 48 x 10 instances, or the chosen group / instance; RCPSP_ONLY narrows it further
    int runs = 0, proven = 0;
    for (int g = group ? group : 1; g <= (group ? group : 48); g++)
        for (int e = exam ? exam : 1; e <= (exam ? exam : 10); e++) {
            if (!onlyInstance(set, g, e)) continue;
            ++runs;
            if (search == "hbae") { if (solveAndWriteHBAE(g, e, set, csv).proven) ++proven; }
            else if (solveRCPSP_TT2(g, e, csv, set) >= 0) ++proven;
        }
    std::cout << "\n" << search << " " << set << ": " << runs << " instances, " << proven << " proven optimal, "
              << runs - proven << " not within " << astar_timeout_seconds << " s, none against the known optima\n"
              << "CSV: " << csv << std::endl;
    return 0;
}
