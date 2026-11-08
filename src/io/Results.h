#pragma once
// Result files and run bookkeeping.
//   resultFile        results/<search>_<set>_<flags>_vNN.csv (RCPSP_RESULTS_DIR overrides the folder)
//   onlyInstance      RCPSP_ONLY=<file>: run only the instances listed there, one "set,group,exam" per line
//   checkAgainstOptimum  every finished instance is compared with data/known_bounds/known_optima.csv; a wrong
//                     answer stops the run (exit 3): a run with a wrong answer is not a result

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include "../common/Globals.h"
#include "ReadInstance.h"

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
inline long getPeakMemoryKB() {
    PROCESS_MEMORY_COUNTERS pmc;
    GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));
    return (long)(pmc.PeakWorkingSetSize / 1024);
}
#else
#include <sys/resource.h>
inline long getPeakMemoryKB() {           // peak resident set size of the process so far
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_maxrss;
}
#endif

// PSPLIB j30/j60/j90: the resource strength cycles 0.2, 0.5, 0.7, 1.0 over the groups.
inline double instanceRS(int group) {
    static const double RSL[4] = {0.2, 0.5, 0.7, 1.0};
    return RSL[((group - 1) % 4 + 4) % 4];
}

inline bool onlyInstance(const std::string& set, int g, int e) {
    static const std::set<std::string> keep = [] {
        std::set<std::string> s;
        const char* f = std::getenv("RCPSP_ONLY");
        if (!f || !*f) return s;
        std::ifstream in(f); std::string line;
        while (std::getline(in, line)) {
            std::string t; for (char c : line) if (c != '\r' && c != ' ' && c != '\t') t += c;
            if (!t.empty()) s.insert(t);
        }
        if (s.empty()) { std::cerr << "RCPSP_ONLY: no instances in " << f << std::endl; std::exit(1); }
        std::cout << "RCPSP_ONLY: " << s.size() << " instances from " << f << std::endl;
        return s;
    }();
    return keep.empty() || keep.count(set + "," + std::to_string(g) + "," + std::to_string(e)) > 0;
}

// The optional bounds of this run, for the file name (e.g. "-energy-sr2000").
inline std::string boundTag() {
    std::string t;
    if (g_tt2_node_tt && g_tt2_node_tt_energy) t += "-energy";
    if (g_tt2_node_tt && g_tt2_node_tt_ser) t += "-ser";
    if (g_tt2_theta) t += "-theta";
    if (g_tt2_singleres) t += "-sr" + std::to_string(g_tt2_singleres_expand);
    if (g_tt2_lookahead) t += "-look" + std::to_string(g_tt2_lookahead_expand);
    if ((g_tt2_theta || g_tt2_singleres || g_tt2_lookahead) && g_tt2_bound_maxrs < 1.0)
        t += "-maxrs" + std::to_string((int)std::lround(g_tt2_bound_maxrs * 100));
    if (g_tt2_lbcs) t += "-lbcs";
    if (g_tt2_lbcc) t += "-lbcc";
    if (g_tt2_lbip0) t += "-ip0";
    if (g_tt2_lbrc) t += "-lbrc";
    if (g_tt2_lber > 0) t += "-lber" + std::to_string(g_tt2_lber);
    if (g_tt2_lber_node > 0) t += "-lbernode" + std::to_string(g_tt2_lber_node);
    if (g_tt2_reopen == 0) t += "-noreopen";
    if (g_tt2_reopen == 1) t += "-reopen";
    return t;
}

// The flags of this run, for the file name (e.g. "dr5-dr4-batch-compact").
inline std::string forwardTag() {
    std::string t;
    auto add = [&](const std::string& s) { if (!t.empty()) t += "-"; t += s; };
    if (g_tt2_dr5) add("dr5");
    if (g_tt2_dr4) add("dr4");
    if (g_tt2_batch) add("batch");
    if (g_tt2_compact) add(g_tt2_compact == 2 ? "compactchk" : "compact");
    if (g_tt2_node_tt) add("nodett");
    const std::string b = boundTag();
    if (!b.empty()) add(b.substr(1));
    if (g_tt2_prof) add("prof");
    if (std::getenv("RCPSP_ONLY") && *std::getenv("RCPSP_ONLY")) add("only");
    return t.empty() ? "plain" : t;
}
inline std::string hbaeTag() {
    std::string t = g_tt2_dr5 ? "dr5" : "nodr5";
    if (g_bi_dr4) t += "-dr4";
    if (g_bi_batch) t += "-batch";
    if (g_tt2_node_tt) t += "-nodett";
    t += boundTag();
    if (g_bi_refute) t += "-biref";
    if (g_tt2_prof) t += "-prof";
    if (std::getenv("RCPSP_ONLY") && *std::getenv("RCPSP_ONLY")) t += "-only";
    return t;
}

// <dir>/<search>_<set>_<tag>_vNN.csv with the first free version; the file is created at once (with the header)
// so parallel runs cannot take the same name.
inline std::string resultFile(const std::string& search, const std::string& set, const std::string& tag, const std::string& header) {
    namespace fs = std::filesystem;
    const char* e = std::getenv("RCPSP_RESULTS_DIR");
    const std::string dir = (e && *e) ? std::string(e) : repoRoot() + "/results";
    std::error_code ec;
    fs::create_directories(dir, ec);
    std::string path;
    for (int v = 1; ; ++v) {
        char vb[16]; std::snprintf(vb, sizeof vb, "_v%02d", v);
        path = dir + "/" + search + "_" + set + "_" + tag + vb + ".csv";
        if (!fs::exists(path, ec)) break;
    }
    std::ofstream f(path);
    if (!f.is_open()) { std::cerr << "Cannot write " << path << std::endl; std::exit(1); }
    f << header << "\n";
    return path;
}

// data/known_bounds/known_optima.csv: instance,size,group,exam,optimum,... (j30: all 480; j60/j90: the ones some
// method proved; see data/README.md)
inline int knownOptimum(const std::string& set, int g, int e) {
    static const std::map<std::string, int> opt = [] {
        std::map<std::string, int> m;
        const std::string path = dataDir() + "/known_bounds/known_optima.csv";
        std::ifstream in(path);
        if (!in.is_open()) { std::cerr << "Note: " << path << " not found, results are not checked" << std::endl; return m; }
        std::string line; std::getline(in, line);   // header
        while (std::getline(in, line)) {
            std::stringstream ss(line); std::string inst, size, gs, es, os;
            if (std::getline(ss, inst, ',') && std::getline(ss, size, ',') && std::getline(ss, gs, ',') &&
                std::getline(ss, es, ',') && std::getline(ss, os, ','))
                m[size + "," + gs + "," + es] = std::atoi(os.c_str());
        }
        return m;
    }();
    auto it = opt.find(set + "," + std::to_string(g) + "," + std::to_string(e));
    return it == opt.end() ? -1 : it->second;
}

// A proven makespan must equal the known optimum, and no schedule (proven or not, e.g. an HBAE incumbent) can be
// shorter than it. Anything else is a bug: stop the run so the CSV cannot be mistaken for a result.
inline void checkAgainstOptimum(const std::string& set, int g, int e, bool proven, int makespan, int ub, const std::string& csv) {
    const int opt = knownOptimum(set, g, e);
    if (opt <= 0) return;
    const char* why = nullptr;
    if (proven && makespan != opt) why = makespan > opt ? "proved a makespan ABOVE the known optimum" : "proved a makespan BELOW the known optimum";
    else if (ub >= 0 && ub < opt) why = "found a schedule shorter than the known optimum";
    if (!why) return;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
              << "WRONG RESULT  " << set << " " << g << ":" << e << " -- " << why
              << "\n  makespan = " << makespan << "   ub = " << ub << "   known optimum = " << opt
              << "\n  partial CSV: " << csv
              << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::exit(3);
}
