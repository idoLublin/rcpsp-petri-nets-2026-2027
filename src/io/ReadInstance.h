#pragma once
// Loading one PSPLIB instance: getPetri reads petri.json, getRCPSP reads rcpsp.json, both from
//   <data dir>/json_outputs_<set>/<set><group>_<exam>/
// The data dir is RCPSP_DATA_DIR if set, otherwise the "data" folder of the repository: found from the working
// directory or up to three levels above it (so a run from an IDE build folder works too).

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include "../common/Globals.h"
#include "../../third_party/json.hpp"

using json = nlohmann::json;

// The repository root: the first of ".", "..", "../..", "../../.." that holds data/known_bounds.
inline std::string repoRoot() {
    static const std::string root = [] {
        namespace fs = std::filesystem;
        std::error_code ec;
        fs::path p = fs::current_path(ec);
        for (int up = 0; up < 4 && !ec; ++up) {
            if (fs::exists(p / "data" / "known_bounds", ec)) return p.string();
            if (!p.has_parent_path() || p == p.parent_path()) break;
            p = p.parent_path();
        }
        return std::string(".");
    }();
    return root;
}

inline std::string dataDir() {
    const char* e = std::getenv("RCPSP_DATA_DIR");
    return (e && *e) ? std::string(e) : repoRoot() + "/data";
}

inline std::string instanceFile(const std::string& set, int group, int exam, const char* file) {
    return dataDir() + "/json_outputs_" + set + "/" + set + std::to_string(group) + "_" + std::to_string(exam) + "/" + file;
}

inline bool openInstanceFile(std::ifstream& in, const std::string& path) {
    in.open(path);
    if (in.is_open()) return true;
    std::cerr << "Failed to open " << path << " (run inside the repository, or set RCPSP_DATA_DIR)" << std::endl;
    return false;
}

inline void getPetri(PetriExample& petriExample, int group, int exam, const std::string& set) {
    petriExample.reset();
    petriExample.place_name_to_id.clear();
    std::ifstream input_file;
    if (!openInstanceFile(input_file, instanceFile(set, group, exam, "petri.json"))) std::exit(2);
    json j;
    try {
        input_file >> j;
    } catch (json::exception& e) {
        std::cerr << "JSON parsing failed: " << e.what() << std::endl;
        std::exit(2);
    }

    // Layout: [placeSize, places..., placedictSize, place dicts..., tranSize, transitions..., ...]
    int placeSize = j[0];
    int placedictSize = j[placeSize + 1];
    int tranSize = j[placedictSize + 1 + placeSize + 1];

    for (int i = 1; i <= placeSize; i++) {
        std::string pName = j[i]["name"];
        petriExample.place_name_to_id[pName] = (int)petriExample.places.size();
        std::vector<std::vector<short>> state;
        if (j[i].contains("state") && j[i]["state"].is_array())
            state = j[i]["state"].get<std::vector<std::vector<short>>>();
        else
            state.push_back({0});
        std::unordered_map<std::string, short> arcsIn, arcsOut;
        if (j[i]["arcs_in"].is_object())  arcsIn  = j[i]["arcs_in"].get<std::unordered_map<std::string, short>>();
        if (j[i]["arcs_out"].is_object()) arcsOut = j[i]["arcs_out"].get<std::unordered_map<std::string, short>>();
        petriExample.places.push_back(Place(pName, arcsIn, arcsOut, state));
    }

    // Transitions: arcs are stored as (place id, weight) instead of place names.
    int offset = placedictSize + 1 + placeSize + 1 + 1;
    for (int i = offset; i < offset + tranSize; i++) {
        std::string tNameStr = j[i]["name"];
        int duration = j[i]["duration"];
        Transition tran((short)std::stoi(tNameStr), (short)duration);
        if (j[i].contains("arcs_in") && j[i]["arcs_in"].is_object())
            for (auto& [key, val] : j[i]["arcs_in"].items())
                tran.arcs_in_indices.push_back({(short)petriExample.place_name_to_id.at(key), (short)(int)val});
        if (j[i].contains("arcs_out") && j[i]["arcs_out"].is_object())
            for (auto& [key, val] : j[i]["arcs_out"].items())
                tran.arcs_out_indices.push_back({(short)petriExample.place_name_to_id.at(key), (short)(int)val});
        petriExample.Transitions.push_back(tran);
    }
}

inline void getRCPSP(RCPSP_example& rcpsp_example, int group, int exam, const std::string& set) {
    ++g_rcpsp_load_gen;   // a (re)loaded instance invalidates the cached per-instance tables
    rcpsp_example.reset();
    std::ifstream input_file;
    if (!openInstanceFile(input_file, instanceFile(set, group, exam, "rcpsp.json"))) std::exit(2);
    json j;
    input_file >> j;

    // Layout: [activities..., predecessors{id: [...]}, successors{id: [...]}, transitive precedences [[a,b],...] (not used),
    //          resources{name: capacity}]
    const int nAct = (int)j.size() - 4;
    for (int i = 0; i < nAct; ++i)
        rcpsp_example.addActivity(Activity(j[i]["duration"], j[i]["name"], j[i]["resource_demands"]));

    auto readLists = [&](const json& part, std::vector<std::vector<short>>& out) {
        std::vector<std::pair<int, json>> sorted;
        for (const auto& item : part.items()) sorted.push_back({std::stoi(item.key()), item.value()});
        std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& [key, value] : sorted)
            for (const auto& val : value)
                out[key - 1].push_back(val.is_string() ? (short)std::stoi(val.get<std::string>()) : (short)val.get<int>());
    };
    rcpsp_example.dependencies.resize(nAct);
    rcpsp_example.backword_dependencies.resize(nAct);
    readLists(j[j.size() - 4], rcpsp_example.backword_dependencies);
    readLists(j[j.size() - 3], rcpsp_example.dependencies);

    for (auto& [key, value] : j[j.size() - 1].items())
        rcpsp_example.addResource(key, value.get<int>());
}
