#pragma once
// The two views of one RCPSP instance, as loaded from data/json_outputs_<set>/<set><group>_<exam>/:
//   RCPSP_example  (rcpsp.json): activities, precedences, resources
//   PetriExample   (petri.json): the timed Petri net built from it (places, transitions, arcs)
// Activity ids are 1-based everywhere (activity i is activities[i-1] and transition Transitions[i-1]).
// Place ids: 0..3 are the resource places, 4.. the activity (precedence) places.

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace P_RCPSP {

class Activity {
public:
    short duration;
    std::string name;
    std::map<std::string, short> resource_demands;   // resource name -> demand, e.g. "R1": 3
    Activity() : duration(0) {}
    Activity(short dur, const std::string& n, const std::map<std::string, short>& resources)
        : duration(dur), name(n), resource_demands(resources) {}
};

class RCPSP_example {
public:
    std::vector<Activity> activities;
    int activity_len;
    std::vector<std::vector<short>> backword_dependencies;   // predecessors of activity i (index i-1)
    std::vector<std::vector<short>> dependencies;            // successors of activity i (index i-1)
    std::vector<std::pair<std::string, short>> resources;    // (name, capacity)
    std::set<std::pair<std::string, std::string>> deep_dependencies;   // transitive precedences, as in the JSON

    RCPSP_example() : activity_len(0) {}
    // Explicit moves: the bidirectional search swaps the forward and the reversed instance on every side change,
    // and std::swap must be O(1) there.
    RCPSP_example(const RCPSP_example&) = default;
    RCPSP_example& operator=(const RCPSP_example&) = default;
    RCPSP_example(RCPSP_example&&) noexcept = default;
    RCPSP_example& operator=(RCPSP_example&&) noexcept = default;

    void addActivity(const Activity& activity) { activities.push_back(activity); }
    void addResource(const std::string& name, short value) { resources.push_back({name, value}); }
    void reset() {
        activities.clear();
        dependencies.clear();
        backword_dependencies.clear();
        resources.clear();
        activity_len = 0;
        deep_dependencies.clear();
    }
};

class Place {
public:
    std::string name;
    std::unordered_map<std::string, short> arcs_in;
    std::unordered_map<std::string, short> arcs_out;
    std::vector<std::vector<short>> state;   // initial marking
    short duration;

    Place(const std::string& placeName,
          const std::unordered_map<std::string, short>& inputArcs = {},
          const std::unordered_map<std::string, short>& outputArcs = {},
          const std::vector<std::vector<short>>& initialState = {},
          short initialDuration = 0)
        : name(placeName), arcs_in(inputArcs), arcs_out(outputArcs), state(initialState), duration(initialDuration) {}
};

class Transition {
public:
    std::vector<std::pair<short, short>> arcs_in_indices;    // (place id, weight)
    std::vector<std::pair<short, short>> arcs_out_indices;   // (place id, weight)
    short duration;
    short name;
    Transition(short Name, short Duration) : duration(Duration), name(Name) {}
};

class PetriExample {
public:
    std::vector<Place> places;
    int place_len;
    std::vector<Transition> Transitions;
    std::unordered_map<std::string, int> place_name_to_id;

    PetriExample() : place_len(0) {}
    // Same as RCPSP_example: O(1) swaps for the bidirectional search.
    PetriExample(const PetriExample&) = default;
    PetriExample& operator=(const PetriExample&) = default;
    PetriExample(PetriExample&&) noexcept = default;
    PetriExample& operator=(PetriExample&&) noexcept = default;

    void reset() {
        places.clear();
        Transitions.clear();
        place_len = 0;
    }
};

}  // namespace P_RCPSP
