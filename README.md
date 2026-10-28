# Exact RCPSP on timed Petri nets — project 2026-2027

An exact solver for the resource-constrained project scheduling problem (RCPSP) that searches the state space of a
timed Petri net (the TTPNR model of the SoCS 2026 paper) with A*. It proves optimal makespans on the PSPLIB sets
j30, j60 and j90 (480 instances each). Three searches share one model:

| search | what it does | mode |
|---|---|---|
| forward | A* from the empty schedule | `tt2_<set>`, `tt2one` |
| backward | the same A* on the reversed instance (all precedences reversed: same optimum, mirrored schedules) | `tt2rev_<set>`, `tt2rev` |
| bidirectional (HBAE) | both at once; the two frontiers meet on a common cut of a schedule | `tt2hbae_<set>`, `tt2hbae` |

The metric is **coverage**: the number of instances proven optimal within 300 s per instance on one core.

## Quick start

```bash
./build.sh                      # g++ (C++20), about 15 s, produces ./rcpsp
./rcpsp tt2one j30 5 3          # forward search on j30 group 5 instance 3
./rcpsp tt2_j30                 # all 480 j30 instances -> results/TT2fwd_j30_dr5-dr4-batch_v01.csv
```

Always run from the repository root (the instances are read from `data/`, the CSVs written to `results/`).
Every finished instance is compared with `data/known_optima.csv`; a wrong answer stops the run (exit code 3).

## Layout

```
src/main.cpp                    modes, flags (read once, by value), the instance loops
src/common/Globals.h            the loaded instance, all options, the per-instance counters
src/io/                         PetriNet.h (instance classes), ReadInstance.h (JSON reader), Results.h (CSV names, known optima)
src/petri/                      the model and the forward search
    State.h                       the marking, the firing rule, which transitions can fire and when, compact rebuild
    Heuristic.h                   critical-path and resource-energy lower bounds
    Environment.h                 successors (DR4, batch, DR5), the heuristic, the state hash
    DominanceTT2.h                DR5 cutset dominance table
    ForwardSearch.h               one A* run, trivial-root shortcut, CSV row
src/backward/ReverseInstance.h  the backward search = the forward search on the reversed instance
src/bidirectional/              HBAE: BAE_RCPSP.h (the search), HBAE.h (two nets in one search), Prover.h (optional prover)
src/bounds/                     deadline reasoning: Propagation.h, NodeDeadlineBound.h (optional per-node bound)
HOG2/                           the parts of the HOG2 search library the build uses (Nathan Sturtevant, see HOG2/LICENSE.txt)
third_party/json.hpp            nlohmann/json (MIT)
data/                           PSPLIB j30/j60/j90 as rcpsp.json + petri.json per instance; known_optima.csv
examples/                       a full server run of the forward search (see below)
build.sh, run_full.sh           build; run whole sets and summarise them
```

The program is a single translation unit: `main.cpp` includes every header once, so there is nothing to link.

## The model in one paragraph

A state is a marking in **relative time**: every token carries the time until it becomes available, counted from
the state's own clock `g`. The identity of a state is (finished activities, activities in progress with their
remaining time); the token lists follow from it. A successor fires one transition — starts one activity — after the
delay `d` at which its predecessors are done and its resources are available (`getAvailableTransitionIndices_TT2`
in `State.h`): the clock advances by `d`, the activity takes its resources, and returns them with its output token
after its duration. The goal is "every activity finished"; its `g` is the makespan. The heuristic
`max(critical path, resource energy)` of what is left is admissible and consistent, so the first goal A* expands is
optimal.

Three rules shrink the search (all sound, all on by default):
- **DR5 cutset dominance** (`DominanceTT2.h`): two states that started the same set of activities — the one that
  is not later on the clock and on every running activity's finish dominates the other, which is dropped.
- **DR4 delayed start** (`Environment.h`): starting an activity at delay `d` is dominated if another ready activity
  could start earlier and already be finished by `d`.
- **Batch** (`Environment.h`): the activities ready *now* may start together in one edge (every feasible subset),
  which trades depth for branching.

## Flags

Flags are environment variables, read once at start-up **by value** (`RCPSP_X=0` is off, `RCPSP_X=1` on). The run
prints the flags in force, and every flag is written to every CSV row.

| flag | default | what it does |
|---|---|---|
| `RCPSP_TIMEOUT_S` | 300 | time limit per instance (seconds) |
| `RCPSP_ONLY` | — | file with `set,group,exam` lines: run only those instances |
| `RCPSP_RESULTS_DIR` / `RCPSP_OUT` | `results` | output folder / one explicit output file |
| `RCPSP_DATA_DIR` | `data` | where `json_outputs_<set>` and `known_optima.csv` are |
| `RCPSP_TT2_DR5` | 1 | DR5 cutset dominance |
| `RCPSP_TT2_DR4` | 1 | DR4 delayed-start dominance |
| `RCPSP_TT2_BATCH` | 1 | batch expansion (`RCPSP_TT2_BATCH_CAP`, default 200000 subsets per node) |
| `RCPSP_TT2_COMPACT` | 0 | 1: stored nodes keep only (finished, active) and the token lists are rebuilt at expansion — the same search with 2-3x less memory on j60/j90; 2: check mode (rebuild and compare every state) |
| `RCPSP_TT2_TT` | 0 | per-node deadline bound: tries to prove "the rest cannot finish within h" and raises h (j30: 23-30% fewer expansions). Knobs `RCPSP_TT2_TT_STEPS` (16), `_FULL` (1), `_MINACT` (2) |
| `RCPSP_TT2_PROF` | 0 | time per search phase in the CSV (costs 6-12%); `RCPSP_TT2_PROF_TRACE=<file>` adds a snapshot every 5 s |
| `RCPSP_BI_DR4` | 1 | HBAE: DR4 on both sides |
| `RCPSP_BI_BATCH` | 0 | HBAE: batch expansion on both sides |
| `RCPSP_BI_REFUTE` | 0 | HBAE: on each new incumbent U, try to prove that no schedule ends by U-1 (`Prover.h`); `RCPSP_BI_REFUTE_MS` (2000 per incumbent), `RCPSP_BI_REFUTE_SHARE` (0.5 of the search time at most) |

DR5 applies to both sides of HBAE as well. `RCPSP_TT2_COMPACT` applies to the forward and backward search only.

## Output

`results/<solver>_<set>_<flags>_vNN.csv`, e.g. `TT2fwd_j60_dr5-dr4-batch_v01.csv`, `TT2bwd_j30_dr5-dr4-batch_v01.csv`,
`HBAE_j90_dr5-dr4_v01.csv`; `vNN` is the first free number, so nothing is overwritten. One row per instance.
Columns to start with:

| column | meaning |
|---|---|
| `solved` | True = proven optimal within the time limit |
| `makespan` | the proven optimum (TT2: 0 when unsolved; HBAE: -1, and `ub` holds the best schedule found) |
| `time`, `expandNumber`, `maxMem` | seconds, A* expansions, peak memory (KB) |
| `maxFExpanded` | largest f expanded = the proven lower bound (useful on unsolved rows) |
| `trivialAtRoot` | 1 = the earliest-start schedule was already resource-feasible, no search needed |
| `knownOptimum` | from `data/known_optima.csv` (-1 if no method has proven one) |

On an unsolved (timed-out) row, fewer expansions in the same time means slower per node, not a better search:
compare `maxFExpanded` for search quality.

## Example: the forward search on the lab server

`examples/TT2fwd_j30-j60-j90_dr5-dr4-batch_full_run.csv` is a full run of this search (October 2026, 300 s per
instance, one core per set): DR5 + DR4 + batch, compact stored nodes (same search, less memory), profiling on.

| set | proven optimal | RS 0.2 / 0.5 / 0.7 / 1.0 (of 120 each) | wrong | peak memory |
|---|---|---|---|---|
| j30 | 480 / 480 (100%) | 120 / 120 / 120 / 120 | 0 | 0.7 GB |
| j60 | 378 / 480 (78.8%) | 47 / 94 / 117 / 120 | 0 | 22.6 GB |
| j90 | 321 / 480 (66.9%) | 3 / 79 / 119 / 120 | 0 | 24.8 GB |

The coverage is lost at low resource strength (RS 0.2): there the critical-path bound is loose, and that is where
better bounds pay off. The file was written by the older code of the `ido` branch; it keeps the columns this code
writes, and its search is the same as this code's default (checked: same expansions on a j30 sample).

## Running on the lab server

```bash
nohup ./run_full.sh tt2 j30 j60 j90 > run_tt2.txt 2>&1 &     # one process per set, summary at the end
RCPSP_TT2_TT=1 nohup ./run_full.sh tt2hbae j60 > run_hbae.txt 2>&1 &
```

A 300 s process can reach about 50 GB on j60 and 63 GB on j90 without compact nodes: check the free memory for
every set you start, and leave room for the other users of the machine. j30 runs take minutes; j60/j90 take hours.

## Working conventions

- **Every new feature goes behind an environment flag, default off**, read by value, printed at start-up and
  written to the CSV header and rows. The default run must stay the same search.
- **Check a change on a j30 spot list first** (`RCPSP_ONLY=<file>`, a few dozen instances, minutes), and compare
  with the default: a pure speed-up must give the same `expandNumber` on every instance; a new bound or rule must
  give 0 wrong answers. Run full sets on the server only.
- **Report a run only if it has 0 wrong answers.** A wrong answer means a bug: find it, fix it, rerun.
- Judge a feature by coverage (instances proven), and look at `maxFExpanded` on the instances that time out.

## History

This branch is a cut-down copy of the research code of the `ido` branch. Removed (still in that branch's history):
the CBS solver, the root-stage CP solver (destructive bound, SGS seed, probe, LNS), the older Petri models (TT,
the old backward class, the explicit-wait model), HDA*, and the experimental options measured as not helping
(symmetry breaking, lazy dominance, Θ-tree bound, incumbent pruning, single-resource sub-solves, configurable
tie-breaks, diagnostic modes). Speed-ups that keep the search identical are now always on (DR5 tested before a
child is built, the batch "close" rule, array-based heuristics), and HBAE is fixed to the configuration measured best.
