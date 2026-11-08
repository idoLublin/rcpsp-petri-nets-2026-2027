# Heuristics and lower bounds

Every bound here is an **admissible lower bound on the remaining makespan** of a search node, so the heuristic
`h = max(the bounds that are on)` stays admissible and A* stays optimal. All optional bounds are **off by default**
and every one is a flag (see the README's flag table); every flag and counter is written to the CSV.

## Read this first

- **A\* expands every node with f < C\*** (C\* = the optimum). A stronger bound saves work at a node only if it
  lifts that node's f to C\* or above; raising f from 50 to 55 when C\* = 60 saves nothing at that node. On j30 C\*
  is known, so you can count, for any bound, how often it actually crossed that line.
- **Judge a bound by coverage** (instances proven optimal within the time limit). On instances that time out,
  compare `maxFExpanded` (the proven lower bound): fewer expansions in the same time means slower per node, not
  a better search.
- **Measure the search alone.** In the research code a root stage (a CP-style solver run once before the search)
  already applies energetic reasoning and shaving; every instance the per-node deadline bound gains on j60/j90
  is one that stage closes by itself. A new bound has to help on the instances the root cannot close.
- **Where the coverage is lost:** RS 0.2 (tight resources). In the example run, `fwd` proves 47 / 120 RS 0.2
  instances on j60 and 3 / 120 on j90 (all of RS 0.7 and 1.0). On the 78 j30 instances whose root bound falls short
  of the optimum, the median gap to the optimum is 28.5 for the critical path alone, 9 for an exact single-resource
  relaxation and 4 for an exact two-resource relaxation (offline, root only): strong bounds exist, the open problem
  is computing them at every node cheaply enough.
- Numbers below marked *research code* were measured on the research version of this solver (same search, more
  options); numbers marked *this code* were measured on this repository (the j30 list and setup are at the end).

## Always on

### Critical path and resource energy (`petri/Heuristic.h`)

- **What:** `h = max(critical path of the unfinished activities, max over resources of remaining work / capacity)`.
  The critical path ignores resources; the energy bound ignores precedence and timing. In-progress activities count
  with their remaining time.
- **How it does:** consistent and cheap (about 5% of the run time). It is exact at the root on the loose instances
  and weak on the tight ones: almost all large gaps at the root are RS 0.2.

## Promising (in `src/bounds/`)

### Per-node deadline bound — `RCPSP_TT2_TT` (`NodeDeadlineBound.h`)

- **What:** a destructive lower bound at every node. Guess a deadline D for the remaining work (start at h). Give
  each unfinished activity a window [earliest start, latest start that still ends by D]. If a window is narrower
  than twice the duration, part of the activity runs no matter where it starts (its compulsory part); if the
  compulsory parts overload a resource, or tightening the windows against them empties one, D is impossible:
  h = D + 1, try again (up to `_STEPS` = 16 times).
- **How it does (research code, lab server, 300 s, search alone):** j60 387 proven vs 375-378 without it, j90 325
  vs 319-321, 0 wrong. Faster as well: on instances both solve, 0.37x the expansions and about 0.3x the time on
  j60. Gains are at RS 0.5 and 0.7; j60 RS 0.2 does not move (46 vs 46-47). On j30, 23-30% fewer expansions.
- **Its limit:** the proof uses compulsory parts only, and they exist only in tight windows. Once D is one full
  activity duration above the critical path of the remaining work, every activity can dodge any interval, so h can
  never rise more than about one duration (at most 10 on PSPLIB) above that critical path. At RS 0.2 the gap is
  15-25.

### Energetic reasoning in the deadline bound — `RCPSP_TT2_TT_ENERGY` (`NodeEnergetic.h`)

- **What:** an extra proof step in the same loop. Over an interval [t1, t2), an activity must do
  `min(t2 - t1, d, es + d - t1, t2 - ls)` units of work inside it wherever it starts; if the forced work of all
  activities exceeds capacity x length on some resource, D is impossible. It needs no tight window, so it is not
  bound by the limit above. It checks every pair of window ends: O(n²) intervals at O(n R) each.
- **How it does (research code, 24 hard j30 instances, 30 s):** 18% fewer expansions on top of the deadline bound
  (3.63 M → 2.98 M), but the bound's time went from 8.7 s to 200 s, and one instance fewer was solved (23 / 24).
  The research code's root stage uses the same check once per deadline, where it is affordable.
- **Open question:** most of the 18% at a fraction of the cost: fewer intervals (Derrien & Petit, relevant
  intervals), a faster checker (Ouellet & Quimper), reuse from parent to child.

### Single-resource relaxation — `RCPSP_TT2_SINGLERES` (`SubProblemBounds.h`, `SubsetSolver.h`)

- **What:** for each resource k, make every other resource unlimited and solve the remaining work exactly with a
  small A* sub-solver (`SubsetSolver.h`); h = the largest of these optima. A sub-solve stops after `_EXPAND`
  expansions (default 2000) and then returns the best bound it has proven, still admissible.
- **How it does (research code):** on j30 it raised h on 46-60% of the nodes where it ran (groups 1-6, RS 0.2 and
  0.5), 0 wrong; no coverage run was ever made. At the root, the exact single-resource relaxation is a median 9
  below the optimum on the 78 bound-short j30 instances, against 28.5 for the critical path.
- **Cost:** R sub-solves per node.

### Look-ahead on the remaining work — `RCPSP_TT2_LOOKAHEAD` (`SubProblemBounds.h`)

- **What:** the remaining work itself, with every real capacity, solved by the same sub-solver under the expansion
  cap; h = the bound it has proven when it stops (the true remaining optimum if it finishes).
- **How it does (research code, as "real-capacity hierarchical RS bound"):** at the root of j30 1:1 it reached the
  optimum (43, base heuristic 38) with a 257 K-expansion sub-solve. In the search with a 2000 cap: 18% fewer
  expansions on group 1 and 56% on group 5, but the larger searches (groups 9 and 13) timed out at 120 s: the
  per-node sub-solve is the cost wall.

## Archived (in `src/bounds/archive/`, measured as weak)

### Suffix energetic reasoning — `RCPSP_TT2_TT_SER` (`archive/SuffixEnergetic.h`)

- **What:** energetic reasoning on the intervals [t1, D) that end at the deadline only (t1 = each earliest start),
  O(n² R) instead of O(n³ R).
- **How it does (research code, 24 hard j30, with the deadline bound):** 3,555,427 → 3,555,790 expansions (no
  effect) for twice the time (44.8 s → 89.3 s). Sound (125 / 125 j30). The time-table and window tightening already
  catch what it finds.

### Θ-tree bound — `RCPSP_TT2_THETA` (`archive/ThetaBound.h`)

- **What:** Vilím's earliest-completion-time bound per resource, `ceil(max over subsets (C · min est + Σ energy) / C)`,
  computed in O(n log n) with a Θ-tree. Stronger than the energy bound.
- **How it does (research code, j30 with DR5):** it beats the base heuristic at many nodes (61% on group 13) but
  rarely lifts f across the line: 0.7-2.6% fewer expansions, and 9.5% more time on the hard group 13.

## Not in this repository

| Bound | What it is | How it did | Where |
|---|---|---|---|
| Shaving at search nodes | after propagation, test each activity at the ends of its window; a window end that fails propagation is cut | the strongest heavier deadline bound measured: in CBS, root gap on 22 hard j30 16 → 8.5, lab server j60 376 → 387, j90 369 → 372; but 98-99% of the run time | CBS solver only (research repo). Here the same step exists at the prover's root: `refutedWin` in `Propagation.h` |
| Root stage | destructive lower bound + schedule seed + deadline probe + LNS, once before the search | a separate CP-style solver; closes most instances by itself | research repo |
| Inflated RS relaxation | raise every capacity to the next RS level and solve | provably no better than the base energy bound on this model (it only loosens it); the look-ahead above is its real-capacity form | not ported |
| State-equation / time-indexed LP | Bonet's state-equation heuristic; the firing-count form is not admissible for makespan, so a time-indexed LP relaxation instead | +0.14 average over the base heuristic at 0.5 s per node; with minimal-forbidden-set cuts +0.7 at about 5 s: root-only at best | Python prototype, research repo (`lp_root_bound/`) |
| Last year's bounds | LBCS (Klein & Scholl LB3, workload along the critical path), LBcc (critical capacity), LBip0 (LB10, incompatible pairs), LBrc (LB2, resource capacity), LBER (energetic root bound - g) | evaluated last year on TT / TT2; the critical path was best on j30, LBER expanded more nodes than it | last year's repositories (`new_heuristic`, LBER port) |
| Root windows as a prune | the root's latest starts, used at every node | inert: 0 of 768 activities tightened | needs the root stage |

## How "this code" was checked

With every optional bound off, this code gives the same search as before the bounds were added (same makespan and
expansions on 38 j30 instances with `fwd`, 10 with `bwd`, 10 with `hbae`).

Each bound on its own, `fwd`, 38 j30 instances (the 24 slowest of the example run, all RS 0.2 / 0.5, plus 14
others), 60 s per instance, WSL laptop, 0 wrong in every row:

| flags | proven | expansions (sum) | time (sum, s) |
|---|---|---|---|
| default | 38 | 5.27 M | 81 |
| `RCPSP_TT2_TT=1` | 38 | 3.48 M | 80 |
| `+ RCPSP_TT2_TT_ENERGY=1` | 37 | 2.85 M | 480 |
| `+ RCPSP_TT2_TT_SER=1` | 38 | 3.49 M | 103 |
| `RCPSP_TT2_THETA=1` | 38 | 4.90 M | 93 |
| `RCPSP_TT2_SINGLERES=1` (cap 2000) | 10 | 0.02 M | 1725 |
| `RCPSP_TT2_LOOKAHEAD=1` (cap 2000) | 10 | 0.05 M | 1719 |

All of them together with `bwd` and with `hbae` (10 instances each): 1 / 10 proven, 0 wrong. The sub-problem
bounds cut expansions by two orders of magnitude but cost far more per node than they save at a cap of 2000:
making them cheap (smaller caps, gating, reuse) is the open problem.
