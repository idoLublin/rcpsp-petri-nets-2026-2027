# Data

The PSPLIB single-mode RCPSP sets j30, j60 and j90 (Kolisch & Sprecher 1997), 480 instances each: 48 groups
("parameters") x 10 instances. Every instance has 4 renewable resources and 30 / 60 / 90 real activities plus a
dummy source and sink (32 / 62 / 92 in total). The resource strength RS cycles over the groups: group 1, 5, 9, ...
has RS 0.2, group 2, 6, ... 0.5, then 0.7 and 1.0.

## Instances: `json_outputs_<set>/<set><group>_<exam>/`

Ready to use, no conversion step: the solver reads these two files directly.

- `rcpsp.json` — the instance: one object per activity (`name`, `duration`, `resource_demands`), then the
  predecessor lists, the successor lists, the transitive precedences (not used by the solver) and the resource
  capacities.
- `petri.json` — the timed Petri net of the same instance: the places (`R1`-`R4` marked with the resource
  capacities, one place per precedence arc such as `post_1_pre_2`, the source `_pre_1` marked with one token and
  the sink `post_<n>`) and one transition per activity with its arcs (e.g. j30 1_1: 54 places, 32 transitions).

## Known optima and bounds: `known_bounds/`

| file | contents |
|---|---|
| `known_optima.csv` | the proven optimum per instance where one is known (j30: all 480; j60: 451; j90: 403). Collected from PSPLIB and from exact solvers (Liu et al. 2023 branch and bound, OR-Tools CP-SAT, Chuffed, Gurobi); the `sources` column says which agree. The solver checks every run against it. |
| `j30opt.sm` | PSPLIB: the optimal makespan of every j30 instance (Demeulemeester & Herroelen) |
| `j60lb.sm`, `j90lb.sm` | PSPLIB: best known upper bound (UB) and lower bound (LB) per instance; `*` marks UB = LB (optimal) |
| `j30hrs.sm`, `j60hrs.sm` | PSPLIB: best known heuristic solutions (makespan and author) |

`known_optima.csv` agrees with the PSPLIB files: the same value on every j30 instance and on every j60 / j90
instance PSPLIB marks optimal, and every known optimum lies within PSPLIB's [LB, UB].
