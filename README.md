# IEEE 33-bus planning and operation model (grid-connected and islanded)

Devices: ONR, DG (PV/WT/MT), SC, OLTC (`TAP`), in-line transformers (`TX`), D-STATCOM, UPFC,
EVCS, BSS, demand response (`DRM`), islanding decisions (`ISL`).
Data in a MATPOWER case (`RC_case33.m`), model in MP-Opt-Model (`RC_om_build.m`), solved with
PlatEMO (GA / NSGA-II / NSGA-III) or the built-in `RC_nsga2.m`. The formulation, notation and
experimental setup are in `RC_formulation.docx`.

## Requirements
- MATLAB R2019b or later
- MATPOWER 8.1 (includes MP-Opt-Model 5.0 and MIPS); MATPOWER 7.1+ works with the legacy `opt_model`
- PlatEMO 4.x (optional; without it the built-in solver is used)

## Quick start
```matlab
addpath(genpath('<matpower folder>'));  addpath(genpath('<PlatEMO folder>'));
RC_validate          % load-flow checks (published 33-bus results, MATPOWER runpf)
RC_main              % one case: edit devices, objectives, m, T, island mode at the top
E = RC_experiments({'A','B','C','D','E'}, struct('T', 24, 'N', 100, 'maxFE', 30000, 'runs', 10));
```

## Main options (`opts`)
| field | values |
|---|---|
| `devices` | any of `'ONR' 'DG' 'SC' 'TAP' 'TX' 'DSTATCOM' 'UPFC' 'EVCS' 'BSS' 'DRM' 'ISL'` |
| `objectives` | `'SWCOST' 'INVCOST' 'COST' 'DEGCOST' 'OPCOST' 'TAC' 'ENS' 'SAIDI' 'SAIFI' 'LOSS' 'VD' 'VSI' 'PEAK' 'LVAR' 'PAR' 'EMISSION' 'EVUNMET' 'EVWAIT' 'DRCOST' 'DISCOMF' 'LSHED' 'WLSHED' 'FDEV' 'SAI'` |
| `m` | 1 (weighted single objective, `weights`) or `numel(objectives)` |
| `island_mode` | `'off'` `'outage'` `'optimal'` `'islanded'` |
| `T` (argument) | 24 (hourly) or 1 (peak snapshot) |
| `mpc`, `set` | case struct / `{'dg.nmax', 4; ...}` overrides |
| `solver`, `alg`, `N`, `maxFE` | `'auto'`/`'platemo'`/`'builtin'`, PlatEMO handle, population, budget |
| `validate`, `refine` | MATPOWER check of the best solution; MIPS polish (m = 1) |

Results are written to `results/<case>_<date>/` (MAT, Excel/CSV tables, PNG/FIG figures, summary).

## Files replaced from the previous version
`RC_cases.m` -> `RC_experiments.m`; `RC_validate_fbs.m` -> `RC_validate.m`. All other `RC_*.m`
files of the previous version are updated; the new device decisions change the decision vector,
so results saved with the old code cannot be decoded by the new one.
