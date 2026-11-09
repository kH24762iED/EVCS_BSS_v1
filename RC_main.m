%% =====================================================================
%  Multi-period planning and operation of the IEEE 33-bus distribution
%  network (grid-connected and islanded) with any combination of
%     'ONR' 'DG' 'SC' 'TAP' 'TX' 'DSTATCOM' 'UPFC' 'EVCS' 'BSS' 'DRM' 'ISL'
%  Objectives (m = 1: weighted single objective; m = 2, 3, 4: multi-objective):
%     'SWCOST' 'INVCOST' 'COST' 'DEGCOST' 'OPCOST' 'TAC' 'ENS' 'SAIDI' 'SAIFI'
%     'LOSS' 'VD' 'VSI' 'PEAK' 'LVAR' 'PAR' 'EMISSION' 'EVUNMET' 'EVWAIT'
%     'DRCOST' 'DISCOMF' 'LSHED' 'WLSHED' 'FDEV' 'SAI'
%  Data     : MATPOWER case + device fields (RC_case33)
%  Model    : MP-Opt-Model (RC_om_build): variables, linear and nonlinear
%             constraints, one nonlinear cost per objective
%  Solver   : PlatEMO 4.x (GA / NSGA-II / NSGA-III) or built-in RC_nsga2
%  Check    : MATPOWER runpf of the best solution (RC_validate_solution)
%% =====================================================================
clear; clc; close all;
cd(fileparts(mfilename('fullpath')));
addpath(genpath(cd));
rng(1);

%% ---- 1) data: MATPOWER case with all device parameters -------------------------
mpc = RC_case33();                    % edit mpc.<device> fields here, e.g.
% mpc.dg.nmax  = 3;                     % up to 3 DGs (0..3 are installed)
% mpc.dg.allowed = [1 3];               % PV, WT and micro-turbine; [1 3] allows PV and MT
% mpc.island.outage = [18 21];          % main-grid outage window (island_mode 'outage')

%% ---- 2) study settings -------------------------------------------------------------
T                = 24;                % 24 = hourly day, 1 = peak-hour snapshot
opts.mpc         = mpc;
opts.devices     = {'ONR', 'DG', 'SC', 'TAP', 'DSTATCOM', 'BSS', 'DRM', 'ISL'};  %all_devices={'ONR','DG','SC','TAP','TX','DSTATCOM','UPFC','EVCS','BSS','DRM','ISL'};
opts.objectives  = {'TAC', 'LOSS', 'LSHED'};
% {'SWCOST','INVCOST','COST','DEGCOST','OPCOST','TAC','ENS','SAIDI','SAIFI', ...
%             'LOSS','VD','VSI','PEAK','LVAR','PAR','EMISSION','EVUNMET','EVWAIT', ...
%             'DRCOST','DISCOMF','LSHED','WLSHED','FDEV','SAI'};
opts.m           = 3;                 % 1 -> weighted sum with opts.weights
opts.weights     = [1 1 1] / 3;       % used only when m = 1
opts.island_mode = 'outage';          % 'off' | 'outage' | 'optimal' | 'island ed'
opts.encoding    = 'loop';            % ONR encoding: 'loop' or 'binary'

%% ---- 3) solver ---------------------------------------------------------------------
opts.solver   = 'auto';               % 'platemo' | 'builtin' | 'auto'
opts.N        = 100;                  % population
opts.maxFE    = 300;                % evaluations (raise for T = 24 with many devices)
opts.validate = true;                 % MATPOWER runpf check of the best solution
opts.refine   = false;                % m = 1 only: MP-Opt-Model NLP polish

%% ---- 4) solve, report, plot, save ----------------------------------------------------
R    = RC_solve(T, opts);
RC_om_eval(R.P, 'display');           % MP-Opt-Model formulation summary
Tb   = RC_report(R);
figs = RC_plot_results(R);
RC_save_results(R, [], Tb, figs);


%% ----- 5) RC experiments
clear; clc; close all;
addpath(genpath(cd));
rng(1);
study.T=24;
study.N=100;
study.maxFE=40000;
study.runs=1;
study.solver='builtin';
study.alg=@NSGAII;
study.mpc=RC_case33();
% study.folder=()
E = RC_experiments('all', study);
test=1;

