function E = RC_experiments(groups, study)
%RC_EXPERIMENTS  Experimental study: each device alone and in combination,
%grid-connected and islanded operation, single- and multi-objective
%formulations, 24-h profile and peak snapshot. Every case is solved,
%reported, plotted and saved; a comparison table and comparison figures
%are written to results/experiments_<date>/.
%
%   E = RC_experiments()                       all groups, default maxFE
%   E = RC_experiments({'A','C'})              selected groups
%   E = RC_experiments('all', study)           study.T, .N, .maxFE, .runs,
%                                                                  .solver, .mpc, .folder
%
%   Group A  individual devices, grid-connected (T = 24)
%   Group B  combinations of devices, grid-connected (T = 24)
%   Group C  islanded operation: outage, microgrid formation, optimal
%            (intentional) islanding, standalone microgrid
%   Group D  number of objectives with all devices: m = 1, 2, 3, 4
%   Group E  peak-hour snapshot (T = 1) versus 24-h profile (T = 24)
%
%   E.cases   case definitions        E.table  comparison of best solutions
%   E.R       solved cases (runs = 1) E.stats  mean/std/best over runs
if nargin < 1 || isempty(groups) || (ischar(groups) && strcmpi(groups, 'all'))
    groups = {'A', 'B', 'C', 'D', 'E'};
end
groups = upper(cellstr(groups));
if nargin < 2, study = struct(); end
T      = gf(study, 'T', 24);       
N = gf(study, 'N', 60);   
maxFE = gf(study, 'maxFE', 12000);
runs   = gf(study, 'runs', 1);     
solver = gf(study, 'solver', 'auto');
mpc0   = gf(study, 'mpc', RC_case33());
folder = gf(study, 'folder', fullfile('results', ['experiments_' datestr(now, 'yyyymmdd_HHMMSS')])); %#ok<TNOW1,DATST>
if ~exist(folder, 'dir')
   mkdir(folder); 
end

%% ---- case matrix ---------------------------------------------------------------------
C = struct('id', {}, 'group', {}, 'devices', {}, 'objectives', {}, 'm', {}, 'island', {}, 'T', {}, 'note', {});
net3 = {'TAC','EMISSION', 'LOSS'};
if ismember('A', groups)
    single = {'ONR', net3; 'DG', net3; 'SC', net3; 'TAP', net3; 'TX', net3; 'DSTATCOM', net3; ...
              'UPFC', net3; 'EVCS', {'TAC','LOSS','EVUNMET'}; 'BSS', {'TAC','PEAK','EVUNMET'}; ...
              'DRM', {'TAC','PEAK','PAR'}};
    for k = 1:size(single, 1)
        C(end+1) = mk(sprintf('A%02d', k), 'A', single(k, 1), single{k, 2}, 3, 'off', T, ['only ' single{k, 1}]); %#ok<AGROW>
    end
end
if ismember('B', groups)
    comb = {{'ONR','DG'}, net3, 'reconfiguration + DG'; ...
            {'ONR','DG','SC'}, net3, 'reconfiguration + DG + capacitors'; ...
            {'DG','SC','TAP','TX'}, net3, 'voltage control (taps + capacitors)'; ...
            {'DG','DSTATCOM','UPFC'}, net3, 'FACTS devices + DG'; ...
            {'EVCS','BSS'}, {'TAC','LOSS','EVUNMET'}, 'e-mobility'; ...
            {'EVCS','BSS','DRM'}, {'TAC','PEAK','EVUNMET'}, 'e-mobility + demand response'; ...
            {'ONR','DG','SC','TAP','TX','DSTATCOM','UPFC'}, net3, 'all network devices'; ...
            {'ONR','DG','SC','TAP','TX','DSTATCOM','UPFC','EVCS','BSS','DRM'}, {'TAC','LOSS','EVUNMET'}, 'all devices'};
    for k = 1:size(comb, 1)
        C(end+1) = mk(sprintf('B%02d', k), 'B', comb{k, 1}, comb{k, 2}, 3, 'off', T, comb{k, 3}); %#ok<AGROW>
    end
end
if ismember('C', groups)
    isl = {{'DG','BSS'}, {'TAC','LSHED','FDEV'}, 'outage', 'outage 18-21 h, one island'; ...
           {'ONR','DG','BSS','DRM','ISL'}, {'TAC','WLSHED','FDEV'}, 'outage', 'outage, microgrid formation'; ...
           {'DG','BSS','ISL'}, {'TAC','LSHED','SAI'}, 'optimal', 'optimal (intentional) islanding'; ...
           {'DG','SC','BSS','DRM','ISL'}, {'TAC','LSHED','FDEV'}, 'islanded', 'standalone islanded network'};
    for k = 1:size(isl, 1)
        C(end+1) = mk(sprintf('C%02d', k), 'C', isl{k, 1}, isl{k, 2}, 3, isl{k, 3}, T, isl{k, 4}); %#ok<AGROW>
    end
end
if ismember('D', groups)
    allD = {'ONR','DG','SC','TAP','TX','DSTATCOM','UPFC','EVCS','BSS','DRM'};
    objD = {{'TAC'}, {'TAC','LOSS'}, {'TAC','LOSS','ENS'}, {'TAC','LOSS','ENS','VD'}};
    for k = 1:4
        C(end+1) = mk(sprintf('D%02d', k), 'D', allD, objD{k}, k, 'off', T, sprintf('m = %d', k)); %#ok<AGROW>
    end
    C(end+1) = mk('D05', 'D', allD, {'TAC','LOSS','VD'}, 1, 'off', T, 'm = 1, weighted sum of 3');
end
if ismember('E', groups)
    dE = {'ONR','DG','SC','TAP','DSTATCOM'};
    C(end+1) = mk('E01', 'E', dE, {'LOSS','VD'}, 2, 'off', 1,  'peak-hour snapshot');
    C(end+1) = mk('E02', 'E', dE, {'LOSS','VD'}, 2, 'off', 24, '24-h profile');
end
%% Our Modified Objective functions



%% ---- solve every case ---------------------------------------------------------------------
E.cases = C;  
E.R = cell(numel(C), 1);  
E.folder = folder;
ind = {'TAC','INVCOST','OPCOST','LOSS','VD','VSI','ENS','SAIDI','PEAK','PAR','EMISSION', ...
       'EVUNMET','EVWAIT','DRCOST','LSHED','WLSHED','FDEV','SAI'};
rows = cell(numel(C), 1);  
vals = nan(numel(C), numel(ind));  
base = vals;
extra = nan(numel(C), 6);  
stats = cell(numel(C), 1);  
devs = cell(numel(C), 1);
for c = 1:numel(C)
    opts = struct('mpc', mpc0, 'devices', {C(c).devices}, 'objectives', {C(c).objectives}, ...
                  'm', C(c).m, 'island_mode', C(c).island, 'N', N, 'maxFE', maxFE, ...
                  'solver', solver, 'validate', true, 'verbose', true);
    Fr = [];
    for r = 1:runs
        rng(r); alg=study.alg;
        R = RC_solve(C(c).T, opts, alg);
        Fr(r, :) = R.F_best; %#ok<AGROW>
        if r == 1, R1 = R; end
    end
    R = R1;
    sub = fullfile(folder, C(c).id);
    Tb  = RC_report(R, false);
    figs = RC_plot_results(R, 'off');
    RC_save_results(R, sub, Tb, figs);
    close(figs);
    E.R{c} = rmfield_safe(R);
    [~, ii] = ismember(ind, R.P.allObj);
    vals(c, :) = R.out_best.Fall(ii);  base(c, :) = R.out_base.Fall(ii);
    extra(c, :) = [min(R.out_best.Vmin), R.out_best.VSImin, R.P.D, R.time, ...
                   double(R.out_best.feasible), size(R.Obj, 1)];
    rows{c} = sprintf('%s %s', C(c).id, C(c).note);
    dl = cellfun(@(a, b2) sprintf('%s@%g', a, b2), Tb.devices.Device, num2cell(Tb.devices.Site), ...
                 'UniformOutput', false);
    devs{c} = strjoin(dl(:)', ', ');
    stats{c} = [mean(Fr, 1); std(Fr, 0, 1); min(Fr, [], 1)];
end

%% ---- comparison tables -------------------------------------------------------------------------
vn = [ind, {'Vmin_pu', 'VSImin', 'D', 'Time_s', 'Feasible', 'nPareto'}];
E.table = array2table([vals, extra], 'VariableNames', vn, 'RowNames', matlab.lang.makeUniqueStrings(rows));
E.table.Devices = devs;
E.table.Objectives = arrayfun(@(q) strjoin(q.objectives, ','), C, 'UniformOutput', false)';
E.table.Island = {C.island}';
E.base = array2table(base, 'VariableNames', ind, 'RowNames', E.table.Properties.RowNames);
chg = 100 * (vals - base) ./ max(abs(base), 1e-12);  chg(abs(base) < 1e-12) = NaN;
E.change = array2table(chg, 'VariableNames', ind, 'RowNames', E.table.Properties.RowNames);
E.stats = stats;
xl = fullfile(folder, 'comparison.xlsx');
writetable(E.table, xl, 'Sheet', 'best', 'WriteRowNames', true);
writetable(E.base, xl, 'Sheet', 'base', 'WriteRowNames', true);
writetable(E.change, xl, 'Sheet', 'change_pct', 'WriteRowNames', true);
writetable(E.table, fullfile(folder, 'comparison.csv'), 'WriteRowNames', true);
if runs > 1
    for c = 1:numel(C)
        S = array2table(stats{c}, 'VariableNames', C(c).objectives, 'RowNames', {'mean', 'std', 'best'});
        writetable(S, xl, 'Sheet', ['stats_' C(c).id], 'WriteRowNames', true);
    end
end

%% ---- comparison figures -------------------------------------------------------------------------
figs = gobjects(0);
show = {'TAC','LOSS','VD','ENS','PEAK','EMISSION','EVUNMET','LSHED'};
labs = {'TAC ($/yr)','Loss (kW)','Voltage deviation','ENS (kWh/yr)','Peak import (MW)', ...
        'CO_2 (kg/day)','EV unserved (kWh/day)','Load shed (kWh/day)'};
for g = unique({C.group})
    k = find(strcmp({C.group}, g{1}));
    figs(end+1) = figure('Name', ['compare_' g{1}], 'Color', 'w', 'Visible', 'off'); %#ok<AGROW>
    for s = 1:numel(show)
        j = find(strcmp(ind, show{s}));
        subplot(2, 4, s);
        barh([base(k, j), vals(k, j)]);
        set(gca, 'YTick', 1:numel(k), 'YTickLabel', {C(k).id}, 'YDir', 'reverse', 'FontSize', 7);
        title(labs{s}); grid on;
        if s == 1, legend('base', 'best', 'Location', 'best'); end
    end
    try, sgtitle(sprintf('Group %s: base case vs best solution', g{1})); catch, end %#ok<NOCOMMA>
end
figs(end+1) = figure('Name', 'compare_heatmap', 'Color', 'w', 'Visible', 'off');
Z = chg;  Z(~isfinite(Z)) = 0;
imagesc(max(min(Z, 100), -100));  colormap(gca, redblue(64));  colorbar;
set(gca, 'XTick', 1:numel(ind), 'XTickLabel', ind, 'XTickLabelRotation', 45, ...
         'YTick', 1:numel(C), 'YTickLabel', {C.id}, 'FontSize', 7);
title('Change of each indicator vs base case (%, blue = improvement)');
for k = 1:numel(figs)
    f = fullfile(folder, get(figs(k), 'Name'));
    print(figs(k), f, '-dpng', '-r300');  savefig(figs(k), [f '.fig']);
end
close(figs);
save(fullfile(folder, 'experiments.mat'), 'E', '-v7.3');
fprintf('\nExperimental study saved in %s\n', folder);
disp(E.table(:, {'TAC', 'LOSS', 'VD', 'ENS', 'LSHED', 'Vmin_pu', 'Feasible', 'Time_s'}));
end

% =========================================================================================
function c = mk(id, group, devices, objectives, m, island, T, note)
c = struct('id', id, 'group', group, 'devices', {devices}, 'objectives', {objectives}, ...
           'm', m, 'island', island, 'T', T, 'note', note);
end

function v = gf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
   v = s.(name);
else
   v = default;
end
end

function R = rmfield_safe(R)
if isfield(R, 'P') && isfield(R.P, 'om'), R.P = rmfield(R.P, 'om'); end
end

function c = redblue(n)
% blue (negative = improvement) - white - red colormap
h = floor(n / 2);  u = linspace(0, 1, h)';
c = [[u u ones(h, 1)]; [ones(n - h, 1) flipud(linspace(0, 1, n - h)') flipud(linspace(0, 1, n - h)')]];
end
