function R = RC_solve(T, opts, alg, N, maxFE)
%RC_SOLVE  Build the model (RC_setup), solve it and post-process the result.
%
%   R = RC_solve(T, opts)
%   R = RC_solve(T, opts, alg, N, maxFE)          (backward compatible call)
%
%   opts (besides the RC_setup options):
%     .solver    'auto' (PlatEMO if on the path, else built-in) | 'platemo' | 'builtin'
%     .alg       PlatEMO algorithm handle; default @GA (m = 1), @NSGAII (m = 2, 3),
%                @NSGAIII (m >= 4)
%     .N, .maxFE population size and evaluation budget (100, 20000)
%     .validate  re-solve the best solution with MATPOWER runpf (true if available)
%     .refine    m = 1: polish the continuous set-points with MP-Opt-Model (false)
%     .verbose   print progress (true)
%
%   R.P          problem data (P.om = MP-Opt-Model object)
%   R.F_base     objectives of the base case,  R.out_base  all its indicators
%   R.Dec, R.Obj feasible non-dominated set (m > 1) or best solutions (m = 1)
%   R.ibest      best compromise (fuzzy satisfying method) or best solution
%   R.x_best, R.F_best, R.out_best   decision vector, objectives, indicators
%   R.hist       evaluation log (objectives, constraint violation per FE)
%   R.val        MATPOWER validation table of the best solution
%   R.time       run time (s)
if nargin < 2 || isempty(opts), opts = struct(); end
if nargin >= 3 && ~isempty(alg),   opts.alg = alg; end
if nargin >= 4 && ~isempty(N),     opts.N = N; end
if nargin >= 5 && ~isempty(maxFE), opts.maxFE = maxFE; end
t0 = tic;
P = RC_setup(T, opts);
N = getopt(opts, 'N', 100);  
maxFE = getopt(opts, 'maxFE', 20000);
verb = getopt(opts, 'verbose', true);

%% base case
[~, ~, R.out_base] = RC_objectives(P.xbase, P);
R.F_base = R.out_base.Fall(P.iObj);

%% solver
solver = lower(getopt(opts, 'solver', 'auto'));
if strcmp(solver, 'auto')
   if exist('platemo', 'file') == 2
      solver = 'platemo';
   else
      solver = 'builtin';
   end
end
if verb
    fprintf('\n[%s] T = %d | \ndevices: %s \nobjectives: %s | m = %d | island: %s | D = %d | solver: %s\n', ...
            datestr(now, 'HH:MM:SS'), T, strjoin(P.devices, '+'), strjoin(P.objectives, ','), ...
            P.m, P.isl.mode, P.D, solver); %#ok<TNOW1,DATST>
end
RC_fitness('reset', maxFE + 2 * N);
switch solver
    case 'platemo'
        alg = getopt(opts, 'alg', []);
        if isempty(alg)
            if P.m == 1, alg = @GA; elseif P.M <= 3, alg = @NSGAII; else, alg = @NSGAIII; end
        end
        [Dec, Obj, Con] = platemo('problem', {@RC_PlatEMO, P}, 'algorithm', alg, ...
                                  'N', N, 'maxFE', maxFE);
        R.alg = func2str(alg);
    case 'builtin'
        [Dec, Obj, Con] = RC_nsga2(P, N, maxFE, struct('verbose', verb));
        R.alg = 'RC_nsga2';
    otherwise
        error('RC_solve: unknown solver %s', solver);
end
R.hist = RC_fitness('get');

%% feasible set, non-dominated filter, best solution
CV   = sum(max(Con, 0), 2);
feas = CV <= 1e-9;
if ~any(feas)
    warning('RC_solve:infeasible', ['No feasible solution: the least-violating one is ' ...
            'reported. Increase maxFE or relax the limits.']);
    feas = CV == min(CV);
end
Dec = Dec(feas, :);  Obj = Obj(feas, :);
[Obj, iu] = unique(Obj, 'rows');  Dec = Dec(iu, :);
if P.m > 1
    nd = true(size(Obj, 1), 1);
    for i = 1:size(Obj, 1)
        nd(i) = ~any(all(Obj <= Obj(i, :), 2) & any(Obj < Obj(i, :), 2));
    end
    Dec = Dec(nd, :);  Obj = Obj(nd, :);
    [~, o] = sort(Obj(:, 1));  R.Dec = Dec(o, :);  R.Obj = Obj(o, :);
    [R.ibest, R.score] = RC_select_best(R.Obj);
else
    [~, o] = sort(Obj(:, 1));  R.Dec = Dec(o, :);  R.Obj = Obj(o, :);
    R.ibest = 1;  R.score = [];
end
R.x_best = R.Dec(R.ibest, :);

%% optional local refinement of the continuous set-points (m = 1)
if P.m == 1 && getopt(opts, 'refine', false)
    R.refine = RC_refine(P, R.x_best);
    if R.refine.improved, R.x_best = R.refine.x; end
end
[~, ~, R.out_best] = RC_objectives(R.x_best, P);
R.F_best = R.out_best.Fall(P.iObj);
R.Fall_pareto = zeros(size(R.Dec, 1), P.nAll);
for i = 1:size(R.Dec, 1), R.Fall_pareto(i, :) = RC_objectives(R.Dec(i, :), P); end

%% MATPOWER validation of the best solution
R.val = [];
if getopt(opts, 'validate', true) && exist('runpf', 'file') == 2
    try
        R.val = RC_validate_solution(P, R.out_best);
    catch err
        warning('RC_solve:validate', 'MATPOWER validation skipped: %s', err.message);
    end
end
R.P = P;  R.opts = opts;  R.solver = solver;  R.N = N;  R.maxFE = maxFE;
R.time = toc(t0);
if verb, fprintf('  done in %.1f s, %d non-dominated / best solutions\n', R.time, size(R.Dec, 1)); end
end

function v = getopt(s, name, default)
if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end
