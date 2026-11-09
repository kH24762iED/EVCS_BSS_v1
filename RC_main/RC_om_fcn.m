function [v, dv] = RC_om_fcn(kind, x, P, k, sc)
%RC_OM_FCN  Cost and constraint functions registered in the MP-Opt-Model
%object (RC_om_build). Both call RC_objectives, which caches the result of
%each decision vector, so all costs of one vector need one load-flow run.
%
%   f       = RC_om_fcn('f', x, P, k, sc)  sc * value of objective P.allObj{k}
%   g       = RC_om_fcn('g', x, P)         nonlinear constraints (column)
%   [v, dv] = RC_om_fcn(...)               + forward-difference derivative
%                                          w.r.t. the continuous variables
%                                          (integer/binary columns are 0)
if iscell(x), x = vertcat(x{:}); end
x = reshape(x, 1, []);
[Fall, G] = RC_objectives(x, P);
if strcmp(kind, 'f'), v = sc * Fall(k); else, v = G(:); end
if nargout < 2, return; end
ic = find(P.enc == 1);
if strcmp(kind, 'f'), dv = zeros(P.D, 1); else, dv = sparse(numel(G), P.D); end
for j = ic
    h  = 1e-6 * max(1, abs(x(j)));
    xp = x;  xp(j) = min(x(j) + h, P.upper(j));
    if xp(j) == x(j), xp(j) = x(j) - h; end
    hj = xp(j) - x(j);
    [Fp, Gp] = RC_objectives(xp, P);
    if strcmp(kind, 'f')
        dv(j) = sc * (Fp(k) - Fall(k)) / hj;
    else
        dv(:, j) = sparse((Gp(:) - G(:)) / hj);
    end
end
end
