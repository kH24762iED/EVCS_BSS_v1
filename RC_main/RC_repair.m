function X = RC_repair(X, P)
%RC_REPAIR  Repair (not penalise) decision vectors, one per row of X:
%   - bounds and integrality of every variable
%   - DR energy neutrality  sum_t m(t) dt rho(t) = 0, projected onto the box
%   - DR curtailment budget sum_t m(t) dt chi(t) <= il_frac * sum_t m(t) dt
%   - BSS rule thresholds   alpha_dis(t) >= alpha_ch(t) + gap
%The repaired vectors satisfy the linear sets marked 'repaired' in
%P.linsets exactly; PlatEMO stores them (RC_PlatEMO.CalDec), and so does
%RC_nsga2, so the search is Lamarckian.
lo = P.lower;  up = P.upper;
X  = min(max(X, lo), up);
ii = P.enc ~= 1;
X(:, ii) = round(X(:, ii));
if isfield(P.bidx, 'DR_shift')
    ix = P.bidx.DR_shift;  w = P.dr_w;  ww = w * w';
    for i = 1:size(X, 1)
        s = X(i, ix);
        for it = 1:50
            s = s - w * ((s * w') / ww);                       % onto w*s = 0
            s = min(max(s, lo(ix)), up(ix));                   % onto the box
            if abs(s * w') <= 1e-10 * sum(w), break; end
        end
        if abs(s * w') > 1e-8 * sum(w), s(:) = 0; end          % fall back to no shift
        X(i, ix) = s;
    end
end
if isfield(P.bidx, 'DR_il')
    ix = P.bidx.DR_il;  B = P.drm.il_frac * sum(P.dr_w);
    for i = 1:size(X, 1)
        e = X(i, ix) * P.dr_w';
        if e > B, X(i, ix) = X(i, ix) * (B / e); end
    end
end
if isfield(P.bidx, 'BSS_ach')
    ia = P.bidx.BSS_ach;  id = P.bidx.BSS_adis;
    X(:, ia) = min(X(:, ia), up(id) - P.bss.gap);
    X(:, id) = min(max(X(:, id), X(:, ia) + P.bss.gap), up(id));
end
end
