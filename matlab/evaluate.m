function res = evaluate(strat, theta, data, p, nUse, tskip)
% Score one strategy (function handle) with parameter theta on a dataset.
% Strategy signature: [xh, xp] = strat(r, el, valid, p, theta), xh/xp are 2xN.
% Errors are position errors [m], RMS over time (t >= tskip) and nUse noise runs.
%   est  : filtered position vs truth now
%   pred : predicted position vs truth tau seconds later
%   predloss : pred RMSE in [t_d, t_d+L+0.5] s (only trajectories with a loss)
H = round(p.tau/p.dt);  N = round(p.T/p.dt) + 1;  k0 = round(tskip/p.dt) + 1;
n = numel(data);
res.est = zeros(1,n); res.pred = zeros(1,n); res.predloss = nan(1,n);
for i = 1:n
    d = data(i);
    kk = k0:N;
    if d.L > 0
        kl = (round(d.t_d/p.dt)+1) : min(N, round((d.t_d+d.L+0.5)/p.dt)+1);
    else
        kl = [];
    end
    se = 0; sp = 0; sl = 0;
    for m = 1:nUse
        [xh, xp] = strat(d.r(m,:), d.el(m,:), d.valid, p, theta);
        ee = sum((xh - d.x(1:2,1:N)).^2, 1);
        ep = sum((xp - d.x(1:2,1+H:N+H)).^2, 1);
        se = se + mean(ee(kk));  sp = sp + mean(ep(kk));
        if ~isempty(kl), sl = sl + mean(ep(kl)); end
    end
    res.est(i) = sqrt(se/nUse);  res.pred(i) = sqrt(sp/nUse);
    if ~isempty(kl), res.predloss(i) = sqrt(sl/nUse); end
end
end
