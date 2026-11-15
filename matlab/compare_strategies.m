function compare_strategies(best_in)
% optional best_in: cell of frozen parameters (one per strategy) to skip tuning
% 1) tune each strategy's parameter(s) on data/tuning.mat (grid search)
% 2) freeze them, score on data/test.mat (same truth, same noise for everyone)
base = fileparts(mfilename('fullpath'));
root = fileparts(base);
addpath(fullfile(base,'strategies'));
p.dt = 0.05; p.T = 10; p.tau = 0.5; p.sig_r = 1; p.sig_el = 0.02*pi/180;
tskip = 1.0;  nTune = 20;
S = load(fullfile(root,'data','tuning.mat')); tuning = S.data;
S = load(fullfile(root,'data','test.mat'));   test   = S.data;

[a1,a2] = ndgrid([5 10 20],[1 3 8]);       g_singer = [a1(:) a2(:)];
[b1,b2] = ndgrid([2 5 10],[1 3 10]);       g_imm    = [b1(:) b2(:)];
[c1,c2] = ndgrid([1 3 10],[0 0.5 1]);      g_damp   = [c1(:) c2(:)];
strats = {
 'kf_cv',      @kf_cv,      [1 2 3 5 8 12 20 30 50]'
 'kf_ca',      @kf_ca,      [1 3 10 30 100]'
 'alphabeta',  @alphabeta,  [0.2 0.3 0.4 0.5 0.6 0.8]'
 'singer',     @singer,     g_singer
 'ekf_cv',     @ekf_cv,     [3 5 8 12 20 30]'
 'adaptive_cv',@adaptive_cv,[1 2 3 5 8]'
 'kfca_damped',@kfca_damped,g_damp
 'imm_cvca',   @imm_cvca,   g_imm };

fid = fopen(fullfile(root,'results','comparison.txt'),'w');
out = @(varargin) (fprintf(varargin{:}) + fprintf(fid, varargin{:}));
out('Tuning: mean 0.5 s prediction RMSE [m] on tuning set (t>=1 s), best parameter per strategy\n');
best = cell(size(strats,1),1);
for s = 1:size(strats,1)
    if nargin > 0, best{s} = best_in{s}; continue; end
    g = strats{s,3};  sc = zeros(size(g,1),1);
    for j = 1:size(g,1)
        r = evaluate(strats{s,2}, g(j,:), tuning, p, nTune, tskip);
        sc(j) = mean(r.pred);
    end
    [mn,ib] = min(sc);  best{s} = g(ib,:);
    out('  %-12s best %-10s tuning RMSE %.2f  (grid %d pts)\n', strats{s,1}, mat2str(best{s}), mn, size(g,1));
end

out('\nTest set (15 trajectories x 100 noise runs, parameters frozen), position RMSE [m], t>=1 s\n');
out('%-12s %-10s %-9s %-11s %-12s %-11s %-11s\n','strategy','param','filtered','pred(mean)','pred(worst)','loss(mean)','loss(worst)');
for s = 1:size(strats,1)
    r = evaluate(strats{s,2}, best{s}, test, p, 100, tskip);
    out('%-12s %-10s %-9.2f %-11.2f %-12.2f %-11.2f %-11.2f\n', strats{s,1}, mat2str(best{s}), ...
        mean(r.est), mean(r.pred), max(r.pred), mean(r.predloss(~isnan(r.predloss))), max(r.predloss));
end

% reference floors: exact true state, no noise, extrapolated tau ahead
H = round(p.tau/p.dt);  N = round(p.T/p.dt)+1;  k0 = round(tskip/p.dt)+1;  kk = k0:N;
fcv = zeros(1,numel(test)); fca = fcv; fl = nan(1,numel(test));
for i = 1:numel(test)
    d = test(i);
    ecv = sum((d.x(:,1:N) + p.tau*d.v(:,1:N) - d.x(:,1+H:N+H)).^2,1);
    eca = sum((d.x(:,1:N) + p.tau*d.v(:,1:N) + 0.5*p.tau^2*d.a(:,1:N) - d.x(:,1+H:N+H)).^2,1);
    fcv(i) = sqrt(mean(ecv(kk)));  fca(i) = sqrt(mean(eca(kk)));
    if d.L > 0
        kl = (round(d.t_d/p.dt)+1):min(N,round((d.t_d+d.L+0.5)/p.dt)+1);
        fl(i) = sqrt(mean(ecv(kl)));
    end
end
out('%-12s %-10s %-9s %-11.2f %-12.2f %-11.2f %-11.2f\n','genie_cv','-','0',mean(fcv),max(fcv),mean(fl(~isnan(fl))),max(fl));
out('%-12s %-10s %-9s %-11.2f %-12.2f\n','genie_ca','-','0',mean(fca),max(fca));
fclose(fid);
end
