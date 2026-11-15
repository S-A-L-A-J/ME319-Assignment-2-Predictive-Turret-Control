function data = generate_dataset(name, n, seed, nMC)
% Generate frozen datasets: truth + dropout mask + noisy measurements.
%   generate_dataset('scenarios')            -> fixed A, B, C
%   generate_dataset('tuning', 10, 1001)     -> sampled family, for choosing parameters
%   generate_dataset('test',   15, 2002)     -> sampled family, for final comparison
% Saves data/<name>.mat (variable: data). nMC = noise realisations per trajectory.
if nargin < 4, nMC = 100; end
p.dt = 0.05;  p.T = 10;  p.tau = 0.5;
p.sig_r  = 1;                 % m
p.sig_el = 0.02*pi/180;       % rad
N = round(p.T/p.dt) + 1;      % 201 measurement times
if ~exist('data','dir'), mkdir('data'); end

if strcmp(name, 'scenarios')
    seed = 1;  n = 3;
    base = struct('p0',[1500;500],'v0',[-50;10],'kind','none','t_on',0,'Tm',0,'am',0,'psi',0,'omega',0);
    Q = repmat(base, 1, 3);
    for j = 2:3
        Q(j).kind = 'pulse';  Q(j).t_on = 1;  Q(j).Tm = 5;
        Q(j).am = hypot(-8,12);  Q(j).psi = atan2(12,-8);
    end
    D = [0 0; 0 0; 1.0 0.5];                    % [t_d L] per scenario, L=0 -> no loss
    labels = {'A','B','C'};
end

seed_rng(seed);
u = @(lo,hi) lo + (hi-lo)*rand;
kinds = {'none','pulse','pulse','turn'};
data = struct([]);
for i = 1:n
    if strcmp(name,'scenarios')
        q = Q(i);  tr = make_truth(q,p);  td = D(i,1);  L = D(i,2);  lab = labels{i};
        assert(tr.ok, 'preset violates envelope');
    else
        kind = kinds{mod(i-1,4)+1};          % cycle: none/pulse/pulse/turn
        dmode = mod(i-1,3);                  % cycle: no loss / random loss / loss during maneuver
        for attempt = 1:5000
            r0 = u(900,1900); phi0 = u(10,60)*pi/180;
            V0 = u(20,120);   eta  = u(0,2*pi);
            q = struct('p0',r0*[cos(phi0);sin(phi0)],'v0',V0*[cos(eta);sin(eta)], ...
                       'kind',kind,'t_on',0,'Tm',0,'am',0,'psi',0,'omega',0);
            if ~strcmp(kind,'none')
                q.t_on = u(1,4);  q.Tm = u(2,6);  q.am = u(5,19.5);  q.psi = u(0,2*pi);
                if strcmp(kind,'turn'), q.omega = (2*(rand>0.5)-1)*u(0.2,0.6); end
            end
            tr = make_truth(q,p);
            if tr.ok, break; end
        end
        assert(tr.ok, 'could not draw an admissible trajectory');
        td = 0;  L = 0;
        if dmode > 0
            L = round(u(0.25,0.5)/p.dt)*p.dt;
            if dmode == 2 && ~strcmp(kind,'none')
                td = u(q.t_on, max(q.t_on, q.t_on+q.Tm-L));
            else
                td = u(1, 9.5);
            end
            td = min(max(round(td/p.dt)*p.dt, 1), p.T - L);
        end
        lab = sprintf('%02d',i);
    end

    valid = true(1,N);
    if L > 0
        k0 = round(td/p.dt);  kl = round(L/p.dt);
        valid(k0+1 : k0+kl) = false;         % samples k0 .. k0+kl-1 lost
    end
    xs = tr.x(:,1:N);
    r  = repmat(hypot(xs(1,:),xs(2,:)), nMC, 1)  + p.sig_r *randn(nMC,N);
    el = repmat(atan2(xs(2,:),xs(1,:)), nMC, 1) + p.sig_el*randn(nMC,N);

    d.id = lab;  d.q = q;  d.t_d = td;  d.L = L;
    d.t = tr.t;  d.x = tr.x;  d.v = tr.v;  d.a = tr.a;     % truth, N+10 samples
    d.valid = valid;  d.r = r;  d.el = el;                  % measurements, nMC x N
    d.env = [tr.rmin tr.rmax tr.vmax tr.amax];
    if i == 1, data = d; else, data(i) = d; end
end

save_data(fullfile('data',[name '.mat']), data);
fprintf('%s: %d trajectories, nMC=%d, saved to data/%s.mat\n', name, n, nMC, name);
fprintf('%-4s %-6s %-14s %-9s %-9s %-9s\n','id','kind','loss [t_d,L]','r range','vmax','amax');
for i = 1:n
    d = data(i);
    fprintf('%-4s %-6s %-14s %4.0f-%-4.0f %8.1f %8.1f\n', d.id, d.q.kind, ...
        ternary(d.L>0, sprintf('[%.2f,%.2f]',d.t_d,d.L), '-'), d.env(1), d.env(2), d.env(3), d.env(4));
end
end

function s = ternary(c,a,b)
if c, s = a; else, s = b; end
end

function save_data(f, data)
if exist('OCTAVE_VERSION','builtin'), save('-mat', f, 'data'); else, save(f, 'data', '-v7'); end
end
