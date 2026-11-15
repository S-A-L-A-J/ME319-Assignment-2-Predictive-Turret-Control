function tr = make_truth(q, p)
% Deterministic true motion from a parameter struct q.
%   q.p0 [2x1] start position, q.v0 [2x1] start velocity
%   q.kind  'none' | 'pulse' | 'turn'
%   q.t_on, q.Tm, q.am, q.psi, q.omega   (maneuver parameters)
% p: .dt .T .tau   -> samples at t = 0, dt, ..., T+tau
h  = 0.001;
tt = 0:h:(p.T + p.tau);
a  = zeros(2, numel(tt));
if ~strcmp(q.kind, 'none')
    s   = tt - q.t_on;
    idx = (s >= 0) & (s <= q.Tm);
    u   = s(idx);
    switch q.kind
        case 'pulse'                       % sin^2 bump, fixed direction
            g   = sin(pi*u/q.Tm).^2;
            ang = q.psi * ones(size(u));
        case 'turn'                        % held magnitude, rotating direction
            rr = 0.5;                      % raised-cosine ramp time [s]
            g  = ones(size(u));
            i1 = u < rr;           g(i1) = 0.5*(1 - cos(pi*u(i1)/rr));
            i2 = u > q.Tm - rr;    g(i2) = 0.5*(1 - cos(pi*(q.Tm-u(i2))/rr));
            ang = q.psi + q.omega*u;
        otherwise
            error('unknown kind');
    end
    a(1,idx) = q.am * g .* cos(ang);
    a(2,idx) = q.am * g .* sin(ang);
end
v = q.v0(:) + cumtrapz(tt, a, 2);
x = q.p0(:) + cumtrapz(tt, v, 2);

in = tt <= p.T + 1e-9;                     % envelope is required on 0..T
R  = hypot(x(1,in), x(2,in));
tr.rmin = min(R);  tr.rmax = max(R);
tr.vmax = max(hypot(v(1,in), v(2,in)));
tr.amax = max(hypot(a(1,in), a(2,in)));
tr.ok   = tr.rmin >= 800 && tr.rmax <= 2000 && tr.vmax <= 150 && tr.amax <= 20;

st   = round(p.dt/h);
tr.t = tt(1:st:end);
tr.x = x(:,1:st:end);
tr.v = v(:,1:st:end);
tr.a = a(:,1:st:end);
end
