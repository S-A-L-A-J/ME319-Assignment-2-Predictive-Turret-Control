function [xh, xp] = alphabeta(r, el, valid, p, alpha)
% Fixed-gain alpha-beta tracker on converted (x,z); beta = alpha^2/(2-alpha).
dt = p.dt; N = numel(r);  beta = alpha^2/(2-alpha);
xh = zeros(2,N); xp = zeros(2,N);
for k = 1:N
    zc = [r(k)*cos(el(k)); r(k)*sin(el(k))];
    if k == 1
        pos = zc; vel = [0;0];
    else
        pos = pos + vel*dt;                         % predict
        if valid(k)
            res = zc - pos;
            pos = pos + alpha*res;
            vel = vel + (beta/dt)*res;
        end
    end
    xh(:,k) = pos;  xp(:,k) = pos + vel*p.tau;
end
end
