function [xh, xp] = imm_cvca(r, el, valid, p, theta)
% IMM of a constant-velocity model and a constant-acceleration model.
% theta = [sig_a (CV noise), sig_j (CA jerk noise)]. Model-switch prob per step = 0.02.
sig_a = theta(1);  sig_j = theta(2);  dt = p.dt;  tau = p.tau;  N = numel(r);
eps_sw = 0.02;  Pi = [1-eps_sw eps_sw; eps_sw 1-eps_sw];     % Pi(i,j): i -> j
Z = zeros(3);
Fcv1 = [1 dt 0; 0 1 0; 0 0 0];       Fca1 = [1 dt dt^2/2; 0 1 dt; 0 0 1];
Hcv1 = [1 tau 0; 0 1 0; 0 0 0];      Hca1 = [1 tau tau^2/2; 0 1 tau; 0 0 1];
g  = [dt^2/2; dt; 0];                Qcv1 = sig_a^2*(g*g');
q1 = [dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 dt];  Qca1 = sig_j^2*q1;
F  = {[Fcv1 Z; Z Fcv1], [Fca1 Z; Z Fca1]};
Fh = {[Hcv1 Z; Z Hcv1], [Hca1 Z; Z Hca1]};
Q  = {[Qcv1 Z; Z Qcv1], [Qca1 Z; Z Qca1]};
Hm = [1 0 0 0 0 0; 0 0 0 1 0 0];
xh = zeros(2,N); xp = zeros(2,N);
mu = [0.7; 0.3];
for k = 1:N
    [zc, R] = meas_conv(r(k), el(k), p);
    if k == 1
        x0 = [zc(1);0;0;zc(2);0;0];
        P0 = zeros(6); P0([1 4],[1 4]) = R;
        P0(2,2) = 100^2; P0(5,5) = 100^2; P0(3,3) = 10^2; P0(6,6) = 10^2;
        xs = {x0, x0};  Ps = {P0, P0};
    else
        cbar = Pi'*mu;
        xm = cell(1,2); Pm = cell(1,2);
        for j = 1:2
            w = (Pi(:,j).*mu)/cbar(j);
            xm{j} = w(1)*xs{1} + w(2)*xs{2};
            Pm{j} = zeros(6);
            for i = 1:2
                dx = xs{i} - xm{j};
                Pm{j} = Pm{j} + w(i)*(Ps{i} + dx*dx');
            end
        end
        lik = ones(2,1);
        for j = 1:2
            xs{j} = F{j}*xm{j};  Ps{j} = F{j}*Pm{j}*F{j}' + Q{j};
            Ps{j} = (Ps{j}+Ps{j}')/2;
            if valid(k)
                S = Hm*Ps{j}*Hm' + R; S=(S+S')/2; y = zc - Hm*xs{j};
                [U,pchol]=chol(S);
                if pchol~=0
                    S=S+1e-9*max(trace(S),1)*eye(2); U=chol(S);
                end
                q=U'\y; loglik=-0.5*(q'*q+2*sum(log(diag(U)))+2*log(2*pi));
                lik(j)=exp(max(loglik,log(realmin)));
                K = (Ps{j}*Hm')/S;
                xs{j} = xs{j} + K*y;
                A = eye(6) - K*Hm;  Ps{j} = A*Ps{j}*A' + K*R*K';
                Ps{j} = (Ps{j}+Ps{j}')/2;
            end
        end
        if valid(k), mu = lik.*cbar;  mu = mu/sum(mu); else, mu = cbar; end
    end
    xc = mu(1)*xs{1} + mu(2)*xs{2};
    yp = mu(1)*(Fh{1}*xs{1}) + mu(2)*(Fh{2}*xs{2});
    xh(:,k) = xc([1 4]);  xp(:,k) = yp([1 4]);
end
end
