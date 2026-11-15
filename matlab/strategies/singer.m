function [xh, xp] = singer(r, el, valid, p, theta)
% Singer model: acceleration is an exponentially-correlated random process.
% theta = [sigma_a, tau_a]: stationary accel std [m/s^2], correlation time [s].
sig = theta(1);  tauA = theta(2);  al = 1/tauA;  N = numel(r);
Fc1 = [0 1 0; 0 0 1; 0 0 -al];  G1 = [0;0;1];
[Ph, Q1] = vanloan(Fc1, G1, 2*al*sig^2, p.dt);
Fh1 = expm(Fc1*p.tau);
Z = zeros(3);
F = [Ph Z; Z Ph];  Q = [Q1 Z; Z Q1];  Fh = [Fh1 Z; Z Fh1];
Hm = [1 0 0 0 0 0; 0 0 0 1 0 0];
xh = zeros(2,N); xp = zeros(2,N);
for k = 1:N
    [zc, R] = meas_conv(r(k), el(k), p);
    if k == 1
        x = [zc(1);0;0;zc(2);0;0];
        P = zeros(6);  P([1 4],[1 4]) = R;
        P(2,2) = 100^2; P(5,5) = 100^2; P(3,3) = sig^2; P(6,6) = sig^2;
    else
        x = F*x;  P = F*P*F' + Q;
        if valid(k)
            S = Hm*P*Hm' + R;  K = (P*Hm')/S;
            x = x + K*(zc - Hm*x);
            A = eye(6) - K*Hm;  P = A*P*A' + K*R*K';
        end
    end
    y = Fh*x;
    xh(:,k) = x([1 4]);  xp(:,k) = y([1 4]);
end
end
