function [xh, xp] = kf_ca(r, el, valid, p, sig_j)
% Constant-acceleration Kalman filter (white-jerk process noise).
% State [x z vx vz ax az]. sig_j = jerk spectral density sqrt [m/s^2/sqrt(s)].
dt = p.dt; N = numel(r);
A1 = [1 dt dt^2/2; 0 1 dt; 0 0 1];                 % per-axis transition
F  = zeros(6);  F([1 3 5],[1 3 5]) = A1;  F([2 4 6],[2 4 6]) = A1;
t  = p.tau; Ah = [1 t t^2/2; 0 1 t; 0 0 1];
Fh = zeros(6); Fh([1 3 5],[1 3 5]) = Ah; Fh([2 4 6],[2 4 6]) = Ah;
q1 = [dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 dt];
Q  = zeros(6); Q([1 3 5],[1 3 5]) = sig_j^2*q1;  Q([2 4 6],[2 4 6]) = sig_j^2*q1;
Hm = zeros(2,6); Hm(1,1) = 1; Hm(2,2) = 1;
xh = zeros(2,N); xp = zeros(2,N);
for k = 1:N
    zc = [r(k)*cos(el(k)); r(k)*sin(el(k))];
    J  = [cos(el(k)) -r(k)*sin(el(k)); sin(el(k)) r(k)*cos(el(k))];
    R  = J*diag([p.sig_r^2 p.sig_el^2])*J';
    if k == 1
        x = [zc;0;0;0;0];  P = zeros(6);  P(1:2,1:2) = R;
        P(3,3) = 100^2; P(4,4) = 100^2; P(5,5) = 10^2; P(6,6) = 10^2;
    else
        x = F*x;  P = F*P*F' + Q;
        if valid(k)
            S = Hm*P*Hm' + R;  K = (P*Hm')/S;
            x = x + K*(zc - Hm*x);
            A = eye(6) - K*Hm;  P = A*P*A' + K*R*K';
        end
    end
    xh(:,k) = x(1:2);
    y = Fh*x;  xp(:,k) = y(1:2);
end
end
