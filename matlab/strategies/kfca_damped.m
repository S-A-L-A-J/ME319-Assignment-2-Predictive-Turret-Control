function [xh, xp] = kfca_damped(r, el, valid, p, theta)
% Constant-acceleration KF; the acceleration term in the 0.5 s forecast is
% scaled by d. theta = [sig_j, d]  (d = 1 is the plain CA forecast, d = 0 is CV forecast).
sig_j = theta(1);  d = theta(2);
dt = p.dt; N = numel(r);
A1 = [1 dt dt^2/2; 0 1 dt; 0 0 1];
F  = zeros(6); F([1 3 5],[1 3 5]) = A1; F([2 4 6],[2 4 6]) = A1;
q1 = [dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 dt];
Q  = zeros(6); Q([1 3 5],[1 3 5]) = sig_j^2*q1; Q([2 4 6],[2 4 6]) = sig_j^2*q1;
Hm = zeros(2,6); Hm(1,1) = 1; Hm(2,2) = 1;
xh = zeros(2,N); xp = zeros(2,N);
for k = 1:N
    [zc, R] = meas_conv(r(k), el(k), p);
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
    xp(:,k) = x(1:2) + p.tau*x(3:4) + d*0.5*p.tau^2*x(5:6);
end
end
