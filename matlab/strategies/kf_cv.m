function [xh, xp] = kf_cv(r, el, valid, p, sig_a)
% Constant-velocity Kalman filter on converted (x,z) measurements.
% State [x z vx vz]. sig_a = process-noise acceleration std [m/s^2].
dt = p.dt; N = numel(r);
F  = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Fh = [1 0 p.tau 0; 0 1 0 p.tau; 0 0 1 0; 0 0 0 1];
G  = [dt^2/2 0; 0 dt^2/2; dt 0; 0 dt];
Q  = sig_a^2 * (G*G');
Hm = [1 0 0 0; 0 1 0 0];
xh = zeros(2,N); xp = zeros(2,N);
for k = 1:N
    zc = [r(k)*cos(el(k)); r(k)*sin(el(k))];
    J  = [cos(el(k)) -r(k)*sin(el(k)); sin(el(k)) r(k)*cos(el(k))];
    R  = J*diag([p.sig_r^2 p.sig_el^2])*J';
    if k == 1
        x = [zc;0;0];  P = zeros(4);  P(1:2,1:2) = R;  P(3,3) = 100^2;  P(4,4) = 100^2;
    else
        x = F*x;  P = F*P*F' + Q;
        if valid(k)
            S = Hm*P*Hm' + R;  K = (P*Hm')/S;
            x = x + K*(zc - Hm*x);
            A = eye(4) - K*Hm;  P = A*P*A' + K*R*K';
        end
    end
    xh(:,k) = x(1:2);
    y = Fh*x;  xp(:,k) = y(1:2);
end
end
