function [xh, xp] = ekf_cv(r, el, valid, p, sig_a)
% Constant-velocity EKF using the raw polar measurement (range, elevation).
dt = p.dt; N = numel(r);
F  = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Fh = [1 0 p.tau 0; 0 1 0 p.tau; 0 0 1 0; 0 0 0 1];
G  = [dt^2/2 0; 0 dt^2/2; dt 0; 0 dt];
Q  = sig_a^2*(G*G');
Rp = diag([p.sig_r^2 p.sig_el^2]);
xh = zeros(2,N); xp = zeros(2,N);
for k = 1:N
    if k == 1
        [zc, R] = meas_conv(r(1), el(1), p);
        x = [zc;0;0];  P = zeros(4);  P(1:2,1:2) = R;  P(3,3) = 100^2;  P(4,4) = 100^2;
    else
        x = F*x;  P = F*P*F' + Q;
        if valid(k)
            rho = hypot(x(1),x(2));
            h   = [rho; atan2(x(2),x(1))];
            Hj  = [x(1)/rho, x(2)/rho, 0, 0; -x(2)/rho^2, x(1)/rho^2, 0, 0];
            S = Hj*P*Hj' + Rp;  K = (P*Hj')/S;
            x = x + K*([r(k); el(k)] - h);
            A = eye(4) - K*Hj;  P = A*P*A' + K*Rp*K';
        end
    end
    xh(:,k) = x(1:2);
    y = Fh*x;  xp(:,k) = y(1:2);
end
end
