function [xh, xp] = adaptive_cv(r, el, valid, p, sig_a)
% CV Kalman filter whose process noise is scaled up when innovations are
% larger than the filter expects (smoothed normalised innovation squared).
dt = p.dt; N = numel(r);
F  = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Fh = [1 0 p.tau 0; 0 1 0 p.tau; 0 0 1 0; 0 0 0 1];
G  = [dt^2/2 0; 0 dt^2/2; dt 0; 0 dt];
Q0 = sig_a^2*(G*G');
Hm = [1 0 0 0; 0 1 0 0];
m = 1;                                   % smoothed NIS/2 (expected value 1)
xh = zeros(2,N); xp = zeros(2,N);
for k = 1:N
    [zc, R] = meas_conv(r(k), el(k), p);
    if k == 1
        x = [zc;0;0];  P = zeros(4);  P(1:2,1:2) = R;  P(3,3) = 100^2;  P(4,4) = 100^2;
    else
        x = F*x;  P = F*P*F' + max(1,m)*Q0;
        if valid(k)
            S = Hm*P*Hm' + R;  K = (P*Hm')/S;
            y = zc - Hm*x;
            m = 0.7*m + 0.3*(y'/S*y)/2;
            x = x + K*y;
            A = eye(4) - K*Hm;  P = A*P*A' + K*R*K';
        end
    end
    xh(:,k) = x(1:2);
    yy = Fh*x;  xp(:,k) = yy(1:2);
end
end
