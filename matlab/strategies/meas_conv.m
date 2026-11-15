function [zc, R] = meas_conv(r, el, p)
% Polar (range, elevation) -> Cartesian measurement and its covariance.
zc = [r*cos(el); r*sin(el)];
J  = [cos(el) -r*sin(el); sin(el) r*cos(el)];
R  = J*diag([p.sig_r^2 p.sig_el^2])*J';
end
