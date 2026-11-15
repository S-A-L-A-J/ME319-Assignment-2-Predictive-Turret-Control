function [Phi, Q] = vanloan(Fc, G, q, dt)
% Discretise  xdot = Fc x + G w  (white w, spectral density q)  over dt (Van Loan).
n = size(Fc,1);
A = [-Fc, G*q*G'; zeros(n), Fc'] * dt;
B = expm(A);
Phi = B(n+1:2*n, n+1:2*n)';
Q   = Phi * B(1:n, n+1:2*n);
end
