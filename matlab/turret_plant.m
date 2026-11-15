function [theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
%TURRET_PLANT First-principles elevation-drive dynamics from the assignment.
% Motor speed is N*theta_dot. Inertia and damping are reflected to the
% motor shaft. The hard stops are inelastic; voltage is limited by the
% controller, not here.
Jt = 1200; Jm = 0.20; N = 100; b = 1500;
Kt = 10; Ke = 10; R = 1.0; L = 0.10;
th_min = -10*pi/180; th_max = 85*pi/180;

Jeq = Jm + Jt/N^2;
beq = b/N^2;
omega_m = N*theta_dot_turret;
i_dot = (V_m - R*i - Ke*omega_m)/L;
omega_dot_m = (Kt*i - beq*omega_m)/Jeq;
theta_ddot_turret = omega_dot_m/N;

% Enforce the physical elevation range when an integrator lands on a stop.
if theta_turret >= th_max && theta_dot_turret >= 0
    theta_dot_turret = 0;
    theta_ddot_turret = min(theta_ddot_turret,0);
elseif theta_turret <= th_min && theta_dot_turret <= 0
    theta_dot_turret = 0;
    theta_ddot_turret = max(theta_ddot_turret,0);
end
end
