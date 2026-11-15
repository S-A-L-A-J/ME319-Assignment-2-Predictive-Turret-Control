function [V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
%TURRET_CONTROLLER Frozen IMM predictor plus LTI elevation compensator.
% Evaluation signature required by ME319 Assignment 2. Call with t=0 (or a
% decreasing time) to reset persistent state. All angles are radians.
%
% A receding-horizon scheduler predicts an intercept point 0.50 s ahead and
% holds its elevation reference until that intercept time. A one-call fire_cmd
% pulse is issued only when the instantaneous estimated ray miss is <=1.25 m.
% The next pulse is inhibited for 1.0 s.

persistent xs Ps mu initialized last_t last_est_t xlp xref last_fire
persistent aim_ref fire_due shot_scheduled

dt_meas = 0.05; tau_pred = 0.50;
sig_r = 1; sig_el = 0.02*pi/180;
sig_a = 2; sig_j = 10; switch_prob = 0.02;
K = 7000; tau_fb = 0.05; tau_ff = 0.02;
Ngear = 100; Ke = 10; Kt = 10; Rm = 1; b = 1500;
beq = b/Ngear^2;
Kvp = (Kt/Ngear)/(Rm*beq + Kt*Ke);
gff = 1/Kvp;
Vmax = 400;                         % stated design assumption
th_min = -10*pi/180; th_max = 85*pi/180;
arm_time = 1.0; recharge = 1.0; fire_gate = 1.25;

if isempty(initialized) || isempty(last_t) || t <= 0 || t < last_t
    xs = {}; Ps = {}; mu = [0.7;0.3]; initialized = false;
    last_t = t; last_est_t = t; xlp = 0; xref = theta_turret;
    last_fire = -inf; aim_ref=theta_turret; fire_due=inf; shot_scheduled=false;
end
fire_cmd = 0;

% Initialize from the first available polar measurement.
if ~initialized && meas_valid
    [z,Rz] = convert_measurement(r_meas,theta_meas,sig_r,sig_el);
    x0 = [z(1);0;0;z(2);0;0];
    P0 = zeros(6); P0([1 4],[1 4]) = Rz;
    P0(2,2)=100^2; P0(5,5)=100^2; P0(3,3)=10^2; P0(6,6)=10^2;
    xs = {x0,x0}; Ps = {P0,P0}; initialized = true;
    last_est_t = t;
end

% Estimator prediction/update at the 20 Hz radar cadence. During loss, the
% prediction still advances but the measurement update is skipped.
if initialized && (t-last_est_t >= dt_meas-1e-9)
    h = t-last_est_t;
    [F,Q,Hm] = model_matrices(h,sig_a,sig_j);
    Pi = [1-switch_prob switch_prob; switch_prob 1-switch_prob];
    cbar = Pi'*mu; xm = cell(1,2); Pm = cell(1,2);
    for j=1:2
        w = Pi(:,j).*mu/cbar(j);
        xm{j}=w(1)*xs{1}+w(2)*xs{2}; Pm{j}=zeros(6);
        for ii=1:2
            dx=xs{ii}-xm{j}; Pm{j}=Pm{j}+w(ii)*(Ps{ii}+dx*dx');
        end
    end
    likelihood=ones(2,1);
    for j=1:2
        xs{j}=F{j}*xm{j}; Ps{j}=F{j}*Pm{j}*F{j}'+Q{j};
        Ps{j}=(Ps{j}+Ps{j}')/2;
        if meas_valid
            [z,Rz]=convert_measurement(r_meas,theta_meas,sig_r,sig_el);
            S=Hm*Ps{j}*Hm'+Rz; S=(S+S')/2; innov=z-Hm*xs{j};
            [U,pchol]=chol(S);
            if pchol~=0
                S=S+1e-9*max(trace(S),1)*eye(2); U=chol(S);
            end
            q=U'\innov; loglik=-0.5*(q'*q+2*sum(log(diag(U)))+2*log(2*pi));
            likelihood(j)=exp(max(loglik,log(realmin)));
            Kgain=(Ps{j}*Hm')/S; xs{j}=xs{j}+Kgain*innov;
            A=eye(6)-Kgain*Hm; Ps{j}=A*Ps{j}*A'+Kgain*Rz*Kgain';
            Ps{j}=(Ps{j}+Ps{j}')/2;
        end
    end
    if meas_valid
        mu=likelihood.*cbar; mu=mu/sum(mu);
    else
        mu=cbar;
    end
    last_est_t=t;
end

if initialized
    xhat=mu(1)*xs{1}+mu(2)*xs{2};
    current_los=atan2(xhat(4),xhat(1)); current_range=hypot(xhat(1),xhat(4));
    [Fh,~,~]=model_matrices(tau_pred,sig_a,sig_j);
    future=mu(1)*(Fh{1}*xs{1})+mu(2)*(Fh{2}*xs{2});
    future_los=atan2(future(4),future(1));
else
    current_los=theta_turret; current_range=inf; future_los=theta_turret;
end

schedule_open=initialized && t>=max(0,arm_time-tau_pred) && ...
    (t+tau_pred-last_fire)>=recharge-1e-9;
if ~shot_scheduled && schedule_open
    aim_ref=future_los; fire_due=t+tau_pred; shot_scheduled=true;
end
if shot_scheduled, reference=aim_ref; else, reference=future_los; end
reference=min(max(reference,th_min),th_max);

% Two-degree-of-freedom controller:
%   Vfb = Gc(s)*(theta_ref-theta), Gc(s)=7000/(0.05s+1)
%   Vff = Gff(s)*theta_ref, Gff(s)=gff*s/(0.02s+1)
% where gff=1/Kvp is the inverse low-frequency plant velocity gain. The
% feedforward changes reference tracking but not the feedback characteristic
% equation; the summed command is then clipped to the stated voltage limit.
h=max(t-last_t,0); e=atan2(sin(reference-theta_turret),cos(reference-theta_turret));
if h>0
    alpha_fb=1-exp(-h/tau_fb); alpha_ff=1-exp(-h/tau_ff);
    xlp=xlp+alpha_fb*(K*e-xlp);
    xref=xref+alpha_ff*(reference-xref);
end
V_m=min(max(xlp+gff*(reference-xref)/tau_ff,-Vmax),Vmax);

if shot_scheduled && t>=fire_due-1e-9
    delta=atan2(sin(theta_turret-current_los),cos(theta_turret-current_los));
    if current_range*cos(delta)>=0
        miss_est=current_range*abs(sin(delta));
    else
        miss_est=current_range;
    end
    if t>=arm_time && (t-last_fire)>=recharge-1e-9 && miss_est<=fire_gate
        fire_cmd=1; last_fire=t;
    end
    shot_scheduled=false;
end
last_t=t;
end

function [z,Rz]=convert_measurement(r,el,sr,se)
z=[r*cos(el);r*sin(el)];
J=[cos(el) -r*sin(el);sin(el) r*cos(el)];
Rz=J*diag([sr^2 se^2])*J';
end

function [F,Q,Hm]=model_matrices(h,sa,sj)
Z=zeros(3); cv=[1 h 0;0 1 0;0 0 0]; ca=[1 h h^2/2;0 1 h;0 0 1];
g=[h^2/2;h;0]; qcv=sa^2*(g*g');
q1=[h^5/20 h^4/8 h^3/6;h^4/8 h^3/3 h^2/2;h^3/6 h^2/2 h]; qca=sj^2*q1;
F={[cv Z;Z cv],[ca Z;Z ca]}; Q={[qcv Z;Z qcv],[qca Z;Z qca]};
Hm=[1 0 0 0 0 0;0 0 0 1 0 0];
end
