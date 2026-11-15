function metrics = validate_tracking_pipeline()
%VALIDATE_TRACKING_PIPELINE Deterministic checks for estimation and prediction.
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'matlab'),fullfile(root,'matlab','strategies'));
p.dt=0.05; p.T=10; p.tau=0.50; p.sig_r=1; p.sig_el=0.02*pi/180;
theta=[2 10]; N=round(p.T/p.dt)+1; H=round(p.tau/p.dt);
t=(0:N+H-1)*p.dt;

% Noise-free constant-velocity trajectory.
x=[1300-38*t; 420+17*t]; v=[-38;17];
[cv_est,cv_pred]=run_case(x,true(1,N),p,theta,N,H);
cv_est_rmse=rmse_after(cv_est,x(:,1:N),p.dt,1.0);
cv_pred_rmse=rmse_after(cv_pred,x(:,1+H:N+H),p.dt,1.0);

% Noise-free constant-acceleration trajectory and a 0.50 s measurement loss.
a=[-3.5;2.0]; xca=[1450;350]+v*t+0.5*a*t.^2;
valid=true(1,N); valid(round(1.0/p.dt)+1:round(1.5/p.dt))=false;
[ca_est,ca_pred]=run_case(xca,valid,p,theta,N,H);
ca_est_rmse=rmse_after(ca_est,xca(:,1:N),p.dt,1.0);
ca_pred_rmse=rmse_after(ca_pred,xca(:,1+H:N+H),p.dt,1.0);
loss_idx=round(1.0/p.dt)+1:round(2.0/p.dt)+1;
loss_pred_rmse=sqrt(mean(sum((ca_pred(:,loss_idx)-xca(:,loss_idx+H)).^2,1)));

assert(all(isfinite([cv_est(:);cv_pred(:);ca_est(:);ca_pred(:)])), ...
    'Estimator or predictor produced a non-finite value.');
assert(cv_est_rmse<0.5 && cv_pred_rmse<1.0, ...
    'Constant-velocity tracking/prediction regression failed.');
assert(ca_est_rmse<1.0 && ca_pred_rmse<2.5 && loss_pred_rmse<4.0, ...
    'Maneuver/dropout tracking regression failed.');

% Each 20 Hz measurement must be released exactly once, even on repeat calls.
clear target_motion_generator
[~,~,v0a]=target_motion_generator(0,'A');
[~,~,v0b]=target_motion_generator(0,'A');
[~,~,v1a]=target_motion_generator(0.0501,'A');
[~,~,v1b]=target_motion_generator(0.0501,'A');
assert(v0a && ~v0b && v1a && ~v1b, ...
    'Measurement-valid pulse is not one-shot at the 20 Hz cadence.');

% Frozen datasets must cover the prediction horizon and obey the envelope.
files={'scenarios.mat','tuning.mat','test.mat'};
for f=1:numel(files)
    S=load(fullfile(root,'data',files{f})); data=S.data;
    for k=1:numel(data)
        assert(size(data(k).x,2)>=N+H && numel(data(k).valid)==N, ...
            'Dataset does not cover the full prediction horizon.');
        assert(data(k).env(1)>=800 && data(k).env(2)<=2000 && ...
               data(k).env(3)<=150 && data(k).env(4)<=20, ...
            'Dataset violates the prescribed motion envelope.');
    end
end

if exist('OCTAVE_VERSION','builtin')
    metrics=struct('cv_est_rmse',cv_est_rmse,'cv_pred_rmse',cv_pred_rmse, ...
        'ca_est_rmse',ca_est_rmse,'ca_pred_rmse',ca_pred_rmse, ...
        'loss_pred_rmse',loss_pred_rmse);
else
    metrics=table(cv_est_rmse,cv_pred_rmse,ca_est_rmse,ca_pred_rmse,loss_pred_rmse);
end
disp(metrics);
fprintf('Tracking and prediction validation: PASS\n');
end

function [xh,xp]=run_case(x,valid,p,theta,N,H)
r=hypot(x(1,1:N),x(2,1:N)); el=atan2(x(2,1:N),x(1,1:N));
[xh,xp]=imm_cvca(r,el,valid,p,theta);
assert(size(xp,2)==N && size(x,2)>=N+H);
end

function value=rmse_after(a,b,dt,t0)
k0=round(t0/dt)+1;
value=sqrt(mean(sum((a(:,k0:end)-b(:,k0:end)).^2,1)));
end
