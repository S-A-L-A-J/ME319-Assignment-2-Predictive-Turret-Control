function [r_meas,theta_meas,meas_valid] = target_motion_generator(t,scenario)
%TARGET_MOTION_GENERATOR Standardized A/B/C measurement interface.
% A measurement is released once when time reaches each 20 Hz sample. Between
% releases, the most recent values are held and meas_valid is false.
persistent data last_t last_scenario last_r last_theta last_sample_k
root=fileparts(fileparts(mfilename('fullpath')));
if isempty(data), S=load(fullfile(root,'data','scenarios.mat')); data=S.data; end
if isnumeric(scenario) && isscalar(scenario) && scenario>=1 && scenario<=numel(data)
    scenario=data(round(scenario)).id;
elseif isstring(scenario)
    scenario=char(scenario);
end
idx=find(strcmp({data.id},scenario),1);
if isempty(idx), error('Unknown scenario: %s',scenario); end
d=data(idx); dt=0.05; N=numel(d.valid);
if isempty(last_t) || t<last_t || isempty(last_scenario) || ~strcmp(last_scenario,scenario)
    last_r=d.r(1,1); last_theta=d.el(1,1); last_t=-inf;
    last_scenario=scenario; last_sample_k=0;
end
k=floor((t+1e-9)/dt)+1;
is_new_sample=k>=1 && k<=N && k>last_sample_k;
if is_new_sample
    last_sample_k=k;
    if d.valid(k), last_r=d.r(1,k); last_theta=d.el(1,k); end
    meas_valid=logical(d.valid(k));
else
    meas_valid=false;
end
r_meas=last_r; theta_meas=last_theta; last_t=t;
end
