function ok = verify_assignment_contract()
%VERIFY_ASSIGNMENT_CONTRACT Check the three evaluator-facing ME319 functions.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'matlab'),fullfile(root,'matlab','strategies'));

assert(nargin(@target_motion_generator)==2 && nargout(@target_motion_generator)==3, ...
    'target_motion_generator must have 2 inputs and 3 outputs.');
assert(nargin(@turret_controller)==6 && nargout(@turret_controller)==2, ...
    'turret_controller must have 6 inputs and 2 outputs.');
assert(nargin(@turret_plant)==5 && nargout(@turret_plant)==3, ...
    'turret_plant must have 5 inputs and 3 outputs.');

clear target_motion_generator turret_controller turret_plant
[r_meas,theta_meas,meas_valid] = target_motion_generator(0,'A');
[V_m,fire_cmd] = turret_controller(0,r_meas,theta_meas,meas_valid,theta_meas,0);
[theta_dot_turret,theta_ddot_turret,i_dot] = turret_plant(0,V_m,theta_meas,0,0);

values=[r_meas theta_meas double(meas_valid) V_m fire_cmd ...
        theta_dot_turret theta_ddot_turret i_dot];
assert(all(isfinite(values)),'Evaluator-facing functions returned a non-finite value.');
assert(isscalar(meas_valid) && (meas_valid==0 || meas_valid==1), ...
    'meas_valid must be a binary scalar.');
assert(isscalar(fire_cmd) && (fire_cmd==0 || fire_cmd==1), ...
    'fire_cmd must be a binary scalar.');

ok=true;
fprintf('ME319 evaluator contract: PASS\n');
fprintf('  target_motion_generator: 2 inputs, 3 outputs\n');
fprintf('  turret_controller:       6 inputs, 2 outputs\n');
fprintf('  turret_plant:            5 inputs, 3 outputs\n');
end
