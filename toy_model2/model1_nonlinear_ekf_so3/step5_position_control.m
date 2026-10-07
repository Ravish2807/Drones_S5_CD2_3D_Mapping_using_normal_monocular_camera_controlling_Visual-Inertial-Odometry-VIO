%% MODEL 1 - STEP 5 : POSITION CONTROL (OUTER LOOP)
% Block 5 of the Model-1 equation flow (docs: Section 3.5).
%
%   INPUT  : estimated position/velocity (p_hat, v_hat) from the EKF (Step 4),
%            desired trajectory (p_d, v_d, pdd_d), gains Kp, Kv
%   OUTPUT : tracking errors e_p, e_v and desired acceleration a_d
%
%   e_p = p_d - p_hat          e_v = v_d - v_hat
%   a_d = pdd_d + Kp e_p + Kv e_v
%
% In the project: so3_controller.py -> compute_position_errors() and the
% a_cmd line of compute_desired_force().  "Where am I vs. where should I be,
% and how hard should I accelerate to get there?"

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model1_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'P', 'xhat_ekf', 't_ekf'}, 'step4_error_state_ekf.m');
P = S.P;

step_header(1, 5, 'Position control (outer loop)', { ...
    'e_p = p_d - p_hat     e_v = v_d - v_hat', 'a_d = pdd_d + Kp e_p + Kv e_v'});

%% Part A - toy example from the documentation
fprintf('\nPart A : toy example (hover setpoint, drone 50 cm low and climbing)\n');
ref = reference_trajectory(0, 'hover');
p_hat = [0.1; -0.2; 1.5];  v_hat = [0; 0; 0.3];
[e_p, e_v, a_d] = position_control(p_hat, v_hat, ref, P);
io_print('IN',  'p_d',   ref.p, 'm',     'desired position (controller.yaml)');
io_print('IN',  'v_d',   ref.v, 'm/s',   'desired velocity');
io_print('IN',  'pdd_d', ref.a, 'm/s^2', 'desired acceleration (feed-forward)');
io_print('IN',  'p_hat', p_hat, 'm',     'estimated position');
io_print('IN',  'v_hat', v_hat, 'm/s',   'estimated velocity');
io_print('IN',  'Kp',    diag(P.Kp)', '1/s^2', 'diag of position gain');
io_print('IN',  'Kv',    diag(P.Kv)', '1/s',   'diag of velocity gain');
io_print('OUT', 'e_p',   e_p, 'm',     'position error');
io_print('OUT', 'e_v',   e_v, 'm/s',   'velocity error');
io_print('OUT', 'a_d',   a_d, 'm/s^2', 'desired acceleration');

%% Part B - chained: use the EKF estimate from Step 4
fprintf('\nPart B : chained from Step 4 (final EKF estimate, helix reference at t = %.0f s)\n', S.t_ekf);
[ph, vh] = unpack_state(S.xhat_ekf);
ref_c = reference_trajectory(S.t_ekf, 'helix');
[e_p_c, e_v_c, a_d_c] = position_control(ph, vh, ref_c, P);
io_print('IN',  'p_d',   ref_c.p, 'm',     'helix reference position');
io_print('IN',  'v_d',   ref_c.v, 'm/s',   'helix reference velocity');
io_print('IN',  'pdd_d', ref_c.a, 'm/s^2', 'helix reference acceleration');
io_print('IN',  'p_hat', ph, 'm', 'from Step 4');
io_print('IN',  'v_hat', vh, 'm/s', 'from Step 4');
io_print('OUT', 'e_p',   e_p_c, 'm', 'position error');
io_print('OUT', 'e_v',   e_v_c, 'm/s', 'velocity error');
io_print('OUT', 'a_d',   a_d_c, 'm/s^2', 'desired acceleration -> Step 6');

%% Part C - what the gains mean: closed-loop error dynamics  e'' + Kv e' + Kp e = 0
fprintf('\nPart C : error dynamics per axis (if the inner loop is perfect)\n');
wn = sqrt(diag(P.Kp)); zeta = diag(P.Kv)./(2*wn);
fprintf('   axis |  Kp  |  Kv  | wn [rad/s] | zeta | 2%% settling time [s]\n');
ax = 'xyz';
for i = 1:3
    fprintf('     %c  | %.2f | %.2f |   %.3f    | %.3f | %.2f\n', ax(i), P.Kp(i,i), ...
        P.Kv(i,i), wn(i), zeta(i), 4/(zeta(i)*wn(i)));
end
dt = 0.01; t = 0:dt:8; e = zeros(3, numel(t)); ed = zeros(3, numel(t));
e(:,1) = e_p; ed(:,1) = e_v;
for k = 1:numel(t)-1
    edd = -P.Kp*e(:,k) - P.Kv*ed(:,k);
    ed(:,k+1) = ed(:,k) + dt*edd;
    e(:,k+1)  = e(:,k)  + dt*ed(:,k+1);
end
fig = figure('Name', 'Step 5 - error dynamics', 'Position', [100 100 600 360]);
plot(t, e, 'LineWidth', 1.5); grid on; xlabel('t [s]'); ylabel('e_p [m]');
legend('e_x', 'e_y', 'e_z'); title('Position error decays: e'''' + K_v e'' + K_p e = 0');
exportgraphics(fig, fullfile(figDir, 'step5_error_decay.png'), 'Resolution', 150);

%% Save outputs (chained values go to Step 6)
ref_chain = ref_c; a_d_chain = a_d_c; a_d_toy = a_d;
save(dataFile, 'ref_chain', 'a_d_chain', 'a_d_toy', '-append');
fprintf('\nSaved ref_chain, a_d_chain, a_d_toy -> %s\n', dataFile);
