%% MODEL 1 - STEP 7 : ATTITUDE CONTROL ON SO(3) (INNER LOOP)
% Block 7 of the Model-1 equation flow (docs: Section 3.7).
%
%   INPUT  : estimated attitude/rate (R_hat, w_hat) from the EKF,
%            desired attitude R_d and force F_d (Step 6), desired rates w_d, wdot_d
%   OUTPUT : attitude error e_R, rate error e_w, torque tau, thrust T
%
%   e_R = 1/2 (R_d' R - R' R_d)^vee
%   e_w = w - R' R_d w_d                    (corrected sign, see docs "Errata")
%   tau = -KR e_R - Kw e_w + w x J w - J([w]x R' R_d w_d - R' R_d wdot_d)
%   T   = F_d . (R e3)
%
% In the project: so3_controller.py -> compute_attitude_error(),
% compute_angular_velocity_error(), compute_moment().

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model1_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'P', 'xhat_ekf', 'F_d_chain', 'R_d_chain'}, 'step6_force_attitude.m');
P = S.P;

step_header(1, 7, 'SO(3) attitude control (inner loop)', { ...
    'e_R = 1/2 (Rd'' R - R'' Rd)^vee        e_w = w - R'' Rd w_d', ...
    'tau = -KR e_R - Kw e_w + w x Jw - J([w]x R''Rd w_d - R''Rd wdot_d)', ...
    'T   = F_d . (R e3)'});

%% Part A - toy example from the documentation
% Drone is rolled 10 deg and still rolling at 0.2 rad/s; it should be level.
fprintf('\nPart A : toy example (10 deg roll, rolling further at 0.2 rad/s)\n');
R_hat = so3_exp([deg2rad(10); 0; 0]);  w_hat = [0.2; 0; 0];
R_d = eye(3); w_d = zeros(3,1); wdot_d = zeros(3,1); F_d = [0; 0; P.m*P.g0];
[e_R, e_w, tau, T] = attitude_control(R_hat, w_hat, R_d, w_d, wdot_d, F_d, P);
io_print('IN',  'R_hat', R_hat, '-',     'estimated attitude (10 deg roll)');
io_print('IN',  'w_hat', w_hat, 'rad/s', 'estimated body rate');
io_print('IN',  'R_d',   R_d,   '-',     'desired attitude (level)');
io_print('IN',  'F_d',   F_d,   'N',     'desired force (hover)');
io_print('OUT', 'e_R',   e_R,   '-',     '[sin(10 deg), 0, 0]');
io_print('OUT', 'e_w',   e_w,   'rad/s', 'rate error');
io_print('OUT', 'tau',   tau,   'N m',   'restoring torque (negative roll)');
io_print('OUT', 'T',     T,     'N',     'thrust = m g0 cos(10 deg)');

%% Part B - chained from Steps 4 and 6
fprintf('\nPart B : chained (R_hat, w_hat from Step 4; R_d, F_d from Step 6)\n');
[~, ~, Rh, wh] = unpack_state(S.xhat_ekf);
[e_R_c, e_w_c, tau_c, T_c] = attitude_control(Rh, wh, S.R_d_chain, zeros(3,1), zeros(3,1), S.F_d_chain, P);
io_print('IN',  'R_hat', Rh, '-', 'from Step 4');
io_print('IN',  'w_hat', wh, 'rad/s', 'from Step 4');
io_print('OUT', 'e_R',   e_R_c, '-', 'attitude error');
io_print('OUT', 'e_w',   e_w_c, 'rad/s', 'rate error');
io_print('OUT', 'tau',   tau_c, 'N m', 'torque command -> Step 8');
io_print('OUT', 'T',     T_c,   'N', 'thrust command -> Step 8');

%% Part C - stability: correct sign vs. the sign printed in the original flow image
% Start 60 deg away from level and let the attitude loop recover (true dynamics).
fprintf('\nPart C : recovery from a 60 deg tilt, Lyapunov function V = 1/2 e_w''J e_w + Psi\n');
dt = 0.002; t = 0:dt:3; N = numel(t);
R0 = so3_exp(deg2rad(60)*[1; 1; 0]/sqrt(2));
[V_ok, ang_ok]   = attitude_sim(R0, +1, P, t);
[V_bad, ang_bad] = attitude_sim(R0, -1, P, t);
io_print('CHK', 'angle(3 s)', ang_ok(end),  'deg', 'with e_w = w - R''R_d w_d  (converges)');
kBad = find(~isnan(ang_bad), 1, 'last');
io_print('CHK', 't blow-up', t(kBad), 's', ...
    'with e_w = w_d - w as in image: |w| > 50 rad/s (tumbling)');
io_print('CHK', 'V monotone', all(diff(V_ok) <= 1e-6*V_ok(1)), '-', ...
    'V never increases with correct sign (1 = true, up to RK4 round-off)');
io_print('CHK', 'V(3s)/V(0)', V_ok(end)/V_ok(1), '-', 'energy-like function has decayed');

fig = figure('Name', 'Step 7 - attitude recovery', 'Position', [100 100 950 360]);
subplot(1,2,1); plot(t, ang_ok, 'LineWidth', 1.6); hold on; plot(t, ang_bad, '--', 'LineWidth', 1.6);
grid on; xlabel('t [s]'); ylabel('angle between R and R_d [deg]'); ylim([0 180]);
legend('e_\omega = \omega - R^TR_d\omega_d (correct)', 'e_\omega = \omega_d - \omega (image sign)', ...
       'Location', 'east');
title('Attitude recovery from 60\circ');
subplot(1,2,2); semilogy(t, V_ok, 'LineWidth', 1.6); grid on;
xlabel('t [s]'); ylabel('V(t)'); title('Lyapunov function decreases monotonically');
exportgraphics(fig, fullfile(figDir, 'step7_attitude_recovery.png'), 'Resolution', 150);

%% Save outputs
u_cmd_chain = [T_c; tau_c]; u_cmd_toy = [T; tau];
save(dataFile, 'u_cmd_chain', 'u_cmd_toy', '-append');
fprintf('\nSaved u_cmd_chain, u_cmd_toy -> %s\n', dataFile);

%% Local function: attitude-only closed loop
function [V, ang] = attitude_sim(R0, sgn, P, t)
% sgn = +1 : e_w = w - R'R_d w_d    sgn = -1 : e_w = w_d - w  (image)
% Uses scalar gains kR = 4, kw = 0.8 so the Lyapunov argument applies exactly.
kR = 4; kw = 0.8;
dt = t(2) - t(1); N = numel(t);
x = pack_state(zeros(3,1), zeros(3,1), R0, zeros(3,1));
V = nan(1, N); ang = nan(1, N);
for k = 1:N
    [~, ~, R, w] = unpack_state(x);
    if norm(w) > 50, break, end            % spinning out of control: stop logging
    e_R = 0.5*vee(R - R');                 % R_d = I  ->  1/2 (R - R')^vee
    e_w = sgn*w;
    V(k)   = 0.5*e_w'*P.J*e_w + kR*0.5*trace(eye(3) - R);
    ang(k) = rad2deg(norm(so3_log(R)));
    tau = -kR*e_R - kw*e_w + cross(w, P.J*w);
    x = rk4_step(x, [P.m*P.g0; tau], dt, P);
end
end
