%% MODEL 1 - STEP 8 : ACTUATOR COMMANDS AND FEEDBACK (CLOSED LOOP)
% Block 8 of the Model-1 equation flow (docs: Section 3.8).
%
%   INPUT  : u = [T, tau] from Step 7, rotor geometry and constants
%   OUTPUT : motor speeds Omega_1..4, the wrench the rotors really produce,
%            and - closing the loop - the full flight: truth -> sensors -> EKF ->
%            position control -> attitude control -> motors -> truth
%
%   [T; tau] = Gamma f,   f_i = kf Omega_i^2   ->   Omega = sqrt(Gamma^-1 u / kf)
%
% In the project: ArduPilot's motor mixer. Our ROS 2 node sends commands,
% ArduPilot converts them to the 4 ESC signals of the Iris in Gazebo.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model1_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'P', 'u_cmd_toy', 'u_cmd_chain', 'Qd', 'Rcam', 'Rgyro', 'dt_ekf'}, ...
                 'step7_attitude_control.m');
P = S.P;

step_header(1, 8, 'Actuator commands and feedback', { ...
    '[T; tau] = Gamma f,  f_i = kf Omega_i^2', 'Omega = sqrt( Gamma^-1 [T; tau] / kf )'});

%% Part A - toy examples from the documentation
fprintf('\nPart A : allocation matrix and toy examples\n');
io_print('IN', 'Gamma', P.Gamma, '-, m', 'rows: T, tau_x, tau_y, tau_z  (Iris geometry)');
io_print('IN', 'kf', P.kf, 'N s^2', 'rotor thrust constant');
uh = [P.m*P.g0; 0; 0; 0];
[Om, f] = motor_allocation(uh, P);
io_print('IN',  'u hover', uh, 'N, N m', 'hover command');
io_print('OUT', 'f',      f,  'N',     'each rotor carries a quarter of the weight');
io_print('OUT', 'Omega',  Om, 'rad/s', 'rotor speeds');
fprintf('          = %.0f RPM each\n', Om(1)*60/(2*pi));
[Om, f, ua] = motor_allocation(S.u_cmd_toy, P);
io_print('IN',  'u toy',  S.u_cmd_toy, 'N, N m', 'command from Step 7 Part A');
io_print('OUT', 'f',      f,  'N',     'right-side rotors (1,4) push harder -> rolls back to level');
io_print('OUT', 'Omega',  Om, 'rad/s', 'rotor speeds');
io_print('CHK', 'u_applied', ua, 'N, N m', 'equals the command (no saturation)');

%% Part B - chained snapshot from Step 7
fprintf('\nPart B : chained from Step 7\n');
[Om_c, ~, ua_c] = motor_allocation(S.u_cmd_chain, P);
io_print('IN',  'u',     S.u_cmd_chain, 'N, N m', 'from Step 7 Part B');
io_print('OUT', 'Omega', Om_c, 'rad/s', 'motor speeds sent to the drone');
io_print('OUT', 'u_applied', ua_c, 'N, N m', 'wrench produced by the rotors');

%% Part C - the whole loop: helix tracking with EKF in the loop (40 s)
fprintf('\nPart C : closed-loop flight (truth -> sensors -> EKF -> control -> motors)\n');
rng(8);
dt = S.dt_ekf; t = 0:dt:40; N = numel(t); camEvery = 4;
sig_p = 0.05; sig_th = 0.02; sig_g = 0.01; sig_F = 0.2; sig_tau = 5e-4;
wind = [0.1; 0; 0];                                     % steady breeze [N]
Hcam  = [eye(3), zeros(3,9); zeros(3,6), eye(3), zeros(3,3)];
Hgyro = [zeros(3,9), eye(3)];

x    = pack_state([0.3; -0.3; 1.8], zeros(3,1), eye(3), zeros(3,1));
xhat = pack_state([0.5; -0.4; 1.9], [0.1; 0; 0], so3_exp([0.03; -0.03; 0.05]), zeros(3,1));
Pcov = blkdiag(0.3^2*eye(3), 0.2^2*eye(3), 0.1^2*eye(3), 0.05^2*eye(3));

L.x = zeros(18,N); L.xh = zeros(18,N); L.pd = zeros(3,N); L.Om = zeros(4,N);
L.eR = zeros(1,N); L.u = zeros(4,N);
for k = 1:N
    % (1) sensors measure the real drone
    [p, ~, R, w] = unpack_state(x);
    zg = w + sig_g*randn(3,1);
    [~, ~, ~, wh] = unpack_state(xhat);
    [xhat, Pcov] = ekf_update(xhat, Pcov, zg - wh, Hgyro, S.Rgyro);
    if mod(k-1, camEvery) == 0
        zp = p + sig_p*randn(3,1);  zR = R*so3_exp(sig_th*randn(3,1));
        [ph, ~, Rh] = unpack_state(xhat);
        [xhat, Pcov] = ekf_update(xhat, Pcov, [zp - ph; so3_log(Rh'*zR)], Hcam, S.Rcam);
    end
    % (2) controller runs on the ESTIMATE
    [ph, vh, Rh, wh] = unpack_state(xhat);
    ref = reference_trajectory(t(k), 'helix');
    [~, ~, a_d]      = position_control(ph, vh, ref, P);
    [F_d, ~, R_d]    = force_attitude(a_d, ref.yaw, P);
    [e_R, ~, tau, T] = attitude_control(Rh, wh, R_d, ref.w, ref.wdot, F_d, P);
    % (3) motors
    [Om, ~, u_app]   = motor_allocation([T; tau], P);
    % log
    L.x(:,k) = x; L.xh(:,k) = xhat; L.pd(:,k) = ref.p; L.Om(:,k) = Om;
    L.eR(k) = norm(e_R); L.u(:,k) = u_app;
    % (4) the real drone moves (wind + gusts), the EKF predicts with the same input
    if k < N
        d = [wind + sig_F*randn(3,1); sig_tau*randn(3,1)];
        x = rk4_step(x, u_app, dt, P, d);
        [xhat, Pcov] = ekf_predict(xhat, Pcov, u_app, dt, P, S.Qd);
    end
end

settle = t > 10;
trackErr = vecnorm(L.x(1:3,:) - L.pd);
estErr   = vecnorm(L.x(1:3,:) - L.xh(1:3,:));
io_print('OUT', 'Omega range', [min(L.Om(:)) max(L.Om(:))], 'rad/s', 'motor speeds stay inside [100, 1100]');
io_print('CHK', 'track RMSE', sqrt(mean(trackErr(settle).^2)), 'm', '|p - p_d| after 10 s');
io_print('CHK', 'est RMSE',   sqrt(mean(estErr(settle).^2)),   'm', '|p - p_hat| after 10 s');
io_print('CHK', 'max |e_R|',  max(L.eR(settle)), '-', 'attitude error after 10 s');

fig = figure('Name', 'Step 8 - closed loop 3D', 'Position', [60 60 700 560]);
plot3(L.pd(1,:), L.pd(2,:), L.pd(3,:), 'k--', 'LineWidth', 1.2); hold on;
plot3(L.x(1,:), L.x(2,:), L.x(3,:), 'Color', [0.1 0.35 0.75], 'LineWidth', 1.6);
plot3(L.xh(1,:), L.xh(2,:), L.xh(3,:), ':', 'Color', [0.85 0.33 0.1], 'LineWidth', 1.2);
grid on; axis equal; view(40, 22); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend('reference p_d(t)', 'true drone', 'EKF estimate', 'Location', 'northeast');
title('Model 1 closed loop: helix tracking with estimation in the loop');
exportgraphics(fig, fullfile(figDir, 'step8_closed_loop_3d.png'), 'Resolution', 150);

fig = figure('Name', 'Step 8 - closed loop signals', 'Position', [60 60 1000 620]);
subplot(2,2,1); plot(t, trackErr, 'LineWidth', 1.3); grid on;
xlabel('t [s]'); ylabel('||p - p_d|| [m]'); title('Tracking error');
subplot(2,2,2); plot(t, estErr, 'LineWidth', 1.3); grid on;
xlabel('t [s]'); ylabel('||p - p\^|| [m]'); title('Estimation error (EKF)');
subplot(2,2,3); plot(t, L.eR, 'LineWidth', 1.3); grid on;
xlabel('t [s]'); ylabel('||e_R||'); title('SO(3) attitude error');
subplot(2,2,4); plot(t, L.Om, 'LineWidth', 1.0); grid on;
xlabel('t [s]'); ylabel('\Omega_i [rad/s]'); title('Motor speeds'); legend('1', '2', '3', '4');
exportgraphics(fig, fullfile(figDir, 'step8_closed_loop_signals.png'), 'Resolution', 150);

%% Save outputs
closed_loop = L; closed_loop.t = t;
save(dataFile, 'closed_loop', '-append');
fprintf('\nSaved closed_loop -> %s\n', dataFile);
