%% MODEL 2 - STEP 10 : APPLY TO THE DRONE
% Block 10 of the Model-2 equation flow (docs: Section 4.10).
%
%   INPUT  : measured state x_k of the NONLINEAR drone, MPC (Step 9) or LQR (Step 6)
%   OUTPUT : thrust/torque u_k = [T_k, tau_k] -> motor allocation -> motor speeds,
%            and the resulting nonlinear flight
%
%   u_k = u_trim + du_k  ->  f = Gamma^-1 u_k  ->  Omega_i = sqrt(f_i / kf)
%
% In the project: the controllers were designed on a LEARNED LINEAR model.
% This step checks that they still work on the real (nonlinear, gusty,
% motor-limited) drone.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'P', 'mpc', 'K_lqr', 'dt', 'nSub', 'u_trim'}, 'step9_mpc_control_input.m');
P = S.P; dt = S.dt;

step_header(2, 10, 'Apply to the drone', { ...
    'u_k = u_trim + du_k = [T_k; tau_k]', 'f = Gamma^-1 u_k,   Omega_i = sqrt(f_i / kf)'});

%% Part A - one sample, by hand
fprintf('\nPart A : a single control sample\n');
x_world = pack_state(P.p_hover + [1.5; -1.0; -1.2], zeros(3,1), eye(3), zeros(3,1));
xd = state_to_dev(x_world, P.p_hover);
du = mpc_solve(S.mpc, xd);
[Om, f] = motor_allocation(S.u_trim + du, P);
io_print('IN',  'x_k', xd, 'mixed', 'measured deviation state');
io_print('OUT', 'du_k', du, 'N, N m', 'MPC output u_0*');
io_print('OUT', 'u_k', S.u_trim + du, 'N, N m', '[T; tau] after adding hover thrust');
io_print('OUT', 'f',   f,  'N', 'rotor thrusts');
io_print('OUT', 'Omega', Om, 'rad/s', 'motor speeds sent to the ESCs');

%% Part B - full nonlinear flights: MPC vs LQR
rng(10);
Kn = 250; t = (0:Kn)*dt;
ctrl = {'MPC', 'LQR'};
for c = 1:2
    x = x_world; Xw = zeros(18, Kn+1); Om = zeros(4, Kn); Xw(:,1) = x;
    for k = 1:Kn
        xm = state_to_dev(x, P.p_hover) + [0.01*ones(3,1); 0.02*ones(3,1); 0.005*ones(3,1); 0.01*ones(3,1)].*randn(12,1);
        if c == 1
            du = mpc_solve(S.mpc, xm);
        else
            du = min(max(-S.K_lqr*xm, S.mpc.u_min), S.mpc.u_max);
        end
        [Om(:,k), ~, u_app] = motor_allocation(S.u_trim + du, P);
        d = [0.2*randn(3,1); 5e-4*randn(3,1)];
        for j = 1:S.nSub, x = rk4_step(x, u_app, dt/S.nSub, P, d); end
        Xw(:,k+1) = x;
    end
    res.(ctrl{c}) = struct('X', Xw, 'Om', Om);
    vmax = max(max(abs(Xw(4:6,:))));
    err  = norm(Xw(1:3,end) - P.p_hover);
    io_print('CHK', [ctrl{c} ' max|v|'], vmax, 'm/s', 'largest velocity component in flight');
    io_print('CHK', [ctrl{c} ' err(10s)'], err, 'm', 'distance to setpoint at t = 10 s');
    io_print('CHK', [ctrl{c} ' Omega'], [min(Om(:)) max(Om(:))], 'rad/s', 'motor speed range');
end

fig = figure('Name', 'Step 10 - nonlinear flight', 'Position', [60 60 1100 430]);
subplot(1,2,1); hold on;
plot3(res.MPC.X(1,:), res.MPC.X(2,:), res.MPC.X(3,:), 'LineWidth', 1.8);
plot3(res.LQR.X(1,:), res.LQR.X(2,:), res.LQR.X(3,:), '--', 'LineWidth', 1.6);
plot3(P.p_hover(1), P.p_hover(2), P.p_hover(3), 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'y');
plot3(x_world(1), x_world(2), x_world(3), 'ko', 'MarkerFaceColor', 'k');
grid on; view(35, 25); axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend('MPC', 'LQR (clipped)', 'setpoint', 'start', 'Location', 'northeast');
title('Nonlinear drone flown with controllers learned from data');
subplot(1,2,2);
plot(t, max(abs(res.MPC.X(4:6,:))), 'LineWidth', 1.6); hold on;
plot(t, max(abs(res.LQR.X(4:6,:))), '--', 'LineWidth', 1.6); yline(1, 'k:', 'max\_velocity = 1 m/s', 'LabelHorizontalAlignment', 'left');
grid on; xlabel('t [s]'); ylabel('max_i |v_i| [m/s]'); legend('MPC', 'LQR (clipped)');
title('MPC respects the speed limit on the real (nonlinear) drone');
exportgraphics(fig, fullfile(figDir, 'step10_nonlinear_flight.png'), 'Resolution', 150);

%% Save
flight = res; flight.t = t;
save(dataFile, 'flight', '-append');
fprintf('\nSaved flight -> %s\n', dataFile);
