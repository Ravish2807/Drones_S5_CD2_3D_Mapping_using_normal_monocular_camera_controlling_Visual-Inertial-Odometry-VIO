%% MODEL 2 - STEP 9 : MPC CONTROL INPUT (RECEDING HORIZON)
% Block 9 of the Model-2 equation flow (docs: Section 4.9).
%
%   INPUT  : current state x_k, the QP of Step 8
%   OUTPUT : optimal plan U* = [u_0*, ..., u_{N-1}*] and the ONE input applied, u_0*
%
%   u_k = [u_0*, u_1*, ..., u_{N-1}*]  ->  apply u_0*, measure, re-solve
%
% In the project: plan 1 s ahead, use only the first 40 ms of the plan,
% then plan again with fresh sensor data. Mistakes never accumulate.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'mpc', 'K_lqr', 'A_dmd', 'B_dmd', 'dt'}, 'step8_qp_formulation.m');
mpc = S.mpc; A = S.A_dmd; B = S.B_dmd; K = S.K_lqr;

step_header(2, 9, 'MPC control input (receding horizon)', { ...
    'U* = [u_0*, ..., u_{N-1}*] = argmin QP(x_k)', 'apply u_k = u_0*,  k <- k + 1,  repeat'});

%% Receding-horizon loop on the learned linear model (8 s)
x0 = [1.5; -1.0; -1.2; zeros(9,1)];
Kn = 200; t = (0:Kn)*S.dt;
Xm = zeros(12, Kn+1); Um = zeros(4, Kn); Xl = Xm; Ul = Um;
Xm(:,1) = x0; Xl(:,1) = x0; nRelax = 0; tSolve = zeros(1, Kn);
for k = 1:Kn
    tic; [Um(:,k), Uplan, info] = mpc_solve(mpc, Xm(:,k)); tSolve(k) = toc;
    nRelax = nRelax + info.relaxed;
    if k == 1, Uplan1 = reshape(Uplan, 4, []); end
    Xm(:,k+1) = A*Xm(:,k) + B*Um(:,k);
    Ul(:,k) = min(max(-K*Xl(:,k), mpc.u_min), mpc.u_max);   % LQR, clipped to motor limits
    Xl(:,k+1) = A*Xl(:,k) + B*Ul(:,k);
end

io_print('IN',  'x_0', x0, 'mixed', 'initial deviation');
io_print('OUT', 'U* at k=0', Uplan1(:, 1:3), 'N, N m', 'first 3 of 25 planned inputs (columns)');
io_print('OUT', 'u_0* (k=0)', Um(:,1), 'N, N m', 'the only one actually applied');
io_print('CHK', 'max|v| MPC', max(max(abs(Xm(4:6,:)))), 'm/s', 'speed limit 1 m/s respected');
io_print('CHK', 'max|v| LQR', max(max(abs(Xl(4:6,:)))), 'm/s', 'clipped LQR has no notion of a speed limit');
io_print('CHK', '||x(8s)|| MPC', norm(Xm(:,end)), '-', 'converged to the setpoint');
io_print('CHK', 'relaxed', nRelax, '-', 'samples where state limits had to be dropped');
io_print('CHK', 'solve time', 1e3*mean(tSolve), 'ms', 'mean QP time (budget: 40 ms)');

fig = figure('Name', 'Step 9 - receding horizon', 'Position', [60 60 1050 420]);
subplot(1,2,1);
plot(t, vecnorm(Xm(4:6,:)), 'LineWidth', 1.6); hold on; plot(t, vecnorm(Xl(4:6,:)), '--', 'LineWidth', 1.6);
yline(1, 'k:', 'v_{max} (per axis)'); grid on; xlabel('t [s]'); ylabel('||v|| [m/s]');
legend('MPC', 'LQR (clipped)'); title('Speed');
subplot(1,2,2);
plot(t, vecnorm(Xm(1:3,:)), 'LineWidth', 1.6); hold on; plot(t, vecnorm(Xl(1:3,:)), '--', 'LineWidth', 1.6);
grid on; xlabel('t [s]'); ylabel('||p - p_{ref}|| [m]'); legend('MPC', 'LQR (clipped)');
title('Distance to the setpoint');
exportgraphics(fig, fullfile(figDir, 'step9_receding_horizon.png'), 'Resolution', 150);

%% Save
lin_sim = struct('t', t, 'Xm', Xm, 'Um', Um, 'Xl', Xl, 'Ul', Ul);
save(dataFile, 'lin_sim', '-append');
fprintf('\nSaved lin_sim -> %s\n', dataFile);
