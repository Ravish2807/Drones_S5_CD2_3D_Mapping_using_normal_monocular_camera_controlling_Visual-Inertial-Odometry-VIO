%% MODEL 1 - STEP 3 : LOCAL ERROR DYNAMICS (LINEARIZED)
% Block 3 of the Model-1 equation flow (docs: Section 3.3).
%
%   INPUT  : A, B from Step 2, nominal trajectory xhat(t), initial error dx(0),
%            input deviation du(t)
%   OUTPUT : predicted error dx(t) from  d(dx)/dt = A dx + B du,
%            discrete transition F = I + A dt, and its accuracy vs. the truth
%
%   dx = x (-) xhat
%   d(dx)/dt = A dx + B du
%
% In the project: the EKF does not track the full state's uncertainty with
% the nonlinear model; it tracks the small ERROR dx with this linear model.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model1_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'P', 'x_hover', 'u_hover', 'A_hover', 'B_hover'}, 'step2_taylor_jacobian.m');
P = S.P; A = S.A_hover; B = S.B_hover;

step_header(1, 3, 'Local error dynamics (linearized)', { ...
    'dx = x (-) xhat = [dp; dv; dtheta; dw]', 'd(dx)/dt = A dx + B du'});

dt = 0.01; t = 0:dt:2; N = numel(t);

%% 1. Discrete-time transition used by the EKF
F_first = eye(12) + A*dt;
F_exact = expm(A*dt);
io_print('IN',  'A',  A, 'mixed', 'continuous error-state matrix (Step 2)');
io_print('IN',  'dt', dt, 's', 'EKF / IMU sample time (100 Hz)');
io_print('OUT', 'F',  F_first, '-', 'F = I + A dt (first order)');
io_print('CHK', 'max|F-expm|', max(abs(F_first(:) - F_exact(:))), '-', ...
    'difference to exact expm(A dt): second order, (A dt)^2/2');

%% 2. Toy example : small initial error  (2 deg roll, 0.2 rad/s yaw rate, 10 cm offset)
dx0_small = [0.1; 0; 0;  0; 0; 0;  deg2rad(2); 0; 0;  0; 0; 0.2];
dx0_large = [0.1; 0; 0;  0; 0; 0;  deg2rad(30); 0; 0;  0; 0; 0.2];
du = [0.5; 0; 0; 0];                       % +0.5 N extra thrust

[dxLin_s, dxTrue_s] = compare(dx0_small, du, S, A, B, P, t);
[dxLin_l, dxTrue_l] = compare(dx0_large, du, S, A, B, P, t);

fprintf('\nToy example: propagate an initial error for 2 s (extra thrust du = +0.5 N)\n');
io_print('IN',  'dx(0) small', dx0_small, 'mixed', '2 deg roll error');
io_print('IN',  'du',          du,        'N, N m', 'input deviation');
io_print('OUT', 'dx(2s) lin',  dxLin_s(:,end),  'mixed', 'linear prediction');
io_print('OUT', 'dx(2s) true', dxTrue_s(:,end), 'mixed', 'true nonlinear error');
eS = norm(dxLin_s(:,end) - dxTrue_s(:,end)) / norm(dxTrue_s(:,end));
eL = norm(dxLin_l(:,end) - dxTrue_l(:,end)) / norm(dxTrue_l(:,end));
io_print('CHK', 'rel.err 2deg',  100*eS, '%', 'linear model is excellent');
io_print('CHK', 'rel.err 30deg', 100*eL, '%', 'linear model degrades for big errors');

% Hand-checkable number: vertical velocity error after 2 s from du = 0.5 N
fprintf('\n  Hand check: dv_z(2 s) = du/m * t = 0.5/1.5*2 = %.4f m/s (linear: %.4f)\n', ...
    0.5/P.m*2, dxLin_s(6,end));

fig = figure('Name', 'Step 3 - error dynamics', 'Position', [100 100 950 360]);
subplot(1,2,1);
plot(t, dxTrue_s(5,:), 'LineWidth', 1.6); hold on; plot(t, dxLin_s(5,:), '--', 'LineWidth', 1.6);
grid on; xlabel('t [s]'); ylabel('\delta v_y [m/s]'); title('2 deg initial roll error');
legend('true nonlinear', 'linear A\deltax + B\deltau', 'Location', 'southwest');
subplot(1,2,2);
plot(t, dxTrue_l(5,:), 'LineWidth', 1.6); hold on; plot(t, dxLin_l(5,:), '--', 'LineWidth', 1.6);
grid on; xlabel('t [s]'); ylabel('\delta v_y [m/s]'); title('30 deg initial roll error');
legend('true nonlinear', 'linear A\deltax + B\deltau', 'Location', 'southwest');
exportgraphics(fig, fullfile(figDir, 'step3_error_dynamics.png'), 'Resolution', 150);

%% 3. Save outputs
F_hover = F_first; dt_ekf = dt;
save(dataFile, 'F_hover', 'dt_ekf', '-append');
fprintf('\nSaved F_hover, dt_ekf -> %s\n', dataFile);

%% Local function
function [dxLin, dxTrue] = compare(dx0, du, S, A, B, P, t)
% Nominal: hover with u_hover. Perturbed: hover (+) dx0 with u_hover + du.
dt = t(2) - t(1); N = numel(t);
xn = S.x_hover;  xp = state_boxplus(S.x_hover, dx0);
Ad = expm([A, B; zeros(4, 16)]*dt);           % exact ZOH discretisation
F = Ad(1:12, 1:12); G = Ad(1:12, 13:16);
dxLin = zeros(12, N); dxTrue = zeros(12, N);
dxLin(:,1) = dx0; dxTrue(:,1) = dx0;
for k = 1:N-1
    xn = rk4_step(xn, S.u_hover, dt, P);
    xp = rk4_step(xp, S.u_hover + du, dt, P);
    dxTrue(:,k+1) = state_boxminus(xp, xn);
    dxLin(:,k+1)  = F*dxLin(:,k) + G*du;
end
end
