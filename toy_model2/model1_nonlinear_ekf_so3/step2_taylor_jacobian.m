%% MODEL 1 - STEP 2 : LINEARIZATION (TAYLOR EXPANSION AND JACOBIANS)
% Block 2 of the Model-1 equation flow (docs: Section 3.2).
%
%   INPUT  : operating point (xhat, uhat), nonlinear model f(x,u) from Step 1
%   OUTPUT : A = df/dx [12x12],  B = df/du [12x4]   evaluated at (xhat, uhat)
%
%   f(x,u) ~ f(xhat,uhat) + A (x - xhat) + B (u - uhat)
%
% The attitude R has 9 numbers but only 3 degrees of freedom, so "x - xhat"
% is taken in error coordinates  dx = [dp; dv; dtheta; dw]  with
% R = Rhat * Exp(dtheta).  That is why A is 12x12, not 18x18.
%
% In the project: the EKF (Step 4) needs A at every time step to predict
% how its uncertainty grows.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model1_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'P', 'x_hover', 'u_hover'}, 'step1_nonlinear_dynamics.m');
P = S.P;

step_header(1, 2, 'Taylor expansion and Jacobians', { ...
    'f(x,u) ~ f(xh,uh) + A (x - xh) + B (u - uh)', ...
    'A = df/dx |(xh,uh)      B = df/du |(xh,uh)'});

%% 1. Jacobians at hover (the operating point of controller.yaml)
[A, B] = quad_jacobians(S.x_hover, S.u_hover, P);
fprintf('\nOperating point: hover at p = [0 0 2] m, R = I, w = 0, T = m g0\n');
io_print('IN',  'xhat',  S.x_hover, 'mixed', 'hover state [p; v; vec(R); w]');
io_print('IN',  'uhat',  S.u_hover, 'N, N m', 'hover input');
io_print('OUT', 'A',     A, 'mixed', 'state Jacobian (12x12)');
io_print('OUT', 'B',     B, 'mixed', 'input Jacobian (12x4)');
fprintf('\n  Key block of A at hover, d(v_dot)/d(theta) = -g0 [e3]x :\n');
disp(A(4:6, 7:9));
fprintf('  Key block of B at hover, d(w_dot)/d(tau) = J^-1 :\n');
disp(B(10:12, 2:4));

%% 2. Verify the analytic Jacobians numerically (at a general, non-hover point)
% The check perturbs the state on the manifold and differentiates the true flow.
xg = pack_state([1; -2; 3], [0.5; 0.2; -0.1], so3_exp([0.2; -0.1; 0.3]), [0.3; -0.2; 0.4]);
ug = [16; 0.01; -0.02; 0.005];
[Ag, Bg] = quad_jacobians(xg, ug, P);
[An, Bn] = numerical_jacobians(xg, ug, P);
fprintf('\nNumerical check at a tilted, rotating, accelerating state:\n');
io_print('CHK', 'max|A-An|', max(abs(Ag(:) - An(:))), '-', 'analytic vs numerical A');
io_print('CHK', 'max|B-Bn|', max(abs(Bg(:) - Bn(:))), '-', 'analytic vs numerical B');

%% 3. Toy example : how good is the first-order Taylor expansion?
% Pitch the drone by theta about body y with hover thrust.
% Linear prediction:   dv_dot_x = g0 * theta
% True nonlinear:       v_dot_x  = g0 * sin(theta)
fprintf('\nToy example: horizontal acceleration caused by a pitch angle\n');
fprintf('   theta [deg] | linear g0*theta | true g0*sin(theta) | error %%\n');
angles = [1 2.865 5 10 20 28.65];
for th = deg2rad(angles)
    x  = pack_state(P.p_hover, zeros(3,1), so3_exp([0; th; 0]), zeros(3,1));
    fx = quad_dynamics(x, S.u_hover, P);
    lin = A(4,:)*[zeros(6,1); 0; th; 0; zeros(3,1)];
    fprintf('   %11.2f | %15.4f | %18.4f | %7.3f\n', rad2deg(th), lin, fx(4), ...
        100*abs(lin - fx(4))/abs(fx(4)));
end

th = linspace(0, deg2rad(45), 100);
fig = figure('Name', 'Step 2 - Taylor accuracy', 'Position', [100 100 600 380]);
plot(rad2deg(th), P.g0*th, 'LineWidth', 1.6); hold on;
plot(rad2deg(th), P.g0*sin(th), 'LineWidth', 1.6); grid on;
xlabel('pitch angle \theta [deg]'); ylabel('horizontal acceleration [m/s^2]');
legend('linear  g_0 \theta  (Taylor, 1st order)', 'nonlinear  g_0 sin\theta', 'Location', 'northwest');
title('Linearization is accurate for small tilts');
exportgraphics(fig, fullfile(figDir, 'step2_taylor_accuracy.png'), 'Resolution', 150);

%% 4. Save outputs
A_hover = A; B_hover = B;
save(dataFile, 'A_hover', 'B_hover', '-append');
fprintf('\nSaved A_hover, B_hover -> %s\n', dataFile);

%% Local function: numerical Jacobians on the manifold
function [An, Bn] = numerical_jacobians(x, u, P)
% Central differences in both perturbation size and time:
%   column i of A ~ [dx(+h,+e) - dx(-h,+e) - dx(+h,-e) + dx(-h,-e)] / (4 h eps)
h = 1e-4; ep = 1e-4;
An = zeros(12); Bn = zeros(12, 4);
for i = 1:12
    e = zeros(12,1); e(i) = ep;
    d = @(sgnE, sgnH) state_boxminus(rk4_step(state_boxplus(x, sgnE*e), u, sgnH*h, P), ...
                                     rk4_step(x, u, sgnH*h, P));
    An(:,i) = (d(1,1) - d(1,-1) - d(-1,1) + d(-1,-1)) / (4*h*ep);
end
for i = 1:4
    e = zeros(4,1); e(i) = ep;
    d = @(sgnH) state_boxminus(rk4_step(x, u + e, sgnH*h, P), rk4_step(x, u - e, sgnH*h, P));
    Bn(:,i) = (d(1) - d(-1)) / (4*h*ep);
end
end
