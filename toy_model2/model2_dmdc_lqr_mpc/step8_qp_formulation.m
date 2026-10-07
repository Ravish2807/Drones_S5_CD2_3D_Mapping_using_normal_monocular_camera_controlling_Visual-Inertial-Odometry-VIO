%% MODEL 2 - STEP 8 : OPTIMIZATION FORMULATION (QUADRATIC PROGRAM)
% Block 8 of the Model-2 equation flow (docs: Section 4.8).
%
%   INPUT  : MPC problem of Step 7 and the current state x0
%   OUTPUT : stacked QP data (H, f, G, h, A_eq, b_eq) and its solution U*
%
%   X = Sx x0 + Su U
%   min_U 1/2 U'HU + f'U   s.t.  G U <= h,   A_eq U = b_eq
%   H = 2 (Su'Qbar Su + Rbar),   f = 2 Su'Qbar Sx x0
%
% In the project: this is the maths a solver chip (or quadprog / OSQP) sees
% 25 times per second. The physics has disappeared into H, f, G and h.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
S = require_vars(dataFile, {'mpc', 'K_lqr'}, 'step7_mpc_problem.m');
mpc = S.mpc;

step_header(2, 8, 'Quadratic program (stacked form)', { ...
    'X = Sx x0 + Su U', 'min 1/2 U''HU + f''U  s.t.  G U <= h,  A_eq U = b_eq', ...
    'H = 2(Su''Qbar Su + Rbar),   f = 2 Su''Qbar Sx x0'});

%% Part A - toy example from the documentation (scalar, N = 2)
fprintf('\nPart A : scalar QP, a = 1.1, b = 0.5, q = r = 1, P_f = 3.12146, x0 = 1\n');
a = 1.1; b = 0.5; Pf = 3.12146; x0 = 1;
toy = mpc_build(a, b, 1, 1, Pf, 2, -0.5, 0.5, [], []);
f = toy.F*x0;
io_print('OUT', 'Sx', toy.Sx, '-', '[a; a^2]');
io_print('OUT', 'Su', toy.Su, '-', '[b 0; ab b]');
io_print('OUT', 'H',  toy.H,  '-', '2(Su''Qbar Su + I)');
io_print('OUT', 'f',  f,      '-', '2 Su''Qbar Sx x0');
Uunc = -toy.H\f;
io_print('OUT', 'U* (no limits)', Uunc, '-', 'first entry = -K x0 = -0.9643 (MPC = LQR)');
Ucon = qp_solve(toy.H, f, toy.G, toy.h0 + toy.E*x0);
io_print('OUT', 'U* (|u|<=0.5)', Ucon, '-', 'both inputs saturate at -0.5');
Aeq = toy.Su(end,:); beq = -toy.Sx(end,:)*x0;          % terminal constraint x_2 = 0
Ueq = qp_solve(toy.H, f, [], [], Aeq, beq);
io_print('OUT', 'U* (x2 = 0)', Ueq, '-', 'with equality A_eq U = b_eq forcing x_2 = 0');
io_print('CHK', 'x2', toy.Sx(end,:)*x0 + toy.Su(end,:)*Ueq, '-', 'terminal state really is 0');

%% Part B - drone: small offset (limits inactive) -> MPC reproduces LQR exactly
fprintf('\nPart B : drone, small offset (5 cm) - constraints inactive\n');
x0s = [0.05; -0.03; 0.02; zeros(9,1)];
[u0s, ~, infoS] = mpc_solve(mpc, x0s);
io_print('IN',  'x0', x0s, 'mixed', 'current deviation state');
io_print('OUT', 'u0* MPC', u0s, 'N, N m', 'first optimal input');
io_print('CHK', '-K x0 LQR', -S.K_lqr*x0s, 'N, N m', 'identical: P_f = P makes MPC = LQR when no limit is active');
io_print('CHK', 'IPM iters', infoS.iterations, '-', 'interior-point iterations');

%% Part C - drone: large offset (limits active)
fprintf('\nPart C : drone, large offset (2 m) - constraints active\n');
x0l = [1.5; -1.0; -1.2; zeros(9,1)];
f = mpc.F*x0l; h = mpc.h0 + mpc.E*x0l;
tic; [U, info] = qp_solve(mpc.H, f, mpc.G, h); tQP = toc;
Xpred = mpc.Sx*x0l + mpc.Su*U;
vPred = reshape(Xpred, 12, []); vPred = vPred(4:6, :);
io_print('IN',  'x0', x0l, 'mixed', 'drone 1.5 m, 1.0 m, 1.2 m away from the setpoint');
io_print('OUT', 'H', mpc.H, '-', sprintf('Hessian %dx%d', size(mpc.H)));
io_print('OUT', 'f', f, '-', 'linear term (depends on x0)');
io_print('OUT', 'G', mpc.G, '-', sprintf('constraint matrix %dx%d', size(mpc.G)));
io_print('OUT', 'u0*', U(1:4), 'N, N m', 'first input of the optimal plan');
io_print('CHK', 'max|v| plan', max(abs(vPred(:))), 'm/s', 'predicted speed never exceeds 1 m/s');
io_print('CHK', 'LQR demand', max(abs(S.K_lqr*x0l)./abs(mpc.u_max)), '-', ...
    'unconstrained LQR would ask for this multiple of the allowed input');
io_print('CHK', 'converged', info.converged, '-', sprintf('%d IPM iterations, %.1f ms', info.iterations, 1e3*tQP));

%% Save
qp_example = struct('x0', x0l, 'U', U, 'H', mpc.H, 'f', f);
save(dataFile, 'qp_example', '-append');
fprintf('\nSaved qp_example -> %s\n', dataFile);
