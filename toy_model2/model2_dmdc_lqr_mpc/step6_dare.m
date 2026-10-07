%% MODEL 2 - STEP 6 : DARE (DISCRETE ALGEBRAIC RICCATI EQUATION)
% Block 6 of the Model-2 equation flow (docs: Section 4.6).
%
%   INPUT  : A, B (Step 2), Q, R (Step 3)
%   OUTPUT : steady-state P solved DIRECTLY, the final LQR gain K, and checks
%
%   P = A'PA - A'PB (R + B'PB)^-1 B'PA + Q
%
% In the project: this is the gain we would actually upload to the drone
% (and P is reused as the terminal cost of the MPC in Step 7).

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
S = require_vars(dataFile, {'A_dmd', 'B_dmd', 'Q', 'R', 'P_rec'}, 'step5_riccati_recursion.m');
A = S.A_dmd; B = S.B_dmd; Q = S.Q; R = S.R;

step_header(2, 6, 'DARE - steady-state Riccati solution', { ...
    'P = A''PA - A''PB (R + B''PB)^-1 B''PA + Q', 'K = (R + B''PB)^-1 B''PA'});

%% Part A - toy example from the documentation (scalar: a quadratic equation)
% 0.25 P^2 - 0.46 P - 1 = 0  ->  P = (0.46 + sqrt(0.46^2 + 1)) / 0.5
fprintf('\nPart A : scalar DARE, a = 1.1, b = 0.5, q = r = 1\n');
Ps_formula = (0.46 + sqrt(0.46^2 + 1))/0.5;
Ps_solver  = dare_solve(1.1, 0.5, 1, 1);
io_print('OUT', 'P formula', Ps_formula, '-', 'root of 0.25P^2 - 0.46P - 1 = 0');
io_print('OUT', 'P solver',  Ps_solver,  '-', 'dare_solve (eigenvector method)');
io_print('OUT', 'K',         lqr_gain(1.1, 0.5, 1, Ps_solver), '-', 'LQR gain');

%% Part B - drone
fprintf('\nPart B : drone\n');
[P, lam] = dare_solve(A, B, Q, R);
K = lqr_gain(A, B, R, P);
res = A'*P*A - A'*P*B*((R + B'*P*B)\(B'*P*A)) + Q - P;
io_print('IN',  'A, B', [size(A) size(B)], '-', 'sizes of the learned model');
io_print('OUT', 'P', P, '-', 'DARE solution (12x12)');
io_print('OUT', 'K', K, 'mixed', 'final LQR gain (4x12)');
io_print('CHK', 'DARE residual', norm(res, 'fro')/norm(P, 'fro'), '-', 'relative, should be ~1e-12');
io_print('CHK', '||P-P_rec||/||P||', norm(P - S.P_rec, 'fro')/norm(P, 'fro'), '-', ...
    'agrees with the Riccati recursion of Step 5');
io_print('CHK', 'max|eig(A-BK)|', max(abs(eig(A - B*K))), '-', 'spectral radius < 1 -> stable');

% Optimal cost check: simulate the linear closed loop and add up the cost
x0 = [0.5; -0.3; 0.2; zeros(9,1)];
xk = x0; J = 0;
for k = 1:5000
    uk = -K*xk; J = J + xk'*Q*xk + uk'*R*uk; xk = A*xk + B*uk;
end
io_print('CHK', 'J simulated', J, '-', 'sum of x''Qx + u''Ru along the closed loop');
io_print('CHK', 'x0''P x0', x0'*P*x0, '-', 'predicted optimal cost - identical');

% What does K mean? Read one row.
fprintf('\n  Reading K (u = -K x), largest entries of two rows:\n');
fprintf('  thrust      dT    = %+.2f*dz %+.2f*v_z            (altitude hold)\n', -K(1,3), -K(1,6));
fprintf('  roll torque tau_x = %+.3f*dy %+.3f*v_y %+.3f*roll %+.3f*w_x  (lean to move sideways)\n', ...
    -K(2,2), -K(2,5), -K(2,7), -K(2,10));

%% Save
P_dare = P; K_lqr = K;
save(dataFile, 'P_dare', 'K_lqr', '-append');
fprintf('\nSaved P_dare, K_lqr -> %s\n', dataFile);
