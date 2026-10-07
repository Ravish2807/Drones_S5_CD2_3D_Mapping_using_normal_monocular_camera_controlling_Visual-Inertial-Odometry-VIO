%% MODEL 2 - STEP 3 : BELLMAN EQUATION (OPTIMAL CONTROL)
% Block 3 of the Model-2 equation flow (docs: Section 4.3).
%
%   INPUT  : learned model (A, B) from Step 2, weights Q (state) and R (input)
%   OUTPUT : the cost J, the Bellman backup V(x) = min_u[...], and the proof
%            that its minimiser is LINEAR in x (which is what makes LQR possible)
%
%   J = sum_{k=0}^{inf} (x_k'Q x_k + u_k'R u_k)
%   V(x_k) = min_u [ x_k'Q x_k + u'R u + V(x_{k+1}) ],   V(x) = x'P x
%
% In the project: Q and R are where WE say what "good flying" means:
% how much a 50 cm position error hurts compared with spending 5 N of thrust.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
S = require_vars(dataFile, {'A_dmd', 'B_dmd'}, 'step2_dmdc.m');
A = S.A_dmd; B = S.B_dmd;

step_header(2, 3, 'Bellman equation and quadratic cost', { ...
    'J = sum_k (x_k''Q x_k + u_k''R u_k)', ...
    'V(x) = min_u [x''Qx + u''Ru + V(Ax + Bu)],   V(x) = x''Px'});

%% Part A - toy example from the documentation (scalar)
% x_{k+1} = 1.1 x_k + 0.5 u_k,  q = r = 1,  start from V_next(x) = 1*x^2, at x = 1
fprintf('\nPart A : scalar Bellman backup at x = 1 with V_next(x) = x^2\n');
a = 1.1; b = 0.5; q = 1; r = 1; Pn = 1; x = 1;
rhs = @(u) q*x^2 + r*u.^2 + Pn*(a*x + b*u).^2;
uBrute = fminsearch(rhs, 0, optimset('TolX', 1e-12, 'TolFun', 1e-12));
[Pnew, Ks] = bellman_backup(a, b, q, r, Pn);
io_print('IN',  'a, b',    [a b], '-', 'scalar model');
io_print('IN',  'q, r',    [q r], '-', 'weights');
io_print('OUT', 'u* brute', uBrute, '-', 'numerical minimum of the right-hand side');
io_print('OUT', 'u* = -Kx', -Ks*x, '-', 'closed form, K = abP/(r + b^2 P) = 0.44');
io_print('OUT', 'V(1)',     rhs(uBrute), '-', 'minimum value');
io_print('OUT', 'P_new',    Pnew, '-', 'closed form: q + a^2 P - (abP)^2/(r + b^2 P) = 1.968');

%% Part B - choose Q and R for the drone (Bryson's rule: 1 / (max acceptable value)^2)
x_max = [0.5*ones(3,1);      % position error   [m]
         1.0*ones(3,1);      % velocity         [m/s]  (max_velocity in controller.yaml)
         0.2; 0.2; 0.5;      % roll, pitch, yaw [rad]
         1.0*ones(3,1)];     % body rates       [rad/s]
u_max = [5; 0.2; 0.2; 0.05]; % dT [N], tau [N m]
Q = diag(1./x_max.^2);
R = diag(1./u_max.^2);
fprintf('\nPart B : drone weights (Bryson''s rule)\n');
io_print('IN',  'x_max', x_max, 'mixed', 'largest acceptable deviation per state');
io_print('IN',  'u_max', u_max, 'N, N m', 'largest acceptable input deviation');
io_print('OUT', 'diag Q', diag(Q), 'mixed', 'state weight');
io_print('OUT', 'diag R', diag(R), 'mixed', 'input weight');

%% Part C - Bellman backup on the drone: closed form vs brute force
rng(3);
x0 = [0.5; -0.3; 0.2; zeros(9,1)];
rhs = @(u) x0'*Q*x0 + u'*R*u + (A*x0 + B*u)'*Q*(A*x0 + B*u);   % V_next = x'Qx
opts = optimset('TolX', 1e-12, 'TolFun', 1e-14, 'MaxFunEvals', 2e5, 'MaxIter', 2e5);
uBrute = fminsearch(rhs, zeros(4,1), opts);
[P1, K1] = bellman_backup(A, B, Q, R, Q);
fprintf('\nPart C : one Bellman backup on the drone (V_next(x) = x''Qx)\n');
io_print('IN',  'x', x0, 'mixed', 'drone 50 cm off in x, 30 cm in y, 20 cm in z');
io_print('OUT', 'u* brute',  uBrute, 'N, N m', 'fminsearch on the right-hand side');
io_print('OUT', 'u* = -K1 x', -K1*x0, 'N, N m', 'closed form');
io_print('CHK', 'V brute vs x''P1x', [rhs(uBrute), x0'*P1*x0], '-', 'the minimum value is quadratic in x');

%% Part D - why feedback is needed: cost of doing nothing
xk = x0; Jzero = 0;
for k = 1:250                         % 10 s
    Jzero = Jzero + xk'*Q*xk;
    xk = A*xk;                        % u = 0
end
io_print('CHK', 'J(u = 0)', Jzero, '-', 'cost after 10 s with u = 0 (keeps growing: drone drifts)');

%% Save
save(dataFile, 'Q', 'R', 'x_max', 'u_max', '-append');
fprintf('\nSaved Q, R, x_max, u_max -> %s\n', dataFile);
