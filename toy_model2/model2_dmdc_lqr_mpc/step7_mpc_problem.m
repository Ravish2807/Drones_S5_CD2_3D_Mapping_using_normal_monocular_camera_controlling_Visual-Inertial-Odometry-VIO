%% MODEL 2 - STEP 7 : MODEL PREDICTIVE CONTROL (MPC) - PROBLEM DEFINITION
% Block 7 of the Model-2 equation flow (docs: Section 4.7).
%
%   INPUT  : model (A, B) from Step 2, weights Q, R (Step 3), terminal weight
%            P_f = P from the DARE (Step 6), horizon N, input and state limits
%   OUTPUT : a fully specified finite-horizon optimal control problem
%
%   min_{u_0..u_{N-1}}  sum_{k=0}^{N-1} (x_k'Q x_k + u_k'R u_k) + x_N'P_f x_N
%   subject to          x_{k+1} = A x_k + B u_k,   u_min <= u_k <= u_max,  |C x_k| <= x_max
%
% In the project: LQR cannot respect limits. MPC can: "never fly faster than
% max_velocity = 1 m/s" (controller.yaml) and "never ask the motors for more
% than they can give" are written directly into the problem.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
S = require_vars(dataFile, {'P', 'A_dmd', 'B_dmd', 'Q', 'R', 'P_dare'}, 'step6_dare.m');
P = S.P;

step_header(2, 7, 'MPC - finite-horizon problem', { ...
    'min  sum_{k=0}^{N-1} (x_k''Q x_k + u_k''R u_k) + x_N''P_f x_N', ...
    's.t. x_{k+1} = A x_k + B u_k,  u_min <= u_k <= u_max,  |C x_k| <= x_max'});

%% Part A - toy example from the documentation (scalar, N = 2)
fprintf('\nPart A : scalar toy problem\n');
fprintf('   min_{u0,u1}  x0^2 + u0^2 + x1^2 + u1^2 + 3.1215 x2^2\n');
fprintf('   s.t. x_{k+1} = 1.1 x_k + 0.5 u_k,  x0 = 1,  |u_k| <= 0.5\n');

%% Part B - the drone
N = 25;                                            % 25 x 40 ms = 1 s look-ahead
T_lo = 5; T_hi = 25;                               % total thrust window [N]
u_min = [T_lo - P.m*P.g0; -0.3; -0.3; -0.05];
u_max = [T_hi - P.m*P.g0;  0.3;  0.3;  0.05];
C = zeros(5, 12); C(1,4) = 1; C(2,5) = 1; C(3,6) = 1; C(4,7) = 1; C(5,8) = 1;
x_max = [1.0; 1.0; 1.0; 0.35; 0.35];               % |v| <= 1 m/s, |roll|,|pitch| <= 20 deg

mpc = mpc_build(S.A_dmd, S.B_dmd, S.Q, S.R, S.P_dare, N, u_min, u_max, C, x_max);
fprintf('\nPart B : drone MPC problem\n');
io_print('IN',  'N',     N,      'steps',  'prediction horizon (1 s)');
io_print('IN',  'P_f',   S.P_dare, '-',    'terminal weight = DARE solution (Step 6)');
io_print('IN',  'u_min', u_min,  'N, N m', 'lower input limits (deviation from hover)');
io_print('IN',  'u_max', u_max,  'N, N m', 'upper input limits');
io_print('IN',  'x_max', x_max,  'm/s, rad', 'limits on |v_x|,|v_y|,|v_z|,|roll|,|pitch|');
io_print('OUT', '#vars', numel(mpc.F*zeros(12,1)), '-', 'decision variables U = [u_0; ...; u_{N-1}]');
io_print('OUT', '#ineq', size(mpc.G, 1), '-', 'inequality constraints');

%% Save
save(dataFile, 'mpc', '-append');
fprintf('\nSaved mpc -> %s\n', dataFile);
