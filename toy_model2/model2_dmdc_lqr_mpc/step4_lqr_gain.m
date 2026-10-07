%% MODEL 2 - STEP 4 : LINEAR QUADRATIC REGULATOR (LQR)
% Block 4 of the Model-2 equation flow (docs: Section 4.4).
%
%   INPUT  : model (A, B), input weight R, value matrix P
%   OUTPUT : optimal gain K and the control law u_k = -K x_k
%
%   K = (R + B'PB)^-1 B'PA         u_k = -K x_k
%
% In the project: K is a 4 x 12 table of numbers. Every 40 ms it turns the
% 12 measured errors into 4 commands (thrust + 3 torques). Nothing else.
% The optimal P comes from Step 5 (iteration) or Step 6 (direct solution);
% here we show the formula with the one-step P from Step 3.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
S = require_vars(dataFile, {'A_dmd', 'B_dmd', 'Q', 'R'}, 'step3_bellman_cost.m');
A = S.A_dmd; B = S.B_dmd; Q = S.Q; R = S.R;

step_header(2, 4, 'LQR gain and control law', { ...
    'K = (R + B''PB)^-1 B''PA', 'u_k = -K x_k'});

%% Part A - toy example from the documentation (scalar, with the converged P)
fprintf('\nPart A : scalar toy example (P = 3.1215 from the DARE, Step 6)\n');
a = 1.1; b = 0.5; r = 1; Ps = 3.12146;
Ks = lqr_gain(a, b, r, Ps);
io_print('IN',  'a, b, r, P', [a b r Ps], '-', 'scalar model, weight and value');
io_print('OUT', 'K',          Ks,         '-', 'gain = abP/(r + b^2 P)');
io_print('OUT', 'a - bK',     a - b*Ks,   '-', 'closed-loop pole (|.| < 1 -> stable)');
fprintf('          open loop  x_k = 1.1^k x_0   grows 10%% per step\n');
fprintf('          closed loop x_k = %.4f^k x_0 shrinks %.0f%% per step\n', a - b*Ks, 100*(1 - (a - b*Ks)));

%% Part B - drone: gain from the one-step value matrix P1 (Step 3 backup)
P1 = bellman_backup(A, B, Q, R, Q);
K1 = lqr_gain(A, B, R, P1);
fprintf('\nPart B : drone gain from a 2-step-horizon value function (not yet optimal)\n');
io_print('IN',  'P', P1, '-', 'value matrix after one Bellman backup');
io_print('OUT', 'K', K1, 'mixed', 'gain (4x12)');
io_print('CHK', 'max|eig(A-BK)|', max(abs(eig(A - B*K1))), '-', ...
    'spectral radius; >= 1 means this short-sighted gain cannot stabilise the drone');

%% Save
K_onestep = K1;
save(dataFile, 'K_onestep', '-append');
fprintf('\nSaved K_onestep -> %s\n', dataFile);
