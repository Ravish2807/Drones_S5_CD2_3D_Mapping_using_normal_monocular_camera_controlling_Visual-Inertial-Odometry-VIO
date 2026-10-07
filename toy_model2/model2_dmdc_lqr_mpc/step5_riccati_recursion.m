%% MODEL 2 - STEP 5 : RICCATI RECURSION
% Block 5 of the Model-2 equation flow (docs: Section 4.5).
%
%   INPUT  : A, B (Step 2), Q, R (Step 3), starting matrix P_0
%   OUTPUT : the sequence P_0, P_1, ... and its steady state P_inf
%
%   P_{k+1} = A'P_k A - A'P_k B (R + B'P_k B)^-1 B'P_k A + Q
%
% In the project: each iteration "looks one more step into the future".
% After enough iterations the drone plans as if the flight never ends.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'A_dmd', 'B_dmd', 'Q', 'R'}, 'step3_bellman_cost.m');
A = S.A_dmd; B = S.B_dmd; Q = S.Q; R = S.R;

step_header(2, 5, 'Riccati recursion', { ...
    'P_{k+1} = A''P_k A - A''P_k B (R + B''P_k B)^-1 B''P_k A + Q'});

%% Part A - toy example from the documentation (scalar)
fprintf('\nPart A : scalar recursion, a = 1.1, b = 0.5, q = r = 1, P_0 = 1\n');
[Pinf_s, hs] = riccati_recursion(1.1, 0.5, 1, 1, 1, 200, 1e-14);
Pk = 1;
fprintf('   k  |  P_k\n   0  |  %.4f\n', Pk);
for k = 1:8
    Pk = bellman_backup(1.1, 0.5, 1, 1, Pk);
    fprintf('   %d  |  %.4f\n', k, Pk);
end
io_print('OUT', 'P_inf', Pinf_s, '-', sprintf('converged after %d iterations', hs.iterations));

%% Part B - drone
fprintf('\nPart B : drone, starting from P_0 = Q\n');
[P_rec, hist] = riccati_recursion(A, B, Q, R, Q, 20000, 1e-12);
io_print('IN',  'P_0', Q, '-', 'start: terminal weight = Q');
io_print('OUT', 'P_inf', P_rec, '-', 'steady-state value matrix (12x12)');
io_print('CHK', 'iterations', hist.iterations, '-', 'to reach relative change < 1e-12');
io_print('CHK', 'min eig P', min(eig(P_rec)), '-', 'positive -> V(x) = x''Px > 0 for every x ~= 0');

fig = figure('Name', 'Step 5 - Riccati convergence', 'Position', [100 100 950 360]);
subplot(1,2,1); semilogy(hist.change, 'LineWidth', 1.5); grid on;
xlabel('iteration k'); ylabel('||P_k - P_{k-1}|| / ||P_k||'); title('Riccati recursion converges');
subplot(1,2,2); plot(hist.trace, 'LineWidth', 1.5); grid on;
xlabel('iteration k'); ylabel('trace(P_k)'); title('Value grows to its infinite-horizon limit');
exportgraphics(fig, fullfile(figDir, 'step5_riccati_convergence.png'), 'Resolution', 150);

%% Save
riccati_iterations = hist.iterations;
save(dataFile, 'P_rec', 'riccati_iterations', '-append');
fprintf('\nSaved P_rec, riccati_iterations -> %s\n', dataFile);
