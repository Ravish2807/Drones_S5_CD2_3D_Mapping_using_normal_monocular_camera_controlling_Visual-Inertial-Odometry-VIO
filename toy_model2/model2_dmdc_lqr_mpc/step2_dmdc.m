%% MODEL 2 - STEP 2 : DMDc (DYNAMIC MODE DECOMPOSITION WITH CONTROL)
% Block 2 of the Model-2 equation flow (docs: Section 4.2).
%
%   INPUT  : snapshot matrices built from the flight data of Step 1
%            X = [x_1 ... x_{N-1}],  X+ = [x_2 ... x_N],  U = [u_1 ... u_{N-1}]
%   OUTPUT : discrete-time linear model  A in R^{12x12},  B in R^{12x4}
%
%   [A B] = X+ [X; U]^+          x_{k+1} ~ A x_k + B u_k
%
% In the project: instead of deriving A and B by hand (Model 1, Step 2) we
% LEARN them from logged flight data. The same code works on a real drone
% whose mass, inertia or motor constants we do not know exactly.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'P', 'runs', 'dt'}, 'step1_flight_data.m');
P = S.P; dt = S.dt;

step_header(2, 2, 'DMDc - learn A and B from data', { ...
    'X = [x_1 ... x_{N-1}]   X+ = [x_2 ... x_N]   U = [u_1 ... u_{N-1}]', ...
    '[A B] = X+ * pinv([X; U])        x_{k+1} ~ A x_k + B u_k'});

%% Part A - toy example from the documentation (scalar, exact)
% True system x_{k+1} = 0.9 x_k + 0.5 u_k, three snapshots, no noise.
fprintf('\nPart A : scalar toy example\n');
Xt = [1, 1.4, 0.76]; Xpt = [1.4, 0.76, 0.934]; Ut = [1, -1, 0.5];
[at, bt] = dmdc(Xt, Xpt, Ut);
io_print('IN',  'X',  Xt,  '-', 'states x_1..x_3');
io_print('IN',  'X+', Xpt, '-', 'states x_2..x_4');
io_print('IN',  'U',  Ut,  '-', 'inputs u_1..u_3');
io_print('OUT', '[a b]', [at bt], '-', 'recovered model (true: 0.9, 0.5)');

%% Part B - the drone: build snapshot matrices from flights 1-4
X = []; Xp = []; U = [];
for r = 1:numel(S.runs) - 1                     % last flight kept for validation
    X  = [X,  S.runs{r}.X(:, 1:end-1)];         %#ok<AGROW>
    Xp = [Xp, S.runs{r}.X(:, 2:end)];           %#ok<AGROW>
    U  = [U,  S.runs{r}.U];                     %#ok<AGROW>
end
fprintf('\nPart B : DMDc on the drone flight data\n');
io_print('IN', 'X',  X,  'mixed', sprintf('state snapshots (12 x %d)', size(X, 2)));
io_print('IN', 'X+', Xp, 'mixed', 'shifted snapshots');
io_print('IN', 'U',  U,  'N, N m', 'input snapshots (4 x M)');

[A, B, info] = dmdc(X, Xp, U);
io_print('OUT', 'A', A, '-', 'learned state matrix (12x12)');
io_print('OUT', 'B', B, '-', 'learned input matrix (12x4)');
io_print('CHK', 'rank[X;U]', info.rank, '-', 'must be 16 = 12 + 4 (enough excitation)');
io_print('CHK', 'sv ratio', info.sv(1)/info.sv(end), '-', 'condition number of [X;U]');
io_print('CHK', 'fit', info.fit, '-', 'relative one-step residual on training data');

%% Compare with the physics model (only possible in simulation)
[Ac, Bc] = quad_jacobians(pack_state(P.p_hover, zeros(3,1), eye(3), zeros(3,1)), [P.m*P.g0; 0; 0; 0], P);
[Atrue, Btrue] = c2d_zoh(Ac, Bc, dt);
fprintf('\nComparison with the exact discretised Jacobians (Model 1, Step 2 + ZOH)\n');
fprintf('   entry        meaning                         DMDc      physics\n');
rowsCols = [1 4; 4 8; 5 7; 6 1; 10 2; 12 4];
names = {'A(1,4)  dt              (p_x from v_x)', 'A(4,8)  ~ g0 dt         (v_x from theta_y)', ...
         'A(5,7)  ~ -g0 dt        (v_y from theta_x)', 'B(6,1)  dt/m            (v_z from dT)', ...
         'B(10,2) dt/J_x          (w_x from tau_x)', 'B(12,4) dt/J_z          (w_z from tau_z)'};
vals = {A, A, A, B, B, B}; ref = {Atrue, Atrue, Atrue, Btrue, Btrue, Btrue};
for i = 1:size(rowsCols, 1)
    rc = rowsCols(i,:);
    fprintf('   %-46s %8.4f  %8.4f\n', names{i}, vals{i}(rc(1), rc(2)), ref{i}(rc(1), rc(2)));
end
io_print('CHK', 'relErr A', norm(A - Atrue, 'fro')/norm(Atrue, 'fro'), '-', '||A - A_phys|| / ||A_phys||');
io_print('CHK', 'relErr B', norm(B - Btrue, 'fro')/norm(Btrue, 'fro'), '-', '||B - B_phys|| / ||B_phys||');

%% Validation on the held-out flight: 1-second (25-step) open-loop prediction
V = S.runs{end}; Kv = size(V.U, 2); H = 25;
errD = []; errP = [];
for k0 = 1:H:Kv-H
    xD = V.X(:,k0); xP = xD;
    for j = 0:H-1
        xD = A*xD + B*V.U(:,k0+j);
        xP = Atrue*xP + Btrue*V.U(:,k0+j);
    end
    errD(end+1) = norm(xD(1:3) - V.X(1:3,k0+H)); %#ok<SAGROW>
    errP(end+1) = norm(xP(1:3) - V.X(1:3,k0+H)); %#ok<SAGROW>
end
io_print('CHK', '1s pred err', mean(errD), 'm', 'DMDc model, unseen flight (position after 1 s)');
io_print('CHK', '1s pred err', mean(errP), 'm', 'physics model, same test (for reference)');

fig = figure('Name', 'Step 2 - DMDc', 'Position', [60 60 1050 400]);
subplot(1,2,1);
plot(real(eig(A)), imag(eig(A)), 'o', 'LineWidth', 1.5); hold on;
plot(real(eig(Atrue)), imag(eig(Atrue)), 'x', 'MarkerSize', 10, 'LineWidth', 1.5);
th = linspace(0, 2*pi, 200); plot(cos(th), sin(th), 'k:');
axis equal; grid on; xlim([0.9 1.1]); ylim([-0.1 0.1]);
legend('eig(A) DMDc', 'eig(A) physics', 'unit circle'); title('Eigenvalues: all on/near 1 (integrators)');
subplot(1,2,2);
k = 1:Kv+1; xs = zeros(12, Kv+1); xs(:,1) = V.X(:,1);
for j = 1:Kv, xs(:,j+1) = A*V.X(:,j) + B*V.U(:,j); end      % one-step-ahead predictions
plot(V.t, V.X(4,:), 'k', 'LineWidth', 1.4); hold on; plot(V.t, xs(4,:), 'r--', 'LineWidth', 1.1);
grid on; xlabel('t [s]'); ylabel('v_x [m/s]'); legend('measured (unseen flight)', 'DMDc one-step prediction');
title('Validation on held-out data');
exportgraphics(fig, fullfile(figDir, 'step2_dmdc_validation.png'), 'Resolution', 150);

%% Save
A_dmd = A; B_dmd = B; A_phys = Atrue; B_phys = Btrue;
save(dataFile, 'A_dmd', 'B_dmd', 'A_phys', 'B_phys', '-append');
fprintf('\nSaved A_dmd, B_dmd, A_phys, B_phys -> %s\n', dataFile);
