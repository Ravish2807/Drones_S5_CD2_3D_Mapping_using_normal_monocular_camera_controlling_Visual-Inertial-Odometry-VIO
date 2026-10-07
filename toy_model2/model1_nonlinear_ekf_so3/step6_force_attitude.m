%% MODEL 1 - STEP 6 : FORCE AND ATTITUDE GENERATION
% Block 6 of the Model-1 equation flow (docs: Section 3.6).
%
%   INPUT  : desired acceleration a_d (Step 5), desired yaw psi_d, mass m, gravity g
%   OUTPUT : desired force F_d, desired thrust axis b3d, desired attitude R_d
%
%   F_d = m (a_d - g)
%   b3d = F_d / ||F_d||
%   R_d = [b1d  b2d  b3d],  b2d = (b3d x b1c)/||b3d x b1c||,  b1d = b2d x b3d
%
% In the project: so3_controller.py -> compute_desired_force() and
% compute_desired_attitude().  A quadrotor can only push along its own
% z-axis, so "accelerate this way" becomes "tilt this way and push".

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model1_data.mat');
S = require_vars(dataFile, {'P', 'a_d_toy', 'a_d_chain', 'ref_chain'}, 'step5_position_control.m');
P = S.P;

step_header(1, 6, 'Force and attitude generation', { ...
    'F_d = m (a_d - g)', 'b3d = F_d / ||F_d||', ...
    'b2d = b3d x b1c / ||.||,  b1d = b2d x b3d,  R_d = [b1d b2d b3d]'});

%% Part A - toy example from the documentation (continues Step 5 Part A)
fprintf('\nPart A : toy example\n');
[F_d, b3d, R_d] = force_attitude(S.a_d_toy, 0, P);
io_print('IN',  'a_d',   S.a_d_toy, 'm/s^2', 'desired acceleration (Step 5, Part A)');
io_print('IN',  'g',     P.g,       'm/s^2', 'gravity vector (ENU)');
io_print('IN',  'm',     P.m,       'kg',    'mass');
io_print('IN',  'psi_d', 0,         'rad',   'desired yaw');
io_print('OUT', 'F_d',   F_d,       'N',     'desired force vector');
io_print('OUT', '||F_d||', norm(F_d), 'N',   'required thrust magnitude');
io_print('OUT', 'b3d',   b3d,       '-',     'desired thrust direction (unit)');
io_print('OUT', 'R_d',   R_d,       '-',     'desired attitude');
io_print('CHK', 'tilt',  acosd(b3d(3)), 'deg', 'angle between b3d and vertical');
io_print('CHK', '||Rd''Rd-I||', norm(R_d'*R_d - eye(3)), '-', 'orthonormality of R_d');
io_print('CHK', 'det(R_d)', det(R_d), '-', 'proper rotation (= +1)');

%% Part B - chained from Step 5 Part B
fprintf('\nPart B : chained from Step 5\n');
[F_d_c, b3d_c, R_d_c] = force_attitude(S.a_d_chain, S.ref_chain.yaw, P);
io_print('IN',  'a_d', S.a_d_chain, 'm/s^2', 'from Step 5');
io_print('OUT', 'F_d', F_d_c, 'N', 'desired force -> Step 7');
io_print('OUT', 'R_d', R_d_c, '-', 'desired attitude -> Step 7');

%% Save outputs
F_d_toy = F_d; R_d_toy = R_d; F_d_chain = F_d_c; R_d_chain = R_d_c;
save(dataFile, 'F_d_toy', 'R_d_toy', 'F_d_chain', 'R_d_chain', '-append');
fprintf('\nSaved F_d_toy, R_d_toy, F_d_chain, R_d_chain -> %s\n', dataFile);
