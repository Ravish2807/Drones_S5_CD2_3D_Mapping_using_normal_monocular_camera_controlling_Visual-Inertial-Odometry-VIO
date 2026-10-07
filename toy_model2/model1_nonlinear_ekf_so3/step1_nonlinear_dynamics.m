%% MODEL 1 - STEP 1 : PHYSICAL SYSTEM AND NONLINEAR DYNAMICS
% Block 1 of the Model-1 equation flow (docs: Section 3.1).
%
%   INPUT  : state  x = [p, v, R, w]   (p,v in R^3, R in SO(3), w in R^3)
%            input  u = [T, tau]       (T in R, tau in R^3)
%   OUTPUT : xdot = f(x,u)  and a simulated open-loop motion x(t)
%
%       p_dot = v
%       v_dot = g + (T/m) R e3
%       R_dot = R [w]x
%       w_dot = J^-1 (tau - w x J w)
%
% In the project: this is "the real drone" (ArduPilot SITL / Gazebo Iris).
% Every later step either predicts, estimates or controls this model.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
resDir = fullfile(here, 'results'); figDir = fullfile(here, 'figures');
if ~isfolder(resDir), mkdir(resDir); end
if ~isfolder(figDir), mkdir(figDir); end
dataFile = fullfile(resDir, 'model1_data.mat');

P = quad_params();
step_header(1, 1, 'Physical system and nonlinear dynamics', { ...
    'p_dot = v', 'v_dot = g + (T/m) R e3', 'R_dot = R [w]x', ...
    'w_dot = J^-1 (tau - w x J w)'});

%% 1. Physical parameters (inputs common to every step)
fprintf('\nPhysical parameters (from drones_controller/config/controller.yaml)\n');
io_print('IN', 'm',  P.m,  'kg',     'vehicle mass');
io_print('IN', 'g',  P.g,  'm/s^2',  'gravity vector, ENU world frame');
io_print('IN', 'J',  P.J,  'kg m^2', 'body inertia matrix');

%% 2. Toy example A : hover equilibrium
% Level attitude, no rotation, thrust exactly equal to weight.
fprintf('\nToy example A : hover equilibrium\n');
x = pack_state([0; 0; 2], zeros(3,1), eye(3), zeros(3,1));
u = [P.m*P.g0; 0; 0; 0];
[pdot, vdot, Rdot, wdot] = unpack_state(quad_dynamics(x, u, P));
io_print('IN',  'R',      eye(3),  '-',       'level attitude');
io_print('IN',  'T',      u(1),    'N',       'thrust = m g0');
io_print('OUT', 'v_dot',  vdot,    'm/s^2',   'zero -> the drone hangs still');
io_print('OUT', 'w_dot',  wdot,    'rad/s^2', 'zero -> no rotation starts');
io_print('OUT', 'R_dot',  Rdot,    '1/s',     'zero -> attitude constant');

%% 3. Toy example B : tilted 10 deg about the body x-axis (roll)
% Same thrust as hover, but the thrust vector is tilted.
fprintf('\nToy example B : 10 deg roll with hover thrust\n');
phi = deg2rad(10);
Rx  = so3_exp([phi; 0; 0]);
x   = pack_state([0; 0; 2], zeros(3,1), Rx, zeros(3,1));
[~, vdot] = unpack_state(quad_dynamics(x, u, P));
io_print('IN',  'R',     Rx,   '-',     'rotation of 10 deg about x');
io_print('OUT', 'v_dot', vdot, 'm/s^2', 'accelerates to -y and sinks slightly');
fprintf('          hand calculation: g0*[0, -sin(10deg), cos(10deg)-1] = [0, %.4f, %.4f]\n', ...
    -P.g0*sin(phi), P.g0*(cos(phi) - 1));

%% 4. Toy example C : gyroscopic coupling
% Spinning about x and z simultaneously with zero torque produces a y-acceleration.
fprintf('\nToy example C : gyroscopic coupling (tau = 0)\n');
w = [1; 0; 1];
x = pack_state([0; 0; 2], zeros(3,1), eye(3), w);
[~, ~, ~, wdot] = unpack_state(quad_dynamics(x, u, P));
io_print('IN',  'w',     w,    'rad/s',   'body rates');
io_print('OUT', 'w_dot', wdot, 'rad/s^2', 'J^-1(-w x Jw) = [0 1 0]''');

%% 5. Open-loop simulation : hover, then a tiny 0.01 N m roll-torque pulse
% Shows why we need feedback: one small kick and the drone drifts away forever.
dt = 0.01; t = 0:dt:4; N = numel(t);
x  = pack_state(P.p_hover, zeros(3,1), eye(3), zeros(3,1));
X  = zeros(18, N); X(:,1) = x;
U  = repmat(u, 1, N);
U(2, t >= 1 & t < 1.1) = 0.01;
for k = 1:N-1
    X(:,k+1) = rk4_step(X(:,k), U(:,k), dt, P);
end
tilt = acosd(min(1, X(15,:)));        % angle between body z and world z

fprintf('\nOpen-loop simulation (4 s, RK4, dt = %.2f s)\n', dt);
io_print('IN',  'u(1.05 s)', U(:, find(t >= 1, 1) + 5), 'N, N m', 'hover thrust + 0.1 s roll-torque pulse');
io_print('OUT', 'p(4 s)',   X(1:3,end), 'm',    'final position (drifted)');
io_print('OUT', 'tilt(4 s)', tilt(end), 'deg',  'final tilt angle (keeps growing)');

fig = figure('Name', 'Step 1 - open loop', 'Position', [100 100 900 350]);
subplot(1,2,1); plot(t, X(1:3,:) - P.p_hover, 'LineWidth', 1.4); grid on;
xlabel('t [s]'); ylabel('p - p_{hover} [m]'); legend('x', 'y', 'z', 'Location', 'northwest');
title('Position after a 0.01 N m torque pulse');
subplot(1,2,2); plot(t, tilt, 'LineWidth', 1.4); grid on;
xlabel('t [s]'); ylabel('tilt [deg]'); title('Tilt angle: open loop is unstable');
exportgraphics(fig, fullfile(figDir, 'step1_open_loop.png'), 'Resolution', 150);

%% 6. Save outputs for the next steps
x_hover = pack_state(P.p_hover, zeros(3,1), eye(3), zeros(3,1));
u_hover = [P.m*P.g0; 0; 0; 0];
save(dataFile, 'P', 'x_hover', 'u_hover');
fprintf('\nSaved P, x_hover, u_hover -> %s\n', dataFile);
