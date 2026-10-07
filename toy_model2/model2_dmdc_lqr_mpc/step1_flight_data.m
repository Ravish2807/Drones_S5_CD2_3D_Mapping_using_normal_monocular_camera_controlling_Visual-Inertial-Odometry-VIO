%% MODEL 2 - STEP 1 : NONLINEAR FLIGHT DATA (FROM DRONE OR SIMULATION)
% Block 1 of the Model-2 equation flow (docs: Section 4.1).
%
%   INPUT  : the drone (here: the nonlinear rigid-body simulator), a stabilising
%            pilot, random excitation, sample time dt
%   OUTPUT : sampled states x_k = [p, v, R, w]_k  -> 12-vector deviations
%            sampled inputs u_k = [T, tau]_k      -> 4-vector deviations
%
%   x_k = [p_k - p_ref; v_k; Log(R_k); w_k] in R^12,   u_k = [T_k - m g0; tau_k] in R^4
%
% In the project: a ROS 2 bag of /ap/v1/pose/filtered, /ap/v1/twist/filtered
% and the commands we send, recorded from ArduPilot SITL / Gazebo.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
resDir = fullfile(here, 'results'); figDir = fullfile(here, 'figures');
if ~isfolder(resDir), mkdir(resDir); end
if ~isfolder(figDir), mkdir(figDir); end
dataFile = fullfile(resDir, 'model2_data.mat');

P = quad_params();
step_header(2, 1, 'Nonlinear flight data', { ...
    'x_k = [p_k - p_ref; v_k; Log(R_k); w_k]   (12x1)', ...
    'u_k = [T_k - m g0; tau_k]                 (4x1)', 'sampled at t_k = k dt'});

%% 1. Recording set-up
dt    = 1/25;            % 25 Hz = control_rate in controller.yaml
nSub  = 8;               % physics integrated at 200 Hz between samples
Tfly  = 25;              % seconds per flight
nRuns = 5;               % 4 for identification + 1 held out for validation
holdK = 5;               % excitation held for 5 samples (0.2 s)
excAmp = [2.0; 0.03; 0.03; 0.01];   % excitation amplitude [N; N m; N m; N m]
sig_meas = [1e-3*ones(6,1); 5e-4*ones(3,1); 1e-3*ones(3,1)];  % logged-state noise
u_trim = [P.m*P.g0; 0; 0; 0];

io_print('IN', 'dt',     dt,     's',      'sample time (25 Hz)');
io_print('IN', 'runs',   nRuns,  '-',      'number of recorded flights');
io_print('IN', 'T_fly',  Tfly,   's',      'duration of each flight');
io_print('IN', 'u_trim', u_trim, 'N, N m', 'hover input (operating point)');
io_print('IN', 'excAmp', excAmp, 'N, N m', 'random excitation amplitude');

%% 2. Fly and record
rng(21);
K = round(Tfly/dt);
runs = cell(1, nRuns);
for r = 1:nRuns
    x = pack_state(P.p_hover + 0.3*randn(3,1), 0.1*randn(3,1), so3_exp(0.05*randn(3,1)), zeros(3,1));
    Xr = zeros(12, K+1); Ur = zeros(4, K); Xw = zeros(18, K+1);
    exc = zeros(4,1);
    for k = 1:K+1
        Xw(:,k) = x;
        Xr(:,k) = state_to_dev(x, P.p_hover) + sig_meas.*randn(12,1);   % logged state
        if k == K+1, break, end
        if mod(k-1, holdK) == 0, exc = excAmp.*(2*rand(4,1) - 1); end
        du = baseline_pilot(Xr(:,k), P) + exc;
        [~, ~, u_app] = motor_allocation(u_trim + du, P);
        Ur(:,k) = u_app - u_trim;                                        % logged input
        d = [0.1*randn(3,1); 2e-4*randn(3,1)];                           % gusts
        for j = 1:nSub
            x = rk4_step(x, u_app, dt/nSub, P, d);
        end
    end
    runs{r} = struct('X', Xr, 'U', Ur, 'Xworld', Xw, 't', (0:K)*dt);
end

%% 3. Outputs
fprintf('\nRecorded data\n');
io_print('OUT', 'x_k (k=1)', runs{1}.X(:,1), 'mixed', 'first logged state of flight 1');
io_print('OUT', 'u_k (k=1)', runs{1}.U(:,1), 'N, N m', 'first logged input of flight 1');
io_print('OUT', 'X run 1',   runs{1}.X, 'mixed', sprintf('12 x %d state history', K+1));
io_print('OUT', 'U run 1',   runs{1}.U, 'mixed', sprintf('4 x %d input history', K));
allX = cell2mat(cellfun(@(s) s.X, runs, 'UniformOutput', false));
io_print('CHK', 'max tilt', rad2deg(max(vecnorm(allX(7:8,:)))), 'deg', ...
    'largest roll/pitch: data stays in the near-hover (linear) regime');

fig = figure('Name', 'Step 1 - flight data', 'Position', [60 60 1100 420]);
subplot(1,2,1);
yyaxis left;  plot(runs{1}.t, runs{1}.X([1 2 3 7 8],:), '-'); ylabel('states');
yyaxis right; plot(runs{1}.t(1:end-1), runs{1}.U(1,:), '-'); ylabel('\delta T [N]');
grid on; xlabel('t [s]'); title('Example flight data (flight 1)');
legend('\delta p_x', '\delta p_y', '\delta p_z', '\theta_x', '\theta_y', '\delta T', 'Location', 'southoutside', 'Orientation', 'horizontal');
subplot(1,2,2); hold on;
for r = 1:nRuns
    plot3(runs{r}.Xworld(1,:), runs{r}.Xworld(2,:), runs{r}.Xworld(3,:), 'LineWidth', 1.1);
end
grid on; view(35, 25); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('3D trajectories of the 5 recorded flights');
exportgraphics(fig, fullfile(figDir, 'step1_flight_data.png'), 'Resolution', 150);

%% 4. Save
save(dataFile, 'P', 'runs', 'dt', 'nSub', 'u_trim');
fprintf('\nSaved P, runs, dt, nSub, u_trim -> %s\n', dataFile);
