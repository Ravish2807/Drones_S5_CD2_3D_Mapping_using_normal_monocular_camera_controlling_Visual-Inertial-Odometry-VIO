%% MODEL 1 - STEP 4 : STATE ESTIMATION WITH AN ERROR-STATE EKF
% Block 4 of the Model-1 equation flow (docs: Section 3.4).
%
%   INPUT  : applied input u_k = [T, tau],  sensor measurements z_k
%            (camera/VIO position + attitude at 25 Hz, gyroscope at 100 Hz),
%            initial guess xhat_0, P_0, noise covariances Q, R_n
%   OUTPUT : estimated state xhat_k = [p, v, R, w]_k and covariance P_k
%
%   Prediction  : xhat_{k+1} = f(xhat_k, u_k)          P_{k+1} = F P_k F' + Q
%   Correction  : K = P H' (H P H' + R_n)^-1
%                 xhat = xhat (+) K (z - h(xhat))       P = (I - K H) P
%
% In the project: this is the "VIO" box. The monocular camera (via visual
% odometry) gives pose, the IMU gives rates; the EKF fuses them with the
% physics model into one smooth, trustworthy state for the controller.

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model1_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'P', 'dt_ekf'}, 'step3_local_error_dynamics.m');
P = S.P; dt = S.dt_ekf;

step_header(1, 4, 'Error-state Extended Kalman Filter', { ...
    'predict : xh = f(xh,u)          P = F P F'' + Q', ...
    'correct : K = P H''(H P H'' + Rn)^-1   xh = xh (+) K(z - h(xh))   P = (I - K H) P'});

%% 1. Toy example (scalar, hand-checkable): altitude estimation
% Prediction: altitude 10 m, commanded climb 0.5 m/s for 1 s, model noise Q = 0.5
% Correction: GPS says 12.5 m with noise variance R = 1
fprintf('\nToy example: one predict + one correct on altitude (scalar EKF)\n');
xk = 10; Pk = 3.5; Fs = 1; Qs = 0.5; climb = 0.5; dts = 1;
xm = xk + climb*dts;          Pm = Fs*Pk*Fs' + Qs;
z  = 12.5; Hs = 1; Rs = 1;
Ks = Pm*Hs'/(Hs*Pm*Hs' + Rs);
xp = xm + Ks*(z - Hs*xm);     Pp = (1 - Ks*Hs)*Pm;
io_print('IN',  'xhat_k, P_k', [xk Pk], 'm, m^2', 'prior altitude and variance');
io_print('OUT', 'xhat-, P-',   [xm Pm], 'm, m^2', 'after prediction (uncertainty grows)');
io_print('IN',  'z, R',        [z Rs],  'm, m^2', 'GPS measurement and its variance');
io_print('OUT', 'K',           Ks,      '-',      'Kalman gain = 4/(4+1)');
io_print('OUT', 'xhat+, P+',   [xp Pp], 'm, m^2', 'after correction (uncertainty shrinks)');

%% 2. Simulate the truth and the sensors (20 s)
% The "real" drone is flown along the helix by its autopilot, which sees the
% TRUE state (like a pilot would). We only record what an onboard computer
% knows: the applied command u_k and the noisy sensor readings z_k.
% The EKF never sees X.
rng(4);
T_end = 20; t = 0:dt:T_end; N = numel(t);
camEvery = 4;                                    % camera/VIO at 25 Hz
sig_p = 0.05; sig_th = 0.02; sig_g = 0.01;       % sensor noise (m, rad, rad/s)
sig_F = 0.2;  sig_tau = 5e-4;                    % wind gusts (N, N m)

X = zeros(18, N); u = zeros(4, N);
X(:,1) = pack_state([0.3; -0.3; 1.8], zeros(3,1), eye(3), zeros(3,1));
for k = 1:N
    [p, v, R, w] = unpack_state(X(:,k));
    ref = reference_trajectory(t(k), 'helix');
    [~, ~, a_d]      = position_control(p, v, ref, P);
    [F_d, ~, R_d]    = force_attitude(a_d, ref.yaw, P);
    [~, ~, tau, T]   = attitude_control(R, w, R_d, ref.w, ref.wdot, F_d, P);
    [~, ~, u(:,k)]   = motor_allocation([T; tau], P);
    if k < N
        d = [sig_F*randn(3,1); sig_tau*randn(3,1)];
        X(:,k+1) = rk4_step(X(:,k), u(:,k), dt, P, d);
    end
end
% measurements
Zp = zeros(3, N); Zth = zeros(3, N); Zg = zeros(3, N);
for k = 1:N
    [p, ~, R, w] = unpack_state(X(:,k));
    Zp(:,k)  = p + sig_p*randn(3,1);
    Zth(:,k) = so3_log(R*so3_exp(sig_th*randn(3,1)));   % stored as rotation vector
    Zg(:,k)  = w + sig_g*randn(3,1);
end

%% 3. Filter set-up
Qd = blkdiag(1e-8*eye(3), (sig_F/P.m*dt)^2*eye(3), 1e-8*eye(3), (sig_tau/P.J(1,1)*dt)^2*eye(3));
Rcam  = blkdiag(sig_p^2*eye(3), sig_th^2*eye(3));
Rgyro = sig_g^2*eye(3);
Hcam  = [eye(3), zeros(3,3), zeros(3,3), zeros(3,3);
         zeros(3,3), zeros(3,3), eye(3), zeros(3,3)];
Hgyro = [zeros(3,9), eye(3)];

[p0, v0, R0, w0] = unpack_state(X(:,1));
xhat = pack_state(p0 + [0.3; -0.2; 0.1], v0 + [0.2; -0.1; 0.1], ...
                  R0*so3_exp([0.05; -0.05; 0.1]), w0);
Pcov = blkdiag(0.3^2*eye(3), 0.2^2*eye(3), 0.1^2*eye(3), 0.05^2*eye(3));

fprintf('\nFilter inputs\n');
io_print('IN', 'u_k',    u(:,1), 'N, N m', 'applied thrust/torque (first sample)');
io_print('IN', 'z_cam',  [Zp(:,1); Zth(:,1)], 'm, rad', 'camera/VIO pose, 25 Hz');
io_print('IN', 'z_gyro', Zg(:,1), 'rad/s', 'gyroscope, 100 Hz');
io_print('IN', 'P_0',    diag(Pcov), 'mixed', 'initial variances (diag)');
io_print('IN', 'R_cam',  diag(Rcam), 'mixed', 'camera noise variances (diag)');

%% 4. Run the error-state EKF
Xh = zeros(18, N); Pd = zeros(12, N); Err = zeros(12, N);
for k = 1:N
    % ---- correction 1: gyroscope (every sample),  h(x) = w ----
    [~, ~, ~, wh] = unpack_state(xhat);
    [xhat, Pcov] = ekf_update(xhat, Pcov, Zg(:,k) - wh, Hgyro, Rgyro);
    % ---- correction 2: camera/VIO pose (every 4th sample),  h(x) = (p, R) ----
    if mod(k-1, camEvery) == 0
        [ph, ~, Rh] = unpack_state(xhat);
        r = [Zp(:,k) - ph; so3_log(Rh'*so3_exp(Zth(:,k)))];
        [xhat, Pcov] = ekf_update(xhat, Pcov, r, Hcam, Rcam);
    end
    Xh(:,k) = xhat; Pd(:,k) = diag(Pcov);
    Err(:,k) = state_boxminus(X(:,k), xhat);
    % ---- prediction to time k+1 ----
    if k < N
        [xhat, Pcov] = ekf_predict(xhat, Pcov, u(:,k), dt, P, Qd);
    end
end

%% 5. Outputs and accuracy
settle = t > 2;                                   % ignore the initial transient
rmse = @(rows) sqrt(mean(sum(Err(rows, settle).^2, 1)));
rawCam = Zp(:, 1:camEvery:end) - X(1:3, 1:camEvery:end);
fprintf('\nFilter outputs\n');
io_print('OUT', 'xhat_N',  Xh(:,end), 'mixed', 'final estimate [p; v; vec(R); w]');
io_print('OUT', 'diag P_N', Pd(:,end), 'mixed', 'final variances');
io_print('CHK', 'RMSE p',  rmse(1:3),  'm',     'position error after 2 s');
io_print('CHK', 'raw cam', sqrt(mean(sum(rawCam.^2, 1))), 'm', 'raw camera position error (for comparison)');
io_print('CHK', 'RMSE v',  rmse(4:6),  'm/s',   'velocity error (never measured directly!)');
io_print('CHK', 'RMSE th', rad2deg(rmse(7:9)), 'deg', 'attitude error');
io_print('CHK', 'RMSE w',  rmse(10:12), 'rad/s', 'body-rate error');
inside = mean(all(abs(Err(:, settle)) <= 3*sqrt(Pd(:, settle)), 1));
io_print('CHK', 'in 3-sigma', 100*inside, '%', 'samples where all 12 errors lie inside +-3 sigma');

labels = {'\delta p_x [m]', '\delta v_x [m/s]', '\delta\theta_x [rad]', '\delta\omega_x [rad/s]'};
rows = [1 4 7 10];
fig = figure('Name', 'Step 4 - EKF errors', 'Position', [80 80 1000 600]);
for i = 1:4
    subplot(2,2,i);
    s3 = 3*sqrt(Pd(rows(i),:));
    fill([t, fliplr(t)], [s3, -fliplr(s3)], [0.85 0.9 1], 'EdgeColor', 'none'); hold on;
    plot(t, Err(rows(i),:), 'Color', [0.1 0.3 0.7], 'LineWidth', 1.1);
    grid on; xlabel('t [s]'); ylabel(labels{i});
    if i == 1, legend('\pm3\sigma from P_k', 'true error x (-) x\^', 'Location', 'northeast'); end
end
sgtitle('Error-state EKF: estimation error stays inside the predicted 3\sigma band');
exportgraphics(fig, fullfile(figDir, 'step4_ekf_errors.png'), 'Resolution', 150);

fig = figure('Name', 'Step 4 - EKF trajectory', 'Position', [80 80 650 500]);
plot3(Zp(1,1:camEvery:end), Zp(2,1:camEvery:end), Zp(3,1:camEvery:end), '.', ...
      'Color', [0.7 0.7 0.7]); hold on;
plot3(X(1,:), X(2,:), X(3,:), 'k', 'LineWidth', 1.6);
plot3(Xh(1,:), Xh(2,:), Xh(3,:), '--', 'Color', [0.85 0.33 0.1], 'LineWidth', 1.4);
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]'); view(35, 25);
legend('camera / VIO measurements', 'true trajectory', 'EKF estimate', 'Location', 'northeast');
title('Fusing noisy camera poses with the physics model');
exportgraphics(fig, fullfile(figDir, 'step4_ekf_trajectory.png'), 'Resolution', 150);

%% 6. Save outputs (the final estimate is handed to the controller in Step 5)
xhat_ekf = Xh(:,end); P_ekf = Pcov; t_ekf = t(end);
save(dataFile, 'xhat_ekf', 'P_ekf', 't_ekf', 'Qd', 'Rcam', 'Rgyro', '-append');
fprintf('\nSaved xhat_ekf, P_ekf, t_ekf, Qd, Rcam, Rgyro -> %s\n', dataFile);
