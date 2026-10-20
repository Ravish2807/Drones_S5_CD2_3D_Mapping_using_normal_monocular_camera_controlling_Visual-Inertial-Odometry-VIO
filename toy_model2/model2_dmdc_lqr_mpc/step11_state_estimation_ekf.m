%% MODEL 2 - STEP 11 : STATE ESTIMATION WITH AN EXTENDED KALMAN FILTER
% Block 11 of the Model-2 equation flow (docs: Section 5.11). It closes the loop:
% apply to drone (10) -> the drone moves -> sensors -> EKF (11) -> x_hat -> MPC (9) / LQR.
%
%   INPUT  : applied input u_k (deviation from hover), camera pixels of known floor
%            markers, gyroscope rates, the LEARNED model A, B (Step 2)
%   OUTPUT : estimated 12-state x_hat_k = [dp; v; theta; w] and its covariance P_k,
%            which the controllers use instead of the true state
%
%   Prediction  (learned model) : x_hat = A x_hat + B u            Sigma = A Sigma A' + Q_e      (M2.19)
%   Process noise from data     : Q_e = cov(x_{k+1} - A x_k - B u_k) on a held-out flight      (M2.20)
%   Measurement (nonlinear)     : pixel = f q1/q3 + c,  q = R_bc' R(theta)' (l - p)            (M2.21-22)
%   Correction                  : L = Sigma H'(H Sigma H' + Rn)^-1,  x_hat = x_hat + L (z - h(x_hat)),
%                                 Sigma = (I - L H) Sigma (I - L H)' + L Rn L'                  (M2.23-24)
%   (Model 2 already uses P, Q, K for LQR, so the docs write Sigma, Q_e, L for the filter;
%    in this code the covariance is the variable Pk and the process noise is Q.)
%
% Why it is needed: Steps 9 and 10 assumed the controller knows the 12-state. A real
% drone measures only pixels and body rates; velocity is never measured. The EKF
% turns those raw readings into the state, using the same learned model as the controller.
% It is "extended" because the camera equation is nonlinear (division by depth, rotation).

clc; clear; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib'));
dataFile = fullfile(here, 'results', 'model2_data.mat');
figDir   = fullfile(here, 'figures');
S = require_vars(dataFile, {'P', 'runs', 'A_dmd', 'B_dmd', 'K_lqr', 'mpc', 'dt', 'nSub', 'u_trim'}, ...
                 'step10_apply_to_drone.m');
P = S.P; A = S.A_dmd; B = S.B_dmd; dt = S.dt;
C = camera_model();

step_header(2, 11, 'State estimation - Extended Kalman Filter', { ...
    'predict : x_hat = A x_hat + B u              Sigma = A Sigma A'' + Q_e', ...
    'measure : pixel = f q1/q3 + c,  q = R_bc'' R(theta)'' (l - p)   (nonlinear -> Jacobian H)', ...
    'correct : L = Sigma H''(H Sigma H'' + Rn)^-1   x_hat = x_hat + L (z - h(x_hat))', ...
    '          Sigma = (I - L H) Sigma (I - L H)'' + L Rn L'''});

%% Part A - toy example from the documentation: altitude from one pixel
% A camera looks down from height h at a floor marker 1 m to the side.
% Pinhole: pixel offset u = f d / h  (nonlinear in h)  ->  H = du/dh = -f d / h^2
fprintf('\nPart A : scalar EKF update with a nonlinear (pinhole) measurement\n');
f = 400; d = 1; h_hat = 2.0; Ph = 0.04; u_meas = 210; Ru = 4;
u_pred = f*d/h_hat;
Hs = -f*d/h_hat^2;
Ss = Hs*Ph*Hs + Ru;
Ks = Ph*Hs/Ss;
h_new = h_hat + Ks*(u_meas - u_pred);
P_new = (1 - Ks*Hs)*Ph;
io_print('IN',  'h_hat, Sigma', [h_hat Ph], 'm, m^2', 'prior altitude and variance');
io_print('IN',  'f, d', [f d], 'px, m', 'focal length, marker offset');
io_print('IN',  'u, Rn', [u_meas Ru], 'px, px^2', 'measured pixel offset and its noise variance');
io_print('OUT', 'u_pred', u_pred, 'px', 'expected pixel f d / h_hat');
io_print('OUT', 'H', Hs, 'px/m', 'slope of the pinhole curve at h_hat');
io_print('OUT', 'L', Ks, 'm/px', 'estimator (Kalman) gain');
io_print('OUT', 'h_new', h_new, 'm', 'corrected altitude');
io_print('OUT', 'Sigma_new', P_new, 'm^2', 'variance: 0.04 -> 0.0004 (sigma 20 cm -> 2 cm)');
io_print('CHK', 'f d / u', f*d/u_meas, 'm', 'exact altitude from the pixel; EKF differs by the linearization error');

%% Part B1 - the camera measurement and its Jacobian
fprintf('\nPart B1 : camera model at hover (2 m above the floor)\n');
x_test = [0.3; -0.2; 0.1; zeros(3,1); 0.05; -0.04; 0.2; zeros(3,1)];
[z, H, ids] = camera_measure(x_test, P.p_hover, C);
Hn = zeros(size(H));  e = 1e-6;
for j = 1:12
    dx = zeros(12,1); dx(j) = e;
    Hn(:,j) = (camera_measure(x_test + dx, P.p_hover, C, ids) - camera_measure(x_test - dx, P.p_hover, C, ids))/(2*e);
end
io_print('IN',  'markers', size(C.L, 2), '-', 'known floor markers (11 x 11 grid, 0.5 m apart)');
io_print('OUT', 'visible', numel(ids), '-', 'markers inside the 640 x 480 image');
io_print('OUT', 'z(1:4)', z(1:4), 'px', 'first two markers (u, v)');
io_print('OUT', 'H', H, 'px/unit', sprintf('%d x 12 Jacobian (zero columns: velocity and rate)', size(H,1)));
io_print('CHK', 'max|H-Hnum|', max(abs(H(:) - Hn(:))), 'px/unit', 'analytic vs numerical Jacobian');

%% Part B2 - process noise Q learned from data (DMDc residuals on the held-out flight)
V  = S.runs{end};
Eres = V.X(:, 2:end) - A*V.X(:, 1:end-1) - B*V.U;      % what the learned model cannot explain
Q_data = cov(Eres');
q_infl = 4;                                             % gusts in flight are 2x those in the data (variance 4x)
Q = q_infl*Q_data;
fprintf('\nPart B2 : process noise from the DMDc residuals of flight 5 (never used for learning)\n');
io_print('IN',  'residuals', Eres, 'mixed', 'x_{k+1} - A x_k - B u_k on the held-out flight');
io_print('OUT', 'sqrt diag Q_e', sqrt(diag(Q))', 'mixed', 'per-step model uncertainty (after x4 gust margin)');
Rn_cam  = C.sigma_px^2;
Rn_gyro = C.sigma_gyro^2*eye(3);

%% Part C - the closed loop with the EKF in it (nonlinear drone, gusts, noisy sensors)
fprintf('\nPart C : nonlinear flight, controllers fed by the EKF estimate instead of the true state\n');
rng(11);
Kn = 250; t = (0:Kn)*dt;
x0_world = pack_state(P.p_hover + [1.5; -1.0; -1.2], zeros(3,1), eye(3), zeros(3,1));
cases = {'MPC + EKF', 'LQR + EKF', 'MPC, true state (Step 10)'};
res = struct();
for c = 1:3
    x  = x0_world;
    xh = state_to_dev(x, P.p_hover) + [0.2; -0.2; 0.1; zeros(3,1); 0.03; -0.03; 0.05; zeros(3,1)];
    Pk = blkdiag(0.2^2*eye(3), 0.2^2*eye(3), 0.05^2*eye(3), 0.05^2*eye(3));
    L.x = zeros(12, Kn+1); L.xh = L.x; L.sd = L.x; L.nvis = zeros(1, Kn+1); L.nis = zeros(1, Kn+1);
    for k = 1:Kn+1
        xd = state_to_dev(x, P.p_hover);
        % ---- sensors (from the TRUE drone) ----
        [zc, ~, ids] = camera_measure(xd, P.p_hover, C);
        zc = zc + C.sigma_px*randn(size(zc));
        zg = xd(10:12) + C.sigma_gyro*randn(3,1);
        % ---- EKF correction (from the ESTIMATE) ----
        [zh, Hc] = camera_measure(xh, P.p_hover, C, ids);
        z  = [zc; zg];  zhat = [zh; xh(10:12)];
        Hm = [Hc; zeros(3,9), eye(3)];
        Rn = blkdiag(Rn_cam*eye(numel(zc)), Rn_gyro);
        [xh, Pk, info] = ekf2_update(xh, Pk, z, zhat, Hm, Rn);
        L.x(:,k) = xd; L.xh(:,k) = xh; L.sd(:,k) = sqrt(diag(Pk));
        L.nvis(k) = numel(ids); L.nis(k) = info.nis/numel(z);
        if k == Kn+1, break, end
        % ---- controller ----
        if c == 3, xc = xd; else, xc = xh; end
        if c == 2
            du = min(max(-S.K_lqr*xc, S.mpc.u_min), S.mpc.u_max);
        else
            du = mpc_solve(S.mpc, xc);
        end
        [~, ~, u_app] = motor_allocation(S.u_trim + du, P);
        % ---- the drone moves (gusts), the EKF predicts with the same applied input ----
        d = [0.2*randn(3,1); 5e-4*randn(3,1)];
        for j = 1:S.nSub, x = rk4_step(x, u_app, dt/S.nSub, P, d); end
        [xh, Pk] = ekf2_predict(xh, Pk, u_app - S.u_trim, A, B, Q);
    end
    res.(sprintf('c%d', c)) = L;
    err = L.x - L.xh;  late = t > 1;
    fprintf('\n  %s\n', cases{c});
    io_print('CHK', 'RMSE p', sqrt(mean(sum(err(1:3,late).^2))), 'm', 'position estimate');
    io_print('CHK', 'RMSE v', sqrt(mean(sum(err(4:6,late).^2))), 'm/s', 'velocity estimate (never measured)');
    io_print('CHK', 'RMSE theta', rad2deg(sqrt(mean(sum(err(7:9,late).^2)))), 'deg', 'attitude estimate');
    io_print('CHK', 'max|v| true', max(max(abs(L.x(4:6,:)))), 'm/s', 'largest true velocity component');
    io_print('CHK', 'err(10 s)', norm(L.x(1:3,end)), 'm', 'true distance to the setpoint');
    io_print('CHK', 'in 3 sigma', 100*mean(all(abs(err(:,late)) <= 3*L.sd(:,late), 1)), '%', ...
        'samples with all 12 errors inside +-3 sigma');
    io_print('CHK', 'mean NIS/m', mean(L.nis(late)), '-', 'normalized innovation, ~1 for a consistent filter');
end

%% Figures
L = res.c1;  err = L.x - L.xh;
fig = figure('Name', 'Step 11 - EKF errors', 'Position', [60 60 1050 560]);
rows = [1 4 7 2];  names = {'\delta p_x [m]', 'v_x [m/s] (never measured)', '\theta_x roll [rad]', '\delta p_y [m]'};
for i = 1:4
    subplot(2,2,i); s3 = 3*L.sd(rows(i),:);
    fill([t fliplr(t)], [s3 -fliplr(s3)], [0.85 0.9 1], 'EdgeColor', 'none'); hold on;
    plot(t, err(rows(i),:), 'Color', [0.1 0.3 0.7], 'LineWidth', 1.1); grid on;
    xlabel('t [s]'); ylabel(names{i});
    if i == 1, legend('\pm3\sigma from P_k', 'true - estimate'); end
end
sgtitle('Model 2 EKF (learned model + camera pixels + gyro): errors stay inside the 3\sigma band');
exportgraphics(fig, fullfile(figDir, 'step11_ekf_errors.png'), 'Resolution', 150);

fig = figure('Name', 'Step 11 - flight', 'Position', [60 60 1100 430]);
subplot(1,2,1);
k0 = 1; [zc, ~, ids] = camera_measure(res.c1.x(:,k0), P.p_hover, C);
plot(zc(1:2:end), zc(2:2:end), 'ks', 'MarkerFaceColor', [0.3 0.3 0.3]); hold on;
k1 = 200; zc = camera_measure(res.c1.x(:,k1), P.p_hover, C);
plot(zc(1:2:end), zc(2:2:end), 'o', 'Color', [0.1 0.4 0.8]);
set(gca, 'YDir', 'reverse'); axis equal; xlim([0 C.W]); ylim([0 C.Hpx]); grid on;
xlabel('u [px]'); ylabel('v [px]'); legend('t = 0 s (0.8 m high)', 't = 8 s (2 m high)');
title('What the camera sees: known floor markers');
subplot(1,2,2);
plot(t, max(abs(res.c1.x(4:6,:))), 'LineWidth', 1.6); hold on;
plot(t, max(abs(res.c2.x(4:6,:))), '--', 'LineWidth', 1.4);
plot(t, max(abs(res.c3.x(4:6,:))), ':', 'LineWidth', 1.6);
yline(1, 'k:', 'max\_velocity', 'LabelHorizontalAlignment', 'left'); grid on;
xlabel('t [s]'); ylabel('max_i |v_i| [m/s]'); legend(cases, 'Location', 'northeast');
title('Flying on the EKF estimate vs. on the true state');
exportgraphics(fig, fullfile(figDir, 'step11_ekf_flight.png'), 'Resolution', 150);

%% Save
ekf2 = struct('Q', Q, 'Q_data', Q_data, 'camera', C, 'results', res, 't', t);
save(dataFile, 'ekf2', '-append');
fprintf('\nSaved ekf2 -> %s\n', dataFile);
