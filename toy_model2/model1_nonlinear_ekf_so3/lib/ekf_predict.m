function [xhat, Pcov, F] = ekf_predict(xhat, Pcov, u, dt, P, Qd)
%EKF_PREDICT  Error-state EKF time update (prediction).
%
%   xhat_{k+1} = f_d(xhat_k, u_k)            nominal state, nonlinear model (RK4)
%   P_{k+1}    = F P_k F' + Q,   F = I + A dt   error covariance, linear model
%
%   INPUT : xhat [18x1] estimate at k,  Pcov [12x12] covariance at k,
%           u [4x1] applied thrust/torque,  dt [s],  P params,  Qd [12x12] process noise
%   OUTPUT: xhat [18x1] predicted estimate,  Pcov [12x12] predicted covariance,
%           F [12x12] discrete error-state transition matrix used
[A, ~] = quad_jacobians(xhat, u, P);   % Jacobian at the current estimate
F    = eye(12) + A*dt;                 % first-order discretisation of expm(A dt)
xhat = rk4_step(xhat, u, dt, P);       % propagate the nonlinear mean
Pcov = F*Pcov*F' + Qd;
Pcov = (Pcov + Pcov')/2;
end
