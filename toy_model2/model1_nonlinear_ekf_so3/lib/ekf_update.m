function [xhat, Pcov, K] = ekf_update(xhat, Pcov, r, H, Rn)
%EKF_UPDATE  Error-state EKF measurement update (correction).
%
%   K    = P H' (H P H' + Rn)^-1          Kalman gain
%   dx   = K r,   r = z (-) h(xhat)       error-state correction (innovation r)
%   xhat = xhat (+) dx                    inject (attitude: R <- R Exp(dtheta))
%   P    = (I - K H) P                    covariance shrinks
%
%   INPUT : xhat [18x1], Pcov [12x12], r [mx1] innovation z (-) h(xhat),
%           H [mx12] measurement Jacobian, Rn [mxm] measurement noise covariance
%   OUTPUT: xhat [18x1] corrected estimate, Pcov [12x12] corrected covariance,
%           K [12xm] Kalman gain
S    = H*Pcov*H' + Rn;
K    = (Pcov*H') / S;
dx   = K*r;
xhat = state_boxplus(xhat, dx);
Pcov = (eye(12) - K*H)*Pcov;
Pcov = (Pcov + Pcov')/2;
end
