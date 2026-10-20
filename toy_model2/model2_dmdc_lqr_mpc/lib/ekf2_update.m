function [xh, Pk, info] = ekf2_update(xh, Pk, z, zhat, H, Rn)
%EKF2_UPDATE  Model-2 EKF measurement update (nonlinear camera, linear gyro).
%
%   r  = z - h(xh)                         innovation (measured - expected)
%   S  = H P H' + Rn                       innovation covariance
%   K  = P H' S^-1                         Kalman gain
%   xh = xh + K r                          the 12-state is a plain vector here
%   P  = (I - K H) P (I - K H)' + K Rn K'  Joseph form (stays symmetric and positive)
%   (M2.23-M2.24; the docs write L for this gain and Sigma for P, because
%    Model 2 already uses K for the LQR gain and P for the value matrix)
%
%   INPUT : xh [12x1], Pk [12x12], z [mx1] measurement, zhat [mx1] = h(xh),
%           H [mx12] Jacobian of h at xh, Rn [mxm] measurement noise covariance
%   OUTPUT: xh, Pk corrected;  info.r innovation, info.nis = r' S^-1 r (consistency check)
r  = z - zhat;
S  = H*Pk*H' + Rn;
K  = (Pk*H')/S;
xh = xh + K*r;
IKH = eye(12) - K*H;
Pk = IKH*Pk*IKH' + K*Rn*K';
Pk = (Pk + Pk')/2;
info.r = r;
info.nis = r'*(S\r);
end
