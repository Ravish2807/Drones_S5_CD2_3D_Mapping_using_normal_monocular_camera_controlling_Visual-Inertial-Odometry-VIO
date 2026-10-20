function [xh, Pk] = ekf2_predict(xh, Pk, u, A, B, Q)
%EKF2_PREDICT  Model-2 EKF time update with the LEARNED (DMDc) model.
%
%   xh_{k+1} = A xh_k + B u_k
%   P_{k+1}  = A P_k A' + Q        (the model is already linear, so F = A)        (M2.19)
%   (the docs write Sigma for this covariance and Q_e for this process noise)
%
%   INPUT : xh [12x1] estimate, Pk [12x12] covariance, u [4x1] applied input
%           (deviation from hover), A [12x12], B [12x4] from DMDc, Q [12x12] process noise
%   OUTPUT: xh [12x1], Pk [12x12] predicted
xh = A*xh + B*u;
Pk = A*Pk*A' + Q;
Pk = (Pk + Pk')/2;
end
