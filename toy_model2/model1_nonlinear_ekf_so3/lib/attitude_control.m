function [e_R, e_w, tau, T] = attitude_control(R_hat, w_hat, R_d, w_d, wdot_d, F_d, P)
%ATTITUDE_CONTROL  Inner loop: SO(3) attitude error, torque and thrust.
%
%   e_R = 1/2 (R_d' R - R' R_d)^vee
%   e_w = w - R' R_d w_d                      (sign as in so3_controller.py, Lee 2010)
%   tau = -KR e_R - Kw e_w + w x J w - J( [w]x R' R_d w_d - R' R_d wdot_d )
%   T   = F_d . (R e3)
%
%   INPUT : R_hat [3x3], w_hat [3x1] rad/s   estimated attitude and body rate
%           R_d [3x3], w_d [3x1] rad/s, wdot_d [3x1] rad/s^2   desired attitude motion
%           F_d [3x1] N desired force,  P params (J, KR, Kw)
%   OUTPUT: e_R [3x1] (~rad), e_w [3x1] rad/s, tau [3x1] N m, T [1x1] N
Rrel = R_hat'*R_d;
e_R  = 0.5*vee(R_d'*R_hat - R_hat'*R_d);
e_w  = w_hat - Rrel*w_d;
tau  = -P.KR*e_R - P.Kw*e_w + cross(w_hat, P.J*w_hat) ...
       - P.J*(hat(w_hat)*Rrel*w_d - Rrel*wdot_d);
T    = dot(F_d, R_hat*P.e3);
end
