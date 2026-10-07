function [e_p, e_v, a_d] = position_control(p_hat, v_hat, ref, P)
%POSITION_CONTROL  Outer loop: tracking errors and desired acceleration.
%
%   e_p = p_d - p_hat,   e_v = v_d - v_hat
%   a_d = pdd_d + Kp e_p + Kv e_v
%
%   INPUT : p_hat, v_hat [3x1] estimated position [m] / velocity [m/s]
%           ref struct: p [3x1] m, v [3x1] m/s, a [3x1] m/s^2  (desired trajectory)
%           P params (Kp, Kv)
%   OUTPUT: e_p [3x1] m,  e_v [3x1] m/s,  a_d [3x1] m/s^2
e_p = ref.p - p_hat;
e_v = ref.v - v_hat;
a_d = ref.a + P.Kp*e_p + P.Kv*e_v;
end
