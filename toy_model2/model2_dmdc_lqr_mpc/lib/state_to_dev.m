function xd = state_to_dev(x, p_ref)
%STATE_TO_DEV  Physical state -> 12-dimensional deviation from hover.
%   The rotation matrix (9 numbers, 3 degrees of freedom) is replaced by its
%   rotation vector theta = Log(R), so the state becomes a plain vector that
%   DMDc / LQR / MPC can work with.
%
%   INPUT : x [18x1] = [p; v; vec(R); w],  p_ref [3x1] hover position [m]
%   OUTPUT: xd [12x1] = [p - p_ref (m); v (m/s); theta (rad); w (rad/s)]
[p, v, R, w] = unpack_state(x);
xd = [p - p_ref; v; so3_log(R); w];
end
