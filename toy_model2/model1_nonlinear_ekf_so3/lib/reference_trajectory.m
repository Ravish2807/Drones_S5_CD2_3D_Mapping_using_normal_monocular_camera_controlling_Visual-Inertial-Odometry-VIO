function ref = reference_trajectory(t, type)
%REFERENCE_TRAJECTORY  Desired motion for the outer loop.
%   'hover' : the controller.yaml setpoint p_d = [0;0;2], yaw 0
%   'helix' : circle of radius 1.5 m at 0.4 rad/s, climbing 5 cm/s from 2 m
%
%   INPUT : t [s], type (char)
%   OUTPUT: ref struct: p, v, a [3x1] (m, m/s, m/s^2), yaw [rad],
%           w [3x1] desired body rate, wdot [3x1] (zero: slow trajectory)
if nargin < 2, type = 'helix'; end
switch type
    case 'hover'
        ref.p = [0; 0; 2]; ref.v = zeros(3,1); ref.a = zeros(3,1);
    case 'helix'
        r = 1.5; W = 0.4; vz = 0.05;
        ref.p = [r*sin(W*t);        r*(1 - cos(W*t));   2 + vz*t];
        ref.v = [r*W*cos(W*t);      r*W*sin(W*t);       vz];
        ref.a = [-r*W^2*sin(W*t);   r*W^2*cos(W*t);     0];
    otherwise
        error('unknown reference type %s', type);
end
ref.yaw  = 0;
ref.w    = zeros(3,1);
ref.wdot = zeros(3,1);
end
