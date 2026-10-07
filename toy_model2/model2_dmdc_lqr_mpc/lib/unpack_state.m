function [p, v, R, w] = unpack_state(x)
%UNPACK_STATE  Split the 18-number stored state into its physical parts.
%   INPUT : x [18x1] = [p(3); v(3); vec(R)(9); w(3)]
%   OUTPUT: p [3x1] position (world, ENU) [m]      v [3x1] velocity (world) [m/s]
%           R [3x3] attitude body->world  [-]      w [3x1] body angular rate [rad/s]
p = x(1:3);
v = x(4:6);
R = reshape(x(7:15), 3, 3);
w = x(16:18);
end
