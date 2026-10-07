function [F_d, b3d, R_d] = force_attitude(a_d, yaw_d, P)
%FORCE_ATTITUDE  Desired force vector and desired attitude R_d.
%
%   F_d = m (a_d - g)                    (g = [0;0;-g0] in ENU)
%   b3d = F_d / ||F_d||                  desired body z-axis (thrust direction)
%   b1c = [cos psi; sin psi; 0]          heading from desired yaw
%   b2d = (b3d x b1c)/||b3d x b1c||,  b1d = b2d x b3d
%   R_d = [b1d  b2d  b3d]
%
%   INPUT : a_d [3x1] desired acceleration [m/s^2], yaw_d [rad], P params
%   OUTPUT: F_d [3x1] N, b3d [3x1] unit vector, R_d [3x3] rotation matrix
F_d = P.m*(a_d - P.g);
nF  = norm(F_d);
if nF < 1e-6
    b3d = P.e3;          % free fall commanded: keep level
else
    b3d = F_d/nF;
end
b1c = [cos(yaw_d); sin(yaw_d); 0];
b2d = cross(b3d, b1c);
if norm(b2d) < 1e-6      % thrust parallel to heading: perturb yaw slightly
    b1c = [cos(yaw_d + 1e-3); sin(yaw_d + 1e-3); 0];
    b2d = cross(b3d, b1c);
end
b2d = b2d/norm(b2d);
b1d = cross(b2d, b3d);
R_d = [b1d, b2d, b3d];
end
