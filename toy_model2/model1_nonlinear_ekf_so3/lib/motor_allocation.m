function [Omega, f, u_applied] = motor_allocation(u, P)
%MOTOR_ALLOCATION  Thrust + torque command -> individual motor speeds.
%
%   f         = Gamma^-1 [T; tau]       per-rotor thrusts             [N]
%   Omega     = sqrt(f / kf)            rotor speeds (saturated)      [rad/s]
%   u_applied = Gamma * kf * Omega.^2   wrench the rotors really make [N, N m]
%
%   INPUT : u [4x1] = [T; tau_x; tau_y; tau_z]  (N, N m),  P params
%   OUTPUT: Omega [4x1] rad/s, f [4x1] N (after saturation), u_applied [4x1]
f_cmd = P.Gamma \ u;
f_min = P.kf*P.Omega_min^2;
f_max = P.kf*P.Omega_max^2;
f     = min(max(f_cmd, f_min), f_max);
Omega = sqrt(f/P.kf);
u_applied = P.Gamma*f;
end
