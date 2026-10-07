function x = rk4_step(x, u, dt, P, d)
%RK4_STEP  One 4th-order Runge-Kutta step of QUAD_DYNAMICS, input held constant.
%   INPUT : x [18x1] state at t,  u [4x1] input,  dt [s],  P params,
%           d [6x1] (optional) disturbance held over the step
%   OUTPUT: x [18x1] state at t+dt  (R re-projected onto SO(3))
if nargin < 5, d = zeros(6,1); end
k1 = quad_dynamics(x,             u, P, d);
k2 = quad_dynamics(x + dt/2*k1,   u, P, d);
k3 = quad_dynamics(x + dt/2*k2,   u, P, d);
k4 = quad_dynamics(x + dt*k3,     u, P, d);
x  = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
% keep R a proper rotation (removes integration drift)
[U, ~, V] = svd(reshape(x(7:15), 3, 3));
x(7:15) = reshape(U*diag([1, 1, det(U*V')])*V', 9, 1);
end
