function xdot = quad_dynamics(x, u, P, d)
%QUAD_DYNAMICS  Continuous-time nonlinear rigid-body model  xdot = f(x,u).
%
%       p_dot = v
%       v_dot = g + (T/m) R e3            (+ d_F/m   wind force)
%       R_dot = R [w]x
%       w_dot = J^-1 (tau - w x J w)      (+ J^-1 d_tau)
%
%   INPUT : x [18x1]  state [p; v; vec(R); w]
%           u [4x1]   input [T (N); tau (N m, body)]
%           P struct  from QUAD_PARAMS
%           d [6x1]   (optional) disturbance [force world (N); torque body (N m)]
%   OUTPUT: xdot [18x1] time derivative of the stored state
if nargin < 4, d = zeros(6,1); end
[~, v, R, w] = unpack_state(x);
T   = u(1);
tau = u(2:4);

p_dot = v;
v_dot = P.g + (T/P.m)*R*P.e3 + d(1:3)/P.m;
R_dot = R*hat(w);
w_dot = P.J \ (tau - cross(w, P.J*w) + d(4:6));

xdot = [p_dot; v_dot; R_dot(:); w_dot];
end
