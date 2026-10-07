classdef DroneLib
%DRONELIB  All helper functions used by the toy-model live scripts.
%   Call them as DroneLib.name(...), e.g.  R = DroneLib.expSO3([0;0;0.1]).
%   The live scripts show every equation step by step; this file only holds
%   the functions that are re-used in many places.
%
%   State  x = [p(3); v(3); R(:)(9); w(3)]   (18 numbers, 12 degrees of freedom)
%   Input  u = [T; tau(3)]                   (N, N m)
%   World frame ENU (z up), body frame FLU, as in drones_controller.

methods (Static)

%% ---------------------------------------------------------------- parameters
function P = params()
% Values copied from drones_controller/config/controller.yaml.
% Rotor constants are those of the ArduPilot Gazebo Iris (also 1.5 kg).
P.m  = 1.5;                         % mass                 [kg]
P.g0 = 9.81;                        % gravity              [m/s^2]
P.g  = [0; 0; -P.g0];               % gravity vector (ENU) [m/s^2]
P.J  = diag([0.02 0.02 0.04]);      % inertia              [kg m^2]
P.e3 = [0; 0; 1];
P.Kp = diag([1.5 1.5 2.0]);         % kp
P.Kv = diag([1.2 1.2 1.5]);         % kv
P.KR = diag([4.0 4.0 2.0]);         % kR
P.Kw = diag([0.8 0.8 0.5]);         % kOmega
P.p_hover = [0; 0; 2];              % desired_x, desired_y, desired_z
% rotors (ArduPilot Quad-X): 1 front-right CCW, 2 back-left CCW,
%                            3 front-left CW,   4 back-right CW
P.rotor_xy = [ 0.13 -0.13  0.13 -0.13;      % x_i [m]
              -0.22  0.22  0.22 -0.22];     % y_i [m]
P.kappa = [-1 -1 1 1];                      % spin direction
P.kf = 8.54858e-6;                          % f_i = kf*Omega_i^2   [N s^2]
P.cm = 0.016;                               % yaw torque per newton [m]
P.Omega_min = 100;  P.Omega_max = 1100;     % motor speed limits [rad/s]
P.Gamma = [ones(1,4); P.rotor_xy(2,:); -P.rotor_xy(1,:); P.kappa*P.cm];
end

%% ---------------------------------------------------------------- SO(3) tools
function S = hat(w)
% hat(w)*b = cross(w,b)
S = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
end

function w = vee(S)
w = [S(3,2); S(1,3); S(2,1)];
end

function R = expSO3(phi)
% rotation vector (axis*angle) -> rotation matrix (Rodrigues)
th = norm(phi);
if th < 1e-10, R = eye(3) + DroneLib.hat(phi); return, end
K = DroneLib.hat(phi/th);
R = eye(3) + sin(th)*K + (1 - cos(th))*K*K;
end

function phi = logSO3(R)
% rotation matrix -> rotation vector (inverse of expSO3)
c  = max(-1, min(1, (trace(R) - 1)/2));
th = acos(c);
if th < 1e-8
    phi = DroneLib.vee(R - R')/2;
else
    phi = th/(2*sin(th))*DroneLib.vee(R - R');
end
end

%% ---------------------------------------------------------------- dynamics
function x = pack(p, v, R, w)
x = [p(:); v(:); R(:); w(:)];
end

function [p, v, R, w] = unpack(x)
p = x(1:3); v = x(4:6); R = reshape(x(7:15), 3, 3); w = x(16:18);
end

function xdot = f(x, u, P, d)
% p_dot = v,  v_dot = g + T/m R e3,  R_dot = R [w]x,  w_dot = J^-1 (tau - w x Jw)
% d = [wind force (world, N); disturbance torque (body, N m)]
if nargin < 4, d = zeros(6,1); end
[~, v, R, w] = DroneLib.unpack(x);
v_dot = P.g + u(1)/P.m*R*P.e3 + d(1:3)/P.m;
R_dot = R*DroneLib.hat(w);
w_dot = P.J \ (u(2:4) - cross(w, P.J*w) + d(4:6));
xdot  = [v; v_dot; R_dot(:); w_dot];
end

function x = rk4(x, u, dt, P, d)
% one Runge-Kutta-4 step with the input held constant
if nargin < 5, d = zeros(6,1); end
k1 = DroneLib.f(x, u, P, d);
k2 = DroneLib.f(x + dt/2*k1, u, P, d);
k3 = DroneLib.f(x + dt/2*k2, u, P, d);
k4 = DroneLib.f(x + dt*k3, u, P, d);
x  = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
[U, ~, V] = svd(reshape(x(7:15), 3, 3));          % keep R a rotation
x(7:15) = reshape(U*diag([1 1 det(U*V')])*V', 9, 1);
end

function [A, B] = jacobians(x, u, P)
% A = df/dx, B = df/du in error coordinates dx = [dp; dv; dtheta; dw]
[~, ~, R, w] = DroneLib.unpack(x);
Z = zeros(3); I = eye(3);
A = [Z I Z Z;
     Z Z -(u(1)/P.m)*R*DroneLib.hat(P.e3) Z;
     Z Z -DroneLib.hat(w) I;
     Z Z Z P.J\(DroneLib.hat(P.J*w) - DroneLib.hat(w)*P.J)];
B = [zeros(3,4);
     R*P.e3/P.m, zeros(3);
     zeros(3,4);
     zeros(3,1), inv(P.J)];
end

%% ---------------------------------------------------------------- reference
function ref = helix(t)
% climbing circle: radius 1.5 m, 0.4 rad/s, +5 cm/s, starting at [0,0,2]
r = 1.5; W = 0.4;
ref.p = [r*sin(W*t); r*(1 - cos(W*t)); 2 + 0.05*t];
ref.v = [r*W*cos(W*t); r*W*sin(W*t); 0.05];
ref.a = [-r*W^2*sin(W*t); r*W^2*cos(W*t); 0];
ref.yaw = 0; ref.w = zeros(3,1); ref.wdot = zeros(3,1);
end

%% ---------------------------------------------------------------- SO(3) controller
function [u, c] = so3_control(p, v, R, w, ref, P)
% Same equations as SO3Controller.compute() in so3_controller.py
c.e_p = ref.p - p;                                   % M1.15
c.e_v = ref.v - v;
c.a_d = ref.a + P.Kp*c.e_p + P.Kv*c.e_v;             % M1.16
c.F_d = P.m*(c.a_d - P.g);                           % M1.17
b3 = c.F_d/norm(c.F_d);                              % M1.18
b1c = [cos(ref.yaw); sin(ref.yaw); 0];
b2 = cross(b3, b1c); b2 = b2/norm(b2);               % M1.19
c.R_d = [cross(b2, b3), b2, b3];
Rrel = R'*c.R_d;
c.e_R = 0.5*DroneLib.vee(c.R_d'*R - R'*c.R_d);       % M1.20
c.e_w = w - Rrel*ref.w;                              % M1.21
tau = -P.KR*c.e_R - P.Kw*c.e_w + cross(w, P.J*w) ... % M1.22
      - P.J*(DroneLib.hat(w)*Rrel*ref.w - Rrel*ref.wdot);
T = dot(c.F_d, R*P.e3);                              % M1.23
u = [T; tau];
end

function [Omega, fr, u_app] = mixer(u, P)
% [T; tau] -> rotor thrusts -> motor speeds (clipped)      M1.24-M1.25
fr = P.Gamma \ u;
fr = min(max(fr, P.kf*P.Omega_min^2), P.kf*P.Omega_max^2);
Omega = sqrt(fr/P.kf);
u_app = P.Gamma*fr;                                  % what the rotors really give
end

%% ---------------------------------------------------------------- error-state EKF
function x = boxplus(x, dx)
[p, v, R, w] = DroneLib.unpack(x);
x = DroneLib.pack(p + dx(1:3), v + dx(4:6), R*DroneLib.expSO3(dx(7:9)), w + dx(10:12));
end

function dx = boxminus(x, xh)
[p, v, R, w] = DroneLib.unpack(x);
[ph, vh, Rh, wh] = DroneLib.unpack(xh);
dx = [p - ph; v - vh; DroneLib.logSO3(Rh'*R); w - wh];
end

function [xh, Pk] = ekf_predict(xh, Pk, u, dt, P, Q)
A  = DroneLib.jacobians(xh, u, P);
F  = eye(12) + A*dt;                                 % M1.9
xh = DroneLib.rk4(xh, u, dt, P);                     % M1.10
Pk = F*Pk*F' + Q;  Pk = (Pk + Pk')/2;                % M1.11
end

function [xh, Pk] = ekf_update(xh, Pk, r, H, Rn)
K  = Pk*H' / (H*Pk*H' + Rn);                         % M1.12
xh = DroneLib.boxplus(xh, K*r);                      % M1.13
Pk = (eye(12) - K*H)*Pk;  Pk = (Pk + Pk')/2;         % M1.14
end

%% ---------------------------------------------------------------- Model 2 tools
function xd = to_dev(x, p_ref)
% 18-number state -> 12-vector deviation from hover   (M2.1)
[p, v, R, w] = DroneLib.unpack(x);
xd = [p - p_ref; v; DroneLib.logSO3(R); w];
end

function du = pilot(xd, P)
% simple PD stabiliser used only while recording data (the role ArduPilot plays)
a = -1.0*xd(1:3) - 1.5*xd(4:6);
th_d = [-a(2)/P.g0; a(1)/P.g0; 0];
du = [P.m*a(3); P.J*(-36*(xd(7:9) - th_d) - 10*xd(10:12))];
end

function P = dare(A, B, Q, R)
% Discrete algebraic Riccati equation without toolboxes (stable eigenvectors)
n = size(A,1);
L = [A, zeros(n); -Q, eye(n)];
M = [eye(n), B*(R\B'); zeros(n), A'];
[V, D] = eig(L, M);
s = abs(diag(D)) < 1;
P = real(V(n+1:end, s)/V(1:n, s));
P = (P + P')/2;
end

function mpc = mpc_build(A, B, Q, R, Pf, N, umin, umax, C, xmax)
% stacked (condensed) QP of the MPC problem      M2.13-M2.15
[n, m] = size(B);
Sx = zeros(n*N, n); Su = zeros(n*N, m*N);
for i = 1:N
    Sx((i-1)*n+1:i*n, :) = A^i;
    for j = 1:i
        Su((i-1)*n+1:i*n, (j-1)*m+1:j*m) = A^(i-j)*B;
    end
end
Qbar = blkdiag(kron(eye(N-1), Q), Pf);
Rbar = kron(eye(N), R);
mpc.H = 2*(Su'*Qbar*Su + Rbar);  mpc.H = (mpc.H + mpc.H')/2;
mpc.F = 2*Su'*Qbar*Sx;                         % f = F*x0
Cb = kron(eye(N), C);
mpc.G  = [eye(m*N); -eye(m*N); Cb*Su; -Cb*Su];
mpc.h0 = [repmat(umax,N,1); -repmat(umin,N,1); repmat(xmax,N,1); repmat(xmax,N,1)];
mpc.E  = [zeros(2*m*N, n); -Cb*Sx; Cb*Sx];     % h = h0 + E*x0
mpc.m = m; mpc.nu = 2*m*N; mpc.Sx = Sx; mpc.Su = Su;
mpc.umin = umin; mpc.umax = umax;
end

function [u0, U] = mpc_solve(mpc, x0)
% solve the QP for the current state, return the first move   (M2.16)
[U, ok] = DroneLib.qp(mpc.H, mpc.F*x0, mpc.G, mpc.h0 + mpc.E*x0);
if ~ok   % state limits impossible from x0 (e.g. after a gust): keep input limits only
    r = 1:mpc.nu;
    U = DroneLib.qp(mpc.H, mpc.F*x0, mpc.G(r,:), mpc.h0(r));
end
u0 = U(1:mpc.m);
end

function [x, ok, it] = qp(H, f, G, h, Aeq, beq)
% min 1/2 x'Hx + f'x  s.t. Gx <= h, Aeq x = beq   (primal-dual interior point)
n = numel(f);
if nargin < 5, Aeq = zeros(0,n); beq = zeros(0,1); end
q = size(Aeq,1); mI = size(G,1);
x = zeros(n,1); y = zeros(q,1); s = max(h - G*x, 1); z = ones(mI,1); ok = false;
for it = 1:80
    rd = H*x + f + G'*z + Aeq'*y;  re = Aeq*x - beq;  ri = G*x + s - h;  mu = s'*z/mI;
    if norm(rd,inf) < 1e-9*(1+norm(f,inf)) && norm(re,inf) < 1e-9 && ...
       norm(ri,inf) < 1e-9*(1+norm(h,inf)) && mu < 1e-9
        ok = true; break
    end
    KKT = [H + G'*((z./s).*G), Aeq'; Aeq, zeros(q)];
    [dx, dy, dz, ds] = DroneLib.newton(KKT, -s.*z, G, rd, re, ri, s, z, n);
    a = DroneLib.step_len(s, ds, z, dz);
    sig = (((s + a*ds)'*(z + a*dz))/mI/mu)^3;
    [dx, dy, dz, ds] = DroneLib.newton(KKT, -s.*z + sig*mu - ds.*dz, G, rd, re, ri, s, z, n);
    a = min(1, 0.99*DroneLib.step_len(s, ds, z, dz));
    x = x + a*dx; y = y + a*dy; z = z + a*dz; s = s + a*ds;
end
end

function [dx, dy, dz, ds] = newton(KKT, rc, G, rd, re, ri, s, z, n)
sol = KKT \ [-rd - G'*((rc + z.*ri)./s); -re];
dx = sol(1:n); dy = sol(n+1:end);
ds = -ri - G*dx;  dz = (rc - z.*ds)./s;
end

function a = step_len(s, ds, z, dz)
a = 1;
i = ds < 0; if any(i), a = min(a, min(-s(i)./ds(i))); end
i = dz < 0; if any(i), a = min(a, min(-z(i)./dz(i))); end
end

end
end
