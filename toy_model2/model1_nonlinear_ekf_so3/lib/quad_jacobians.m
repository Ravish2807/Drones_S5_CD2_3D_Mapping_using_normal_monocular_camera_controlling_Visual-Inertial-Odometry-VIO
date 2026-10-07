function [A, B] = quad_jacobians(xhat, uhat, P)
%QUAD_JACOBIANS  Analytic Jacobians of f(x,u) in error-state coordinates.
%
%   dx = [dp; dv; dtheta; dw] (12x1),  du = [dT; dtau] (4x1),  R = Rhat*Exp(dtheta)
%
%        | 0  I        0                0                       |
%   A =  | 0  0  -(T/m) Rhat [e3]x      0                       |
%        | 0  0      -[w]x              I                       |
%        | 0  0        0        J^-1([J w]x - [w]x J)           |
%
%        | 0            0    |
%   B =  | Rhat e3/m    0    |
%        | 0            0    |
%        | 0          J^-1   |
%
%   INPUT : xhat [18x1] linearisation state,  uhat [4x1] linearisation input,  P params
%   OUTPUT: A [12x12] = df/dx,  B [12x4] = df/du,  both evaluated at (xhat, uhat)
[~, ~, Rh, wh] = unpack_state(xhat);
Th = uhat(1);
Z = zeros(3); I = eye(3);

A = [Z, I, Z,                          Z;
     Z, Z, -(Th/P.m)*Rh*hat(P.e3),     Z;
     Z, Z, -hat(wh),                   I;
     Z, Z, Z,  P.J \ (hat(P.J*wh) - hat(wh)*P.J)];

B = [zeros(3,1),     zeros(3);
     Rh*P.e3/P.m,    zeros(3);
     zeros(3,1),     zeros(3);
     zeros(3,1),     inv(P.J)];
end
