function [Pnew, K] = bellman_backup(A, B, Q, R, Pnext)
%BELLMAN_BACKUP  One application of the Bellman equation for a quadratic V.
%
%   V(x) = min_u [ x'Qx + u'Ru + V_next(Ax + Bu) ],   V_next(x) = x' Pnext x
%   minimiser:  u* = -K x,   K = (R + B'Pnext B)^-1 B'Pnext A
%   value:      V(x) = x' Pnew x,
%               Pnew = Q + A'Pnext A - A'Pnext B (R + B'Pnext B)^-1 B'Pnext A
%
%   INPUT : A, B model, Q [nxn] state weight, R [mxm] input weight, Pnext [nxn]
%   OUTPUT: Pnew [nxn] value matrix one step earlier, K [mxn] optimal gain
K    = lqr_gain(A, B, R, Pnext);
Pnew = Q + A'*Pnext*A - A'*Pnext*B*K;
Pnew = (Pnew + Pnew')/2;
end
