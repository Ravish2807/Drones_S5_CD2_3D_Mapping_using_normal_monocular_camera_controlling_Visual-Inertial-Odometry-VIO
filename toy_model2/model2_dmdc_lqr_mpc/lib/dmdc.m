function [A, B, info] = dmdc(X, Xp, U)
%DMDC  Dynamic Mode Decomposition with control (least-squares system identification).
%
%   X  = [x_1 ... x_{N-1}],  Xp = [x_2 ... x_N],  U = [u_1 ... u_{N-1}]
%   [A B] = Xp * pinv([X; U])        ->   x_{k+1} ~ A x_k + B u_k
%
%   INPUT : X [n x M], Xp [n x M] (snapshots shifted by one sample), U [m x M]
%   OUTPUT: A [n x n], B [n x m]  discrete-time model
%           info.sv   singular values of Omega = [X; U]  (excitation check)
%           info.rank numerical rank of Omega (must be n + m)
%           info.fit  relative one-step residual ||Xp - A X - B U||_F / ||Xp||_F
n = size(X, 1);
Omega = [X; U];
G = Xp * pinv(Omega);
A = G(:, 1:n);
B = G(:, n+1:end);
info.sv   = svd(Omega);
info.rank = rank(Omega);
info.fit  = norm(Xp - A*X - B*U, 'fro') / norm(Xp, 'fro');
end
