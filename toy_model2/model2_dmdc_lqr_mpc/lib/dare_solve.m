function [P, lam] = dare_solve(A, B, Q, R)
%DARE_SOLVE  Direct solution of the Discrete Algebraic Riccati Equation.
%
%   P = A'PA - A'PB (R + B'PB)^-1 B'PA + Q
%
%   Method (no toolbox): the optimal state/co-state pair obeys
%       [I  G ] [x_{k+1}]   [ A  0] [x_k]
%       [0  A'] [l_{k+1}] = [-Q  I] [l_k],      G = B R^-1 B',  l_k = P x_k.
%   The n stable generalised eigenvectors [X1; X2] (|lambda| < 1) span
%   {l = P x}, hence P = X2 X1^-1.
%
%   INPUT : A [nxn], B [nxm], Q [nxn] >= 0, R [mxm] > 0
%   OUTPUT: P [nxn] stabilising solution,  lam [nx1] closed-loop eigenvalues
n = size(A, 1);
G = B*(R\B');
L = [A, zeros(n); -Q, eye(n)];
M = [eye(n), G; zeros(n), A'];
[V, D] = eig(L, M);
lamAll = diag(D);
stable = abs(lamAll) < 1;
if nnz(stable) ~= n
    error('dare_solve: expected %d stable eigenvalues, found %d (system not stabilisable?)', ...
          n, nnz(stable));
end
X1 = V(1:n, stable); X2 = V(n+1:end, stable);
P = real(X2/X1);
P = (P + P')/2;
lam = lamAll(stable);
end
