function [P, hist] = riccati_recursion(A, B, Q, R, P0, maxIt, tol)
%RICCATI_RECURSION  Iterate the discrete Riccati difference equation to steady state.
%
%   P_{k+1} = A'P_k A - A'P_k B (R + B'P_k B)^-1 B'P_k A + Q
%
%   (k counts "steps-to-go": P_k is the value matrix of a k-step horizon problem)
%
%   INPUT : A, B, Q, R, P0 [nxn] start (terminal weight), maxIt, tol (relative change)
%   OUTPUT: P [nxn] converged matrix, hist.change(k) = ||P_k - P_{k-1}||/||P_k||,
%           hist.trace(k), hist.iterations
if nargin < 6, maxIt = 10000; end
if nargin < 7, tol = 1e-12; end
P = P0;
hist.change = zeros(1, maxIt); hist.trace = zeros(1, maxIt);
for k = 1:maxIt
    Pn = bellman_backup(A, B, Q, R, P);
    hist.change(k) = norm(Pn - P, 'fro') / norm(Pn, 'fro');
    hist.trace(k)  = trace(Pn);
    P = Pn;
    if hist.change(k) < tol, break, end
end
hist.change = hist.change(1:k); hist.trace = hist.trace(1:k); hist.iterations = k;
end
