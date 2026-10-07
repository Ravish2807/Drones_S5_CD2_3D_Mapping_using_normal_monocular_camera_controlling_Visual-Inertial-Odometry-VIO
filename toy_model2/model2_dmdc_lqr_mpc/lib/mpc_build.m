function mpc = mpc_build(A, B, Q, R, Pf, N, u_min, u_max, C, x_max)
%MPC_BUILD  Condensed (stacked) quadratic program of a linear MPC problem.
%
%   Prediction:  X = Sx x0 + Su U,   X = [x_1; ...; x_N],  U = [u_0; ...; u_{N-1}]
%   Cost:        sum_{k=0}^{N-1} (x_k'Qx_k + u_k'Ru_k) + x_N'Pf x_N
%              = 1/2 U'HU + f'U + const,   H = 2(Su'Qbar Su + Rbar),  f = F x0,
%                F = 2 Su'Qbar Sx
%   Constraints: u_min <= u_k <= u_max,   |C x_k| <= x_max  (k = 1..N)
%              -> G U <= h0 + E x0
%
%   INPUT : A [nxn], B [nxm], Q, R, Pf (terminal weight), N horizon,
%           u_min, u_max [mx1], C [cxn] constrained outputs, x_max [cx1] (C = [] for none)
%   OUTPUT: mpc struct with Sx, Su, Qbar, Rbar, H, F, G, h0, E and all inputs
[n, m] = size(B);
Sx = zeros(n*N, n); Su = zeros(n*N, m*N);
Ai = eye(n);
for i = 1:N
    Ai = A*Ai;                                     % A^i
    Sx((i-1)*n+1:i*n, :) = Ai;
    for j = 1:i
        Su((i-1)*n+1:i*n, (j-1)*m+1:j*m) = A^(i-j)*B;
    end
end
Qbar = blkdiag(kron(eye(N-1), Q), Pf);
Rbar = kron(eye(N), R);
H = 2*(Su'*Qbar*Su + Rbar);  H = (H + H')/2;
F = 2*Su'*Qbar*Sx;

Gu = [eye(m*N); -eye(m*N)];
hu = [repmat(u_max, N, 1); -repmat(u_min, N, 1)];
if isempty(C)
    G = Gu; h0 = hu; E = zeros(size(Gu, 1), n);
else
    Cbar = kron(eye(N), C);
    G  = [Gu; Cbar*Su; -Cbar*Su];
    h0 = [hu; repmat(x_max, N, 1); repmat(x_max, N, 1)];
    E  = [zeros(size(Gu, 1), n); -Cbar*Sx; Cbar*Sx];
end
mpc = struct('A', A, 'B', B, 'Q', Q, 'R', R, 'Pf', Pf, 'N', N, 'u_min', u_min, ...
    'u_max', u_max, 'C', C, 'x_max', x_max, 'Sx', Sx, 'Su', Su, 'Qbar', Qbar, ...
    'Rbar', Rbar, 'H', H, 'F', F, 'G', G, 'h0', h0, 'E', E, 'nInputRows', size(Gu, 1));
end
