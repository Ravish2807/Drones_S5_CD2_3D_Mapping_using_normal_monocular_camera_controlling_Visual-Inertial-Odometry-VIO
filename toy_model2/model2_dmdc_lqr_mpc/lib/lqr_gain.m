function K = lqr_gain(A, B, R, P)
%LQR_GAIN  Optimal feedback gain for a given quadratic value function V(x) = x'Px.
%
%   K = (R + B'PB)^-1 B'PA        u_k = -K x_k
%
%   INPUT : A [nxn], B [nxm], R [mxm] input weight, P [nxn] value matrix
%   OUTPUT: K [mxn]
K = (R + B'*P*B) \ (B'*P*A);
end
