function [Ad, Bd] = c2d_zoh(A, B, dt)
%C2D_ZOH  Exact zero-order-hold discretisation (no toolbox needed).
%   expm([A B; 0 0] dt) = [Ad Bd; 0 I]
%   INPUT : A [nxn], B [nxm] continuous model, dt [s]
%   OUTPUT: Ad [nxn], Bd [nxm] such that x_{k+1} = Ad x_k + Bd u_k
[n, m] = size(B);
E  = expm([A, B; zeros(m, n + m)]*dt);
Ad = E(1:n, 1:n);
Bd = E(1:n, n+1:end);
end
