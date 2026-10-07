function [x, info] = qp_solve(H, f, G, h, Aeq, beq)
%QP_SOLVE  Convex quadratic program, primal-dual interior point (Mehrotra).
%
%   min_x  1/2 x'Hx + f'x    s.t.   G x <= h,   Aeq x = beq
%
%   A self-contained replacement for quadprog (Optimization Toolbox is not
%   required). Slack s >= 0 turns G x <= h into G x + s = h; the KKT system
%       H x + f + G'z + Aeq'y = 0,  Aeq x = beq,  G x + s = h,  s.*z = 0
%   is solved by Newton steps that keep (s, z) > 0.
%
%   INPUT : H [nxn] >= 0, f [nx1], G [pxn], h [px1], Aeq [qxn], beq [qx1] (optional)
%   OUTPUT: x [nx1] minimiser,  info.converged, info.iterations, info.z (multipliers)
n = numel(f);
if nargin < 5 || isempty(Aeq), Aeq = zeros(0, n); beq = zeros(0, 1); end
if isempty(G),  G = zeros(0, n); h = zeros(0, 1); end
m = size(G, 1); q = size(Aeq, 1);

if m == 0                                          % equality constraints only
    sol = [H, Aeq'; Aeq, zeros(q)] \ [-f; beq];
    x = sol(1:n); info = struct('converged', true, 'iterations', 1, 'z', zeros(0,1));
    return
end

x = zeros(n, 1); y = zeros(q, 1);
s = max(h - G*x, 1); z = ones(m, 1);
tol = 1e-9; info.converged = false;
for it = 1:80
    rd = H*x + f + G'*z + Aeq'*y;                 % dual residual
    re = Aeq*x - beq;                              % equality residual
    ri = G*x + s - h;                              % inequality residual
    mu = (s'*z)/m;                                 % duality gap
    if norm(rd, inf) < tol*(1 + norm(f, inf)) && norm(re, inf) < tol && ...
       norm(ri, inf) < tol*(1 + norm(h, inf)) && mu < tol
        info.converged = true; break
    end
    D = z./s;
    KKT = [H + G'*(D.*G), Aeq'; Aeq, zeros(q)];
    [Lf, Uf, Pf] = lu(KKT);
    newton = @(rc) newton_step(rc, Lf, Uf, Pf, G, rd, re, ri, s, z, n);

    % predictor (affine scaling) step
    [dx, dy, dz, ds] = newton(-s.*z);
    a  = max_step(s, ds, z, dz);
    sigma = (((s + a*ds)'*(z + a*dz))/m / mu)^3;
    % corrector step
    [dx, dy, dz, ds] = newton(-s.*z + sigma*mu - ds.*dz);
    a = min(1, 0.99*max_step(s, ds, z, dz));
    x = x + a*dx; y = y + a*dy; z = z + a*dz; s = s + a*ds;
end
info.iterations = it; info.z = z;
end

function [dx, dy, dz, ds] = newton_step(rc, Lf, Uf, Pf, G, rd, re, ri, s, z, n)
rhs = [-rd - G'*((rc + z.*ri)./s); -re];
sol = Uf \ (Lf \ (Pf*rhs));
dx = sol(1:n); dy = sol(n+1:end);
ds = -ri - G*dx;
dz = (rc - z.*ds)./s;
end

function a = max_step(s, ds, z, dz)
a = 1;
i = ds < 0; if any(i), a = min(a, min(-s(i)./ds(i))); end
i = dz < 0; if any(i), a = min(a, min(-z(i)./dz(i))); end
end
