function x = state_boxplus(xhat, dx)
%STATE_BOXPLUS  Inject an error state:  x = xhat (+) dx.
%   INPUT : xhat [18x1] stored state, dx [12x1] = [dp; dv; dtheta; dw]
%   OUTPUT: x [18x1] with p+dp, v+dv, R*Exp(dtheta), w+dw
[p, v, R, w] = unpack_state(xhat);
x = pack_state(p + dx(1:3), v + dx(4:6), R*so3_exp(dx(7:9)), w + dx(10:12));
end
