function dx = state_boxminus(x, xhat)
%STATE_BOXMINUS  Error state  dx = x (-) xhat  on R^9 x SO(3).
%   The attitude error uses the right (body) perturbation R = Rhat*Exp(dtheta).
%   INPUT : x, xhat [18x1] stored states
%   OUTPUT: dx [12x1] = [dp; dv; dtheta; dw]
[p,  v,  R,  w ] = unpack_state(x);
[ph, vh, Rh, wh] = unpack_state(xhat);
dx = [p - ph; v - vh; so3_log(Rh'*R); w - wh];
end
