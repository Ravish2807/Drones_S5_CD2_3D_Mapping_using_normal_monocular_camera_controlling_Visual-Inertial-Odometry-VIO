function R = so3_exp(phi)
%SO3_EXP  Exponential map so(3) -> SO(3) (Rodrigues' formula).
%   INPUT : phi [3x1]  rotation vector  (axis * angle)        [rad]
%   OUTPUT: R   [3x3]  rotation matrix, R'*R = I, det(R) = +1
th = norm(phi);
if th < 1e-10
    R = eye(3) + hat(phi);
    return
end
K = hat(phi/th);
R = eye(3) + sin(th)*K + (1 - cos(th))*(K*K);
end
