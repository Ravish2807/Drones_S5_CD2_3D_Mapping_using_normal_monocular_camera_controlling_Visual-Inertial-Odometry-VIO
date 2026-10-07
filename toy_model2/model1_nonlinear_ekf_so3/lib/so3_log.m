function phi = so3_log(R)
%SO3_LOG  Logarithm map SO(3) -> so(3), inverse of SO3_EXP.
%   INPUT : R   [3x3]  rotation matrix
%   OUTPUT: phi [3x1]  rotation vector (axis * angle), |phi| <= pi   [rad]
c  = max(-1, min(1, (trace(R) - 1)/2));
th = acos(c);
if th < 1e-8
    phi = vee(R - R')/2;
elseif pi - th < 1e-6
    % 180 deg rotation: axis is the eigenvector of R with eigenvalue +1
    [V, D] = eig(R);
    [~, i] = min(abs(diag(D) - 1));
    phi = th * real(V(:, i));
else
    phi = th/(2*sin(th)) * vee(R - R');
end
end
