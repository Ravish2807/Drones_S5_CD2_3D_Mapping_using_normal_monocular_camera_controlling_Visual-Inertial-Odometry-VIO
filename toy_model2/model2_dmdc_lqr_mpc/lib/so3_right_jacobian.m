function Jr = so3_right_jacobian(theta)
%SO3_RIGHT_JACOBIAN  Right Jacobian of SO(3):  Exp(theta + d) ~ Exp(theta) Exp(Jr(theta) d).
%
%   Jr = I - (1 - cos(phi))/phi^2 [theta]x + (phi - sin(phi))/phi^3 [theta]x^2,  phi = |theta|
%
%   INPUT : theta [3x1] rotation vector [rad]
%   OUTPUT: Jr [3x3]   (equals I - 1/2 [theta]x for small angles)
phi = norm(theta);
S = hat(theta);
if phi < 1e-6
    Jr = eye(3) - 0.5*S;
else
    Jr = eye(3) - (1 - cos(phi))/phi^2*S + (phi - sin(phi))/phi^3*(S*S);
end
end
