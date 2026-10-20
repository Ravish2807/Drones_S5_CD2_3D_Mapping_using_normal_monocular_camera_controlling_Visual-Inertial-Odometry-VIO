function [z, H, ids] = camera_measure(xd, p_ref, C, ids)
%CAMERA_MEASURE  Pinhole projection of the ground markers (M2.21) and its Jacobian (M2.22).
%
%   Marker l in the camera frame:  q = R_bc' R(theta)' (l - p),   p = p_ref + dp
%   Pixel (pinhole model):         pi(q) = [f q1/q3 + cx;  f q2/q3 + cy]
%   Jacobian w.r.t. the 12-state  x = [dp; v; theta; w]:
%       dq/ddp    = -R_bc' R'
%       dq/dtheta =  R_bc' [R'(l - p)]x Jr(theta)
%       dpi/dq    = (f/q3) [1 0 -q1/q3; 0 1 -q2/q3]
%
%   INPUT : xd [12x1] deviation state, p_ref [3x1] hover point, C camera_model,
%           ids (optional) marker indices to use; if omitted, every marker that is
%           in front of the camera and inside the image is used
%   OUTPUT: z [2m x 1] predicted pixels [px], H [2m x 12] Jacobian [px per state unit],
%           ids [1 x m] the markers used
p  = p_ref + xd(1:3);
th = xd(7:9);
R  = so3_exp(th);
Jr = so3_right_jacobian(th);
Rcw = C.R_bc'*R';                          % world -> camera rotation
if nargin < 4
    q   = Rcw*(C.L - p);
    pix = [C.f*q(1,:)./q(3,:) + C.cx; C.f*q(2,:)./q(3,:) + C.cy];
    ids = find(q(3,:) > 0.2 & pix(1,:) > 0 & pix(1,:) < C.W & pix(2,:) > 0 & pix(2,:) < C.Hpx);
end
m = numel(ids);
z = zeros(2*m, 1);  H = zeros(2*m, 12);
for i = 1:m
    a   = R'*(C.L(:, ids(i)) - p);         % marker seen from the drone, body frame
    q   = C.R_bc'*a;                       % ... and in the camera frame
    rows = 2*i-1:2*i;
    z(rows) = [C.f*q(1)/q(3) + C.cx; C.f*q(2)/q(3) + C.cy];
    dpi = (C.f/q(3))*[1 0 -q(1)/q(3); 0 1 -q(2)/q(3)];
    H(rows, 1:3) = dpi*(-Rcw);
    H(rows, 7:9) = dpi*(C.R_bc'*hat(a)*Jr);
end
end
