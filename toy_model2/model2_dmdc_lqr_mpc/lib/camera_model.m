function C = camera_model()
%CAMERA_MODEL  Downward-looking monocular camera and known ground markers.
%   The camera is fixed under the drone and looks straight down. It sees
%   markers printed on the floor (think AprilTags) whose world positions are
%   known. Each marker gives one pixel measurement (u, v).
%
%   OUTPUT: C struct
%     f [px] focal length, cx, cy [px] principal point, W, Hpx [px] image size
%     R_bc [3x3]  camera axes written in the body frame:
%                 x_c = body x (forward), y_c = -body y, z_c = -body z (down)
%     L [3 x nL]  marker positions in the world (ENU) [m], all on the floor z = 0
%     sigma_px [px] pixel noise,  sigma_gyro [rad/s] gyroscope noise
C.f   = 400;
C.cx  = 320;  C.cy = 240;
C.W   = 640;  C.Hpx = 480;
C.R_bc = diag([1, -1, -1]);
[gx, gy] = meshgrid(-2.5:0.5:2.5, -2.5:0.5:2.5);   % 11 x 11 grid, 0.5 m spacing
C.L = [gx(:)'; gy(:)'; zeros(1, numel(gx))];
C.sigma_px   = 1.0;
C.sigma_gyro = 0.01;
end
