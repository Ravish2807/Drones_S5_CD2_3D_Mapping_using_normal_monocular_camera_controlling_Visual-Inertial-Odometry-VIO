function P = quad_params()
%QUAD_PARAMS  Physical, controller and estimator parameters of the toy drone.
%   The mass, inertia, gains and setpoint are copied from the project file
%   drones_controller/config/controller.yaml so that this toy model behaves
%   like the ROS 2 controller. Rotor constants are those of the ArduPilot
%   Gazebo "Iris" quadrotor (also 1.5 kg), the model used in our SITL setup.
%
%   OUTPUT: P  struct with fields (dimension, unit):
%     m [1x1, kg], g0 [1x1, m/s^2], g [3x1, m/s^2] (ENU, points down)
%     J [3x3, kg m^2], e3 [3x1]
%     Kp, Kv [3x3, 1/s^2, 1/s]   KR [3x3, N m], Kw [3x3, N m s]
%     rotor_xy [2x4, m], kappa [1x4], kf [N/(rad/s)^2], cm [m]
%     Gamma [4x4] allocation matrix, Omega_min/max [rad/s]

% ---- rigid body (controller.yaml: mass, gravity, inertia) ----
P.m  = 1.5;
P.g0 = 9.81;
P.g  = [0; 0; -P.g0];          % ENU world frame: gravity points to -z
P.J  = diag([0.02, 0.02, 0.04]);
P.e3 = [0; 0; 1];

% ---- controller gains (controller.yaml: kp, kv, kR, kOmega) ----
P.Kp = diag([1.5, 1.5, 2.0]);
P.Kv = diag([1.2, 1.2, 1.5]);
P.KR = diag([4.0, 4.0, 2.0]);
P.Kw = diag([0.8, 0.8, 0.5]);

% ---- setpoint (controller.yaml: desired_x/y/z/yaw) ----
P.p_hover = [0; 0; 2.0];
P.yaw_hover = 0.0;

% ---- rotors: ArduPilot Quad-X numbering, body frame FLU ----
%   1 front-right CCW, 2 back-left CCW, 3 front-left CW, 4 back-right CW
P.rotor_xy = [ 0.13, -0.13,  0.13, -0.13;     % x_i [m]
              -0.22,  0.22,  0.22, -0.22];    % y_i [m]
P.kappa    = [-1, -1, +1, +1];                % yaw reaction sign (CCW -> -1)
P.kf = 8.54858e-6;                            % thrust f_i = kf*Omega_i^2
P.cm = 0.016;                                 % yaw torque = kappa*cm*f_i
P.Omega_min = 100;                            % idle speed  [rad/s]
P.Omega_max = 1100;                           % max speed   [rad/s]

% [T; tau_x; tau_y; tau_z] = Gamma * [f1; f2; f3; f4]
P.Gamma = [ones(1,4);
           P.rotor_xy(2,:);        % tau_x =  sum y_i f_i
          -P.rotor_xy(1,:);        % tau_y = -sum x_i f_i
           P.kappa*P.cm];          % tau_z =  sum kappa_i cm f_i
end
