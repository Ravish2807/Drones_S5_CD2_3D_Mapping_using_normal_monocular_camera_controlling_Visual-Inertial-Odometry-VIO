function run_all_model1()
%RUN_ALL_MODEL1  Runs Model 1 (nonlinear dynamics -> EKF -> SO(3) control) end to end.
%   Each step prints its INPUTS and OUTPUTS, saves its results to
%   results/model1_data.mat and its figures to figures/.
here = fileparts(mfilename('fullpath'));
steps = {'step1_nonlinear_dynamics', 'step2_taylor_jacobian', 'step3_local_error_dynamics', ...
         'step4_error_state_ekf', 'step5_position_control', 'step6_force_attitude', ...
         'step7_attitude_control', 'step8_actuators_feedback'};
for i = 1:numel(steps)
    run_step(fullfile(here, steps{i}));   % .mlx or .m, whichever exists
end
fprintf('\nModel 1 complete. Figures in %s\n', fullfile(here, 'figures'));
end

function run_step(file)
% Separate workspace per step, so each script's "clear" stays local.
run(file);
end
