function run_all_model2()
%RUN_ALL_MODEL2  Runs Model 2 (flight data -> DMDc -> LQR / MPC -> drone) end to end.
%   Each step prints its INPUTS and OUTPUTS, saves its results to
%   results/model2_data.mat and its figures to figures/.
here = fileparts(mfilename('fullpath'));
steps = {'step1_flight_data', 'step2_dmdc', 'step3_bellman_cost', 'step4_lqr_gain', ...
         'step5_riccati_recursion', 'step6_dare', 'step7_mpc_problem', ...
         'step8_qp_formulation', 'step9_mpc_control_input', 'step10_apply_to_drone'};
for i = 1:numel(steps)
    run_step(fullfile(here, steps{i}));   % .mlx or .m, whichever exists
end
fprintf('\nModel 2 complete. Figures in %s\n', fullfile(here, 'figures'));
end

function run_step(file)
% Separate workspace per step, so each script's "clear" stays local.
run(file);
end
