clc;clear; close all;
Util.plot_setup();

run_name_all = {"mpc_random_ic_p_alter_match";
                          "game_2_agent_random_ic_p_alter10";};

result_folder = "results"; % no parallel results
% result_folder = "results_parallel"; % parallel results
% parallel results should use the raw data, if use the condensed data, it
% would not able to rerun the same run due to the random seed mismatch
% between parallel and non parallel
target_folder = fullfile(result_folder, "figures");
type_id = 1;
seed = 996; % 996 is the closest to the same initial condition, for non parallel run used in the paper

% the closet to the same initial condition, for parallel run, very similar to the one used in the paper.
% however, if you attempt to run this results, it would not be able to
% regenerate the results due to the random seed mismatch, should try to get
% the raw and plot rather than rerun. This is why the result is not used in
% the paper.
% seed = 864;

% used the following to find the run closest to nominal IC.
% for each run, the seed appears to be the same behavior.
% still better double check on different machine for parallel cases.
% run_name = run_name_all{1};
% result_root_folder = fullfile(result_folder, run_name);
% summary_file = fullfile(result_root_folder, "summary.mat");
% load(summary_file, "run_summary");
% all_norms = zeros(1, length(run_summary.x_ini_all(1, :, 1)));
% for idx = 1:length(run_summary.x_ini_all(1, :, 1))
%     all_norms(idx) = norm(run_summary.x_ini_all(:, idx, 1) - [-20, 4, -20, 4]', 2);
% end
% [~, seed] = min(all_norms);


for iter = 1:length(run_name_all)
    run_name = run_name_all{iter};
    result_root_folder = fullfile(result_folder, run_name);

    close all;
    % game.GenSummary.rerun_one_run(result_root_folder, type_id, seed, true);
    fig_name = run_name + "_"+ num2str(type_id) + "_" + sprintf("%03d", seed);
    save_name{1} = fig_name + "_frame";
    save_name{2} = fig_name + "_profile";
    game.GenSummary.plot_one_run(result_root_folder, type_id, seed, 2, target_folder, save_name);
end
