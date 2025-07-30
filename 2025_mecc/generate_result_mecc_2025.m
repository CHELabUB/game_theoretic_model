% set PARALLEL_PROCESS to false if want to reproduce the exact results in the paper
% set PARALLEL_PROCESS to true if you want to use parallel processing
% the random seed will be different if using parallel processing and may very among machines.
results_root = "results";
results_root_temp = "temp_raw_results";
PARALLEL_PROCESS = false;

% results_root = "results_parallel";
% results_root_temp = "temp_raw_results_parallel";
% PARALLEL_PROCESS = true;

%% 2 game
clc;close all;
clearvars -except results_root results_root_temp PARALLEL_PROCESS; 

P_ALTER = 1.0;
ADD_MEASUREMENT_NOISE = false;
ADD_ACTION_UNCERTAINTY = false;
run_name = "game_2_agent_random_ic_p_alter10";
RESULT_ROOT_FOLDER = results_root_temp + filesep + run_name;
game.run_state_dependent_game_intersection_game;
target_root_folder = results_root + filesep + run_name;
copy_list = {"figures", "summary.mat"};
Util.move_summary_results(RESULT_ROOT_FOLDER, target_root_folder, copy_list)

% % Should not change results
clc;close all;
clearvars -except results_root results_root_temp PARALLEL_PROCESS;

P_ALTER = 0.7;
ADD_MEASUREMENT_NOISE = false;
ADD_ACTION_UNCERTAINTY = false;
run_name = "game_2_agent_random_ic_p_alter07";
RESULT_ROOT_FOLDER = results_root_temp + filesep + run_name;
game.run_state_dependent_game_intersection_game;
target_root_folder = results_root + filesep + run_name;
copy_list = {"figures", "summary.mat"};
Util.move_summary_results(RESULT_ROOT_FOLDER, target_root_folder, copy_list)

% % game + MPC
clc;close all;
clearvars -except results_root results_root_temp PARALLEL_PROCESS;

P_ALTER = 1.0;
P_ALTER_MODEL = P_ALTER;
ADD_MEASUREMENT_NOISE = false;
ADD_ACTION_UNCERTAINTY = false;
run_name = "mpc_random_ic_p_alter_match";
RESULT_ROOT_FOLDER = results_root_temp + filesep + run_name;
game.run_state_dependent_game_intersection_mpc;
target_root_folder = results_root + filesep + run_name;
copy_list = {"figures", "summary.mat"};
Util.move_summary_results(RESULT_ROOT_FOLDER, target_root_folder, copy_list)

clc;close all;
clearvars -except results_root results_root_temp PARALLEL_PROCESS;
P_ALTER = 0.7;
P_ALTER_MODEL = 1.0;
ADD_MEASUREMENT_NOISE = false;
ADD_ACTION_UNCERTAINTY = false;
run_name = "mpc_random_ic_p_alter_overestimation";
RESULT_ROOT_FOLDER = results_root_temp + filesep + run_name;
game.run_state_dependent_game_intersection_mpc;
target_root_folder = results_root + filesep + run_name;
copy_list = {"figures", "summary.mat"};
Util.move_summary_results(RESULT_ROOT_FOLDER, target_root_folder, copy_list)

clc;close all;
clearvars -except results_root results_root_temp PARALLEL_PROCESS;
P_ALTER = 0.5;
P_ALTER_MODEL = 1.0;
ADD_MEASUREMENT_NOISE = false;
ADD_ACTION_UNCERTAINTY = false;
run_name = "mpc_random_ic_p_alter_overestimation_2";
RESULT_ROOT_FOLDER = results_root_temp + filesep + run_name;
game.run_state_dependent_game_intersection_mpc;
target_root_folder = results_root + filesep + run_name;
copy_list = {"figures", "summary.mat"};
Util.move_summary_results(RESULT_ROOT_FOLDER, target_root_folder, copy_list)

clc;close all;
clearvars -except results_root results_root_temp PARALLEL_PROCESS;
P_ALTER = 0.3;
P_ALTER_MODEL = 1.0;
ADD_MEASUREMENT_NOISE = false;
ADD_ACTION_UNCERTAINTY = false;
run_name = "mpc_random_ic_p_alter_overestimation_3";
RESULT_ROOT_FOLDER = results_root_temp + filesep + run_name;
game.run_state_dependent_game_intersection_mpc;
target_root_folder = results_root + filesep + run_name;
copy_list = {"figures", "summary.mat"};
Util.move_summary_results(RESULT_ROOT_FOLDER, target_root_folder, copy_list)

clc;close all;
clearvars -except results_root results_root_temp PARALLEL_PROCESS;
P_ALTER = 1.0;
P_ALTER_MODEL = 0.7;
ADD_MEASUREMENT_NOISE = false;
ADD_ACTION_UNCERTAINTY = false;
run_name = "mpc_random_ic_p_alter_underestimation";
RESULT_ROOT_FOLDER = results_root_temp + filesep + run_name;
game.run_state_dependent_game_intersection_mpc;
target_root_folder = results_root + filesep + run_name;
copy_list = {"figures", "summary.mat"};
Util.move_summary_results(RESULT_ROOT_FOLDER, target_root_folder, copy_list)

clc;close all;
clearvars -except results_root results_root_temp PARALLEL_PROCESS;
P_ALTER = 1.0;
P_ALTER_MODEL = 0.95;
ADD_MEASUREMENT_NOISE = false;
ADD_ACTION_UNCERTAINTY = false;
run_name = "mpc_random_ic_p_alter_underestimation_2";
RESULT_ROOT_FOLDER = results_root_temp + filesep + run_name;
game.run_state_dependent_game_intersection_mpc;
target_root_folder = results_root + filesep + run_name;
copy_list = {"figures", "summary.mat"};
Util.move_summary_results(RESULT_ROOT_FOLDER, target_root_folder, copy_list)

clc;close all;
clearvars -except results_root results_root_temp PARALLEL_PROCESS;
P_ALTER = 1.0;
P_ALTER_MODEL = 0.98;
ADD_MEASUREMENT_NOISE = false;
ADD_ACTION_UNCERTAINTY = false;
run_name = "mpc_random_ic_p_alter_underestimation_3";
RESULT_ROOT_FOLDER = results_root_temp + filesep + run_name;
game.run_state_dependent_game_intersection_mpc;
target_root_folder = results_root + filesep + run_name;
copy_list = {"figures", "summary.mat"};
Util.move_summary_results(RESULT_ROOT_FOLDER, target_root_folder, copy_list)
