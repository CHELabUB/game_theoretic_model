%% Put potentially Altered global parameter here
% clc;clear; close all;
% P_ALTER = 0.7;
% ADD_MEASUREMENT_NOISE = false;
% ADD_ACTION_UNCERTAINTY = false;
% RESULT_ROOT_FOLDER = "temp_data" + filesep + "game_2_agent_random_p_alter_ic";

% Plotter setup
Util.plot_setup();

%% Load Game parameters
game_param = game.default_game_param(P_ALTER);
%% Simulation setup
adapt_role = true;
% adapt_role = false;

%% Add some randomness
% note that for all noises
% if w is defined as matrix, will randomize as multivariate normal distribution
% if w is defined as vector, will randomize as multivariate uniform distribution
% the noise is added directly to the state of the simulation
% the first 4 belongs to eb (red, game, other), the second 4 belongs to nb (blue, game)

% add measurement noise
% note that since this is effects the role estimation directly, usually set consistent with the game W
sim_param.add_noise = ADD_MEASUREMENT_NOISE;
sim_param.noise_W = game_param.noise_W;
sim_param.noise_W = zeros(4);
sim_param.noise_W(1:2, 1:2) = game_param.noise_W(1:2, 1:2);
sim_param.noise_W(7:8, 7:8) = game_param.noise_W(1:2, 1:2);

% add action uncertainty at control action
% should be size of the control action, which is 2 * num of vehicles = 4
sim_param.add_action_uncertainty = ADD_ACTION_UNCERTAINTY;
sim_param.uncertainty = zeros(4, 1);
sim_param.uncertainty(1) = 0.03;
sim_param.uncertainty(4) = 0.03;

sim_param.randomize_initial_condition = true;
cov_pos_ic = 5.0;
cov_vel_ic = 1.0;

% sim_param.ini_condition_W = zeros(8);
% sim_param.ini_condition_W(1, 1) = cov_pos_ic;
% sim_param.ini_condition_W(2, 2) = cov_vel_ic;
% sim_param.ini_condition_W(7, 7) = cov_pos_ic;
% sim_param.ini_condition_W(8, 8) = cov_vel_ic;

sim_param.ini_condition_W = zeros(8, 1);
% only very the longitudinal position and speed of the nb (game) vehicle
sim_param.ini_condition_W(1) = 0;
sim_param.ini_condition_W(2) = 0;
sim_param.ini_condition_W(7) = cov_pos_ic;
sim_param.ini_condition_W(8) = cov_vel_ic;

%% time step setting
% for none sampled trajectory
% need 0.5, 10, 2 was the the following 3
% for sample trajectory, sim time step need to be smaller than the sample dt
% or we need to update the controller
sim_param.dt = 0.1; 
sim_param.steps = 61;
sim_param.frame_rate = 10;
sim_param.adapt_role = adapt_role;

%% Misc
% print off
game_param.print_off = true;
%% run loops set up
v0 = 4;
x0 = - 20;
% the x_ini is given from nb game agent perspective
% the first 2 are NB, the second 2 are EB
sim_type = {[x0;v0;x0;v0], "Leader", "NBICVaryLeader";
            [x0;v0;x0;v0], "Follower", "NBICVaryFollower"};

result_root_folder = RESULT_ROOT_FOLDER;
agent_game = 2;

seed_can = 1:1000;
type_id_can = 1:2;

run_info = struct();
run_info.common_parameters = struct("game_param", game_param, "sim_param", sim_param);
run_info.type = sim_type;
run_info.seed_can = seed_can;
run_info.type_id_can = type_id_can;

%% Run Sim
% Preallocate cell arrays
sim_params = cell(numel(type_id_can), numel(seed_can));
game_params = cell(numel(type_id_can), numel(seed_can));

for type_id = type_id_can
    sim_param.x_ini = sim_type{type_id, 1};
    sim_param.initial_role = sim_type{type_id, 2};
    % Add the following in case that that when role is not changing, the
    % initial role will not be used to override the game_param role
    game_param.role = sim_type{type_id, 2};
    sim_param.run_type = sim_type{type_id, 3};
    target_folder = result_root_folder + filesep + sim_param.run_type;
    if ~exist(target_folder, 'dir')
        mkdir(target_folder);
    end
    
    for seed = seed_can
        sim_param.seed = seed;
        sim_param.run_name = fullfile( result_root_folder, sim_param.run_type, sprintf("%03d", sim_param.seed));

        sim_params{type_id, seed} = sim_param;
        game_params{type_id, seed} = game_param;
    end
end

% % Run the simulation in parallel
% downside about parallel is that the random seed would be different 
% (and it may be different from different machine)
if PARALLEL_PROCESS
    if isempty(gcp('nocreate'))
        parpool;
    end
    parfor seed_idx = 1:length(seed_can)
        for type_id_idx = 1:length(type_id_can)
            sim_param = sim_params{type_id_idx, seed_idx};
            game_param = game_params{type_id_idx, seed_idx};
            if agent_game == 1
                game.simIntersectionRunGame(game_param, sim_param);
            else
                game.simIntersectionRunTwoGameAgent(game_param, sim_param);
            end
        end
    end
else
    for seed_idx = 1:length(seed_can)
        for type_id_idx = 1:length(type_id_can)
            sim_param = sim_params{type_id_idx, seed_idx};
            game_param = game_params{type_id_idx, seed_idx};
            if agent_game == 1
                game.simIntersectionRunGame(game_param, sim_param);
            else
                game.simIntersectionRunTwoGameAgent(game_param, sim_param);
            end
        end
    end
end

%% Summary
% can run standalone, only need the following info
save_plot = true;
if length(seed_can) > 1
    [run_summary, run_figs] = game.GenSummary.get_summary(result_root_folder, sim_type, seed_can, type_id_can);
    run_summary.run_info = run_info;
    fig_target_folder = fullfile(pwd, result_root_folder, "figures");
    if ~exist(fig_target_folder, 'dir')
        mkdir(fig_target_folder);
    end
    for i = 1:length(run_figs.handle)
    if save_plot
        saveas(run_figs.handle(i), fullfile(fig_target_folder, run_figs.names{i} + ".png"));
        save(fullfile(result_root_folder, "summary.mat"), "run_summary");
    end
    end
end
%% Plot one run result (prefer doing this from somewhere else)
% game.GenSummary.plot_one_run(result_root_folder, type_id, seed, true);
