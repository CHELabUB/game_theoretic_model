%% Put potentially Altered global parameter here
% clc;clear; close all;
% P_ALTER = 0.7;
% P_ALTER_MODEL = P_ALTER;
% ADD_MEASUREMENT_NOISE = true;
% ADD_ACTION_UNCERTAINTY = true;
% RESULT_ROOT_FOLDER = "temp_data" + filesep + "mpc_test";

% Plotter setup
Util.plot_setup();

%% Game parameters
game_param = game.default_game_param(P_ALTER);

%% MPC set up
mpc_param = game_param;
% mpc may need to have a finer mash to deal with intermediate states

% if use sample trajectory, this would not matter
% mpc_param.deltaT = 0.5; 
% mpc_param.N = floor(mpc_param.T / mpc_param.deltaT);

% this cannot be too small, or it would be overly constrained
mpc_param.epsilon = 0.02; % chance constraints

% % only add significant noise level to longitudinal positions and speed
% % change this if one wants something different
% mpc_param.noise_W(1, 1) = cov_pos;
% mpc_param.noise_W(2, 2) = cov_vel;

% this type only matters for the old state dependent role transition.
% mpc_param.prob_seq_type = 1;
% % should not have too much differences. But just added in case.
% % 1 is consider changes along the way (default)
% % 2 is uniform

% allowing the agent_model to be different from the game agent
game_param_model = game_param;
p_alter = P_ALTER_MODEL;
game_param_model.alter_matrix(:, :, 1) = [1, p_alter; 0, 1 - p_alter];
game_param_model.alter_matrix(:, :, 2) = [1 - p_alter, 0; p_alter, 1];

mpc_param.agent_model = game_param_model;

%% Simulation setup
adapt_role = true;
% adapt_role = false;

%% Add some randomness
% note that for all noises
% if w is defined as matrix, will randomize as multivariate normal distribution
% if w is defined as vector, will randomize as multivariate uniform distribution
% the noise is added directly to the state of the simulation
% the first 4 belongs to eb (red, mpc), the second 4 belongs to nb (blue, game)

% add measurement noise
% note that since this is effects the role estimation directly, usually set consistent with the game W
sim_param.add_noise = ADD_MEASUREMENT_NOISE;
sim_param.noise_W = mpc_param.noise_W;
sim_param.noise_W = zeros(4);
sim_param.noise_W(1:2, 1:2) = mpc_param.noise_W(1:2, 1:2);
sim_param.noise_W(7:8, 7:8) = mpc_param.noise_W(1:2, 1:2);

% add action uncertainty at control action
% should be size of the control action, which is 2 * num of vehicles = 4
sim_param.add_action_uncertainty = ADD_ACTION_UNCERTAINTY;
sim_param.uncertainty = zeros(4, 1);
sim_param.uncertainty(1) = 0.03; % only the nb vehicle has uncertainty. No uncertainty for the eb vehicle

sim_param.randomize_initial_condition = true;
cov_pos_ic = 5.0;
cov_vel_ic = 1.0;

% sim_param.ini_condition_W = zeros(8);
% sim_param.ini_condition_W(1, 1) = cov_pos_ic;
% sim_param.ini_condition_W(2, 2) = cov_vel_ic;
% sim_param.ini_condition_W(7, 7) = cov_pos_ic;
% sim_param.ini_condition_W(8, 8) = cov_vel_ic;

sim_param.ini_condition_W = zeros(8, 1);
% only very the longitudinal position and speed of the nb vehicle
sim_param.ini_condition_W(1) = 0;
sim_param.ini_condition_W(2) = 0;
sim_param.ini_condition_W(7) = cov_pos_ic;
sim_param.ini_condition_W(8) = cov_vel_ic;

%% time step setting
% for none sampled trajectory
% need 0.5, 10, 2 was the the following 3
% for sample trajectory, sim time step need to be smaller than the sample dt
% or we need to update the controller
% Note for MPC, it takes longer (both vehicle traveling slower)
sim_param.dt = 0.1; 
sim_param.steps = 61; % for some of the unfinished case, temporary set to larger value to make sure it is finished.
sim_param.frame_rate = 10; % and increase to corresponding values. Need to set + 1 so that role can be interpreted correctly.
sim_param.adapt_role = adapt_role;

%% Misc
game_param.print_off = true;
mpc_param.print_off = true;
mpc_param.agent_model.print_off = true;
%% run loops set up

v0 = 4;
x0 = - 20;
% the x_ini is given from nb game agent perspective
% the first 2 are NB, the second 2 are EB
sim_type = { [x0;v0;x0;v0], "Leader", "NBICVaryLeader";
                    [x0;v0;x0;v0], "Follower", "NBICVaryFollower"};

result_root_folder = RESULT_ROOT_FOLDER;

seed_can = 1:1000;
type_id_can = 1:2;

run_info = struct();
run_info.common_parameters = struct("game_param", game_param, "mpc_param", mpc_param, "sim_param", sim_param);
run_info.type = sim_type;
run_info.seed_can = seed_can;
run_info.type_id_can = type_id_can;

%% Run Sim
% Preallocate cell arrays
sim_params = cell(numel(type_id_can), numel(seed_can));
game_params = cell(numel(type_id_can), numel(seed_can));
mpc_params = cell(numel(type_id_can), numel(seed_can));

for type_id = type_id_can
    sim_param.x_ini = sim_type{type_id, 1};
    sim_param.initial_role = sim_type{type_id, 2};
    % Add the following in case that that when role is not changing, the
    % initial role will not be used to override the game_param role
    game_param.role = sim_type{type_id, 2};
    mpc_param.agent_model.role = game_param.role;
    sim_param.run_type = sim_type{type_id, 3};
    target_folder = fullfile(pwd, result_root_folder + filesep + sim_param.run_type);
    if ~exist(target_folder, 'dir')
        mkdir(target_folder);
    end

    for seed = seed_can
        sim_param.seed = seed;
        sim_param.run_name = fullfile( result_root_folder, sim_param.run_type, sprintf("%03d", sim_param.seed));
        game_params{type_id, seed} = game_param;
        mpc_params{type_id, seed} = mpc_param;
        sim_params{type_id, seed} = sim_param;
    end
end

%% Run the simulation in parallel
% downside about parallel is that the random seed would be different 
% (and it may be different from different machine)
if PARALLEL_PROCESS
    if isempty(gcp('nocreate'))
        parpool;
    end
    parfor seed_idx = 1:length(seed_can)
        for type_id_idx = 1:length(type_id_can)
            game_param = game_params{type_id_idx, seed_idx};
            mpc_param = mpc_params{type_id_idx, seed_idx};
            sim_param = sim_params{type_id_idx, seed_idx};
            game.simIntersectionRun(game_param, mpc_param, sim_param)
        end
    end
else
    for seed_idx = 1:length(seed_can)
        for type_id_idx = 1:length(type_id_can)
            game_param = game_params{type_id_idx, seed_idx};
            mpc_param = mpc_params{type_id_idx, seed_idx};
            sim_param = sim_params{type_id_idx, seed_idx};
            game.simIntersectionRun(game_param, mpc_param, sim_param)
        end
    end
end
%% Summary
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
else
    run_summary.run_info = run_info;
    save(fullfile(result_root_folder, "summary.mat"), "run_summary");
end
%% Plot one run result
% game.GenSummary.plot_one_run(result_root_folder, type_id, seed, true);
