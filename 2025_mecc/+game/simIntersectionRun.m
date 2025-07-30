function simIntersectionRun(game_param, mpc_param, sim_param)
tStart = tic;
fprintf("Simulating %s \n", sim_param.run_name);
%% Scenario setup
%% Define relation between game and simulation
% TODO: need to bring the following to the simulation class
% Note, game does not have direction in its mind. It only needs the
% longitudinal position and speed, start from negative, and intersection
% point at 0. 
% The order is [ego_pos; ego_speed; other_pos; other_speed]

% Simulation is based on 2-D coordinate, each vehicle has 4 states 
% x, vx, y, vy. 
% The first vehicle travels from west to east (x axis)
% The second vehicle travels from south to north (y axis)
% In this simulation set up
% The first vehicle in the simulation is using MPC
% The second vehicle in the simulation is using game-theoretic model for
% decision making

% The x_game_to_sim maps the state (sequence) of game to the state (sequence) of simulation 
% It therefore defines how ego and other vehicle in game got mapped to the 2-D plane, 
% This should corresponds to which vehicle is using the game-theoretical
% model to make decision

% the following x_game_to_sim definition maps the "ego" vehicle in the game travels from south to north
% and the "other" vehicle from west to east
% which means that the second vehicle in the simulation is using the game-theoretic model
% and thus it is the "ego" in game, while the first vehicle vehicle is the
% "other" vehicle in the game

x_game_to_sim = @(x_seq)    [x_seq(3, :);
                             x_seq(4, :);
                             x_seq(3, :) * 0;
                             x_seq(4, :) * 0;
                             x_seq(1, :) * 0;
                             x_seq(2, :) * 0;
                             x_seq(1, :);
                             x_seq(2, :);];

% convert the u in game role order 
% (ego = veh 0 in game = second vehicle in sim, 
%  other = veh 1 in game = first vehicle in sim)
% to sim role order.
u_game_to_sim = @(u0_game, u1_game) [u1_game; 0; 0; u0_game];

% inverse conversion of the above not used in simulation but got saved and
% used in plot
u_sim_to_game = @(u_sim) [u_sim(4,:); u_sim(1,:)];

x_sim_to_game = @(x) [x(7, :);
                      x(8, :);
                      x(1, :);
                      x(2, :);];

x_sim_to_game_other = @(x) [x(1, :);
                            x(2, :);];


% not used in simulation but got saved and used in plot!!
% plotter need x, y, yaw, v;
% from x vx y vy to x, y, yaw, v
x_sim_to_plot_one = @(x) [x(1, :);
                                         x(3, :);
                                         atan2(x(4,:), x(2,:));
                                         sqrt(x(2,:).^2 + x(4,:).^2)];
% the game vehicle is blue (north bound)
% Not used as plot usually happens outside
x_sim_to_plot = @(x) [x_sim_to_plot_one(x(5:8,:));
                                  x_sim_to_plot_one(x(1:4,:));];


% again the first vehicle is using mpc
x_sim_to_mpc = @(x) x(1:2, :); 
% the second vehicle will be the other from the perspective of first
% vehicle, this is used to get the state of the other vehicle from sim
% state
x_sim_to_mpc_other = @(x) x(7:8, :);

%% Check parameters and set up initial condition
rng(sim_param.seed);

if isfield(game_param, "sample_traj") && game_param.sample_traj
    if sim_param.dt > game_param.sample_dt
        fprintf("Warning: simulation dt is greater than game sample dt, this may cause issue in simulation. \n");
    end
    if sim_param.dt > mpc_param.sample_dt
        fprintf("Warning: simulation dt is greater than mpc sample dt, this may cause issue in simulation. \n");
    end
    max_dec = abs(game_param.u_min);
else
    if sim_param.dt > game_param.deltaT
        fprintf("Warning: simulation dt is greater than game dt, this may cause issue in simulation. \n");
    end
    if sim_param.dt > mpc_param.dt
        fprintf("Warning: simulation dt is greater than mpc dt, this may cause issue in simulation. \n");
    end
    max_dec = abs(min(game_param.u0_action_set));
end

% (TODO) Need a better check on whether the initial state allow collision be avoided by the
% current control resolution.
safe_condition = @(s, v) v^2 / (0 - s - game_param.reward.min_dist) / 2  < max_dec;
x_ini = sim_param.x_ini;
if ~isfield(sim_param, "randomize_initial_condition")
    sim_param.randomize_initial_condition = false;
end
if sim_param.randomize_initial_condition
    % randomize the initial condition until it is safe
    safe_check = false;
    while ~safe_check
        if all(size(sim_param.ini_condition_W) == [8, 8])
            % multivariate normal distribution
            x_ini = x_ini + x_sim_to_game(mvnrnd(zeros(8, 1), sim_param.ini_condition_W, 1)');
        else
            % multivariate uniform distribution
            x_ini = x_ini + x_sim_to_game(unifrnd(-sim_param.ini_condition_W(:), sim_param.ini_condition_W(:)));
        end
        safe_check = safe_condition(x_ini(1), x_ini(2)) && safe_condition(x_ini(3), x_ini(4));
    end    
else 
    assert(safe_condition(x_ini(1), x_ini(2)))
    assert(safe_condition(x_ini(3), x_ini(4)))
end
sim_param.x_ini_used = x_ini;

if ~isfield(sim_param, "add_action_uncertainty")
    sim_param.uncertainty = 0;
    sim_param.add_action_uncertainty = false;
end
%% Initialize objects
% game agent is north bound (nb)
game_param.agent_name = ["NB(game)"; "EB(mpc)"];
game_obj = game.LFGStateDependentRole(game_param);
game_obj.fix_role(~sim_param.adapt_role)
game_obj.assign_role(sim_param.initial_role); 

% mpc agent is east bound (eb)
mpc_param.agent_name = ["EB(mpc)"; "NB(game)"];
mpc_obj = game.LFGMPC(mpc_param);
if isfield(mpc_param, "agent_model")
    if isequal(mpc_param.agent_model, rmfield(game_param, "agent_name"))
        fprintf("Wanted to add a model to mpc_agent, but it is the same as game_param, so reuse. \n");
        game_agent_model_obj = game_obj;
    else
        agent_model_diff = Util.compare_structs(mpc_param.agent_model, rmfield(game_param, "agent_name"));
        fprintf("mpc agent will add a model with parameter different from game_param. \n");
        game_agent_model_obj = game.LFGStateDependentRole(mpc_param.agent_model);
    end
else
    fprintf("mpc agent will use the game agent obj directly as agent model (perfect model). \n");
    game_agent_model_obj = game_obj;
end

if isfield(game_param, "veh_dim")
    veh_dim = game_param.veh_dim;
else
    printf("Warning: veh_dim not provided, simulation class will using default values. \n");
    veh_dim = [];
end

sim_obj = game.SimIntersection(x_game_to_sim(x_ini), ...
                                                'dt', sim_param.dt, ...
                                                'add_noise', sim_param.add_noise, ...
                                                'noise_W', sim_param.noise_W, ...
                                                'uncertainty', sim_param.uncertainty, ...
                                                'add_action_uncertainty', sim_param.add_action_uncertainty, ...
                                                'veh_dim', veh_dim);

assert(all(x_sim_to_game(sim_obj.x) == x_ini))
%% simulation log
steps = sim_param.steps;
time_sim = zeros(1, steps + 1);
time_sim(1) = sim_obj.time;
x_sim = zeros(8, steps + 1);
x_sim_mea = zeros(8, steps + 1);
x_sim(:, 1) = sim_obj.x;
u_sim = zeros(4, steps);
u_sim_act = zeros(4, steps);
u_action = zeros(2, steps);

game_info_all =cell(1, steps);
mpc_info_all = cell(1, steps);
role_trans_prob = zeros(2, 2, steps);
% only game agent actually carries the role properties with meaning
% but both agents are estimating the role by the other agent
role_sim = strings(1, steps);
role_sim(1) = game_obj.role;
role_sim_next = strings(1, steps);
infeasible_log = zeros(1, steps);

% game agent (nb) role estimation on mpc agent (eb)
trace_prob_other_on_ego = zeros(2, steps);
% mpc agent (eb) role estimation on game agent (nb)
trace_prob_ego_on_other = zeros(2, steps);

optimal_chance_constraint_range = zeros(2, steps);
chance_constraint_range = zeros(2, steps);
flag_who_arrived_first = 0;
flag_who_arrived_stop_line_first = 0;
stop_line_pos = abs(game_param.stop_distance); % a positive value;
% % play with the tolerance a bit, right now it is 0.2; 0.2 does the trick, not sure if this is the largest
% % this is because all the non-terminal cases are very close to the finish line, (just perhaps need longer time)
% % but instead of increase speed, the minimum distance when both are at the intersection line (despite both are almost stopped)
% % does not actually get into danger.
stop_line_pos_for_termination_check = stop_line_pos + 0.2;
collision_log = zeros(1, steps + 1);

game_agent_other_measurement.time = game_obj.T1;
mpc_agent_other_measurement.time = mpc_obj.T1;
%% sim
collision_flag = sim_obj.check_collision();

assert(~collision_flag);
collision_log(1) = collision_flag;
idx_end = steps + 1;

for i = 1:steps
    %% get state from simulation, this should happen every step
    x_sim_mea(:, i) = sim_obj.get_state_measurement();
    x_mea_current = x_sim_mea(:, i);
    x_game = x_sim_to_game(x_mea_current);
    x_mpc = x_sim_to_mpc(x_mea_current);

    % TODO: need to update if T1 is different from sample time
    game_agent_other_measurement.trace = x_sim_to_game_other(x_mea_current);
    mpc_agent_other_measurement.trace = x_sim_to_mpc_other(x_mea_current);

    %% Decision making by both agents, this should follow decision making schedule
    % each agent may have its own decision making schedule
    % other vehicle (nb) is the game agent
    [u0_game, u1_game, game_info] = game_obj.step(x_game, game_agent_other_measurement);
    % ego vehicle (eb) is the mpc agent
    [u1_mpc, mpc_info] = mpc_obj.get_proactive_sequence(x_mpc, mpc_agent_other_measurement, game_agent_model_obj);
    
    % log information that is of interest (e.g., plotting)
    % game agent (nb) other vehicle
    role_sim(i) = game_obj.role;
    trace_prob_other_on_ego(:, i) = game_obj.role_estimation;
    role_trans_prob(:, :, i) = game_obj.role_trans_prob;

    if game_obj.role_update_schedule == 2
        % if role change will be enforced in the next step
        role_sim_next(i) = game_obj.role;
    end
    game_info_all{i} = {u0_game, u1_game, game_info};
    
    % mpc agent (eb) ego vehicle
    trace_prob_ego_on_other(:, i) = mpc_obj.role_estimation;        
    optimal_chance_constraint_range(:, i) = [max(mpc_info.step_chance_constraint_all(mpc_info.max_reward_idx, :));
                                                                 min(mpc_info.step_chance_constraint_all(mpc_info.max_reward_idx, :))];
    % to really check feasibility, should record the maximum of the minimum
    lowest_chance_for_each_profile = min(mpc_info.step_chance_constraint_all')';
    assert(all(size(lowest_chance_for_each_profile) == [size(mpc_info.step_chance_constraint_all, 1), 1]));
    chance_constraint_range(:, i) = [max(lowest_chance_for_each_profile);
                                                    min(lowest_chance_for_each_profile)];
    mpc_info_all{i} = {u1_mpc, mpc_info};

    infeasible_log(i) = mpc_info.infeasible;
    u_action(:, i) = [game_info.action(1); mpc_info.action];

    game_obj.print_role_estimation_info();
    mpc_obj.print_role_estimation_info();

    %% Execute decision in the simulation environment
    % (TODO) implement tracking and different steps.
    u_sim(:, i) = u_game_to_sim(u0_game(1), u1_mpc);
    % in case there are noise
    u_sim_act(:, i) = sim_obj.step(u_sim(:, i));
    time_sim(i + 1) = sim_obj.time;
    x_sim(:, i + 1) = sim_obj.x;
    collision_log(i + 1) = sim_obj.check_collision();

    %% check for finish flags, based on the assumption that x_ini would not means finish
    % note that the simulation will keep running even after one of the agent cross the stop line and intersection point
    [flag_who_arrived_first, flag_who_arrived_stop_line_first, could_stop] = ...
        game.simIntersectionFinishCheck(x_sim_to_game(sim_obj.x), stop_line_pos_for_termination_check, ...
                            flag_who_arrived_first, ...
                            flag_who_arrived_stop_line_first);
    if could_stop
        % could potentially stop here instead of going to the end
        idx_end = i + 1;
    end
end
%% post simulation statistics
switch flag_who_arrived_stop_line_first
    case 1
        fprintf("eastbound (mpc) vehicle arrived first. \n");
    case 2
        fprintf("northbound (game) vehicle arrived first. \n");
    case 3
        fprintf("both vehicle arrived at the same time (subject to dt resolution). \n");
    otherwise
        fprintf("wired run, neither arrived at the intersection. \n");
end

if any(collision_log)
    fprintf("Collision detected. \n");
    collision_flag = true;
end
infeasible_flag = false;
if any(infeasible_log)
    fprintf("Infeasible happens for mpc. \n");
    infeasible_flag = true;
end

% For the current intersection set up, this should not happen if sqrt(2) * stop_line > safety tolerant 
% since neither of the vehicle has entered the stop line so they should be at least
% if extend the check to the end, then the minimum distance might be smaller than the tolerance 
% if the intersection zone is small
min_dist = min(sqrt((x_sim(1, 1:idx_end) - x_sim(5, 1:idx_end)).^2 ...
                                                    + (x_sim(3, 1:idx_end) - x_sim(7, 1:idx_end)).^2));
if min_dist < game_param.reward.min_dist
    warning("Warning: minimum distance between vehicles is less than the minimum distance allowed. \n");
end

time_used = toc(tStart);
% to facilitate plotting
trace_prob_nb_on_eb = trace_prob_other_on_ego;
trace_prob_eb_on_nb = trace_prob_ego_on_other;

%% save the results
save_name = sim_param.run_name;
sim_param = rmfield(sim_param, 'run_name');
save(save_name, ...
        '-regexp', '^(?!(game_obj|game_agent_model_obj|mpc_obj|sim_obj|save_name|tStart)$).');
fprintf("Simulation took %.4f [sec],  result saved to %s. \n", time_used, save_name);
end
