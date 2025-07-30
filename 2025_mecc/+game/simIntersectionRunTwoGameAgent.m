function simIntersectionRunTwoGameAgent(game_param, sim_param)
tStart = tic;
fprintf("Simulating two game agent %s \n", sim_param.run_name);
%% Scenario setup
%% Define relation between game and simulation
% TODO: need to bring the following to the simulation class
% note, game does not have direction in its mind. It only needs the
% longitudinal position and speed, start from negative, and intersection
% point at 0.

% Simulation is based on 2-D coordinate, each vehicle has 4 states 
% x, vx, y, vy. 
% The first vehicle travels from west to east (x axis)
% The second vehicle travels from south to north (y axis)
% The following map x_game_to_sim defines how ego and other
% vehicle in game got mapped to the 2-D plane, here the flexibility comes
% into play.

% the following maps ego vehicle travels from south to north (nb)
% and the other vehicle from west to east
% which means that the second vehicle in the simulation is the ego in game
% and the first vehicle vehicle is the other vehicle

x_game_nb_to_sim = @(x_seq)   [x_seq(3, :);
                                             x_seq(4, :);
                                             x_seq(3, :) * 0;
                                             x_seq(4, :) * 0;
                                             x_seq(1, :) * 0;
                                             x_seq(2, :) * 0;
                                             x_seq(1, :);
                                             x_seq(2, :);];

u_game_nb_to_sim = @(u_ego, u_other) [u_other; 0; 0; u_ego];

u_sim_to_game_nb = @(u_sim) [u_sim(4,:); u_sim(1,:)];

x_sim_to_game_nb = @(x) [x(7, :);
                                     x(8, :);
                                     x(1, :);
                                     x(2, :);];

x_sim_to_game_nb_ego = @(x) [x(7, :); x(8, :);];
x_sim_to_game_nb_other = @(x) [x(1, :); x(2, :);];


% the following maps ego vehicle travels from west to east (eb)
% and the other vehicle from south to north
% which means that the first vehicle in the simulation is the ego in game
% and the second vehicle vehicle is the other vehicle

x_game_eb_to_sim = @(x_seq)   [x_seq(1, :);
                                             x_seq(2, :);
                                             x_seq(1, :) * 0;
                                             x_seq(2, :) * 0;
                                             x_seq(3, :) * 0;
                                             x_seq(4, :) * 0;
                                             x_seq(3, :);
                                             x_seq(4, :);];
u_game_eb_to_sim = @(u_ego, u_other) [u_ego; 0; 0; u_other];

u_sim_to_game_eb = @(u_sim) [u_sim(1,:); u_sim(4,:)];

x_sim_to_game_eb = @(x) [x(1, :);
                                     x(2, :);
                                     x(7, :);
                                     x(8, :);];

x_sim_to_game_eb_ego = @(x) [x(1, :); x(2, :);];
x_sim_to_game_eb_other = @(x) [x(7, :); x(8, :);];


% plotter need x, y, yaw, v;
% from x vx y vy to x, y, yaw, v
x_sim_to_plot_one = @(x) [x(1, :);
                                         x(3, :);
                                         atan2(x(4,:), x(2,:));
                                         sqrt(x(2,:).^2 + x(4,:).^2)];
% the following order makes tue northbound vehicle is blue
x_sim_to_plot = @(x) [x_sim_to_plot_one(x(5:8,:));
                                  x_sim_to_plot_one(x(1:4,:));];

%% Check parameters and set up initial condition
rng(sim_param.seed);

if isfield(game_param, "sample_traj") && game_param.sample_traj
    if sim_param.dt > game_param.sample_dt
        fprintf("Warning: simulation dt is greater than sample dt, this may cause issue in simulation. \n");
    end
    max_dec = abs(game_param.u_min);
else
    if sim_param.dt > game_param.deltaT
        fprintf("Warning: simulation dt is greater than game dt, this may cause issue in simulation. \n");
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
            x_ini = x_ini + x_sim_to_game_nb(mvnrnd(zeros(8, 1), sim_param.ini_condition_W, 1)');
        else
            % multivariate uniform distribution
            x_ini = x_ini + x_sim_to_game_nb(unifrnd(-sim_param.ini_condition_W(:), sim_param.ini_condition_W(:)));
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
game_param.agent_name = ["NB"; "EB"];
game_obj_nb = game.LFGStateDependentRole(game_param);
game_obj_nb.fix_role(~sim_param.adapt_role)
game_obj_nb.assign_role(sim_param.initial_role); 

game_param.agent_name = ["EB"; "NB"];
game_obj_eb = game.LFGStateDependentRole(game_param);
game_obj_eb.fix_role(~sim_param.adapt_role)

% let the eb vehicle has different role than nb vehicle
if sim_param.initial_role == "Leader"
    game_obj_eb.assign_role("Follower");
else
    game_obj_eb.assign_role("Leader");
end 

if isfield(game_param, "veh_dim")
    veh_dim = game_param.veh_dim;
else
    printf("Warning: veh_dim not provided, simulation class will using default values. \n");
    veh_dim = [];
end

sim_obj = game.SimIntersection(x_game_nb_to_sim(x_ini), ...
                                                'dt', sim_param.dt, ...
                                                'add_noise', sim_param.add_noise, ...
                                                'noise_W', sim_param.noise_W, ...
                                                'uncertainty', sim_param.uncertainty, ...
                                                'add_action_uncertainty', sim_param.add_action_uncertainty, ...
                                                'veh_dim', veh_dim);

assert(all(x_sim_to_game_nb(sim_obj.x) == x_ini))
%% simulation log
steps = sim_param.steps;
time_sim = zeros(1, steps + 1);
x_sim = zeros(8, steps + 1);
time_sim(1) = sim_obj.time;
x_sim(:, 1) = sim_obj.x;
x_sim_mea = zeros(8, steps + 1);
u_sim = zeros(4, steps);
u_sim_act = zeros(4, steps);
u_action_nb = zeros(2, steps);
u_action_eb = zeros(2, steps);
u_action = zeros(2, steps);

game_info_all = cell(1, steps);
role_trans_prob_nb = zeros(2, 2, steps);
role_trans_prob_eb = zeros(2, 2, steps);
role_trans_prob = zeros(2, steps);

role_sim = strings(2, steps);
role_sim(:, 1) = [game_obj_nb.role;
                  game_obj_eb.role];
role_sim_next = strings(2, steps);
flag_who_arrived_first = 0;
flag_who_arrived_stop_line_first = 0;
stop_line_pos = abs(game_param.stop_distance); % a positive value;
% % play with the tolerance a bit, right now it is 0.2; 0.2 does the trick, not sure if this is the largest
% % this is because all the non-terminal cases are very close to the finish line, (just perhaps need longer time)
% % but instead of increase speed, the minimum distance when both are at the intersection line (despite both are almost stopped)
% % does not actually get into danger.
stop_line_pos_for_termination_check = stop_line_pos + 0.2;
flag_role_conflict_exists = false;
collision_log = zeros(1, steps + 1);

% Each game agent maintains a role estimation of the other agent it is interacting
% save the trace of the role probability estimation
trace_prob_nb_on_eb = zeros(2, steps);
trace_prob_eb_on_nb = zeros(2, steps);

other_measure_nb.time = game_obj_nb.T1;
other_measure_eb.time = game_obj_eb.T1;

%% simulation

collision_flag = sim_obj.check_collision();
assert(~collision_flag);
collision_log(1) = collision_flag;
idx_end = steps + 1;

for i = 1:steps
    %% get state from simulation, this should happen every step
    x_sim_mea(:, i) = sim_obj.get_state_measurement();
    x_mea_current = x_sim_mea(:, i);
    x_game_nb = x_sim_to_game_nb(x_mea_current);
    x_game_eb = x_sim_to_game_eb(x_mea_current);
    % get new measurement for game
    other_measure_nb.trace = x_sim_to_game_nb_other(x_mea_current);
    other_measure_eb.trace = x_sim_to_game_eb_other(x_mea_current);

    %% decision making by both agents, this should follow decision making schedule
    [u_nb_ego, ~, game_info_nb] = game_obj_nb.step(x_game_nb, other_measure_nb);
    [u_eb_ego, ~, game_info_eb] = game_obj_eb.step(x_game_eb, other_measure_eb);
    
    % log information that is of interest (e.g., plotting)
    role_sim(:, i) = [game_obj_nb.role;
                      game_obj_eb.role];

    role_trans_prob_nb(:, :, i) = game_info_nb.role_trans_prob;
    role_trans_prob_eb(:, :, i) = game_info_eb.role_trans_prob;

    % to facilitate plotting (does not contains complete information)
    role_trans_prob(1, 1, i) = role_trans_prob_nb(1, 1, i);
    role_trans_prob(2, 2, i) = role_trans_prob_eb(2, 2, i);

    u_action_nb(:, i) = game_info_nb.action;
    u_action_eb(:, i) = game_info_eb.action;
    u_action(:, i) = [u_action_nb(1, i); u_action_eb(1, i)];
    
    if game_obj_nb.role_update_schedule == 2
        % if role change will be enforced in the next step
        role_sim_next(:, i) = [game_info_nb.role;
                               game_info_eb.role];
    end
    game_info_all{i} = {u_nb_ego, u_eb_ego, game_info_nb, game_info_eb};

    % log the role estimation results by each agent
    % role estimation northbound vehicle on eastbound vehicle
    trace_prob_nb_on_eb(:, i) = game_obj_nb.role_estimation;
    game_obj_nb.print_role_estimation_info();

    % role estimation eastbound vehicle on northbound vehicle
    trace_prob_eb_on_nb(:, i) = game_obj_eb.role_estimation;
    game_obj_eb.print_role_estimation_info();
    
    %% Execute decision in the simulation environment
    % (TODO) implement tracking and different steps.
    u_sim(:, i) = u_game_nb_to_sim(u_nb_ego(1), u_eb_ego(1));
    % in case there are noise
    u_sim_act(:, i) = sim_obj.step(u_sim(:, i));
    time_sim(i + 1) = sim_obj.time;
    x_sim(:, i + 1) = sim_obj.x;
    collision_log(i+1) = sim_obj.check_collision();

    %% check for finish flags, based on the assumption that x_ini would not means finish
    % note that the simulation will keep running even after one of the agent cross the stop line and intersection point
    [flag_who_arrived_first, flag_who_arrived_stop_line_first, could_stop] = ...
        game.simIntersectionFinishCheck(x_sim_to_game_nb(sim_obj.x), stop_line_pos_for_termination_check, ...
                            flag_who_arrived_first, ...
                            flag_who_arrived_stop_line_first);
    if could_stop
        % could potentially stop here instead of going to the end
        idx_end = i + 1;
    end
end
%% post simulation statistics
switch flag_who_arrived_first
    case 1
        fprintf("eastbound (game) vehicle arrived first. \n");
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
if any(role_sim(1, :) == role_sim(2, :))
    fprintf("Role conflict detected. \n");
    flag_role_conflict_exists = true;
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
u_sim_to_game = u_sim_to_game_nb;

%% save the results
save_name = sim_param.run_name;
sim_param = rmfield(sim_param, 'run_name');
save(save_name, ...
        '-regexp', '^(?!(game_obj_nb|game_obj_eb|sim_obj|save_name)$).');
fprintf("Simulation took %.4f [sec],  result saved to %s. \n", time_used, save_name);
end
