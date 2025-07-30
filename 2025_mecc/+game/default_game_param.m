%% Put potentially Altered global parameter here
% clc;clear; close all;
% P_ALTER = 0.7;
% P_ALTER_MODEL = P_ALTER;
% ADD_MEASUREMENT_NOISE = true;
% ADD_ACTION_UNCERTAINTY = true;
% RESULT_ROOT_FOLDER = "temp_data" + filesep + "mpc_test";

function game_param = default_game_param(P_ALTER)
%% Game parameters
veh_width = 2.5;
veh_length = 5.0;
lane_width = 4.0;

% reward terms related
veh_dim.veh_width = veh_width;
veh_dim.veh_length = veh_length;
veh_dim.veh_d = veh_length / 3;
veh_dim.lane_width = lane_width;

game_param.veh_dim = veh_dim;

% initial role (will be overridden at the beginning of the simulation)
% game_param.role = "Leader"; 
% game_param.role = "Follower"; 

% 1: use a fixed provided transition probability matrix, given by game_param.alter_matrix
% 2: use a the defined state dependent transition probability matrix
%        cal_state_dependent_role_trans_prob method in LFGStateDependentRole Game class
% 3: (prefer) use the prescribed role and the alter matrix to calculate the transition probability
%        Needs game_param.alter_matrix and game_param.role_prescription_type to be defined
% other value: no role change (through an identity matrix)
game_param.role_update_type = 3;

% 1: prescribe role with Maximum likelihood estimation of the other agent's role
% 2: prescribe the complementary role to the estimated role with the same probability
% other value: prescribe both role with equal probability
game_param.role_prescription_type = 1;

% 1 (or other values): at each time stamp, update role first then take action is taken
% 2: take action first then update role
% update role after action is taken
game_param.role_update_schedule = 2;

% state transition coefficient
if game_param.role_update_type == 1
    game_param.alter_matrix = [0.9, 0.1; 0.1, 0.9];
end

% needed if game_param.role_update_type = 2
if game_param.role_update_type == 2
    % the larger the sharper changes when start with neutral (dx = 0)
    % in 10 range pretty much just 2 possibilities e.g. , 10, 12
    % probably should try something smaller to make the results more randomized.
    % ~0.2 level pretty much very similar distribution for different initial positions
    game_param.k_lead = 1;
    game_param.k_follower = 1.2;
end

% More interesting case and often used.
if game_param.role_update_type == 3
    game_param.alter_matrix = cat(3, zeros(2, 2), zeros(2, 2));
    % will definitely change role
    p_alter = P_ALTER;
    game_param.alter_matrix(:, :, 1) = [1, p_alter; 0, 1 - p_alter];
    game_param.alter_matrix(:, :, 2) = [1 - p_alter, 0; p_alter, 1];
end

% reward
game_param.reward.w_collision = 100; % weight on collision
game_param.reward.w_control = 0.1;
game_param.reward.lambda = 0.99; % 
game_param.reward.min_dist = 1.5 * veh_dim.veh_length;

% action space definition
% by default, ego is traveling x, other is traveling y

% col = action dimension
% row = num of actions
game_param.deltaT = 1;
game_param.N = 4;
delta_a = 2;
game_param.u0_action_set = [delta_a; -delta_a; 0.0;];
game_param.u1_action_set =  [delta_a; -delta_a; 0.0;];

game_param.v_max = 10;
game_param.v_min = 0;
game_param.T = game_param.deltaT * game_param.N;

% % if want to sample the trajectory
game_param.sample_traj = true;
game_param.full_sample_len = 50;
game_param.sample_dt = 0.1;
game_param.N_speed_mesh  = 10; 
% 10 is something still do able for MPC search
% this to make sure the front bumper is stopped at the stop line.
game_param.stop_distance = lane_width + veh_length / 2;
game_param.u_max = 1;
game_param.u_min = -2;
ovm_params.alpha = 0.4;
ovm_params.beta = 0.5;

% to make the two equivalent
ovm_params.alpha_v = ovm_params.alpha + ovm_params.beta;
ovm_params.kappa = 0.6;
ovm_params.hst = 5;
ovm_params.vmax = game_param.v_max;
ovm_params.hgo = ovm_params.hst + ovm_params.vmax / ovm_params.kappa;
ovm_params.l_veh = veh_length;
game_param.ovm_params = ovm_params;

% role estimation parameter (only used for one vehicle)
game_param.noise_W = zeros(2);
% note that this noise level would change how much the role estimation is certain about the role
% even when the action taken is precisely at the value that the other agent with the role is taking
% it does not actually corresponds to with the actual noise level in the simulation.
% only add significant noise level to longitudinal positions and speed
% TODO: could this be too large?
cov_pos = 0.03;
cov_vel = 0.01;
game_param.noise_W(1, 1) = cov_pos;
game_param.noise_W(2, 2) = cov_vel;

end