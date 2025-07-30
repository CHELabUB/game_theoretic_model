% misc do not print stuff, by default is print
% game_param.print_off = true;

%%
veh_width = 2.5;
veh_length = 5.0;
lane_width = 4.0;


% reward terms related
veh_dim.veh_width = veh_width;
veh_dim.veh_length = veh_length;
veh_dim.veh_d = veh_length / 3;
veh_dim.lane_width = lane_width;


game_param.veh_dim = veh_dim;
game_param.role = "Follower";
game_param.reward.w_collision = 100; % weight on collision
game_param.reward.w_control = 0.1;
game_param.reward.lambda = 0.99; % 
game_param.reward.min_dist = 1.5 * veh_dim.veh_length;
game_param.v_max = 10;
game_param.v_min = 0;

% for trajectory sampling
game_param.sample_traj = true;
game_param.full_sample_len = 10; % shorter than usual
game_param.sample_dt = 0.1;
game_param.N_speed_mesh  = 5;
game_param.stop_distance = 0; % start from negative distance 
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

% for role estimation
game_param.noise_W = diag([0.1, 0.2]);

% reward (LFGClass test) 
game_param.reward.w_collision = 100; % weight on collision
game_param.reward.w_control = 0.1;
game_param.reward.lambda = 0.99; % 
game_param.reward.min_dist = 1.5 * veh_dim.veh_length;

% state transition matrix related
game_param.k_lead = 1;
game_param.k_follower = 1.2;
% game_param.alter_matrix = cat(3, [0.7, 0.45; 0.3, 0.55], [0.8, 0.35; 0.2, 0.65]);

%% LFGMPCClass test
mpc_param = game_param;
% chance constraints (LFGMPCClass test)
mpc_param.epsilon = 0.02; % chance constraints
