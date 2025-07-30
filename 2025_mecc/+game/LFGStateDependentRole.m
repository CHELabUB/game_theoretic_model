classdef LFGStateDependentRole < game.LFGBaseClass
    %LFGSTATEDEPENDENTROLE Summary of this class goes here
    %   Leader follower game object 
    %   for intersection and 2 vehicle only, in this model only 1-D motion for each vehicle, starts from negative to positive,
    %   and assumes that the only conflict point would be when the distance is 0. 
    %   From visualization perspective, how these two direction are actually projected does not matter
    % Given a intersection state, gives the corresponding action prediction
    % over the horizon of both agents.
    % only define the properties in addition to the base class
    properties
        param % abstract in base class
        class_name % abstract in base class
        role_fixed
        role_trans_prob
        role_prob
        role_update_type
        role_update_schedule
        role_prescription_type
    end
    
    methods
        function obj = LFGStateDependentRole(param)
            %LFGSTATEDEPENDENTROLE Construct an instance of this class
            %   Detailed explanation goes here
            obj.param = param;
            if isfield(obj.param, "agent_name")
                obj.agent_name = obj.param.agent_name;
            end
            obj.debug_print("This agent is a LFGStateDependentRole agent.\n");
            obj.class_name = "LFGStateDependentRole";
            obj.debug_print(sprintf("Agent name '%s' for the ego vehicle, '%s' for the other vehicle .\n", ...
                                    obj.agent_name(1), obj.agent_name(2)));
            % this ratio is used for reward calculation and remain the same as long as horizon does not change.
            if ~obj.param.sample_traj
                ratio = zeros(obj.param.N, 1);
                for i = 1:obj.param.N
                    ratio(i) = obj.param.reward.lambda ^ (i - 1);
                end
                obj.initialize_action_sequence();
            else
                ratio = zeros(obj.param.full_sample_len, 1);
                obj.debug_print("Sample trajectory is used, the ratio is calculated based on the sample_dt.\n");
                for i = 1:obj.param.full_sample_len
                    ratio(i) = obj.param.reward.lambda ^ (i - 1);
                end
            end
            obj.param.reward.ratio_vector = ratio;
            if isfield(obj.param, "role")
                obj.assign_role(obj.param.role);
            else
                obj.debug_print("No role given, default will be Follower.\n")
                obj.assign_role("Follower");
            end

            obj.debug_print("by default the role can be changing.\n");
            obj.role_fixed = true;
            % default transition probability gives no role changes
            obj.role_trans_prob = eye(2);
            
            % discrete system implementation detail
            if isfield(obj.param, "role_update_type")
                obj.role_update_type = obj.param.role_update_type;
            else
                obj.debug_print("No role_update_type given, use default role update type.\n");
                obj.role_update_type = 1;
            end
            % role update type, will effect how the role transition probability is calculated
            if ~isfield(obj.param, "alter_matrix")
                obj.param.alter_matrix = eye(2);
                obj.debug_print("Warning: no alter_matrix is provided, set to eye(2).\n");
            end
            switch obj.role_update_type
                case 1
                    obj.debug_print("The transition probability matrix is fixed \n");
                    assert(all(size(obj.param.alter_matrix(:, :, 1)) == [2, 2]));
                case 2
                    obj.debug_print("Will update role based explicitly on state. \n");
                case 3
                    obj.debug_print("based on prescribed role and alter matrix. \n");
                    assert(all(size(obj.param.alter_matrix) == [2, 2, 2]));
                otherwise
                    obj.debug_print("Error value, no role changes.\n");
                    obj.role_update_type = 1;
            end
            % Schedule for role update, whether now or next step
            if isfield(obj.param, "role_update_schedule")
                obj.role_update_schedule = obj.param.role_update_schedule;
            else
                obj.debug_print("No role_update_schedule given, use default value.\n");
                obj.role_update_schedule = 1;
            end
            switch obj.role_update_schedule 
                case 1
                    obj.debug_print("The role will be updated at the current step. \n");
                case 2
                    obj.debug_print("The role will be updated at the next step. \n");
                otherwise
                    obj.debug_print("Error, use default role update schedule, which is at the current step. \n");
                    obj.role_update_schedule = 1;
            end
            % role prescription type, how the role is prescribed based on the role estimation of the other vehicle
            if isfield(obj.param, "role_prescription_type")
                obj.role_prescription_type = obj.param.role_prescription_type;
            else
                obj.debug_print("No role_prescription_type given, use default value.\n");
                obj.role_prescription_type = 1;
            end
            switch obj.role_prescription_type
                case 1
                    obj.debug_print("Role prescription is based on the estimated role of the other vehicle with maximum likelihood. \n");
                case 2
                    obj.debug_print("Role prescription is based on the probability of role estimation of the other vehicle. \n");
                otherwise
                    obj.debug_print("Error, use default role prescription type, which is based on the role estimation of the other vehicle. \n");
                    obj.role_prescription_type = 1;
            end
            obj.role_estimation = [0.5; 0.5];
            if isfield(obj.param, 'T1')
                obj.T1 = obj.param.T1;
            else
                % only for sampled base so the smallest should be with sample dt
                % obj.T1 = obj.dt;
                obj.T1 = obj.param.sample_dt;
            end
            obj.x_other_trace_predicted.time = 0:obj.param.sample_dt:obj.T1;
            % all zero will theoretically give the same probability for both roles
            % but could numerically challenging and gives NaN
            % obj.x_other_trace_predicted.trace = zeros(2, length(obj.x_other_trace_predicted.time(2:end)), 2);
            obj.status = []; % empty means no previous solution yet.
            % check parameter for anomaly
            obj.check_param();
        end
        % role adaptation
        function assign_role(obj, prescribed_new_role)
            if obj.role_fixed
                obj.debug_print(sprintf("Error, %s role is fixed. \n", obj.agent_name(1)));
                return
            end
            if prescribed_new_role == "Leader"
                obj.debug_print(sprintf("%s assumes Leader role.\n", obj.agent_name(1)));
            else 
                if prescribed_new_role == "Follower"
                    obj.debug_print(sprintf("%s assumes Follower role.\n", obj.agent_name(1)));
                else
                    obj.debug_print(sprintf("Wrong role, either Leader or Follower, default to Follower role for %s.\n", obj.agent_name(1)));
                    prescribed_new_role = "Follower";
                end
            end
            obj.role = prescribed_new_role;
        end
        function fix_role(obj, fix)
            if fix
                if obj.role_fixed
                    obj.debug_print(sprintf("%s role is already fixed.\n", obj.agent_name(1)));
                else
                    obj.debug_print(sprintf("%s role will be fixed.\n", obj.agent_name(1)));
                    obj.role_fixed = true;
                end
            else
                if ~obj.role_fixed
                    obj.debug_print(sprintf("%s role is already unfixed.\n", obj.agent_name(1)));
                else
                    obj.debug_print(sprintf("%s role will be unfixed.\n", obj.agent_name(1)));
                    obj.role_fixed = false;
                end
            end
        end
        function print_role_estimation_info(obj)
            obj.debug_print(sprintf("Actual %s role: %s \n", obj.agent_name(1), obj.role));
            obj.debug_print(sprintf("%s view %s's role: l %f, f %f \n", obj.agent_name(1), obj.agent_name(2), ...
                                    obj.role_estimation(1), obj.role_estimation(2)));
        end
        function trans_prob = cal_state_dependent_role_trans_prob(obj, x)
            % A simple explicit state dependent role transition probability matrix
            % does not depend on the current role of the vehicles
            % does not depend on the role estimation of the other vehicle

            % update this function to make more complex definitions.

            % The state x is 1-D motion [ego_s, ego_v, other_s, other_v], 
            % distance to the merge point 0, initially should both start from negative, 
             
            % calculate the difference between distance to merge dx
            % if both have yet to reach the intersection point, then both distance should be positive
            % A negative dx means ego (0) is closer to intersecting point
            dx = (0 - x(1)) - (0 - x(3));

            % from leader
            expdx_l = obj.param.k_lead * dx;
            % probability to stay leader, more likely if dx < 0 (expdx_l < 0)
            pll = 1 / (1 + exp(expdx_l));
            % probability from leader to follower
            pfl = 1 - pll;

            % from follower
            expdx_f = -obj.param.k_follower * dx;
            % probability stay follower, more likely if dx > 0 (expdx_f < 0)
            pff = 1 / (1 + exp(expdx_f));
            % probability from follower to leader
            plf = 1 - pff;
            trans_prob = [pll, plf; pfl, pff];

            % should not do this in case this is called outside of the step function
            % obj.role_trans_prob = trans_prob;
        end
        function trans_prob = cal_role_transition_probability(obj, x, prob_role_prescribed)
            %% core function to calculate the role transition probability
            %% it may depend on
            % current state of the game
            % current role of the ego vehicle
            % the prescribed role for the ego vehicle
            switch obj.role_update_type
                case 1
                    % transition matrix is fixed
                    trans_prob = obj.param.alter_matrix(:, :, 1);
                case 2
                    % transition matrix is based explicitly on state only
                    % old implementation, not used any more
                    trans_prob = obj.cal_state_dependent_role_trans_prob(x);
                case 3
                    % based on prescribed role and alter matrix
                    % note that this provided alter matrix need to be valid probability matrix
                    % it may not need to be symmetric
                    trans_prob = obj.param.alter_matrix(:, :, 1) * prob_role_prescribed(1) + ...
                                 obj.param.alter_matrix(:, :, 2) * prob_role_prescribed(2);
                otherwise
                    % no role changes % technically should not be here.
                    trans_prob = eye(2);
            end
            % should not do this in case this is called outside of the step function
            % obj.role_trans_prob = trans_prob;
        end
        function new_role = get_new_role(obj, v)
            if ~exist('v', 'var')
                v = rand(1); % pick a uniform value between 0 and 1
            end
            new_role = obj.role;
            pll = obj.role_trans_prob(1, 1);
            pff = obj.role_trans_prob(2, 2);
            if obj.role == "Leader"
                if v > pll
                    obj.debug_print(sprintf("current %s role is Leader, need to change to Follower sample %.2f > %.2f.\n", obj.agent_name(1), v, pll));
                    new_role = "Follower";
                else
                    obj.debug_print(sprintf("current %s role is Leader, remain, sample %.2f <= %.2f.\n", obj.agent_name(1), v, pll));
                end
            else
                if v > pff
                    obj.debug_print(sprintf("current %s role is Follower, need to change to Leader sample %.2f > %.2f.\n", obj.agent_name(1), v, pff));
                    new_role = "Leader";
                else
                    obj.debug_print(sprintf("current %s role is Follower, remain, sample %.2f <= %.2f.\n", obj.agent_name(1), v, pff));
                end
            end
        end
        function prescribed_role_prob = cal_prescribed_role_prob(obj, other_role_est)
            % calculate the prescribed role probability based on the role estimation of the other vehicle
            % in case when the new role can be given as deterministic (e.g. maximum likelihood estimation)
            % it will also be returned.
            % prescribed_new_role = [];
            switch obj.role_prescription_type
                case 1
                     % maximum likelihood estimation
                    % remark: this is because for two roles, the complement of each role is the other role
                    % it is thus equivalent to 
                    %   [~, idx] = max(1 - other_role_est);
                    %   prescribed_new_role = obj.roles(idx);
                    if other_role_est(1) == other_role_est(2)
                        % tie breaker (only for two roles)
                        % keep the current role
                        obj.debug_print(sprintf("Warning, %s has role estimation on %s is equal, keep the current one \n", obj.agent_name(1), obj.agent_name(2)));
                        if obj.role == "Leader"
                            prescribed_role_prob = [1; 0];
                        else
                            prescribed_role_prob = [0; 1];
                        end
                        % prescribed_new_role = obj.role;
                    else
                        [~, idx] = max(other_role_est);
                        prescribed_role_prob = 0 * other_role_est;
                        % take the complementary role's index
                        prescribed_role_prob(obj.roles_complement_index(idx)) = 1;
                        % this gives a deterministic role prescription
                        % however, this does not mean the final transition matrix will be deterministic
                        % prescribed_new_role = obj.roles_complement(idx);
                    end
                case 2
                    % continuous, prefer to take complementary role with the same probability
                    prescribed_role_prob = other_role_est(obj.roles_complement_index);
                    % only works for two roles
                    % prescribed_role_prob = 1 - other_role_est;
                otherwise
                    % should not be here, but default to 0.5 for each role
                    % no preference
                    prescribed_role_prob = 0.5 * ones(2, 1);
            end
        end
        function [u0, u1, game_info] = step(obj, x_ini, other_measure)
            % Note: it seems that matlab allows missing inputs (order
            % matters) can be check with exist("other_measure"), but this
            % is not a good practice

            % First part, get the role transition matrix
            % step on prior role estimation, the order is always leader -> follower
            other_role_est = obj.role_estimation;
            % acquire the prescribed role probability from role estimation
            prescribed_role_prob = obj.cal_prescribed_role_prob(other_role_est);
            % determine the role transition matrix
            trans_prob = obj.cal_role_transition_probability(x_ini, prescribed_role_prob);
            % assign the role_trans_prob matrix here for role adaptation.
            % only do this during the step.
            obj.role_trans_prob = trans_prob;

            % Second part, get the action based on the role
            % here make difference between whether do role before or after the action
            if ~isfield(obj.x_other_trace_predicted, "trace")
                % initialize the trace
                obj.x_other_trace_predicted.trace = cat(3, other_measure.trace, other_measure.trace);
            end
            if obj.role_update_schedule == 2
                % first get action and then update role
                [u0, u1, game_info] = obj.get_veh_decision(x_ini);
                prescribed_new_role = obj.get_new_role();
                obj.assign_role(prescribed_new_role);
            else
                % first update role and then get action
                prescribed_new_role = obj.get_new_role();
                obj.assign_role(prescribed_new_role);
                % get the action based on new roles
                [u0, u1, game_info] = obj.get_veh_decision(x_ini);
            end
            obj.status = game_info;
            % update role estimation
            obj.update_role_estimation(other_measure.trace);
            % and update prediction for future decision
            % assuming that the prediction tensor is always (:,:, 1) for leader, (:,:, 2) for the follower
            % this allows the role_estimation is also leader -> follower
            [~, other_trace_prediction] = obj.get_full_prediction(x_ini, other_measure.time);
            obj.x_other_trace_predicted.trace = other_trace_prediction;
        end
        function r_bars = cal_combine_reward(obj, ego_ini, ego_trace, ego_control_trace, ...
                                                                       other_ini, other_trace,  other_control_trace)
            % (TODO) add a unit test for this function
            % (TODO) for safety calculation, make the collision term also speed dependent, this way the vehicle would at least slow down
            % for intersection case it is easier to calculate together
            % each vehicle should only have 2 state, the distance and speed and it is only 1-D
            ego_idx = 1;
            other_idx = 1;
            ego_idx_v = 2;
            other_idx_v = 2;
            ego_liveness = ego_trace(ego_idx, :) - ego_ini(ego_idx);
            ego_control_effort = abs(ego_control_trace);
            other_liveness = other_trace(other_idx, :) - other_ini(other_idx);
            other_control_effort = abs(other_control_trace);
            % NOTE: this is an simplified calculation based on the assumption that the two directions are orthogonal!!!
            dist = sqrt(ego_trace(ego_idx, :).^2 + other_trace(other_idx, :).^2);
            collision_indicator = dist < obj.param.reward.min_dist;
            collision_ego = collision_indicator .* (1 + ego_trace(ego_idx_v, :));
            collision_other = collision_indicator .* (1 + other_trace(other_idx_v, :));
            step_reward_ego = ego_liveness - obj.param.reward.w_collision * collision_ego - obj.param.reward.w_control * ego_control_effort;
            step_reward_other = other_liveness - obj.param.reward.w_collision * collision_other - obj.param.reward.w_control * other_control_effort;
            r_bar0 = step_reward_ego * obj.param.reward.ratio_vector;
            r_bar1 = step_reward_other * obj.param.reward.ratio_vector;
            r_bars = [r_bar0, r_bar1];
        end
        function [Rbar0, Rbar1, all_trajectories] = calculate_full_Rbar(obj, x_ini, u0_tensor, u1_tensor)
            tStart = tic;
            n_seq_u0 = size(u0_tensor, 3);
            n_seq_u1 = size(u1_tensor, 3);
            obj.debug_print(sprintf("R matrix calculation for u0 u1 of size %d %d may take some time...\n", n_seq_u0, n_seq_u1));
            s0_trace = obj.get_state_trace(x_ini(1:2), u0_tensor);
            s1_trace = obj.get_state_trace(x_ini(3:4), u1_tensor);
            % for both R matrixes
            % each role corresponds to one u1 sequence, 
            % each column corresponds to one u0 sequence
            [Rbar0, Rbar1] = obj.calculate_pair_reward_from_u_and_s(x_ini, s0_trace, u0_tensor, s1_trace, u1_tensor);
            time_used = toc(tStart);
            all_trajectories = {s0_trace, s1_trace};
            obj.debug_print(sprintf("R matrix calculation takes %.4f [sec] for u0 u1 of size %d %d.\n", time_used, n_seq_u0, n_seq_u1));
        end
        function [Rbar0, Rbar1, all_trajectories] = calculate_full_Rbar_sample(obj, x_ini)
            tStart = tic;
            [s0_trace, u0_trace] = obj.generate_traj_sample(x_ini(1:2));
            [s1_trace, u1_trace] = obj.generate_traj_sample(x_ini(3:4));
            n_seq_u0 = size(u0_trace, 3);
            n_seq_u1 = size(u1_trace, 3);
            obj.debug_print(sprintf("R matrix calculation for based on sample, size varies may take some time...\n"));
            % for both R matrixes
            % each role corresponds to one u1 sequence, 
            % each column corresponds to one u0 sequence
            [Rbar0, Rbar1] = obj.calculate_pair_reward_from_u_and_s(x_ini, s0_trace, u0_trace, s1_trace, u1_trace);
            time_used = toc(tStart);
            all_trajectories = {s0_trace, s1_trace, u0_trace, u1_trace};
            obj.debug_print(sprintf("R matrix calculation takes %.4f [sec] for u0 u1 of size %d %d.\n", time_used, n_seq_u0, n_seq_u1));
        end        
        function [Rbar0, Rbar1] = calculate_pair_reward_from_u_and_s(obj, x_ini, s0_trace, u0_tensor, s1_trace, u1_tensor)
            % given u and s trace, calculate the combined reward
            n_seq_u0 = size(u0_tensor, 3);
            n_seq_u1 = size(u1_tensor, 3);
            % each role corresponds to one u1 sequence, 
            % each column corresponds to one u0 sequence
            Rbar1 = zeros(n_seq_u1, n_seq_u0);
            Rbar0 = zeros(n_seq_u1, n_seq_u0);
            for i = 1:n_seq_u1
               u1_sequence = u1_tensor(:, :, i);
               for j = 1:n_seq_u0
                   u0_sequence = u0_tensor(:, :, j);
                   rbars = obj.cal_combine_reward(x_ini(1:2), s0_trace(:, :, j), u0_sequence, ...
                                                                                x_ini(3:4), s1_trace(:, :, i), u1_sequence);
                   Rbar0(i, j) = rbars(1); 
                   Rbar1(i, j) = rbars(2);
               end
            end
        end
        function [u0, u1, game_info] = get_veh_decision(obj, x_ini)
            if ~obj.param.sample_traj
                % default use discretized action sequence
                u0_trace = obj.u0_tensor;
                u1_trace = obj.u1_tensor;
                % make ego's own decision.
                [Rbar0, Rbar1, all_trajectories] = obj.calculate_full_Rbar(x_ini, u0_trace, u1_trace);
                x0_tensor = all_trajectories{1};
                x1_tensor = all_trajectories{2};
                t_steps = (1:obj.param.N) * obj.dt;
            else
                [Rbar0, Rbar1, all_trajectories] = obj.calculate_full_Rbar_sample(x_ini);
                x0_tensor = all_trajectories{1};
                x1_tensor = all_trajectories{2};
                u0_trace = all_trajectories{3};
                u1_trace = all_trajectories{4};
                t_steps = (1:obj.param.full_sample_len) * obj.param.sample_dt;
            end

            % technically only need to calculate one, but consider that getting index is not complex, calculate both.
            [U, Q_u0_as_follower] = obj.get_follower_sequences_idx(Rbar0, Rbar1);
            u0_sequence_u0_as_follower = u0_trace(:,:,U{1});
            u1_sequence_u0_as_follower = u1_trace(:,:,U{2});
            u0_action_u0_as_follower = U{1}; % will be role, only two dimensions
            u1_action_u0_as_follower = U{2}; % will be column, only two dimensions
            x0_sequence_u0_as_follower = x0_tensor(:,:,U{1});
            x1_sequence_u0_as_follower = x1_tensor(:,:,U{2});

            [U, Q_u0_as_leader] = obj.get_leader_sequences_idx(Rbar0, Rbar1);
            u0_sequence_u0_as_leader = u0_trace(:,:,U{1});
            u1_sequence_u0_as_leader = u1_trace(:,:,U{2});
            u0_action_u0_as_leader = U{1};
            u1_action_u0_as_leader = U{2};
            x0_sequence_u0_as_leader = x0_tensor(:,:,U{1});
            x1_sequence_u0_as_leader = x1_tensor(:,:,U{2});

            % Besides the action (u), the target trajectories is also provided to allow tracking by the vehicle
            % this implicitly allows different resolution between game and simulation
            if obj.role == "Leader"
                if size(u0_sequence_u0_as_leader, 3) > 1
                    obj.debug_print("Warning, u0 as leader is not unique, take the first one. \n");
                end
                u0 = u0_sequence_u0_as_leader(:, :, 1);
                u0_action = u0_action_u0_as_leader(1);
                x0_target_trace = x0_sequence_u0_as_leader(:, :, 1);
                if size(u1_sequence_u0_as_leader, 3) > 1
                    obj.debug_print("Warning, u1 as follower is not unique, take the first one. \n");
                end
                u1 = u1_sequence_u0_as_leader(:, :, 1);
                u1_action = u1_action_u0_as_leader(1);
                x1_target_trace = x1_sequence_u0_as_leader(:, :, 1);
            else
                if obj.role == "Follower"
                    if size(u0_sequence_u0_as_follower, 3) > 1
                        obj.debug_print("Warning, u0 as leader is not unique, take the first one. \n");
                    end
                    u0 = u0_sequence_u0_as_follower(:, :, 1);
                    u0_action = u0_action_u0_as_follower(1);
                    x0_target_trace = x0_sequence_u0_as_follower(:, :, 1);
                    if size(u1_sequence_u0_as_follower, 3) > 1
                        obj.debug_print("Warning, u0 as leader is not unique, take the first one. \n");
                    end
                    u1 = u1_sequence_u0_as_follower(:, :, 1);
                    u1_action = u1_action_u0_as_follower(1);
                    x1_target_trace = x1_sequence_u0_as_follower(:, :, 1);
                else
                    error("Should not be here.\n")
                end
            end

            game_info.x0_target_trace = x0_target_trace;
            game_info.x1_target_trace = x1_target_trace;
            % the expected trajectories if the other agent is running as leader / follower
            % note that it is in a different order.
            game_info.other_expected_trace = cat(3, x1_sequence_u0_as_follower, x1_sequence_u0_as_leader);
            game_info.role_estimated_prob = obj.role_estimation;
            game_info.u0_sequence_u0_as_follower = u0_sequence_u0_as_follower;
            game_info.u1_sequence_u0_as_follower = u1_sequence_u0_as_follower;
            game_info.u0_sequence_u0_as_leader = u0_sequence_u0_as_leader;
            game_info.u1_sequence_u0_as_leader = u1_sequence_u0_as_leader;
            game_info.x0_sequence_u0_as_follower = x0_sequence_u0_as_follower;
            game_info.x1_sequence_u0_as_follower = x1_sequence_u0_as_follower;
            game_info.x0_sequence_u0_as_leader = x0_sequence_u0_as_leader;
            game_info.x1_sequence_u0_as_leader = x1_sequence_u0_as_leader;
            game_info.Q_u0_as_follower = Q_u0_as_follower;
            game_info.Q_u0_as_leader = Q_u0_as_leader;
            game_info.Rbar0 = Rbar0;
            game_info.Rbar1 = Rbar1;
            game_info.role = obj.role;
            game_info.role_trans_prob = obj.role_trans_prob;
            game_info.action = [u0_action; u1_action];
            game_info.t_steps = t_steps;
            game_info.x_ini = x_ini;
        end
        function [ego_prediction, other_prediction, game_info] = get_full_prediction(obj, x_ini, prediction_horizon)
            % This function would fully calculate the prediction based on the current state
            % it will formulate the game and calculate the prediction
            % TODO: for now only check x_ini to see if can reuse.
            % obj.status is only assigned when executing "step" function, get_veh_decision would not save the status
            if ~isempty(obj.status) && isequal(obj.status.x_ini, x_ini)
                % judged based on initial conditions
                % if the status is already calculated, just use it
                obj.debug_print("Use the existing status for prediction as the initial condition is the same.\n");
                game_info = obj.status;
            else
                % if the status is not calculated, calculate it
                [~, ~, game_info] = obj.get_veh_decision(x_ini);
            end
            if ~exist("prediction_horizon", "var")
                % provide prediction with the full horizon and time step
                prediction_horizon = [0, game_info.t_steps];
            else
                if prediction_horizon(end) > game_info.t_steps(end)
                    error("Error Prediction horizon exceeds the game horizon.\n");
                end
            end
            [ego_prediction, other_prediction] = obj.prediction_core(x_ini, game_info, prediction_horizon);
            obj.debug_print(sprintf("prediction output order %s %s.\n", obj.agent_name(1), obj.agent_name(2)));
        end
        function [ego_prediction, other_prediction] = prediction_core(~, x_ini, game_info, prediction_horizon)
            raw_horizon = [0, game_info.t_steps];
            % note that the optimal trajectories may not be unique,
            % take the first one to be consistent with optimal
            % selection
            ego_leader_trace = interp1(raw_horizon, [x_ini, [game_info.x0_sequence_u0_as_leader(:, :, 1);...
                                                                                   game_info.x1_sequence_u0_as_leader(:, :, 1)]]', ...
                                                                                   prediction_horizon)';
            other_leader_trace = interp1(raw_horizon, [x_ini, [game_info.x0_sequence_u0_as_follower(:, :, 1); ...
                                                                                      game_info.x1_sequence_u0_as_follower(:, :, 1)]]', ...
                                                                                      prediction_horizon)';
            % TODO: Is there a better way?
            % for both ego prediction and other prediction, the role
            % dependent prediction is ordered in leader -> follower role 
            ego_prediction = cat(3, ego_leader_trace(1:2, :), other_leader_trace(1:2, :));
            other_prediction = cat(3, other_leader_trace(3:4, :), ego_leader_trace(3:4, :));
        end
    end
end

