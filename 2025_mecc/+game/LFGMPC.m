classdef LFGMPC < game.LFGBaseClass
    %LFGMPC Summary of this class goes here
    %   An discrete MPC algorithm where the interactive agent's future is
    %   provided using LFG with initial role probability, and the state dependent role
    %   transition probability is assumed to be known.
    % Specifically, this is for intersection and 2 vehicle only, 
    %   where only 1-D motion for each vehicle, starts from negative to positive,
    %   and assumes that the only conflict point would be when the distance is 0. 
    
    properties
        param % abstract in base class
        class_name % abstract in base class
        num_roles
    end
    
    methods
        function obj = LFGMPC(param)
            %LFGMPC Construct an instance of this class
            %   Detailed explanation goes here
            obj.param = param;
            if isfield(obj.param, "agent_name")
                obj.agent_name = obj.param.agent_name;
            end
            obj.debug_print("This Agent is a LFGMPC agent.\n");
            obj.class_name = "LFGMPC";
            obj.debug_print(sprintf("Agent name '%s' for the ego vehicle, '%s' for the other vehicle .\n", ...
                                    obj.agent_name(1), obj.agent_name(2)));
            % obj.dt = obj.param.deltaT;
            obj.num_roles = length(obj.roles);
            % this ratio is used for reward calculation and remain the same as long as horizon does not change.
            if ~obj.param.sample_traj
                ratio = zeros(obj.param.N, 1);
                for i = 1: obj.param.N
                    ratio(i) = obj.param.reward.lambda ^ (i - 1);
                end
            else
                ratio = zeros(obj.param.full_sample_len, 1);
                obj.debug_print("Sample trajectory is used, the ratio is calculated based on the sample_dt.\n");
                for i = 1: obj.param.full_sample_len
                    ratio(i) = obj.param.reward.lambda ^ (i - 1);
                end
            end
            obj.param.reward.ratio_vector = ratio;
            obj.initialize_action_sequence();
            obj.role_estimation = [0.5; 0.5];
            if isfield(obj.param, 'T1')
                obj.T1 = obj.param.T1;
            else
                % only for sampled base so the smallest should be with sample dt
                % obj.T1 = obj.dt;
                obj.T1 = obj.param.sample_dt;
            end
            if ~isfield(obj.param, 'prob_seq_type')
                obj.param.prob_seq_type = 1;
                obj.debug_print("Chance constrain prob_seq_type is not given, set to 1 (default).\n");
            end
            obj.x_other_trace_predicted.time = 0:obj.param.sample_dt:obj.T1;
            obj.status = [];
            % check parameter for anomaly
            obj.check_param();
        end
        function print_role_estimation_info(obj)
            obj.debug_print(sprintf("%s view %s's role: l %f, f %f \n", obj.agent_name(1), obj.agent_name(2), ...
                                    obj.role_estimation(1), obj.role_estimation(2)));
        end
        function [step_reward, safety_satisfy] = calculate_safety_and_reward(obj, ego_ini, ego_state_trace, ego_control_trace, other_state_trace_tensor)
            n_other_trace = size(other_state_trace_tensor, 3);
            % the following does not need to be true any more if there is a tree expand
            % assert(n_other_trace == obj.num_roles);
            ego_idx = 1;
            other_idx = 1;
            ego_idx_v = 2;
            % for reward does not count initial state because it can not be changed
            trace_length = size(ego_state_trace, 2);
            safety_satisfy = zeros(n_other_trace, trace_length);
            step_reward = zeros(n_other_trace, trace_length);
            for i = 1:n_other_trace
                 dist = sqrt(ego_state_trace(ego_idx, :).^2 + other_state_trace_tensor(other_idx, :, i).^2);
                 safety_satisfy(i, :) = dist >= obj.param.reward.min_dist;
                 step_reward(i, :) = obj.cal_reward(ego_ini, ego_state_trace, ego_control_trace, other_state_trace_tensor(:, :, i));
                 % similar to game objective function, penalize if the safety is not satisfied
                 step_reward(i, :) = step_reward(i, :) - obj.param.reward.w_collision * ~safety_satisfy(i, :) .* (1 + ego_state_trace(ego_idx_v, :));
            end
        end
        function step_reward_ego = cal_reward(obj, ego_ini, ego_trace, ego_control_trace, other_trace)
            % For this simple intersection case, the reward (if not consider collision) is only about liveness and control effort
            % the collision is considered in the safety constraints does not need to be considered here. As a results the other trace is not used.
            ego_idx = 1;
            % other_idx = 1;
            ego_liveness = ego_trace(ego_idx, :) - ego_ini(ego_idx);
            ego_control_effort = abs(ego_control_trace);
            % dist = sqrt(ego_trace(ego_idx, :).^2 + other_trace(other_idx, :).^2);
            % collision = dist < obj.param.reward.min_dist;
            step_reward_ego = ego_liveness - obj.param.reward.w_control * ego_control_effort;
            % step_reward_ego is a row, obj.param.reward.ratio_vector is a
            % column
            step_reward_ego = step_reward_ego .* obj.param.reward.ratio_vector(:)';
        end
        function [u_mpc, mpc_info] = get_proactive_sequence(obj, x_ini, other_measure, agent_model)
            % This function requires the agent_model to give closed loop behavior prediction
            % note that this agent_model should be wrapped so that it can convert mpc states to game states
            % agent_model should have the following functions:
            %  get_full_prediction(x_T1, time_seq) -> other_state_prediction, ego_state_prediction
            %  get_x_errors(x1, x2) -> the error between two state trajectories
            %  estimate_role(prior, error) -> post
            other_role_prob = obj.role_estimation;
            if ~isfield(obj.x_other_trace_predicted, "trace")
                % initialize the trace
                obj.x_other_trace_predicted.trace = cat(3, other_measure.trace, other_measure.trace);
            end
            x_t_in_agent_view = [other_measure.trace(:, 1); x_ini];
            % for now only support sample trajectories
            assert(obj.param.sample_traj);
            if agent_model.role_update_type == 3 && agent_model.role_prescription_type == 1
                obj.debug_print(sprintf("Assume %s is using a MLE + estimation based estimation.\n", obj.agent_name(2)));
            end
            trace_length = obj.param.full_sample_len;
            tStart = tic;

            % first generate the first part of the prediction on the other vehicle's trajectories
            % this part should be "inferred" from the agent model, as it does not depend on the ego's action/profile
            % (TODO) need to do coordinate change to the other vehicle's perspective
            %        for now it is OK in the game both vehicle are 1-d motion, so coordinate is the same

            % Remark for both predictions from agent model, it is in the order of leader->follower role for each agents
            % this means that the first dimension (:, :, 1) for both agents are when they are taking leader roles,
            % and the trajectories with the same (third) index ARE NOT complement to each other
            %  i.e.,  other_state_prediction_part1(:, :, 1) when the other is taking the leader role
            %         ego_state_prediction_part1(:, :, 1) when the ego is taking the leader role
            [other_state_prediction_part1, ego_state_prediction_part1] = agent_model.get_full_prediction(x_t_in_agent_view, ...
                                                                                                         0:obj.param.sample_dt:obj.T1);
            trace_length_part1 = size(other_state_prediction_part1, 2);
            trace_length_part2 = trace_length - trace_length_part1;

            % will build this full prediction from 2 parts, and because there are two roles from T1 at each spot,
            % there will be 4 possible trajectories for the agent
            other_state_full_prediction = zeros(obj.num_roles, obj.param.full_sample_len, obj.num_roles * obj.num_roles);
            
            % the first part is given by the estimation at current role
            % every n_roles rows are the same corresponds to the same role
            for i_role = 1:obj.num_roles
                for j_role = 1:obj.num_roles
                    idx = (i_role - 1) * obj.num_roles + j_role;
                    other_state_full_prediction(:, 1:trace_length_part1, idx) = ...
                    other_state_prediction_part1(:, 1:trace_length_part1, i_role);
                end
            end

            % for the ego sample trajectories sample, it is full horizon
            % (TODO/Question) Does it make sense to resample ego from the intermediate point T1?
            [ego_state_trace_tensor, ego_control_trace_tensor] = obj.generate_traj_sample(x_ini);

            n_ego_trace = size(ego_state_trace_tensor, 3);
            n_other_trace = size(other_state_full_prediction, 3);

            step_reward_all = zeros(n_other_trace, trace_length, n_ego_trace);
            cum_reward_all = zeros(1, n_ego_trace);
            step_safety_constraint_all = zeros(n_other_trace, trace_length, n_ego_trace);
            step_chance_constraint_all = zeros(n_ego_trace, trace_length);
            chance_constraint_all = zeros(1, n_ego_trace);

            % (TODO) this may be a major limitation?
            % assume others estimation on ego always starts from uniform distribution.
            % essentially always takes complimentary role to the role with maximum likelihood.
            other_estimate_ego_prior = ones(obj.num_roles, 1) / obj.num_roles;

            other_state_full_prediction_prob = zeros(obj.num_roles * obj.num_roles, 1);

            for i = 1:n_ego_trace
                % For each ego trace, first build the full prediction for the other vehicle

                % At T1, the other vehicle make a role estimation on ego.
                % This estimation is based on the error of ego vehicle between t and T1
                % against the prediction of ego vehicle by the other vehicle generated at t.
                % thus does not depend on the role taken by the other vehicle at t.
                ego_errors = agent_model.get_x_errors(ego_state_trace_tensor(:, 1:trace_length_part1, i), ego_state_prediction_part1);
                % The other vehicle at T1 will take use the action by the ego vehicle between 0 to T1 to make an estimation on ego's role
                role_est_agent_on_ego = agent_model.estimate_role(other_estimate_ego_prior, ego_errors);

                prescribed_role_prob = agent_model.cal_prescribed_role_prob(role_est_agent_on_ego);
                
                % finish building the rest of the trajectories from this point.
                for i_role = 1:obj.num_roles
                    % state needs to be in the game order, that is, 
                    %   the first two are for the other vehicle,
                    %   the second two are for the ego vehicle
                    x_T1_in_agent_view = [other_state_prediction_part1(:, trace_length_part1, i_role); 
                                          ego_state_trace_tensor(:, trace_length_part1, i)];
                    % get the role transition probability matrix
                    trans_prob = agent_model.cal_role_transition_probability(x_T1_in_agent_view, prescribed_role_prob);
                    idx = (i_role - 1) * obj.num_roles + 1 : i_role * obj.num_roles;
                    other_state_full_prediction_prob(idx) = trans_prob(:, i_role) * other_role_prob(i_role);
                    % calling the get full prediction essentially solve another game from the starting point.
                    [other_state_prediction_part2, ~, ~] = agent_model.get_full_prediction(x_T1_in_agent_view);
                    for j_role = 1:obj.num_roles
                        idx = (i_role - 1) * obj.num_roles + j_role;
                        other_state_full_prediction(:, trace_length_part1 + 1:end, idx) = other_state_prediction_part2(:, 2:trace_length_part2 + 1, j_role);                   
                    end
                end
                
                % each state builds a new (potentially different other trajectories)
                % ready to calculate the reward and check safety constrains
                [step_reward, safety_satisfy] = obj.calculate_safety_and_reward(x_ini, ... 
                                                                                ego_state_trace_tensor(:, :, i), ... 
                                                                                ego_control_trace_tensor(:, :, i), ...
                                                                                other_state_full_prediction);
                step_safety_constraint_all(:, :, i) = safety_satisfy;
                step_reward_all(:, :, i) = step_reward;
                % the probability for each trajectories by the others is the same at each time step, 
                % all given by other_state_full_prediction_prob
                cum_reward_all(i) = other_state_full_prediction_prob(:)' * sum(step_reward, 2);
                % if the probability is 0, the corresponding constraint is 0
                step_chance_constraint = other_state_full_prediction_prob(:)' * safety_satisfy;
                step_chance_constraint_all(i, :) = step_chance_constraint;
                chance_constraint_all(i) = all(step_chance_constraint > 1- obj.param.epsilon);
            end

            % once the reword is built, the rest of the calculation is the same as the normal MPC
            % it is possible that no feasible solution exist.
            if all(chance_constraint_all == 0)
                warning("Warning, no feasible solution by MPC; find the solution with maximum objective function still. \n");
                infeasible = true;
            else
                infeasible = false;
                cum_reward_all(chance_constraint_all == 0) = -inf;
            end
            [max_reward, max_reward_idx] = max(cum_reward_all);
            u_action = max_reward_idx(1);

            time_used = toc(tStart);
            obj.debug_print(sprintf("Role influencing MPC takes %.4f [sec] for action size of %d.\n", time_used, size(ego_state_trace_tensor, 3)));
            u_mpc_seq = ego_control_trace_tensor(:, :, max_reward_idx);
            u_mpc = u_mpc_seq(1);
            mpc_info = struct();
            mpc_info.u_optimal_sequence = u_mpc_seq;
            mpc_info.max_reward = max_reward;
            mpc_info.max_reward_idx = max_reward_idx;
            % additional logs with everything.
            mpc_info.cum_reward_all = cum_reward_all;
            mpc_info.chance_constraint_all = chance_constraint_all;
            mpc_info.step_chance_constraint_all = step_chance_constraint_all; % size changes
            mpc_info.step_reward_all = step_reward_all; % size changes
            mpc_info.step_safety_constraint_all = step_safety_constraint_all; % size changes
            mpc_info.action = u_action;
            mpc_info.infeasible = infeasible;
            obj.status = mpc_info;

            % lastly update the role estimation for the next round
            obj.update_role_estimation(other_measure.trace);
            % and also update the prediction results using agent model
            [other_state_prediction, ~] = agent_model.get_full_prediction(x_t_in_agent_view, other_measure.time);
            obj.x_other_trace_predicted.trace = other_state_prediction;
        end
    end
end

