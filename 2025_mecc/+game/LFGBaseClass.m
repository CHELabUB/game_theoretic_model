classdef LFGBaseClass < handle
    properties (Abstract)
        % make it abstract so that it required to be redefined by subclass
        % Note that this also make the class abstract and thus cannot be
        % instantiated
        param
        class_name
    end
    % these can not be redefined, but can still be overridden by subclass
    properties
        role
        agent_name = ["ego", "other"]
        roles = ["Leader", "Follower"]
        roles_complement = ["Follower", "Leader"] % the complement of each roles (straightforward for 2 roles, not in general)
        roles_complement_index = [2, 1] % the complement of each roles (straightforward for 2 roles, not in general)
        u0_tensor % action sequence tensor by ego
        u1_tensor % action sequence tensor by the other agent
        dt % sequence based time step
        status % status of the class, used to keep the latest solution info
        T1 % used for role estimation schedule
        role_estimation % carry the role estimation for the other vehicle
        x_other_trace_predicted % a tensor of the other vehicle's state prediction based on different roles
    end
    methods
        function obj = LFGBaseClass(varargin)
            %LFGBASECLASS Construct an instance of this class
            % This super class does nothing at initialization.
            % Base class contains basic sample generation, role estimation, and utility functions.
            % Reward calculation are to be implemented by subclass.
            % fprintf("Need to be overridden by subclass\n");
        end
        function debug_print(obj, msg)
            if ~isfield(obj.param, "print_off") || obj.param.print_off == false
                fprintf(msg);
            end
        end
        function initialize_action_sequence(obj)
            % the action sequence is independent of states.
            if isfield(obj.param, "u0_action_set")
                obj.debug_print("Initialize u0 actions sets with all possible combinations.\n");
                obj.u0_tensor = obj.generate_all_sequences(obj.param.u0_action_set, obj.param.N, "u0");
            end
            if isfield(obj.param, "u1_action_set")
                obj.debug_print("Initialize u1 actions sets with all possible combinations.\n");
                obj.u1_tensor = obj.generate_all_sequences(obj.param.u1_action_set, obj.param.N, "u1");
            end
        end
        % % generate trajectories from all possible sequences
        function u_tensor = generate_all_sequences(obj, action_set, N, name)
            % this generate all the possible sequence
            % col num = action dimension
            % row num = num of actions
            num_of_actions = size(action_set, 1);
            action_dimension = size(action_set, 2);
            u_candidate = game.gen_sequence(num_of_actions, N);
            num_of_sequences = length(u_candidate);
            u_tensor = zeros(action_dimension, N, num_of_sequences);
            for i = 1:num_of_sequences
                u_tensor(:, :, i) = action_set(u_candidate(:, i), :)';
            end
            obj.debug_print(sprintf("%s is generated from %.d actions of dim %.d, for horizon %.d, get %.d sequences.\n", ...
                name, num_of_actions, action_dimension, N, num_of_sequences));
        end
        % Generate state sequences for longitudinal motion only.
        function s_trace_tensor = get_state_trace(obj, x_ini, u_tensor)
            n_seq = size(u_tensor, 3);
            % exclude first state
            s_trace_tensor = zeros(2, obj.param.N, n_seq);
            for i = 1:n_seq
                s_trace_tensor(:, :, i) = obj.veh_dynamics(x_ini, u_tensor(:, :, i));
            end
        end
        function s_trace = veh_dynamics(obj, x_ini, u_seq)
            % no initial states
            m = size(u_seq, 2);
            s_trace = zeros(2, m);
            for i = 1:m
                a = u_seq(1, i);
                x_ini = game.const_a_step(x_ini(1), x_ini(2), a, obj.dt, obj.param.v_max, obj.param.v_min);
                s_trace(:, i) = x_ini;
            end
        end
        function issue_found = check_param(obj)
            % check some anomaly in parameter setting and through
            % MATLAB warning.
            % by no means complete
            % should be called at the end of the child class constructor
            issue_found = 0;
            if obj.param.u_min >= obj.param.u_max
                warning("param u_min %.2f should be no larger than u_max %.2f. \n", obj.param.u_min, obj.param.u_max);
                issue_found = issue_found + 1;
            end
            if obj.param.u_min >= 0
                warning("param u_min %.2f should be negative. \n", obj.param.u_min);
                issue_found = issue_found + 1;
            end
            if obj.param.u_max <= 0
                warning("param u_max %.2f should be positive. \n", obj.param.u_max);
                issue_found = issue_found + 1;
            end
            if obj.param.v_min >= obj.param.v_max
                warning("param v_min %.2f should be no larger than v_max %.2f. \n", obj.param.v_min, obj.param.v_max);
                issue_found = issue_found + 1;
            end
            if obj.param.v_min < 0
                warning("param v_min %.2f should not be negative. \n", obj.param.v_min);
                issue_found = issue_found + 1;
            end
            if obj.param.v_max <= 0
                warning("param v_max %.2f should be positive. \n", obj.param.v_max);
                issue_found = issue_found + 1;
            end
            if issue_found > 0
                warning("Please check the parameter setting. \n");
            else
                obj.debug_print(sprintf("Parameter check passed for obj %s of class %s. \n", obj.agent_name, obj.class_name));
            end
        end
        % % generate trajectory from samples
        function [s, u] = generate_traj_sample(obj, x_ini, T)
            % generate action primitives
            % note that because there is no interaction when generating the trajectories, 
            % there are couple of different options
            % (1) with established controller (this can be for different
            % speed) Many papers e.g., the A Multiple Model Kalman Filtering
            % (2) with some basic acceleration combination pruning
            % (fluctuating acceleration leave out)
            % (3) Sampling the action space with continuous actions (Thrun's paper)
            % advantage of having fewer primitive is to increase difference
            % in terms of decision between different roles.
            % First version using sample based.
            % for intersection cases
            % Get to maximum speed, bring vehicle to stop at the stop line or keep current speed
            % may be also have an option of emergency stop?
            
            if ~exist('T', 'var')
                T = obj.param.full_sample_len * obj.param.sample_dt;
                sample_length = obj.param.full_sample_len;
            else
                sample_length = ceil(T / obj.param.sample_dt);
                T = obj.param.sample_dt * sample_length;
            end
            % u_max > 0, u_min < 0
            v_max_reachable = min(obj.param.v_max, x_ini(2) + obj.param.u_max * T);
            v_min_reachable = max(obj.param.v_min, x_ini(2) + obj.param.u_min * T);
            if v_min_reachable <=0
                % check if the vehicle can stop before the intersection
                if x_ini(1) + x_ini(2) / abs(obj.param.u_min) < -obj.param.stop_distance
                    s_target_zero = -obj.param.stop_distance;
                else
                    s_target_zero = [];
                end
            else
                % can not go to stop, then just try to reach the minimum speed possible.
                s_target_zero = [];
            end
            % v_mesh should be of length at least 2.
            v_mesh = linspace(v_min_reachable, v_max_reachable, obj.param.N_speed_mesh);
            s = zeros(2, sample_length, obj.param.N_speed_mesh + 1);
            % "keep current speed"
            [s_trace, u_trace] = obj.regulate_speed(x_ini, x_ini(2), [], sample_length);
            s(:, :, 1) = s_trace;
            u(:, :, 1) = u_trace;
            % or go to other speed, including stop the vehicle
            % 2nd trajectory is to bring vehicle to stop.
            [s_trace, u_trace] = obj.regulate_speed(x_ini, v_mesh(1), s_target_zero, sample_length);
            s(:, :, 2) = s_trace;
            u(:, :, 2) = u_trace;
            for i = 2:obj.param.N_speed_mesh
                % if isfield(obj.param, 'mesh_at_intersection') && obj.param.mesh_at_intersection
                %     if stop is possible, then regulate all the speed at the same location
                %     how ever this does not make sense as the vehicle may need to slow down
                %     to achieve speed only at location.
                %     [s_trace, u_trace] = obj.regulate_speed(x_ini, v_mesh(i), s_target_zero, sample_length);
                %     
                % else
                %     The current way: only regulate speed, vmesh is trying to achieve the speed asap.
                %     [s_trace, u_trace] = obj.regulate_speed(x_ini, v_mesh(i), [], sample_length);
                % end
                [s_trace, u_trace] = obj.regulate_speed(x_ini, v_mesh(i), [], sample_length);
                s(:, :, i + 1) = s_trace;
                u(:, :, i + 1) = u_trace;
            end
        end
        function [s_trace, u_trace] = regulate_speed(obj, x_ini, v_target, s_target, full_sample_len)
            ovm_params = obj.param.ovm_params;
            % use car following model to regulate speed
            sample_dt = obj.param.sample_dt;
            % if ~exist('full_sample_len', 'var')
            %     full_sample_len = obj.param.full_sample_len;
            % end
            s_trace = zeros(2, full_sample_len);
            u_trace = zeros(1, full_sample_len);
            x = x_ini;
            h_target = v_target / ovm_params.kappa + ovm_params.hst + ovm_params.l_veh;
            if nargin < 4 || isempty(s_target)
                % only regulate speed
                for i = 1:full_sample_len
                    u = obj.car_following(x, [x(1) + h_target, v_target]);
                    x = game.const_a_step(x(1), x(2), u, sample_dt, obj.param.v_max, obj.param.v_min);
                    u_trace(i) = u;
                    s_trace(:, i) = x;
                end
            else
                % also regulate distance
                for i = 1:full_sample_len
                    u = obj.car_following(x, [s_target + ovm_params.hst + ovm_params.l_veh, v_target]);
                    x = game.const_a_step(x(1), x(2), u, sample_dt, obj.param.v_max, obj.param.v_min);
                    u_trace(i) = u;
                    s_trace(:, i) = x;
                end
            end
        end
        function u = car_following(obj, x, x_lead)
            s = x(1);
            v = x(2);
            s1 = x_lead(1);
            v1 = x_lead(2);
            % gives back capped ovm responses.
            u = obj.cap_u(game.OVM(s1 - s - obj.param.ovm_params.l_veh, v, v1, obj.param.ovm_params));
        end
        function u_capped = cap_u(obj, u)
            u_capped = max(obj.param.u_min, min(u, obj.param.u_max));
        end

        % reward and helper functions
        function r_bar = cal_reward(obj)
            % to be specified for different case
            % for intersection case
            r_bar = 0;
            error('Error: not Implemented.\n')
        end

        %% role estimations (this part does not depend on the environment
        function [role_post, probs] = estimate_role(obj, role_priors, x_errors)
            % estimate the other's role based on the errors
            % This is a simple Bayesian estimation with assumption that role would stay constant.
            % Note this errors may be generated based on multiple time instances in nontrivial manners.
            % note that this can be used for both agents, as long as the the x_errors of size [n_states, n_roles]
            sigma = obj.param.noise_W;
            % use the same Gaussian error for role probability assessment.
            [n_states, n_roles] = size(x_errors);
            assert(all(size(role_priors) == [n_roles, 1]))
            assert(all(size(sigma) == [n_states, n_states]))
            probs = zeros(1, n_roles);
            for i = 1:n_roles
                probs(i) = mvnpdf(x_errors(:, i), zeros(n_states, 1), sigma);
            end
            role_post = (probs'.*role_priors) / (probs * role_priors);
        end
        function x_errors = get_x_errors(~, x_actual_traces, x_predicted_traces)
            % get the errors of the target trajectories
            % this is used for role estimation
            n_roles = size(x_predicted_traces, 3);
            n_states = size(x_actual_traces, 1);
            x_errors = zeros(n_states, n_roles);
            for i = 1:n_roles
                % (TODO) is this the best way? 
                % Use the mean absolute error as the error metric for now
                x_errors(:, i) = mean(abs(x_actual_traces - x_predicted_traces(:, :, i)), 2);
            end
        end
        function update_role_estimation(obj, x_other_trace_actual)
            % update the role estimation based on the actual traces
            % this is used for role estimation
            % Note that the role estimation does not matter for the order
            % of the predicted trace. The corresponding role estimation
            % will corresponds to the order of the trace prediction role
            x_errors = obj.get_x_errors(x_other_trace_actual, obj.x_other_trace_predicted.trace);
            [role_post, ~] = obj.estimate_role(obj.role_estimation, x_errors);
            obj.role_estimation = role_post;
        end
        %% core game functions based on reward table determine the optimal actions
        % this part does not depend on the environment.
        function [U, Q] = get_leader_sequences_idx(obj, Rbar0, Rbar1)
            % a full brute forth search for U(index) and Q          
            % each row corresponds to one u1 sequence, 
            % each column corresponds to one u0 sequence

            % vehicle 0 (ego) is the leader
            % vehicle 1 (other vehicle) is the follower
            % veh 1 select the worse case for each control sequence
            Q1_min = min(Rbar1, [], 2);
            Q1_max_min = max(Q1_min);
            U1_optimal_idx = find(Q1_min == Q1_max_min);
            if size(U1_optimal_idx, 1) > 1
               obj.debug_print("U1 optimal set is not unique.\n");
            end

            Q1_follower = Q1_min;

            % base on veh 1's section, veh 0 select the best out of all
            % U0_options

            % for leader first it gets the optimal sequences by U1
            % takes the maximum among each roles
            if size(U1_optimal_idx, 1) > 1
                Q0_max = max(Rbar0(U1_optimal_idx, :));
            else
                Q0_max = Rbar0(U1_optimal_idx, :);
            end

            U0_optimal_idx = find(Q0_max == max(Q0_max));
            if size(U0_optimal_idx, 2) > 1
                obj.debug_print("U0 optimal set is not unique.\n");
            end
            Q0_leader = Q0_max(:);

            % summarize the Q and U mapping for potential probability
            % analysis, veh0
            
            U = {U0_optimal_idx, U1_optimal_idx};
            % should both be column vectors.
            Q = {Q0_leader, Q1_follower};
        end
        function [U, Q] = get_follower_sequences_idx(obj, Rbar0, Rbar1)
            % a full brute forth search for U and Q
            % Note to give flexibility, 
            % Rbar0 and Rbar1 are of same size
            % each row corresponds to one u1 sequence, 
            % each column corresponds to one u0 sequence

            % vehicle 0 is the follower 
            % vehicle 1 is the leader 
            % veh 0 will consider the worse cases and find max min solution
            Q0_min = min(Rbar0, [], 1);
            Q0_max_min = max(Q0_min);
            U0_optimal_idx = find(Q0_min == Q0_max_min);
            if size(U0_optimal_idx, 2) > 1
               obj.debug_print("U0 optimal set is not unique.\n");
            end
            Q0_follower = Q0_min(:);

            % base on veh 0's section, veh 1 select the best out of all
            % U1_options

            % for leader first it gets the optimal sequences by U1
            % takes the maximum among each rows
            if size(U0_optimal_idx, 2) > 1
                Q1_max = max(Rbar1(:, U0_optimal_idx), [], 2);
            else
                Q1_max = Rbar1(:, U0_optimal_idx);
            end
            U1_optimal_idx = find(Q1_max == max(Q1_max));
            if size(U1_optimal_idx, 1) > 1
                obj.debug_print("U1 optimal set is not unique.\n");
            end

            Q1_leader = Q1_max(:);
            
            % summarize the Q and U mapping for potential probability
            % analysis, veh0
            
            U = {U0_optimal_idx, U1_optimal_idx};
            % should both be column vectors.
            Q = {Q0_follower, Q1_leader};
        end
        %% utility functions
        function plot_trace(obj, u_trace, s_trace, fig_num)
            % for debugging purpose mostly
            n_traces = size(s_trace, 3);
            if ~exist('fig_num', 'var')
                fig_num = 1;
            end
            figure(fig_num);
            subplot(3, 1, 1);hold on;grid on; box on;
            ylabel('$s$ [m]', "Interpreter", "latex");
            subplot(3, 1, 2);hold on;grid on; box on;
            ylabel('$v$ [m/s]', "Interpreter", "latex");
            subplot(3, 1, 3);hold on;grid on; box on;
            ylabel('$a~{\rm [m/s^2]}$', "Interpreter", "latex");
            xlabel("$t$ [s]", "Interpreter", "latex");
            if ~obj.param.sample_traj
                t = obj.dt * (1:obj.param.N);
            else
                full_sample_len = length(s_trace(1, :, 1));
                t = obj.param.sample_dt * (1:full_sample_len);
            end
            figure(fig_num);
            legends = cell(1, n_traces);
            for i = 1:n_traces
                subplot(3, 1, 1);
                plot(t, s_trace(1, :, i));
                subplot(3, 1, 2);
                plot(t, s_trace(2, :, i));
                subplot(3, 1, 3);
                if ~isempty(u_trace)
                    plot(t, u_trace(1, :, i));
                end
                legends{i} = num2str(i);
            end
            subplot(3,1,1);legend(legends)
        end
    end
end