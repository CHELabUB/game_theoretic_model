classdef SimIntersection < handle
    %SIM_INTERSECTION Summary of this class goes here
    %   Detailed explanation goes here
    % this class
    properties (SetAccess = protected)
        % may be accessed but not changed after set.
        dt
        n_veh
        time
        x
        noise_W
        add_noise
        uncertainty
        add_action_uncertainty
        v_max
        v_min
        print_off
        veh_dim
    end
    
    methods
        function obj = SimIntersection(x_ini, varargin)
            %SIM_INTERSECTION Construct an instance of this class
            %   Detailed explanation goes here
            obj.set_param(varargin{:})
            obj.reset(x_ini);
        end
        function debug_print(obj, msg)
            if obj.print_off
                fprintf(msg);
            end
        end
        function set_param(obj, varargin)
            % private parameters
            % add more optional parameters that may be set this way.
            %% Game parameters
            veh_width = 2.5;
            veh_length = 5.0;
            default_veh_dim.veh_width = veh_width;
            default_veh_dim.veh_length = veh_length;
            
            defaults = {'dt', 0.1, ...
                        'noise_W', 0, 'add_noise', false, ...
                        'uncertainty', 0, 'add_action_uncertainty', false, ...
                        'v_max', 10, 'v_min', 0,...
                        'print_off', false, 'veh_dim', default_veh_dim};
            p = Util.SetOptions(defaults, varargin);
            obj.dt = p.dt;
            obj.noise_W = p.noise_W;
            obj.add_noise = p.add_noise;
            obj.v_max = p.v_max;
            obj.v_min = p.v_min;
            obj.print_off = p.print_off;
            obj.add_action_uncertainty = p.add_action_uncertainty;
            obj.uncertainty = p.uncertainty;
            if isempty(p.veh_dim)
                p.veh_dim = default_veh_dim;
            else
                obj.veh_dim = p.veh_dim;
            end
            if obj.add_noise
                v = eig(obj.noise_W);
                % for simplicity, consider only diagonal covariance (no
                % dependency between each other.
                % obj.noise_W = diag(obj.noise_W);
                if max(v) <= 1e-6
                    obj.debug_print("Noise covariance matrix too small, simulation will not add noise.\n");
                    obj.add_noise = false;
                end
            else
                obj.debug_print("simulation will not add noise.\n");
            end
        end
        function reset(obj, x_ini)
            obj.time = 0.0;
            obj.x = x_ini;
            obj.n_veh = round(length(x_ini) / 4);
        end
        function x_new = pred_step(obj, x, u)
            x_new = x;
            for i_veh = 1:obj.n_veh
                x_idx = (i_veh - 1) * 4 + (1:4);
                u_idx = (i_veh - 1) * 2 + (1:2);
                x_sub = obj.x(x_idx);
                u_sub = u(u_idx);
                % (TODO)this part need to consider travel direction
                % so far only assume positive direction.
                x_new(x_idx) = [game.const_a_step(x_sub(1), x_sub(2), u_sub(1), obj.dt, obj.v_max, obj.v_min);
                                game.const_a_step(x_sub(3), x_sub(4), u_sub(2), obj.dt, obj.v_max, obj.v_min);];
            end
        end
        function u_act = step(obj, u)
            % generic step, treat all other vehicles the same
            % assume constant acceleration model for both x and y
            % direction, no rotation
            % each vehicle has 4 states, x, vx, y, vy
            % each vehicle has 2 control, ux, uy
            if obj.add_action_uncertainty
                if all(size(obj.uncertainty) == [2 * obj.n_veh, 2 * obj.n_veh])
                    % multivariate normal distribution
                    u_act = u + mvnrnd(zeros(2 * obj.n_veh, 1), obj.uncertainty, 1);
                else
                    % multivariate uniform distribution
                    u_act = u + unifrnd(-obj.uncertainty(:), obj.uncertainty(:));
                end
            else
                u_act = u;
            end
            x_new = pred_step(obj, obj.x, u_act);
            obj.x = x_new;
            obj.time = obj.time + obj.dt;
        end
        function step_tracking(obj, target_trajectories)
            % Implement tracking control
            % For now assume all vehicles does perfect tracking
            % how to allow separate control framework for each vehicle? e.g. one is game vehicle that does perfect tracking, 
            % the other does MPC tracking or other type of tracking
            % assume target is given as t, x, vx, y, vy for each vehicle
            x_new = interp1(target_trajectories(:, 1), target_trajectories(:, 2:end), obj.time);
            if obj.add_noise
                if all(size(obj.noise_W) == [2 * obj.n_veh, 2 * obj.n_veh])
                    noise = mvnrnd(zeros(4 * obj.n_veh, 1), obj.noise_W, 1);
                else
                    noise = unifrnd(-obj.noise_W(:), obj.noise_W(:));
                end
                obj.x = x_new + noise(:);
            else
                obj.x = x_new;
            end
            obj.time = obj.time + obj.dt;
        end
        function collide = check_collision(obj)
            % (TODO) add a unit test for this function
            % (TODO) The current setup can use some improvement
            % Also probably want to add dependence on speed. e.g. if both are stopped at the intersection
            % then the minimum distance may be violated but there is no danger of collision.
            % check if any collision happens
            if obj.n_veh > 2
                collide = false;
                fprintf("Warning: collision check is not implemented properly for more thant 2 vehicles yet. \n");
                return;
            end
            % for intersection case only, 
            % vehicle 1 is traveling west to east, vehicle 2 is traveling south to north
            mini_dist = (obj.veh_dim.veh_length + obj.veh_dim.veh_width ) / 2;
            collide = obj.x(1) - obj.x(5) <= mini_dist && ...
                      obj.x(1) - obj.x(5) >= - mini_dist && ...
                      obj.x(3) - obj.x(7) <= mini_dist && ...
                      obj.x(3) - obj.x(7) >= - mini_dist;
        end
        function x_mea = get_state_measurement(obj)
            % (TODO) add a unit test for this function
            % get the state measurement
            if obj.add_noise
                if all(size(obj.noise_W) == [4 * obj.n_veh, 4 * obj.n_veh])
                    noise = mvnrnd(zeros(4 * obj.n_veh, 1), obj.noise_W, 1);
                else
                    noise = unifrnd(-obj.noise_W(:), obj.noise_W(:));
                end
                x_mea = obj.x + noise(:);
            else
                x_mea = obj.x;
            end
        end
    end
end

