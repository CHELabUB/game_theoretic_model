classdef TestLFGClass < skeletonClass
    properties
        LFGObject
    end
    methods(TestMethodSetup)
          function setup(testCase)
              run(fullfile('test_data', 'sample_game_param.m'));
              testCase.LFGObject = game.LFGStateDependentRole(game_param);
          end
    end
    methods (Test)
        function testReward(testCase)
            obj = testCase.LFGObject;
            ego_ini = [-10; 5];
            dt = 0.2;
            N = obj.param.full_sample_len;
            ego_trace = [ego_ini(1) + ego_ini(2) * dt * (1:N); ego_ini(2) * ones(1, N)];
            % note that this is for testing purpose so the control trace is not actually used to generate the state trace
            ego_control_trace = zeros(1, N) + 0.1;
            other_ini = [-10; 4];
            other_trace = [other_ini(1) + other_ini(2) * dt * (1:N); other_ini(2) * ones(1, N)];
            other_control_trace = zeros(1, N) + 0.2;
            r_bars = obj.cal_combine_reward(ego_ini, ego_trace, ego_control_trace, ...
                                            other_ini, other_trace, other_control_trace);
            % this is before the change with collision term depending on the speed.
            % r_bars_exp = [-414.378191933796, -424.833244839417];

            % this is after the change with collision term depending on the speed.
            r_bars_exp = [-2744.77693649357,  -2289.15224048724];
            testCase.verifyEqual(r_bars, r_bars_exp, "AbsTol", 1e-10);
        end
        function testRoleTransitionMatrix(testCase)
            obj = testCase.LFGObject;

            function trans_prob = check_once(other_role_est, x_ini)
                % matlab allows that if there is only one output, then the second output is not assigned
                obj.role_estimation = other_role_est;
                prescribed_role_prob = obj.cal_prescribed_role_prob(other_role_est);
                trans_prob = obj.cal_role_transition_probability(x_ini, prescribed_role_prob);
            end
            
            % type 1: static matrix regardless of estimation
            other_role_est = rand(2, 1);
            x_ini = rand(4, 1);
            obj.role_update_type = 1;
            % Note, the random matrix is not normalized thus is not a legitimate transition matrix
            % This is OK for testing purpose
            obj.param.alter_matrix = cat(3, rand(2, 2), zeros(2, 2));
            exp = obj.param.alter_matrix(:, :, 1);
            % new role could be assigned based on the role_prescription_type but it doesn't really matter
            act = check_once(other_role_est, x_ini);
            testCase.verifyEqual(act, exp, "AbsTol", 1e-16);
            
            % type 2: depends explicitly on the states
            obj.role_update_type = 2;
            obj.param.k_lead = 1;
            obj.param.k_follower = 1.2;
            x_ini = rand(4, 1);
            x_ini(1) = -5; x_ini(3) = -8;
            % because ego is closer, it should be more likely to stay as leader and transient towards leader
            exp = [0.95257412682243, 0.973403006423134
                   0.0474258731775666, 0.0265969935768659];
            % new role could be assigned based on the role_prescription_type but it doesn't really matter
            act = check_once(other_role_est, x_ini);
            testCase.verifyEqual(act, exp, "AbsTol", 1e-14);

            % type 3: depends on the estimation
            function check_case3_mle(p_alter, exp_list)
                % with one p_alter, check 4 possible scenarios

                function m = get_random_trans_matrix()
                    v = rand(2, 1);
                    m = [v(1), 1 - v(2); 1 - v(1), v(2);];
                end

                obj.param.alter_matrix = cat(3, get_random_trans_matrix(), ...
                                                               get_random_trans_matrix());
                if ~isempty(p_alter)
                    obj.param.alter_matrix(:, :, 1) = [1, p_alter; 0, 1 - p_alter];
                    obj.param.alter_matrix(:, :, 2) = [1 - p_alter, 0; p_alter, 1];
                end
                % initial position should not matter so randomize it
                x_ini = rand(4, 1);
                % transition matrix based on role estimation
                obj.role_update_type = 3;
                % maximum likelihood estimation
                obj.role_prescription_type = 1;
                
                role_index = randi(2); % random role assignment
                obj.role = obj.roles(role_index);
    
                p_leader = rand() * 0.5 + 0.5; % leader
                other_role_est = [p_leader; 1 - p_leader];
                exp = obj.param.alter_matrix(:, :, 2);
                if ~isempty(exp_list)
                    exp = exp_list{1};
                end

                act = check_once(other_role_est, x_ini);
                testCase.verifyEqual(act, exp, "other is more likely a Leader", "AbsTol", 1e-14);
    
                p_leader = rand() * 0.5; % follower
                other_role_est = [p_leader; 1 - p_leader];
                exp = obj.param.alter_matrix(:, :, 1);
                if ~isempty(exp_list)
                    exp = exp_list{2};
                end
                act = check_once(other_role_est, x_ini);
                testCase.verifyEqual(act, exp, "other is more likely a Follower", "AbsTol", 1e-14);
    
                p_leader = 0.5; % tie breaker, keep the current roles.
                other_role_est = [p_leader; 1 - p_leader];
                if role_index == 1
                    exp = obj.param.alter_matrix(:, :, 1);
                    if ~isempty(exp_list)
                        exp = exp_list{3};
                    end
                else
                    exp = obj.param.alter_matrix(:, :, 2);
                    if ~isempty(exp_list)
                        exp = exp_list{4};
                    end
                end
                act = check_once(other_role_est, x_ini);
                testCase.verifyEqual(act, exp, "tie breaker", "AbsTol", 1e-14);
            end

            check_case3_mle([], []);
            % a special case is if p_alter = 1;
            % the corresponding state transition matrix should be deterministic
            % regardless of initial roles
            exp_list = { [0, 0; 1, 1], [1, 1; 0, 0], [1, 1; 0, 0], [0, 0; 1, 1]};
            check_case3_mle(1.0, exp_list);
            
            % with continuous role prescription
            obj.role_prescription_type = 2; % smooth
            % make an legit role_estimation
            p_other_leader_est = rand(1);
            other_role_est = [p_other_leader_est; 1 - p_other_leader_est];
            % special p_alter matrix with p_alter = 1;
            p_alter = 1;
            obj.param.alter_matrix(:, :, 1) = [1, p_alter; 0, 1 - p_alter];
            obj.param.alter_matrix(:, :, 2) = [1 - p_alter, 0; p_alter, 1];

            % should give stationary matrix
            % this test works only for two roles
            exp = [other_role_est(2:-1:1), other_role_est(2:-1:1)];
            exp2 = [1 - other_role_est, 1 - other_role_est];
            act = check_once(other_role_est, x_ini);
            testCase.verifyEqual(act, exp, "AbsTol", 1e-14);
            testCase.verifyEqual(act, exp2, "AbsTol", 1e-14);
        end
        function testGetNewRole(testCase)
            obj = testCase.LFGObject;
            % a stochastic matrix
            obj.role_trans_prob = eye(2) * 0.5;
            % get a v > pff/pll should change role
            v = rand(1) * 0.5 + 0.5;
            obj.role = "Leader";
            new_role = get_new_role(obj, v);
            testCase.verifyEqual(new_role, "Follower");
            obj.role = "Follower";
            new_role = get_new_role(obj, v);
            testCase.verifyEqual(new_role, "Leader");

            % get a v < pff/pll should keep role
            v = rand(1) * 0.5;
            obj.role = "Leader";
            new_role = get_new_role(obj, v);
            testCase.verifyEqual(new_role, "Leader");
            obj.role = "Follower";
            new_role = get_new_role(obj, v);
            testCase.verifyEqual(new_role, "Follower");

            % get a v = 0.5 tie breaker, should keep role
            v = 0.5;
            obj.role = "Leader";
            new_role = get_new_role(obj, v);
            testCase.verifyEqual(new_role, "Leader");
            obj.role = "Follower";
            new_role = get_new_role(obj, v);
            testCase.verifyEqual(new_role, "Follower");

            % a deterministic transition matrix
            % should always give the same follower role
            obj.role_trans_prob = [0, 0; 1, 1];
            for i = 1:100
                role_index = randi(2); % random role assignment
                obj.role = obj.roles(role_index);
                new_role = get_new_role(obj);
                testCase.verifyEqual(new_role, "Follower");
            end
            obj.role_trans_prob = [1, 1; 0, 0];
            for i = 1:100
                role_index = randi(2); % random role assignment
                obj.role = obj.roles(role_index);
                new_role = get_new_role(obj);
                testCase.verifyEqual(new_role, "Leader");
            end
        end
        function testStep(testCase, dump_result)
            % make sure results may be reproduced
            baseline_folder = fullfile('test_data', 'full_runs', 'game_same_two_game_agents');
            
            % define one run as a nested function
            function run_one_scenario(baseline_name)
                load(fullfile(baseline_folder, baseline_name, '000.mat'), 'game_param', 'sim_param', 'x_ini', ...
                    'x_sim_to_game_nb', 'x_sim_to_game_eb', 'x_sim_to_game_nb_other', 'x_sim_to_game_eb_other', ...
                    'x_game_nb_to_sim');
                %% Initialize objects
                game_obj_nb = game.LFGStateDependentRole(game_param);
                game_obj_nb.fix_role(~sim_param.adapt_role)
                game_obj_nb.assign_role(sim_param.initial_role); 

                game_obj_eb = game.LFGStateDependentRole(game_param);
                game_obj_eb.fix_role(~sim_param.adapt_role)

                % let the eb vehicle has different role than nb vehicle
                if sim_param.initial_role == "Leader"
                    game_obj_eb.assign_role("Follower");
                else
                    game_obj_eb.assign_role("Leader");
                end

                % test for the first step
                sim_x_ini = x_game_nb_to_sim(x_ini);
                x_game_nb = x_sim_to_game_nb(sim_x_ini);
                x_game_eb = x_sim_to_game_eb(sim_x_ini);
                
                % get new measurement for game
                other_measure_nb.time = game_obj_nb.T1;
                other_measure_eb.time = game_obj_eb.T1;
                other_measure_nb.trace = x_sim_to_game_nb_other(sim_x_ini);
                other_measure_eb.trace = x_sim_to_game_eb_other(sim_x_ini);
                [u_nb_ego, ~, game_info_nb] = game_obj_nb.step(x_game_nb, other_measure_nb);
                [u_eb_ego, ~, game_info_eb] = game_obj_eb.step(x_game_eb, other_measure_eb);

                data = testCase.json.read(fullfile(baseline_folder, baseline_name, '000.json'));
                exp = data.game_info_all{1};
                % order {u_nb_ego, u_eb_ego, game_info_nb, game_info_eb}
                testCase.verifyEqual(u_nb_ego(:), exp{1}(:), baseline_name, "AbsTol", 1e-10);
                testCase.verifyEqual(u_eb_ego(:), exp{2}(:), baseline_name, "AbsTol", 1e-10);

                testCase.verifyEqual(testCase.json.verify_as_json(game_info_nb), ...
                                     testCase.json.verify_as_json(exp{3}), baseline_name, "AbsTol", 1e-10);
                testCase.verifyEqual(testCase.json.verify_as_json(game_info_eb), ...
                                     testCase.json.verify_as_json(exp{4}), baseline_name, "AbsTol", 1e-10);
                
                if dump_result
                    target_folder = fullfile(testCase.test_results_root_folder, "step");
                    if ~exist(target_folder, "dir")
                        mkdir(target_folder)
                    end
                    testCase.json.save(struct("u_nb_ego", u_nb_ego, "u_eb_ego", u_eb_ego, ...
                                              "game_info_nb", game_info_nb, "game_info_eb", game_info_eb), ...
                                       fullfile(target_folder, baseline_name + ".json"));
                end
            end
            all_baselines = ["SamePosLeader";
                             "SamePosFollower"; 
                             "OtherCloserLeader";
                             "OtherCloserFollower";
                             "GameAgentCloserLeader"
                             "GameAgentCloserFollower"];
            for i = 1:length(all_baselines)
                run_one_scenario(all_baselines(i));
            end
        end
        function testFullRuns(testCase, dump_result)
            all_baselines = ["SamePosLeader";
                             "SamePosFollower"; 
                             "OtherCloserLeader";
                             "OtherCloserFollower";
                             "GameAgentCloserLeader"
                             "GameAgentCloserFollower"];
            % all_baselines = ["SamePosLeader"];
            test_result_root_folder = testCase.test_results_root_folder;
            baseline_folder = fullfile('test_data', 'full_runs', 'game_same_two_game_agents');
            for type_id = 1:length(all_baselines)
                % make sure start from the same configuration
                baseline_name = all_baselines(type_id);
                % exp = load(fullfile(baseline_folder, baseline_name, '000.mat'));
                exp = testCase.json.read(fullfile(baseline_folder, baseline_name, '000.json'));
                sim_param = exp.sim_param;
                game_param = exp.game_param;

                target_folder = fullfile(test_result_root_folder, "full_runs", baseline_name);
                if ~exist(target_folder, "dir")
                    mkdir(target_folder)
                end
                mkdir(target_folder);
                % only override save target name
                sim_param.run_name = fullfile(target_folder, "000");
                game.simIntersectionRunTwoGameAgent(game_param, sim_param);
                act = load(sim_param.run_name + ".mat");

                % only need to check some important variables
                vars_to_check = ["game_info_all", ... 
                                 "x_sim", "u_sim", "role_sim", "u_action", "collision_log",...
                                 "trace_prob_nb_on_eb", "trace_prob_eb_on_nb",...
                                 "flag_who_arrived_first", "flag_who_arrived_stop_line_first",...
                                 ];
                for i = 1:length(vars_to_check)
                    if ~isfield(exp, vars_to_check(i))
                        exp.(vars_to_check(i)) = NaN;
                    end
                    testCase.verifyEqual(testCase.json.verify_as_json(act.(vars_to_check(i))), ...
                                         testCase.json.verify_as_json(exp.(vars_to_check(i))), ...
                                         baseline_name + " " + vars_to_check(i), "AbsTol", 1e-10);
                end
                if dump_result
                    vars_to_save = [["sim_param", "game_param"], vars_to_check];
                    testCase.json.convert_mat_to_json(sim_param.run_name + ".mat", ...
                                                      fullfile(target_folder, "000.json"), vars_to_save);
                end
            end
        end
    end
end
