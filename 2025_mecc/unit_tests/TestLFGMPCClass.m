classdef TestLFGMPCClass < skeletonClass
    properties
        LFGObject
        LFGAgent
    end
    methods(TestMethodSetup)
        function setup(testCase)
            run(fullfile('test_data', 'sample_game_param.m'));
            % need an game agent for the LFGMPC class
            testCase.LFGAgent = game.LFGStateDependentRole(game_param);
            testCase.LFGObject = game.LFGMPC(mpc_param);
        end
    end
    methods (Test)
        function testReward(testCase, dump_result)
            % test the reward function
            % when there is a collision
            x_mpc = [-8; 10];
            x_other = [-8; 9];
            mpc_obj = testCase.LFGObject;
            mpc_obj.param.print_off = true;
            N = mpc_obj.param.full_sample_len;
            dt = mpc_obj.param.sample_dt;
            t_steps = dt * (1:N);
            mpc_state_trace = x_mpc + [x_mpc(2); 0] * t_steps;
            % note that the control does not correspond to the state
            % but for unit test it is OK.
            mpc_control_trace = 0.5 * t_steps;
            other_trace1 = x_other + [x_other(2); 0] * t_steps;
            other_trace2 = x_other + [x_other(2); dt] * t_steps;
            other_state_trace_tensor = cat(3, other_trace1, other_trace2);
            [step_reward, safety_satisfy] = mpc_obj.calculate_safety_and_reward(x_mpc, mpc_state_trace, mpc_control_trace, other_state_trace_tensor);
            act.step_reward = step_reward;
            act.safety_satisfy = safety_satisfy;
            exp = testCase.json.read(fullfile("test_data", "LFGMPCClass", "reward.json"));
            testCase.verifyEqual(testCase.json.verify_as_json(act), ...
                                 testCase.json.verify_as_json(exp), "AbsTol", 1e-10);
            if dump_result
                testCase.json.save(act, fullfile(testCase.test_results_root_folder, "reward.json"));
            end
        end
        function testOptProactive(testCase, dump_result)
            % consider an infeasible case and see what's the solution of the optimization
            % not initially collided but will collide due to speed
            x_mpc = [-5; 6];
            x_other = [-5; 5.5];
            mpc_obj = testCase.LFGObject;
            mpc_obj.param.print_off = true;
            N = mpc_obj.param.full_sample_len;
            game_obj = testCase.LFGAgent; % used as agent_model
            % For unit test, skip game class calculation but give the prediction results directly
            % the prediction only need the following quantities
            % index 0 : ego in the game agent class, thus is the other vehicle in mpc class
            % index 1 : other in the game agent class, thus is the ego vehicle in the mpc class
            dt = mpc_obj.param.sample_dt;
            t_steps = dt * (1:N);
            x_mpc_trace1 = x_mpc + [x_mpc(2); 0] * t_steps;
            x_mpc_trace2 = x_mpc + [x_mpc(2); dt] * t_steps;
            x_other_trace1 = x_other + [x_other(2); 0] * t_steps;
            x_other_trace2 = x_other + [x_other(2); dt] * t_steps;
            
            game_info.x_ini = [x_other; x_mpc];
            game_info.x0_sequence_u0_as_follower = x_other_trace1;
            game_info.x1_sequence_u0_as_follower = x_mpc_trace1;

            game_info.x0_sequence_u0_as_leader = x_other_trace2;
            game_info.x1_sequence_u0_as_leader = x_mpc_trace2;
            game_info.t_steps = t_steps;
            
            game_obj.status = game_info;
            
            x_other_measurement.trace = x_other;
            x_other_measurement.time = mpc_obj.T1;
            [u_mpc, mpc_info] = mpc_obj.get_proactive_sequence(x_mpc, x_other_measurement, game_obj);
            testCase.verifyTrue(mpc_info.infeasible);
            if dump_result
                act.u_mpc = u_mpc;
                act.mpc_info = mpc_info;
                testCase.json.save(act, fullfile(testCase.test_results_root_folder, "opt_proactive.json"));
            end
        end
        function testLFGMPCStep(testCase, dump_result)
            baseline_folder = fullfile('test_data', 'full_runs', 'mpc_sample_branch');

            % define one run as a nested function
            function run_one_scenario(baseline_name)
                load(fullfile(baseline_folder, baseline_name, '000.mat'), 'game_param', 'sim_param', 'mpc_param', 'x_ini', ...
                        'x_sim_to_game', 'x_sim_to_mpc', 'x_sim_to_game_other', 'x_sim_to_mpc_other', ...
                        'x_game_to_sim');
                % initialize objects
                game_obj = game.LFGStateDependentRole(game_param);
                game_obj.fix_role(~sim_param.adapt_role)
                game_obj.assign_role(sim_param.initial_role); 

                % mpc agent is east bound (eb)
                mpc_obj = game.LFGMPC(mpc_param);

                game_agent_other_measurement.time = game_obj.T1;
                mpc_agent_other_measurement.time = mpc_obj.T1;
                % no measurement noise
                sim_x_ini = x_game_to_sim(x_ini);
                x_mea_current = sim_x_ini;

                x_game = x_sim_to_game(x_mea_current);
                x_mpc = x_sim_to_mpc(x_mea_current);

                game_agent_other_measurement.trace = x_sim_to_game_other(x_mea_current);
                mpc_agent_other_measurement.trace = x_sim_to_mpc_other(x_mea_current);

                [u0_game, u1_game, game_info] = game_obj.step(x_game, game_agent_other_measurement);
                % ego vehicle (eb) is the mpc agent
                [u1_mpc, mpc_info] = mpc_obj.get_proactive_sequence(x_mpc, mpc_agent_other_measurement, game_obj);

                data = testCase.json.read(fullfile(baseline_folder, baseline_name, '000.json'));
                exp = data.game_info_all{1};
                % order {u0_game, u1_game, game_info}
                testCase.verifyEqual(u0_game(:), exp{1}(:), baseline_name, "AbsTol", 1e-10);
                testCase.verifyEqual(u1_game(:), exp{2}(:), baseline_name, "AbsTol", 1e-10);

                testCase.verifyEqual(testCase.json.verify_as_json(game_info), ...
                                        testCase.json.verify_as_json(exp{3}), baseline_name, "AbsTol", 1e-10);
                % TODO, update the baseline
                % mpc_info_all{i} = {u1_mpc, mpc_info};
                exp = data.mpc_info_all{1};
                testCase.verifyEqual(u1_mpc(:), exp{1}(:), baseline_name, "AbsTol", 1e-10);
                fields_to_skip = ["step_chance_constraint_all", "step_reward_all", "step_safety_constraint_all", "infeasible"];
                testCase.verifyEqual(testCase.json.verify_as_json(mpc_info, fields_to_skip), ...
                                     testCase.json.verify_as_json(exp{2}, fields_to_skip), baseline_name, "AbsTol", 1e-10);

                if dump_result
                    act.u0_game = u0_game;
                    act.u1_game = u1_game;
                    act.game_info = game_info;
                    act.u1_mpc = u1_mpc;
                    act.mpc_info = mpc_info;
                    target_folder = fullfile(testCase.test_results_root_folder, "step");
                    if ~exist(target_folder, "dir")
                        mkdir(target_folder)
                    end
                    testCase.json.save(act, fullfile(target_folder, baseline_name + ".json"));
                end
            end
            all_baselines = ["SamePosLeader";
                             "SamePosFollower"; 
                             "MPCCloserLeader";
                             "MPCCloserFollower";
                             "GameAgentCloserLeader"
                             "GameAgentCloserFollower"];
            % all_baselines = ["SamePosLeader"];
            for i = 1:length(all_baselines)
                fprintf("Running %s\n", all_baselines(i));
                run_one_scenario(all_baselines(i));
            end
        end
        function testFullRuns(testCase, dump_result)
            all_baselines = ["SamePosLeader";
                             "SamePosFollower"; 
                             "MPCCloserLeader";
                             "MPCCloserFollower";
                             "GameAgentCloserLeader"
                             "GameAgentCloserFollower"];
            % all_baselines = ["SamePosLeader"];
            % test_result_root_folder = fullfile("unit_tests", "temp_test_results");
            test_result_root_folder = testCase.test_results_root_folder;
            baseline_folder = fullfile('test_data', 'full_runs', 'mpc_sample_branch');

            for type_id = 1:length(all_baselines)
                % make sure start from the same configuration
                baseline_name = all_baselines(type_id);
                % exp = load(fullfile(baseline_folder, baseline_name, '000.mat'));
                exp = testCase.json.read(fullfile(baseline_folder, baseline_name, '000.json'));
                sim_param = exp.sim_param;
                game_param = exp.game_param;
                mpc_param = exp.mpc_param;

                target_folder = fullfile(test_result_root_folder, "full_runs", baseline_name);
                if ~exist(target_folder, "dir")
                    mkdir(target_folder)
                end
                % only override save target name
                sim_param.run_name = fullfile(target_folder, "test");
                game.simIntersectionRun(game_param, mpc_param, sim_param);
                act = load(sim_param.run_name + ".mat");

                % only need to check some important variables
                vars_to_check = ["game_info_all", "mpc_info_all", ... 
                                 "x_sim", "u_sim", "role_sim", "u_action", "collision_log",...
                                 "trace_prob_ego_on_other", "trace_prob_other_on_ego"...
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
                    vars_to_save = ["sim_param", "game_param", "mpc_param", vars_to_check];
                    testCase.json.convert_mat_to_json(sim_param.run_name + ".mat", ...
                                                      fullfile(target_folder, "000.json"), vars_to_save);
                end
            end
        end
        function testFullRunWithInfeasiblFlag(testCase, dump_result)
            test_result_root_folder = testCase.test_results_root_folder;
            baseline_folder = fullfile('test_data', 'full_runs', 'mpc_sample_branch');
            % make sure start from the same configuration
            baseline_name = "NBICVaryFollower";
            % exp = load(fullfile(baseline_folder, baseline_name, '956.mat'));
            exp = testCase.json.read(fullfile(baseline_folder, baseline_name, '956.json'));
            sim_param = exp.sim_param;
            game_param = exp.game_param;
            mpc_param = exp.mpc_param;

            target_folder = fullfile(test_result_root_folder, "full_runs", baseline_name);
            if ~exist(target_folder, "dir")
                mkdir(target_folder)
            end
            % only override save target name
            sim_param.run_name = fullfile(target_folder, "test");
            game.simIntersectionRun(game_param, mpc_param, sim_param);
            act = load(sim_param.run_name + ".mat");

            % only need to check some important variables
            vars_to_check = ["game_info_all", "mpc_info_all", ... 
                             "x_sim", "u_sim", "role_sim", "u_action", "collision_log",...
                             "trace_prob_ego_on_other", "trace_prob_other_on_ego",...
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
            % TODO, update the baseline and remove the following
            fields_to_skip = [];
            for i = 1:length(act)
                testCase.verifyEqual(testCase.json.verify_as_json(act.mpc_info_all{i}{1}), ...
                                     testCase.json.verify_as_json(exp.mpc_info_all{i}{1}), ...
                                     baseline_name + " " + vars_to_check(i), "AbsTol", 1e-10);
                testCase.verifyEqual(testCase.json.verify_as_json(act.mpc_info_all{i}{2}, fields_to_skip), ...
                                     testCase.json.verify_as_json(exp.mpc_info_all{i}{2}, fields_to_skip), ...
                                     baseline_name + " " + vars_to_check(i), "AbsTol", 1e-10);
            end
            if dump_result
                vars_to_save = ["sim_param", "game_param", "mpc_param", vars_to_check];
                testCase.json.convert_mat_to_json(sim_param.run_name + ".mat", ...
                                                  fullfile(target_folder, "956.json"), vars_to_save);
            end
        end
    end
end
