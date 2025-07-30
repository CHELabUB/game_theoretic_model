classdef TestLFGBaseClass < skeletonClass
    properties
        LFGObject
    end

    methods(TestMethodSetup)
        % method create cannot be reused because it is used at the base class for TestClassSetup
        function setup(testCase)
            run(fullfile('test_data', 'sample_game_param.m'));
            testCase.LFGObject = game.LFGExampleClass(game_param);
        end
    end
    methods (Test)
        function testParamChecker(testCase)
            generic_obj = game.LFGExampleClass(testCase.LFGObject.param);
            generic_obj.param.v_min = - 1;
            generic_obj.param.v_max = - 2;
            generic_obj.param.u_min = 1;
            generic_obj.param.u_max = -1;
            testCase.verifyEqual(generic_obj.check_param, 6);
        end
        function testFindOptimalIndex(testCase)
            obj = testCase.LFGObject;
            % test find optimal action function
            % each row corresponds to one u1 sequence, 
            % each column corresponds to one u0 sequence
            Rbar = [9, 8, 7;
                    4, 5, 6;
                    2, 3, 1];
            [U, Q] = obj.get_leader_sequences_idx(Rbar, Rbar);
            % u1 is follower
            % find the minimum of each row
            % then find the maximum of the minimums
            testCase.verifyEqual(U{1}, 1);
            testCase.verifyEqual(Q{1}, [9; 8; 7]);
            testCase.verifyEqual(U{2}, 1);
            testCase.verifyEqual(Q{2}, [7; 4; 1]);
            [U, Q] = obj.get_follower_sequences_idx(Rbar, Rbar);
            % u0 is follower
            % find the minimum of each column
            % then find the maximum of the minimums
            testCase.verifyEqual(U{1}, 2);
            testCase.verifyEqual(Q{1}, [2; 3; 1]);
            % u1 is the leader find the maximum in the corresponding column
            testCase.verifyEqual(U{2}, 1);
            testCase.verifyEqual(Q{2}, [8; 5; 3]);
        end
        function testTrajectorySample(testCase, dump_result)
            x_ini = [-20, 5];
            [s, u] = testCase.LFGObject.generate_traj_sample(x_ini);
            % can inspect using the figures
            % testCase.LFGObject.plot_trace(u, s);
            testCase.verifyEqual(size(s, 3), 6);
            testCase.verifyEqual(size(u, 3), 6);
            exp = testCase.json.read(fullfile("test_data", "sample_traj_results.json"));
            testCase.verifyEqual(s, exp.s);
            testCase.verifyEqual(u, exp.u);
            if dump_result
                testCase.json.save(struct("s", s, "u", u), ...
                                   fullfile(testCase.test_results_root_folder, "sample_traj_results.json"));
            end
            % This example is used to investigate whether the it make sense
            % to force all speed to be at achieved at the interesection.
            % x_ini = [-20, 5];
            % [s, u] = testCase.LFGObject.generate_traj_sample(x_ini, 5);
            % % can inspect using the figures
            % % testCase.LFGObject.plot_trace(u, s);
            % testCase.verifyEqual(size(s, 3), 6);
            % testCase.verifyEqual(size(u, 3), 6);
            % % exp = testCase.json.read(fullfile("test_data", "sample_traj_results_long.json"));
            % % testCase.verifyEqual(s, exp.s);
            % % testCase.verifyEqual(u, exp.u);
            % if dump_result
            %     testCase.json.save(struct("s", s, "u", u), ...
            %                        fullfile(testCase.test_results_root_folder, "sample_traj_results_long.json"));
            % end
        end
        function testCalError(testCase)
            x_actual_traces = [1, 2, 3, 4, 5;
                               -2, -3, -4, -5, -6;
                               3, 4, 5, 6, 7];
            x_predicted_traces = cat(3, zeros(3, 5), ones(3, 5));
            act = testCase.LFGObject.get_x_errors(x_actual_traces, x_predicted_traces);
            exp = [3, 4, 5;
                   2, 5, 4]';
            testCase.verifyEqual(act, exp, "AbsTol", 1e-14);
        end
        function testRoleEstimation(testCase)
            role_priors = [0.4; 0.6];
            x_errors = [0.1, 0.2;
                        0.05, 0.1];
            [role_post, ~] = testCase.LFGObject.estimate_role(role_priors, x_errors);
            % only within tolerance
            testCase.verifyEqual(role_post, [0.4411; 0.5589], "AbsTol", 1e-5);
            % n_states = 3;
            role_priors = [0.4; 0.6];
            % n_states x n_roles
            x_errors = [0.1, 0.2, 0.3;
                        0.05, 0.1, 0.2]';
            testCase.LFGObject.param.noise_W = diag([0.1, 0.2, 0.3]);
            [role_post, ~] = testCase.LFGObject.estimate_role(role_priors, x_errors);
            % only within tolerance
            testCase.verifyEqual(role_post, [0.354046; 0.645953], "AbsTol", 1e-5);
        end
    end
end
