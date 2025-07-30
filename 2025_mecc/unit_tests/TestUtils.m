% To load predefined data
% exp = load('test_data/xxx.mat');
% For better readability, prefer explicit results.

classdef TestUtils < skeletonClass
    methods (Test)
        function testOVM(testCase)
            ovm_params.alpha = 0.4;
            ovm_params.beta = 0.5;
            ovm_params.alpha_v = ovm_params.alpha + ovm_params.beta;
            ovm_params.kappa = 0.6;
            ovm_params.hst = 5;
            ovm_params.vmax = 10;
            ovm_params.hgo = ovm_params.hst + ovm_params.vmax / ovm_params.kappa;
            % very close
            act = game.OVM(5, 5, 5, ovm_params);
            exp = -2.0;
            testCase.verifyEqual(act, exp);
            
            % slightly slow down
            act = game.OVM(15, 6, 5, ovm_params);
            exp = -0.5;
            testCase.verifyEqual(act, exp);

            % at the equilibrium
            act = game.OVM(15, 6, 6, ovm_params);
            exp = 0.0;
            testCase.verifyEqual(act, exp);
            
            % mildly speed up
            act = game.OVM(16, 6, 7, ovm_params);
            exp = 0.74;
            testCase.verifyEqual(act, exp, "AbsTol", 1e-8);

            % very far
            act = game.OVM(50, 5, 5, ovm_params);
            exp = 2.0;
            testCase.verifyEqual(act, exp);
        end
        function testGenSequence(testCase, dump_result)
            exp = testCase.json.read(fullfile("test_data", "gen_sequence_results.json"));
            act1 = game.gen_sequence(2, 3);
            testCase.verifyEqual(act1, exp.result1);
            act2 = game.gen_sequence(3, 3);
            testCase.verifyEqual(act2, exp.result2);
            if dump_result
                testCase.json.save(struct("result1", act1, "result2", act2), ...
                                   fullfile(testCase.test_results_root_folder, "gen_sequence_results.json"));
            end
        end
        function testConstAStep(testCase)
            s0 = 0;
            dt = 1;
            v_max = 10;
            v_min = 0;
            % constant speed
            v0 = 8; a = 0;
            exp = [8; 8];
            act = game.const_a_step(s0, v0, a, dt, v_max, v_min);
            testCase.verifyEqual(act, exp);            
            % speed up
            v0 = 9; a = 1;
            exp = [9.5; 10];
            act = game.const_a_step(s0, v0, a, dt, v_max, v_min);
            testCase.verifyEqual(act, exp);
            % speed up to max
            v0 = 9.5; a = 1;
            exp = [9.875; 10];
            act = game.const_a_step(s0, v0, a, dt, v_max, v_min);
            testCase.verifyEqual(act, exp);
            % slow down
            v0 = 5; a = -4;
            exp = [3; 1];
            act = game.const_a_step(s0, v0, a, dt, v_max, v_min);
            testCase.verifyEqual(act, exp);
            % stop
            v0 = 7; a = -10;
            exp = [2.45; 0];
            act = game.const_a_step(s0, v0, a, dt, v_max, v_min);
            testCase.verifyEqual(act, exp);
        end
        function testSpeedControl(testCase)
            % set up
            v_max = 20;
            v_min = 0;
            ovm_params.alpha = 0.4;
            ovm_params.beta = 0.5;
            % to make the two equivalent
            ovm_params.alpha_v = ovm_params.alpha + ovm_params.beta;
            ovm_params.kappa = 0.6;
            ovm_params.hst = 5;
            ovm_params.vmax = v_max;
            ovm_params.hgo = ovm_params.hst + ovm_params.vmax / ovm_params.kappa;
            dt = 0.1;
            full_sample_len = 100;
            
            % define a nested function
            function [s_trace, u_trace, s_trace2, u_trace2] = get_compare_results(v0, v_target, s_stop)
                s_trace = zeros(2, full_sample_len + 1);
                u_trace = zeros(1, full_sample_len);
                u_trace2 = u_trace;
                
                s_trace(:, 1) = [0; v0];
                s_trace2 = s_trace;
                for i = 1:full_sample_len
                    u_trace(:, i) = game.speed_control(s_trace(2, i), v_target, ovm_params);
                    s_trace(:, i + 1) = game.const_a_step(s_trace(1, i), s_trace(2, i), u_trace(:, i), dt, v_max, v_min);
                    if v_target > 0
                        h = v_target / ovm_params.kappa + ovm_params.hst;
                    else
                        h = s_stop + ovm_params.hst - s_trace2(1, i);
                    end
                    u_trace2(:, i) = game.OVM(h, s_trace2(2, i), v_target, ovm_params);
                    s_trace2(:, i + 1) = game.const_a_step(s_trace2(1, i), s_trace2(2, i), u_trace2(:, i), dt, v_max, v_min);
                end
                return 
            end
            
            % when initial speed and target speed are non zero, they the two methods should be the same
            v0 = rand(1) * 5 + 5;
            v_target = rand(1) * 5 + 5;
            s_stop = 5;
            [s_trace, u_trace, s_trace2, u_trace2] = get_compare_results(v0, v_target, s_stop);
            testCase.verifyEqual(s_trace, s_trace2, "AbsTol", 1e-12);
            testCase.verifyEqual(u_trace, u_trace2, "AbsTol", 1e-12);

            v0 = 4;
            v_target = 0;
            s_stop = 16;
            exp = testCase.json.read(fullfile("test_data", "speed_control_results.json"));
            [s_trace, u_trace, s_trace2, u_trace2] = get_compare_results(v0, v_target, s_stop);
            testCase.verifyEqual(s_trace, exp.s_trace, "AbsTol", 1e-12);
            testCase.verifyEqual(s_trace2, exp.s_trace2, "AbsTol", 1e-12);
            % json by default will loaded as columns
            testCase.verifyEqual(u_trace, exp.u_trace', "AbsTol", 1e-12);
            testCase.verifyEqual(u_trace2, exp.u_trace2', "AbsTol", 1e-12);
            % the OVM way should roughly get to the stop position with target speed.
            testCase.verifyEqual(s_trace2(1, end), s_stop, "AbsTol", 0.2);
            testCase.verifyEqual(s_trace2(2, end), v_target, "AbsTol", 0.1);
        end

    end
end
