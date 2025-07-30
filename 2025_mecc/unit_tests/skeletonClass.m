% skeletonClass - A template class for new test classes.
% it contains basic json interface for load and dumping results
% it also contains the target folder for saving the test results

classdef skeletonClass < matlab.unittest.TestCase
    properties
        json
        test_results_root_folder
    end
    properties (TestParameter)
        dump_result = {false};
    end
    methods(TestClassSetup)
        function create(testCase)
            testCase.json = Util.JSONIO();
            test_name = class(testCase);
            testCase.test_results_root_folder = fullfile("unit_tests", "temp_test_results", test_name);
            if exist(testCase.test_results_root_folder, 'dir')
                rmdir(testCase.test_results_root_folder, "s");
            end
            mkdir(testCase.test_results_root_folder);
        end
    end
end
