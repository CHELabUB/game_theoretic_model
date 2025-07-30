% run unit tests for the code
% need to make sure the root of the repo is added to the search path
% https://www.mathworks.com/matlabcentral/answers/646283-unit-test-can-t-see-the-function-it-needs-to-test
% Note: similar to python unittest, every complete and successful test will print a "." to the screen. 
% Due to the output print by the function this "." may appears as an additional . in some of the logging lines. 
% This is a bad design by matlab where both print shared the same stdout.

import matlab.unittest.TestSuite
import matlab.unittest.parameters.Parameter
dump_result = {true}; % if want to dump the results
% dump_result = {false}; % default, if do not want to dump the results
param = Parameter.fromData('dump_result', dump_result);

% run all the tests in the folder
suiteFolder = TestSuite.fromFolder('unit_tests', "ExternalParameters", param);
all_results = run(suiteFolder);
disp(all_results);

% run a single test
% test_case = TestSuite.fromFile(fullfile("unit_tests", "TestUtils.m"), "ExternalParameters", param);
% test_case = TestSuite.fromFile(fullfile("unit_tests", "TestLFGBaseClass.m"), "ExternalParameters", param);
% test_case = TestSuite.fromFile(fullfile("unit_tests", "TestLFGMPCClass.m"), "ExternalParameters", param);
% test_case = TestSuite.fromFile(fullfile("unit_tests", "TestLFGClass.m"), "ExternalParameters", param);
% result = run(test_case);
% disp(result);
