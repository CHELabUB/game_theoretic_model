classdef LFGExampleClass < game.LFGBaseClass
    % An example class of the abstract class so it can be instantiated.
    % only define the properties in addition to the base class
    properties
        param % abstract in base class
        class_name % abstract in base class
    end
    methods
        function obj = LFGExampleClass(param)
            % need to redefine this method
            obj.param = param;
            obj.class_name = "LFGExampleClass";
            % agent_name not used and not redefined, so update for better
            % print out.
            obj.agent_name = "basic_class_example";
            % the following may fail if the parameters are not set up properly
            obj.check_param();
        end
    end
end
