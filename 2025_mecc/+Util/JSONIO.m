classdef JSONIO < handle
    %JSONIO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        PrettyPrint
    end
    
    methods
        function obj = JSONIO()
            obj.PrettyPrint = true;
        end
        function save(obj, data, JSONFILE_name)
            encodedJSON = jsonencode(data, PrettyPrint=obj.PrettyPrint);
            fid=fopen(JSONFILE_name, 'w');
            fprintf(fid, encodedJSON);
            fclose('all'); 
        end
        function data = read(~, JSONFILE_name)
            fid = fopen(JSONFILE_name, 'r');
            raw = fread(fid, inf);
            str = char(raw');
            fclose(fid);
            data = jsondecode(str);
        end
        function convert_mat_to_json(obj, mat_file, target_file, vars_to_save)
            raw_data = load(mat_file);
            obj.convert_data_to_json(raw_data, target_file, vars_to_save);
        end
        function convert_data_to_json(obj, raw_data, target_file, vars_to_save)
            % the data to be saved better be matrixes
            if ~isempty(vars_to_save)
                data_to_save = struct();
                for i = 1:length(vars_to_save)
                    data_to_save.(vars_to_save(i)) = raw_data.(vars_to_save(i));
                end
            else
                data_to_save = raw_data;
            end
            obj.save(data_to_save, target_file);
        end
        function convert_json_to_mat(obj, json_file, target_file)
            data = obj.read(json_file);
            save(target_file, '-struct', 'data');
        end
        function converted = verify_as_json(~, raw, var_to_skip)
            % var_to_skip is an array of strings
            if nargin == 2 || isempty(var_to_skip)
                converted = jsondecode(jsonencode(raw));
            else
                sub = struct();
                vars = fieldnames(raw);
                for i = 1:length(vars)
                    if ~ismember(vars{i}, var_to_skip)
                        sub.(vars{i}) = raw.(vars{i});
                    end
                end
                converted = jsondecode(jsonencode(sub));
            end
        end
    end
end

