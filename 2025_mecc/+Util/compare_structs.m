function summary = compare_structs(struct1, struct2)
    % Get the field names of both structures
    fields1 = fieldnames(struct1);
    fields2 = fieldnames(struct2);
    
    % Find the fields that are common to both structures
    common_fields = intersect(fields1, fields2);
    
    % Find the fields that are unique to each structure
    unique_fields1 = setdiff(fields1, fields2);
    unique_fields2 = setdiff(fields2, fields1);
    
    % Print the differences
    fprintf('Differences between structures:\n');
    
    % Print the fields that are unique to struct1
    if ~isempty(unique_fields1)
        fprintf('  Fields unique to struct1:\n');
        for i = 1:length(unique_fields1)
            fprintf('    %s\n', unique_fields1{i});
        end
    end
    
    % Print the fields that are unique to struct2
    if ~isempty(unique_fields2)
        fprintf('  Fields unique to struct2:\n');
        for i = 1:length(unique_fields2)
            fprintf('    %s\n', unique_fields2{i});
        end
    end
    
    % Print the fields that have different values
    common_fields_diff = {};
    for i = 1:length(common_fields)
        field = common_fields{i};
        value1 = struct1.(field);
        value2 = struct2.(field);
        if ~isequal(value1, value2)
            fprintf('  Field %s has different values:\n', field);
            fprintf('    struct1: %s\n', num2str(value1));
            fprintf('    struct2: %s\n', num2str(value2));
            common_fields_diff{end + 1} = field;
        end
    end
    summary.common_fields_diff = common_fields_diff;
    summary.unique_fields1 = unique_fields1;
    summary.unique_fields2 = unique_fields2;
end
