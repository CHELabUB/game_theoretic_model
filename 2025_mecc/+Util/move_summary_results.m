function move_summary_results(sourceDir, targetDir, fileList)
    % Check if the source directory exists
    if ~isfolder(sourceDir)
        error('Source directory does not exist.');
    end
    
    % Check if the target directory exists, if not create it
    if ~isfolder(targetDir)
        mkdir(targetDir);
    end
    
    % Loop through the file list
    for i = 1:numel(fileList)
        % Get the current file/folder name
        name = fileList{i};
        
        % Construct the source and target paths
        sourcePath = fullfile(sourceDir, name);
        targetPath = fullfile(targetDir, name);
        
        % Check if the current item is a file
        if isfile(sourcePath)
            % Copy the file to the target directory
            copyfile(sourcePath, targetPath);
        elseif isfolder(sourcePath)
            % Copy the folder to the target directory
            copyfile(sourcePath, targetPath, 'f');
        else
            warning('Item "%s" does not exist in the source directory.', name);
        end
    end
end
