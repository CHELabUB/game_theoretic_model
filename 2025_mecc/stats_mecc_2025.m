clc;clear; close all;
%%
% results_root = "results"; % no parallel results, should be reproducible on different machine. 
results_root = "results_parallel"; % parallel results, may not be reproduced exactly on different machine.
%% Game the summary and print the table in latex format
%% MPC + game
mpc_runs = {"mpc_random_ic_p_alter_match";
                "mpc_random_ic_p_alter_overestimation";
                "mpc_random_ic_p_alter_overestimation_2";
                "mpc_random_ic_p_alter_overestimation_3";
                "mpc_random_ic_p_alter_underestimation_3";
                "mpc_random_ic_p_alter_underestimation_2";
                "mpc_random_ic_p_alter_underestimation";
                };
get_p_alter= @(run_summary) ...
    [run_summary.run_info.common_parameters.game_param.alter_matrix(1,2);
        run_summary.run_info.common_parameters.mpc_param.agent_model.alter_matrix(1,2);];
% belief / actual
print_p_alter = @(value) sprintf("%.2f/%.2f", value(1), value(2));
mpc_stats = cell(1, length(mpc_runs));
run_type = {};
mpc_run_flag = [];
mpc_run_flag_update = [];
mpc_collision_flag = [];
mpc_min_dist_flag = [];
run_set_folder = {};
for iter = 1:length(mpc_runs)
    run_name = mpc_runs{iter};
    fprintf("checking %s \n", run_name);
    result_root_folder = fullfile(results_root, run_name);

    clear run_summary;
    load(fullfile(result_root_folder, "summary.mat"), "run_summary");
    mpc_stats{iter} = game.GenSummary.get_summary_stats(run_summary);
    for jter = 1:length(run_summary.run_info.type(:, 2))
        run_type{end + 1} = run_summary.run_info.type{jter, 2} + "," + print_p_alter(get_p_alter(run_summary));
        run_set_folder{end + 1} = fullfile(result_root_folder, run_summary.run_info.type{jter, 3});
    end
    % run_type = [run_type, get_p_alter(run_summary)];
    
    if isfield(mpc_stats{iter}, "run_flag_update")
        fprintf("Using update terminal condition. \n");
        mpc_run_flag_update = [mpc_run_flag_update, mpc_stats{iter}.run_flag_update.value];
        mpc_min_dist_flag = [mpc_min_dist_flag, mpc_stats{iter}.min_dst_flag_update.value];
    else
        mpc_run_flag = [mpc_run_flag, mpc_stats{iter}.run_flag.value];
        mpc_min_dist_flag = [mpc_min_dist_flag, mpc_stats{iter}.min_dst_flag.value];
    end
    mpc_collision_flag = [mpc_collision_flag, mpc_stats{iter}.collision_flag.value];
end
run_set_folder = run_set_folder(:);

% because there is no arrival at the same time, drop column 3.
if ~all(mpc_run_flag_update(4, :) == 0)
    table = [mpc_run_flag_update([1, 2, 4], :)', mpc_collision_flag(2,:)'];
    table_ext = [mpc_run_flag_update([1, 2, 4], :)', mpc_min_dist_flag(2, :)'];
    col_name = mpc_stats{1}.run_flag.row_name([1,2]);
    col_name{3} = "NF";
    col_name{4} = "Collision";
else
    % if there is no NF case
    table = [mpc_run_flag_update([1, 2], :)', mpc_collision_flag(2,:)'];
    table_ext = [mpc_run_flag_update([1, 2], :)', mpc_min_dist_flag(2, :)'];
    col_name = mpc_stats{1}.run_flag.row_name([1,2]);
    % col_name{3} = "NF";
    col_name{3} = "Collision";
end

row_name = run_type;
mpc_stats_latex = game.GenSummary.table2latex(table, col_name, row_name, 2)
col_name_ext = col_name;
col_name_ext{end} = "$\not\in\Xcal_{\rm safe}$";
mpc_stats_ext_latex = game.GenSummary.table2latex(table_ext, col_name_ext, row_name, 2)

%% Game
clearvars -except results_root;

game_runs = {"game_2_agent_random_ic_p_alter10";
             "game_2_agent_random_ic_p_alter07";
            };
game_stats = cell(1, length(game_runs));
game_run_flag = [];
game_run_flag_update = [];
game_collision_flag = [];
game_min_dist_flag = [];
for iter = 1:length(game_runs)
    run_name = game_runs{iter};
    fprintf("checking %s \n", run_name);
    result_root_folder = fullfile(results_root, run_name);
    clear run_summary;
    load(fullfile(result_root_folder, "summary.mat"), "run_summary");
    game_stats{iter} = game.GenSummary.get_summary_stats(run_summary);
    
    if isfield(game_stats{iter}, "run_flag_update")
        fprintf("Using update terminal condition. \n");
        game_run_flag_update = [game_run_flag_update, game_stats{iter}.run_flag_update.value];
        game_min_dist_flag = [game_min_dist_flag, game_stats{iter}.min_dst_flag_update.value];
    else
        game_run_flag = [game_run_flag, game_stats{iter}.run_flag.value];
        game_min_dist_flag = [game_min_dist_flag, game_stats{iter}.min_dst_flag.value];
    end
    game_collision_flag = [game_collision_flag, game_stats{iter}.collision_flag.value];
    
end

if ~all(game_run_flag_update(4, :) == 0)
    table = [game_run_flag_update([1, 2, 4], :)', game_collision_flag(2,:)'];
    table_ext = [game_run_flag_update([1, 2, 4], :)', game_min_dist_flag(2,:)'];
    col_name = game_stats{1}.run_flag.row_name([1,2]);
    col_name{3} = "NF";
    col_name{4} = "Collision";
else
    table = [game_run_flag_update([1, 2], :)', game_collision_flag(2,:)'];
    table_ext = [game_run_flag_update([1, 2], :)', game_min_dist_flag(2,:)'];
    col_name = game_stats{1}.run_flag.row_name([1,2]);
    col_name{3} = "Collision";
end
row_name = {"$Full = 1.0 $, Leader", "Full, Follower", ...
                     "Part, Leader", "Part Follower"};

game_stats_latex = game.GenSummary.table2latex(table, col_name, row_name, 2)

col_name_ext = col_name;
col_name_ext{end} = "$\not\in\Xcal_{\rm safe}$";
game_stats_ext_latex = game.GenSummary.table2latex(table_ext, col_name_ext, row_name, 2)
