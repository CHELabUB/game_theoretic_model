classdef GenSummary
    % can run without defining the class object
    methods(Static)
        function run_data = load_one_run(result_root_folder, type_id, seed)
            run_name = game.GenSummary.get_run_name(result_root_folder, type_id, seed);
            if ~exist(run_name, "file")
                run_data = [];
                return
            end
            run_data = load(run_name);
        end
        function [summary, figs] = get_summary(result_root_folder, sim_type, seed_can, type_id_can)
            if nargin < 4
                % look for the already generated summary
                summary_file = fullfile(result_root_folder, "summary.mat");
                load(summary_file, "run_summary");
                summary = run_summary;
                figs = game.GenSummary.get_summary_plot(run_summary);
                game.GenSummary.print_summary(run_summary)
                return;
            end
            run_flag_all = zeros(max(seed_can), max(type_id_can));
            collision_flag_all = zeros(max(seed_can), max(type_id_can));
            min_dist_all = zeros(max(seed_can), max(type_id_can));
            min_dist_flag = ones(max(seed_can), max(type_id_can));
            x_ini_all = zeros(4, max(seed_can), max(type_id_can));
            run_flag_update_all = zeros(max(seed_can + 1), max(type_id_can));
            min_dist_update_all = zeros(max(seed_can), max(type_id_can));
            min_dist_flag_update = ones(max(seed_can), max(type_id_can));
            run_flag_no_role_change_all = zeros(max(seed_can + 1), max(type_id_can));
            % mpc only
            has_infeasible = zeros(max(seed_can), max(type_id_can));
            func = @(mpc_info) (isfield(mpc_info{2}, "infeasible") & mpc_info{2}.infeasible);
            summary_report = [""];
            for type_id = type_id_can
                sim_param.run_type = sim_type{type_id, 3};
                for seed = seed_can
                    sim_param.seed = seed;
                    sim_param.run_name = fullfile(result_root_folder, sim_param.run_type, sprintf("%03d", sim_param.seed));
                    raw_sim_param = load(sim_param.run_name, "sim_param");
                    raw_sim_param = raw_sim_param.sim_param;
                    clear x_ini;
                    load(sim_param.run_name, "x_ini");
                    x_ini_all(:, seed + 1, type_id) = x_ini;
                    clear flag_who_arrived_first;
                    load(sim_param.run_name, "flag_who_arrived_first");
                    if ~exist("flag_who_arrived_first", "var")
                        s = sprintf("collision_flag not exist: %s\n", sim_param.run_name);
                        summary_report(end+1) = s;  
                        fprintf("%s", s);
                    end
                    run_flag_all(seed + 1, type_id) = flag_who_arrived_first;
                    if flag_who_arrived_first == 0
                        s = sprintf("funky run, neither arrived at (0, 0): %s\n", sim_param.run_name);
                        summary_report(end+1) = s;  
                        fprintf("%s", s);
                    end
                    if flag_who_arrived_first == 3
                        s = sprintf("Rare case both arrived at (0, 0) at the same time: %s\n", sim_param.run_name);
                        summary_report(end+1) = s;  
                        fprintf("%s", s);
                    end
                    clear collision_flag;
                    load(sim_param.run_name, "collision_flag");
                    if ~exist("collision_flag", "var")
                        s = sprintf("collision_flag not exist: %s\n", sim_param.run_name);
                        summary_report(end+1) = s;  
                        fprintf("%s", s);
                    end
                    collision_flag_all(seed + 1, type_id) = collision_flag;
                    if collision_flag == 1
                        s = sprintf("Collision detected: %s\n", sim_param.run_name);
                        summary_report(end+1) = s;  
                        fprintf("%s", s);
                    end
                    load(sim_param.run_name, "x_sim");
                    load(sim_param.run_name, "game_param");
                    min_dist_all(seed + 1, type_id) = min(sqrt((x_sim(1, :) - x_sim(5, :)).^2 + (x_sim(3, :) - x_sim(7, :)).^2));
                    min_dist_flag(seed + 1, type_id) = (min_dist_all(seed + 1, type_id) > game_param.reward.min_dist);

                    clear idx_end;
                    load(sim_param.run_name, "idx_end");
                    min_dist_update_all(seed + 1, type_id) = min(sqrt((x_sim(1, 1:idx_end) - x_sim(5, 1:idx_end)).^2 ...
                                                          + (x_sim(3, 1:idx_end) - x_sim(7, 1:idx_end)).^2));
                    min_dist_flag_update(seed + 1, type_id) = (min_dist_update_all(seed + 1, type_id) > game_param.reward.min_dist);
                    clear flag_who_arrived_stop_line_first;
                    load(sim_param.run_name, "flag_who_arrived_stop_line_first");
                    run_flag_update_all(seed + 1, type_id) = flag_who_arrived_stop_line_first;
                    % use arrived at stop line as indicator for who arrive first.
                    if flag_who_arrived_stop_line_first == 0
                        s = sprintf("funky run, neither arrived at the stop line: %s\n", sim_param.run_name);
                        summary_report(end+1) = s;  
                        fprintf("%s", s);
                    end
                    if flag_who_arrived_stop_line_first == 3
                        s = sprintf("Rare case both arrived at the stop line at same time: %s\n", sim_param.run_name);
                        summary_report(end+1) = s;  
                        fprintf("%s", s);
                    end
                    clear role_sim role_sim_next;
                    load(sim_param.run_name, "role_sim", "role_sim_next");
                    no_role_changes = true;
                    for iter = 1:size(role_sim, 1)
                        if ~(no_role_changes && ...
                                all(size(unique(role_sim(iter, :))) == [1, 1]) && ...
                                all(size(unique(role_sim_next(iter, :))) == [1, 1]))
                            no_role_changes = false;
                        end
                    end
                    run_flag_no_role_change_all(seed + 1, type_id) = no_role_changes;

                    % this is only for mpc game run
                    vars = whos('-file', sim_param.run_name);
                    if ~any(strcmp({vars.name}, 'mpc_info_all'))
                        has_infeasible(seed + 1, type_id) = 0;
                    else
                        clear mpc_info_all;
                        load(sim_param.run_name, "mpc_info_all");
                        has_infeasible(seed + 1, type_id) = any(cellfun(func, mpc_info_all));
                    end
                    
                end
            end
            % assume that this is the same for all the runs.
            if ~isfield(raw_sim_param, "randomize_initial_condition")
                sim_param.randomize_initial_condition = false;
            else
                sim_param.randomize_initial_condition = raw_sim_param.randomize_initial_condition;
            end
            run_flag_all = run_flag_all(seed_can + 1, type_id_can);
            collision_flag_all = collision_flag_all(seed_can + 1, type_id_can);
            min_dist_all = min_dist_all(seed_can + 1, type_id_can);
            min_dist_flag = min_dist_flag(seed_can + 1, type_id_can);
            x_ini_all = x_ini_all(:, seed_can + 1, type_id_can);
            run_flag_update_all = run_flag_update_all(seed_can + 1, type_id_can);
            min_dist_update_all = min_dist_update_all(seed_can + 1, type_id_can);
            min_dist_flag_update = min_dist_flag_update(seed_can + 1, type_id_can);
            run_flag_no_role_change_all = run_flag_no_role_change_all(seed_can + 1, type_id_can);
            
            has_infeasible = has_infeasible(seed_can + 1, type_id_can);

            summary.run_flag_all = run_flag_all;
            summary.collision_flag_all = collision_flag_all;
            summary.min_dist_all = min_dist_all;
            summary.min_dist_flag = min_dist_flag;
            summary.x_ini_all = x_ini_all;
            summary.run_flag_update_all = run_flag_update_all;
            summary.min_dist_update_all = min_dist_update_all;
            summary.min_dist_flag_update = min_dist_flag_update;
            summary.run_flag_no_role_changes_flag_all = run_flag_no_role_change_all;

            summary.has_infeasible = has_infeasible;
            summary.report = summary_report(2:end);
        
            % to facilitate plot
            run_info.common_parameters = struct("sim_param", sim_param, "game_param", game_param);
            run_info.type = sim_type;
            run_info.seed_can = seed_can;
            run_info.type_id_can = type_id_can;
            summary_for_plot = summary;
            summary_for_plot.run_info = run_info;
            figs = game.GenSummary.get_summary_plot(summary_for_plot);
        end
        function print_summary(run_summary)
            for i = 1:length(run_summary.report)
                fprintf("%s", run_summary.report(i));
                % disp(run_summary.report{i}(1:end-1))
            end
        end
        %% TODO include the above get infeasible part
        function run_flag_update_all = add_metric_to_run(run_result_folder, override)
            summary_file = fullfile(run_result_folder, "summary.mat");
            load(summary_file, "run_summary");
            summary = run_summary;
            % % an example of adding a new metric
            % Some_new_summary_flag = "run_no_role_changes_flag_all";
            % if isfield(summary, Some_new_summary_flag)
            %     if nargin < 2 || ~override
            %         run_flag_update_all = summary.run_flag_update_all;
            %         warning(sprintf("%s already exist, no need to update.", Some_new_summary_flag));
            %         return;
            %     else
            %         warning(sprintf(" %s already exist, no need to update. Overwrite.", Some_new_summary_flag));
            %     end
            % end
            % sim_type = summary.run_info.type;
            % type_id_can = summary.run_info.type_id_can;
            % seed_can = summary.run_info.seed_can;
            % run_flag_update_all = zeros(max(seed_can + 1), max(type_id_can));
            % for type_id = type_id_can
            %     for seed = seed_can
            %         sim_param.seed = seed;
            %         sim_param.run_name = fullfile(run_result_folder, sim_type{type_id, 3}, sprintf("%03d", sim_param.seed));
            %         clear role_sim role_sim_next;
            %         load(sim_param.run_name, "role_sim", "role_sim_next");
            %         no_role_changes = true;
            %         for iter = 1:size(role_sim, 1)
            %             if ~(no_role_changes && ...
            %                     all(size(unique(role_sim(iter, :))) == [1, 1]) && ...
            %                     all(size(unique(role_sim_next(iter, :))) == [1, 1]))
            %                 no_role_changes = false;
            %             end
            %         end
            %         run_flag_update_all(seed + 1, type_id) = no_role_changes;
            %     end
            % end
            % run_summary.run_no_role_changes_flag_all = run_flag_update_all;
            %% Update flag 

            save(summary_file, "run_summary");
        end
        function stats = get_summary_stats(summary)
            type_id_can = summary.run_info.type_id_can;
            sim_type = summary.run_info.type(type_id_can, 3);
            run_flag_all = summary.run_flag_all;
            collision_flag_all = summary.collision_flag_all;
            min_dist_flag = summary.min_dist_flag;
            % util get the frequency of each column
            get_pct = @(s) s ./ sum(s) * 100;
            % for both 2 game run and mpc + game rune,
            % 1 means eastbound arrives first,
            % 2 means northbound arrives first,
            % 3 means both arrive at the same time.
            % 0 means neither arrives.
            category_name = {'EB', 'NB', 'same time','not finished'};
            get_hist = @(flag) hist(categorical(flag, [1, 2, 3, 0], category_name));
            run_flag_summary_pct.col_name = sim_type;
            run_flag_summary_pct.row_name = category_name;
            run_flag_summary_pct.value = get_pct(get_hist(run_flag_all));
            if isfield(summary, "run_flag_update_all")
                run_flag_update_all = summary.run_flag_update_all;
                run_flag_summary_pct_update.col_name = sim_type;
                run_flag_summary_pct_update.row_name = category_name;
                run_flag_summary_pct_update.value = get_pct(get_hist(run_flag_update_all));
            end
            
            category_name = {'No collision','collision'};
            collision_flag_summary_pct.col_name = sim_type;
            collision_flag_summary_pct.row_name = category_name;
            get_hist = @(flag) hist(categorical(flag,[0, 1], category_name));
            collision_flag_summary_pct.value = get_pct(get_hist(collision_flag_all));

            category_name = {'Yes','No'};
            min_dist_flag_summary_pct.col_name = sim_type;
            min_dist_flag_summary_pct.row_name = category_name;
            get_hist = @(flag) hist(categorical(flag,[1, 0], category_name));
            min_dist_flag_summary_pct.value = get_pct(get_hist(min_dist_flag));
            if isfield(summary, "run_flag_update_all")
                min_dist_flag_update = summary.min_dist_flag_update;
                min_dist_flag_update_summary_pct.col_name = sim_type;
                min_dist_flag_update_summary_pct.row_name = category_name;
                min_dist_flag_update_summary_pct.value = get_pct(get_hist(min_dist_flag));
            end
            stats.run_flag = run_flag_summary_pct;
            stats.collision_flag = collision_flag_summary_pct;
            stats.min_dst_flag = min_dist_flag_summary_pct;
            if isfield(summary, "run_flag_update_all")
                stats.run_flag_update = run_flag_summary_pct_update;
                stats.min_dst_flag_update = min_dist_flag_update_summary_pct;
            end
        end
        function figs = get_summary_plot(input, save_plot)
            if isstruct(input)
                save_plot = false;
                summary = input;
            else
                % is string
                result_root_folder = input;
                summary_file = fullfile(result_root_folder, "summary.mat");
                load(summary_file, "run_summary");
                summary = run_summary;
                if nargin < 2
                    save_plot = false;
                end
            end
            run_flag_all = summary.run_flag_all;
            collision_flag_all = summary.collision_flag_all;
            % min_dist_all = summary.min_dist_all;
            min_dist_flag = summary.min_dist_flag;
            x_ini_all = summary.x_ini_all;
            if isfield(summary, "has_infeasible")
                has_infeasible = summary.has_infeasible;
            end
            type_id_can = summary.run_info.type_id_can;
            sim_type = summary.run_info.type;
            game_param = summary.run_info.common_parameters.game_param;
            sim_param = summary.run_info.common_parameters.sim_param;
            % util get the frequency of each column
            get_pct = @(s) s ./ sum(s) * 100;
            % for both 2 game run and mpc + game rune,
            % 1 means eastbound arrives first,
            % 2 means northbound arrives first,
            % 3 means both arrive at the same time.
            % 0 means neither arrives.
            plot_hist = @(flag) hist(categorical(flag,[1, 2, 3, 0],{'EB', 'NB', 'same time','not finished'}));
            run_flag_summary = plot_hist(run_flag_all);
            run_flag_summary_pct = get_pct(run_flag_summary);
            fig_nums = zeros(1, 3);
            fig_nums(1) = figure;
            plot_hist(run_flag_all)
            disp(run_flag_summary_pct)
            legend(sim_type(type_id_can, 3))
            title("Who finished first?");
            plot_hist = @(flag) hist(categorical(flag,[0, 1],{'No collision','collision'}));
            collision_flag_summary = plot_hist(collision_flag_all);
            collision_flag_summary_pct = get_pct(collision_flag_summary);
            fig_nums(2) = figure;
            plot_hist(collision_flag_all)
            disp(collision_flag_summary_pct)
            legend(sim_type(type_id_can, 3))
            title("Any collision?");
            plot_hist = @(flag) hist(categorical(flag,[1, 0],{'Yes','No'}));
            min_dist_flag_summary = plot_hist(min_dist_flag);
            min_dist_flag_summary_pct = get_pct(min_dist_flag_summary);
            fig_nums(3) = figure;
            plot_hist(min_dist_flag);
            disp(min_dist_flag_summary_pct)
            legend(sim_type(type_id_can, 3))
            title(sprintf('Min dist greater than %.3f?', game_param.reward.min_dist));
            figure_names = {'who_arrives_first', 'any_collision', 'mini_dist'};
        
            if sim_param.randomize_initial_condition
                colors = dictionary([0, 1, 2, 3], ["k", "r", "b", "g"]);
                flags_description = dictionary([0, 1, 2, 3], ["not finished", "eb", "nb", "same time"]);
                if isfield(summary, "run_flag_update_all")
                    [fig_all_part, fig_all_names_part] = game.GenSummary.plot_flag_vs_ic(summary.run_flag_update_all, flags_description, colors, ...
                                                       x_ini_all, sim_type(type_id_can, 3), "results_distribute_");
                else
                    [fig_all_part, fig_all_names_part] = game.GenSummary.plot_flag_vs_ic(summary.run_flag_all, flags_description, colors, ...
                                                       x_ini_all, sim_type(type_id_can, 3), "results_distribute_");
                end
                fig_nums = [fig_nums, fig_all_part];
                figure_names = [figure_names, fig_all_names_part];
            end
        
            figs.handle = fig_nums;
            figs.names = figure_names;
            if save_plot
                fig_target_folder = fullfile(result_root_folder, "figures");
                if ~exist(fig_target_folder, 'dir')
                    mkdir(fig_target_folder);
                end
                for i = 1:length(figs.handle)
                    saveas(figs.handle(i), fullfile(fig_target_folder, figs.names{i} + ".png"));
                end
             end
        end
        function [fig_nums, fig_names]= plot_flag_vs_ic(run_flag_all, flags_description, colors, ...
                                                       x_ini_all, sim_type, fig_header)
            
            num_type = size(run_flag_all, 2);
            fig_nums = zeros(1, num_type);
            fig_names = cell(1, num_type);
            for type_id = 1:num_type
                all_flags = unique(run_flag_all);
                fig_nums(type_id) = figure; hold on;grid on; box on;
                legends = cell(1, length(all_flags));
                for i = 1:length(all_flags)
                    idxs = find(run_flag_all(:, type_id) == all_flags(i));
                    scatter(x_ini_all(1, idxs, type_id) - x_ini_all(3, idxs, type_id), ...
                            x_ini_all(2, idxs, type_id) - x_ini_all(4, idxs, type_id), ...
                            colors(all_flags(i)), 'filled');
                    pct = length(idxs)/length(x_ini_all) * 100;
                    legends{i} = flags_description(all_flags(i)) + sprintf(" %.2f %%", pct);
                end
                legend(legends, 'Location', 'SouthWest');
                title(sim_type{type_id});
                ylabel("$v_{\rm nb} - v_{\rm eb}$ [m/s]");
                xlabel("$s_{\rm nb} - s_{\rm eb}$ [m]");
                fig_names{type_id} = fig_header + sim_type{type_id};
            end
        end
        function  run_name = get_run_name(result_root_folder, type_id, seed)
            summary_file = fullfile(result_root_folder, "summary.mat");
            load(summary_file, "run_summary");
            summary = run_summary;
            sim_type = summary.run_info.type;
            run_type = sim_type{type_id, 3};
            run_name = fullfile(result_root_folder, run_type, sprintf("%03d.mat", seed));
        end
        function [fig_all, fig_all_names] = plot_one_run(result_root_folder, type_id, seed, ...
                                                                              save_plot, save_target, save_name);
            if nargin < 4
                save_plot = 0;
            end
            publish = false;
            ext = "";
            if save_plot == 2
                publish = true;
                frame_vertical = true;
                ext = "_pub";
            end
            if nargin < 5
                save_target = [];
                save_name = [];
            end
            if nargin < 6
                save_name = [];
            end

            summary_file = fullfile(result_root_folder, "summary.mat");
            load(summary_file, "run_summary");
            summary = run_summary;
            sim_type = summary.run_info.type;

            run_type = sim_type{type_id, 3};
            run_to_plot = fullfile(result_root_folder, run_type, sprintf("%03d.mat", seed));
            if ~exist(run_to_plot, 'file')
                fprintf("File not exist, will rerun: %s\n", run_to_plot);
                game.GenSummary.rerun_one_run(result_root_folder, type_id, seed);
            else
                fprintf("%s exists, reload. \n", run_to_plot);
            end
            game.GenSummary.check_for_inconsistency(summary, run_to_plot);
            load(run_to_plot);

            if exist("mpc_param", "var")
                % if save_plot
                %     fig_target_folder_frame = fullfile(result_root_folder, "figures", run_type, sprintf("%03d", sim_param.seed), "frames");
                %     if ~exist(fig_target_folder_frame, 'dir')
                %         mkdir(fig_target_folder_frame)
                %     end
                % end
                fig_all  = game.plotIntersectionRun(run_to_plot, "publish", publish, "frame_vertical", frame_vertical);
            else
                fig_all  = game.plotIntersectionRunGame(run_to_plot, "publish", publish, "frame_vertical", frame_vertical);
            end
            title_name = sprintf("%s  %03d", sim_type{type_id, 3}, seed);
            % load(run_to_plot, "sim_param");
            if save_plot
                if isempty(save_target)
                    fig_target_folder = fullfile(result_root_folder, "figures", run_type, sprintf("%03d", sim_param.seed));
                else
                    fig_target_folder = save_target;
                end
                if ~exist(fig_target_folder, 'dir')
                    mkdir(fig_target_folder)
                end
            end
            fig_all_names = cell(1, length(fig_all));
            for i = 1:length(fig_all)
                set(fig_all(i), 'Name', title_name);
                if save_plot
                    if isempty(save_name)
                        name = num2str(i) + ext;
                    else
                        name = save_name{i};
                    end
                    fig_all_names{i} = fullfile(fig_target_folder, name + ".fig");
                    saveas(fig_all(i), fig_all_names{i});
                    fig_all_names{i} = fullfile(fig_target_folder, name + ".png");
                    exportgraphics(fig_all(i), fig_all_names{i},'BackgroundColor','none','ContentType','vector');
                end
            end
        end
        function check_for_inconsistency(summary, run_results)
            % if rerun a test, alert if the result is different from the summary suggested
            % from the summary

            run_flag_all = summary.run_flag_all ;
            collision_flag_all = summary.collision_flag_all;
            x_ini_all = summary.x_ini_all;
            load(run_results, "collision_flag", "flag_who_arrived_first", "x_ini", "sim_param");
            row_idx = find(summary.run_info.seed_can == sim_param.seed);
            col_idx = find(cellfun(@(x) x == sim_param.run_type, summary.run_info.type(:,3)));
            if run_flag_all(row_idx, col_idx) ~= flag_who_arrived_first
                warning("Inconsistency in flag_who_arrived_first: %s\n", run_results);
            end
            if collision_flag_all(row_idx, col_idx) ~= collision_flag
                warning("Inconsistency in collision_flag: %s\n", run_results);
            end
            if ~isequal(x_ini_all(:, row_idx, col_idx), x_ini)
                warning("Inconsistency in x_ini: %s\n", run_results);
            end
        end
        function rerun_one_run(result_root_folder, type_id, seed, debug_print)
            % TODO, make this also for game results rerun
            summary_file = fullfile(result_root_folder, "summary.mat");
            load(summary_file, "run_summary");
            game_param = run_summary.run_info.common_parameters.game_param;
            if nargin < 4
                debug_print = false;
            end
            game_param.print_off = ~debug_print;
            mpc_run = false;
            if isfield(run_summary.run_info.common_parameters, "mpc_param")
                mpc_run = true;
                mpc_param = run_summary.run_info.common_parameters.mpc_param;
                mpc_param.print_off = ~debug_print;
            end
            sim_param = run_summary.run_info.common_parameters.sim_param;
            sim_type = run_summary.run_info.type;
            
            % set the parameters same way as in the mpc loop
            sim_param.x_ini = sim_type{type_id, 1};
            sim_param.initial_role = sim_type{type_id, 2};
            % Add the following in case that that when role is not changing, the
            % initial role will not be used to override the game_param role
            game_param.role = sim_type{type_id, 2};
            mpc_param.agent_model.role = game_param.role;
            sim_param.run_type = sim_type{type_id, 3};
            target_folder = fullfile(result_root_folder + filesep + sim_param.run_type);
            if ~exist(target_folder, 'dir')
                mkdir(target_folder);
            end
            sim_param.seed = seed;
            sim_param.run_name = fullfile(result_root_folder, sim_param.run_type, sprintf("%03d", sim_param.seed));
            if mpc_run
                game.simIntersectionRun(game_param, mpc_param, sim_param);
            else
                game.simIntersectionRunTwoGameAgent(game_param, sim_param);
            end
        end
        %% Util
        function is_same = are_two_runs_same(run1, run2)
            function results = strip_data(results)
                results = rmfield(results, 'tStart'); % run executed time would be different
                results = rmfield(results,'time_used'); % very according to machines
                results = rmfield(results.sim_param, "run_name"); % actual save name depends on root folder, may be different
            end
            run1_data = strip_data(load(run1));
            run2_data = strip_data(load(run2));
            is_same = isequal(run1_data, run2_data);
        end
        function latex_table = table2latex(table, col_names, row_names, num_digits)
            % Get the number of rows and columns in the table
            num_rows = size(table, 1);
            num_cols = size(table, 2);

            if isempty(col_names)
                col_names = arrayfun(@(x) num2str(x), 1:num_cols, 'UniformOutput', false);
            end
            if isempty(row_names)
                row_names = arrayfun(@(x) num2str(x), 1:num_rows, 'UniformOutput', false);
            end

            % Initialize the LaTeX table string
            latex_table = sprintf('\\begin{tabular}{|%s}\n', repmat('c|', 1, num_cols + 1));
            
            latex_table = sprintf('%s \\hline \n &', latex_table);
            % Add the column names
            for j = 1:num_cols
                col_name = col_names{j};
                latex_table = sprintf('%s%s', latex_table, col_name);
                if j < num_cols
                    latex_table = sprintf('%s &', latex_table);
                else
                    latex_table = sprintf('%s \\\\ \\hline \n', latex_table);
                end
            end
            % latex_table = sprintf('%s\\hline\n%s & %s \\\\ \n', latex_table, col1_name, col2_name);
            
            % Loop over the rows of the table
            for i = 1:num_rows
                % Loop over the columns of the table
                for j = 1:num_cols
                    % Get the value in the current cell
                    value = table(i, j);
                    
                    % Format the value as a string with the specified number of digits
                    value_str = sprintf('%.*f', num_digits, value);
                    
                    % Add the value to the LaTeX table string
                    if j == 1
                        latex_table = sprintf('%s %s & %s', latex_table, row_names{i}, value_str);
                    else
                        latex_table = sprintf('%s & %s', latex_table, value_str);
                    end
                end
                
                % Add a newline character and a line break
                latex_table = sprintf('%s \\\\ \\hline \n', latex_table);
            end
            
            % Add the final line break and the end of the tabular environment
            latex_table = sprintf('%s  \\end{tabular}\n', latex_table);
        end
    end
end
