function fig_all  = plotIntersectionRunGame(run_name, varargin)
    defaults = {"save_frame", [], "publish", false, "frame_vertical", false};
    options = Util.SetOptions(defaults, varargin);

    load(run_name);
    fig_frames(1) = game.PlotIntersection.plot_one_run(time_sim, x_sim, role_sim(1, :), [], options.frame_vertical);

    if options.publish
        width = 7;
        height = 5.5;
        num_rows = 4;
        num_cols = 1;
        nb_colors = {"bo-",};
        eb_colors = {"rx-.",};
    else
        width = 9;
        height = 13;
        num_rows = 7;
        num_cols = 1;
        nb_colors = {"bo-", "gs--"};
        eb_colors = {"rx-.", "m+:"};
    end

    fig_profile = Util.fig_setup_on_screen(width, height);

    ax = zeros(num_rows, num_cols);
    iter = 0;

    iter = iter + 1;
    ax(iter) = subplot(num_rows, num_cols, iter);hold on;box on;grid on;

    if size(role_sim, 1) > 1
        plot(time_sim(1:end - 1), categorical(role_sim(1, :), {'Follower', 'Leader'}), nb_colors{1});
        plot(time_sim(1:end - 1), categorical(role_sim(2, :), {'Follower', 'Leader'}), eb_colors{1});
        blue_role_prob = (role_sim(1, :) == "Leader");
        red_role_prob = (role_sim(2, :) == "Leader");
        if ~options.publish
            legend("Blue (nb) Actual Role", "Red (eb) Actual Role");
        end
    else
        plot(time_sim(1:end-1), categorical(role_sim, {'Follower', 'Leader'}), nb_colors{1});
        blue_role_prob = (role_sim == "Leader");
        red_role_prob = (role_sim == "Follower");
        if ~options.publish
            legend("Blue (nb) Actual Role");
        end
    end

    if game_param.role_update_type == 2
        plot(time_sim, categorical(role_sim_next), 'bd-');
        blue_role_prob = (role_sim_next == "Leader");
        red_role_prob = (role_sim_next == "Follower");
    end

    iter = iter + 1;
    ax(iter) = subplot(num_rows, num_cols, iter);hold on;box on;grid on;
    plot(time_sim(1:end-1), trace_prob_nb_on_eb(1, :), nb_colors{1});
    if ~options.publish
        plot(time_sim(1:end-1), red_role_prob(1:end), eb_colors{2});
    end
    plot(time_sim(1:end-1), trace_prob_eb_on_nb(1, :), eb_colors{1});
    if ~options.publish
        plot(time_sim(1:end-1), blue_role_prob(1:end), nb_colors{2});
    end
    ylim([0.0, 1.0]);

    if ~options.publish
        ylabel("$p$", 'Rotation',0);
        legend({"blue (nb) thinks red as Leader", "red (eb) is leader", ...
                "red (eb) thinks blue as Leader", "blue (nb) is leader" }, "Interpreter", "Latex");
    else
        % plot legend line
        x_legend = linspace(2, 2.7, 4);
        plot(x_legend, x_legend * 0 + 0.3, eb_colors{1});
        plot(x_legend, x_legend * 0 + 0.6, nb_colors{1});
    end

    if ~options.publish
        iter = iter + 1;
        ax(iter) = subplot(num_rows, num_cols, iter);hold on;box on;grid on;
        plot(time_sim(1:end - 1), reshape(role_trans_prob(1, 1, :), [1, steps]), nb_colors{2});
        plot(time_sim(1:end - 1), reshape(role_trans_prob(2, 2, :), [1, steps]), eb_colors{2});
        ylim([0.0, 1.0]);
        ylabel("$\pi$", 'Rotation',0);
        legend({"$\pi_{\rm ll}$", "$\pi_{\rm ff}$"}, "Interpreter", "Latex");

        iter = iter + 1;
        ax(iter) = subplot(num_rows, num_cols, iter);hold on;box on;grid on;
        % negative means red car (eb) is closer.
        deltaS = (0 - x_sim(1, :)) - (0 - x_sim(7, :));
        % means already arrived, no need to consider.
        deltaS(x_sim(1, :) > 0 | x_sim(7, :) > 0) = 0;
        plot(time_sim, deltaS, nb_colors{1});
        if any(collision_log)
            fprintf("Collision detected. \n");
            plot(time_sim, collision_log, 'k+:');
            legend({" < 0 means red car (eb) is closer", "collision flag"});
        else
            ylabel("$\Delta s$", 'Rotation',0);
            legend("< 0 means red car (eb) is closer");
        end
    end

    iter = iter + 1;
    ax(iter) = subplot(num_rows, num_cols, iter);hold on;box on;grid on;
    % In simulation the first set of states belongs to eb vehicle 
    plot(time_sim, x_sim(8, :), nb_colors{1});
    plot(time_sim, x_sim(2, :), eb_colors{1});
    if ~options.publish
        legend({"blue nb", "red eb"}, "Interpreter", "Latex");
        ylabel("$[\rm m/s]$", 'Rotation',0);
    else
        ylim([0.0, 10.0]);
    end
    iter = iter + 1;
    ax(iter) = subplot(num_rows, num_cols, iter);hold on;box on;grid on;
    uu = u_sim_to_game(u_sim); 
    % translate to nb vehicle perspective so the first one is for nb blue vehicle
    plot(time_sim(1:end - 1), uu(1, :), nb_colors{1});
    plot(time_sim(1:end - 1), uu(2, :), eb_colors{1});
    if ~options.publish
        ylabel("$a [{\rm m/s^2}] $", 'Rotation',0);
    else
        % plot legend line
        x_legend = linspace(3.5, 4.2, 4);
        plot(x_legend, x_legend * 0 + 0.0, nb_colors{1});
        plot(x_legend, x_legend * 0 - 1.0, eb_colors{1});
        ylim([-2.0, 1.0]);
    end

    if ~options.publish
        iter = iter + 1;
        ax(iter) = subplot(num_rows, num_cols, iter);hold on;box on;grid on;
        % nb game first (because this is the main actor in game), then eb second
        plot(time_sim(1:end - 1), u_action(1, :), nb_colors{1});
        plot(time_sim(1:end - 1), u_action(2, :), eb_colors{1});
    end

    linkaxes(ax, 'x');
    if ~options.publish
        ylabel("idx", 'Rotation',0);
        xlabel("time");
    else
        xlim([0, 6])
    end
    
    fig_all = [fig_frames, fig_profile];
    if exist("flag_who_arrived_stop_line_first", "var")
        fprintf("Use new flag. \n");
        flag = flag_who_arrived_stop_line_first;
    else
        flag = flag_who_arrived_first;
        fprintf("Use old flag. \n");
    end
    switch flag
        case 1
            fprintf("eastbound (red, other in nb game) vehicle arrived first. \n");
        case 2
            fprintf("northbound (blue, ego in nb game) vehicle arrived first. \n");
        case 3
            fprintf("both vehicle arrived at the same time (subject to dt resolution). \n");
        otherwise
            fprintf("wired run, neither arrived at the intersection. \n");
    end
end
