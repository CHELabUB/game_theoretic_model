classdef PlotIntersection
    % can run without defining the class object
    methods(Static)
        function fig = drawIntersection(time, redCarPos, blueCarPos, redCarSpeed, blueCarSpeed, blueCarRole, ax)
            % horPos: [x, y] of the horizontal road segment
            horPos = [0, 0];
            % vertPos: [x, y] of the vertical road segment
            vertPos = [0, 0];
            roadWidth = 4;
            vehWidth = 2.5;
            vehLength = 5.0;
            grassTexture = imread(fullfile('asset', 'grass_texture.png')); % load grass texture
            [redCar, redCarAlpha] = game.PlotIntersection.removeWhiteBackground(fullfile('asset', 'red_car.png')); 
            [blueCar, blueCarAlpha] = game.PlotIntersection.removeWhiteBackground(fullfile('asset', 'blue_car.png')); 

            % blueCarRole: to be displayed in the title
        
            if nargin < 7 || isempty(ax)
                width = 5; % inch
                height = 5; % inch
                fig = Util.fig_setup_on_screen(width, height); 
            else
                axes(ax);
                fig = gcf;
            end
        
            % create grass background
            hold on;
            [x, y] = meshgrid(-25:5, -25:5);
            surface(x, y, zeros(size(x)), 'FaceColor', 'texturemap', 'CData', grassTexture, 'EdgeColor', 'none');
        
            % Draw horizontal road segment
            fill([horPos(1)-roadWidth/2, horPos(1)+roadWidth/2, horPos(1)+roadWidth/2, horPos(1)-roadWidth/2], ...
                 [-25, -25, 5, 5], [0.6, 0.6, 0.6], 'EdgeColor', 'none');
        
            % Draw vertical road segment
            fill([-25, 5, 5, -25], ...
                 [vertPos(2)-roadWidth/2, vertPos(2)-roadWidth/2, vertPos(2)+roadWidth/2, vertPos(2)+roadWidth/2], ...
                 [0.6, 0.6, 0.6], 'EdgeColor', 'none');
        
            % Draw the intersection center
            fill([horPos(1)-roadWidth/2, horPos(1)+roadWidth/2, horPos(1)+roadWidth/2, horPos(1)-roadWidth/2], ...
                 [vertPos(2)-roadWidth/2, vertPos(2)-roadWidth/2, vertPos(2)+roadWidth/2, vertPos(2)+roadWidth/2], ...
                 [0.6, 0.6, 0.6], 'EdgeColor', 'none');
        
            % Draw the intersection corners
            theta = linspace(0, pi/2, 100);
            r = roadWidth / 2; % radius of the corner
            x1 = horPos(1) - 2*r + r*cos(theta);
            y1 = vertPos(2) - 2*r + r*sin(theta);
            x2 = horPos(1) + 2*r - r*cos(theta);
            y2 = vertPos(2) - 2*r + r*sin(theta);
            x3 = horPos(1) - 2*r + r*cos(theta);
            y3 = vertPos(2) + 2*r - r*sin(theta);
            x4 = horPos(1) + 2*r - r*cos(theta);
            y4 = vertPos(2) + 2*r - r*sin(theta);
        
            fill([horPos(1)-roadWidth/2, x1, horPos(1)-roadWidth/2], [vertPos(2)-roadWidth/2, y1, vertPos(2)+roadWidth/2], [0.6, 0.6, 0.6], 'EdgeColor', 'none'); % 左下角
            fill([horPos(1)+roadWidth/2, x2, horPos(1)+roadWidth/2], [vertPos(2)-roadWidth/2, y2, vertPos(2)+roadWidth/2], [0.6, 0.6, 0.6], 'EdgeColor', 'none'); % 右下角
            fill([horPos(1)-roadWidth/2, x3, horPos(1)-roadWidth/2], [vertPos(2)+roadWidth/2, y3, vertPos(2)-roadWidth/2], [0.6, 0.6, 0.6], 'EdgeColor', 'none'); % 左上角
            fill([horPos(1)+roadWidth/2, x4, horPos(1)+roadWidth/2], [vertPos(2)+roadWidth/2, y4, vertPos(2)-roadWidth/2], [0.6, 0.6, 0.6], 'EdgeColor', 'none'); % 右上角
        
            % draw road center lines
            plot([-25, horPos(1)-roadWidth], [vertPos(2), vertPos(2)], 'w--', 'LineWidth', 0.8); % west
            plot([horPos(1)+roadWidth, 5], [vertPos(2), vertPos(2)], 'w--', 'LineWidth',  0.8); % east
            plot([horPos(1), horPos(1)], [-25, vertPos(2)-roadWidth], 'w--', 'LineWidth',  0.8); % south
            plot([horPos(1), horPos(1)], [vertPos(2)+roadWidth, 5], 'w--', 'LineWidth',  0.8); % north
        
            % draw the crossing line
            line([horPos(1)-roadWidth/2, horPos(1)+roadWidth/2], [vertPos(2)-roadWidth, vertPos(2)-roadWidth], 'Color', 'w', 'LineWidth', 0.8); % south
            line([horPos(1)-roadWidth/2, horPos(1)+roadWidth/2], [vertPos(2)+roadWidth, vertPos(2)+roadWidth], 'Color', 'w', 'LineWidth', 0.8); % north
            line([horPos(1)-roadWidth, horPos(1)-roadWidth], [vertPos(2)-roadWidth/2, vertPos(2)+roadWidth/2], 'Color', 'w', 'LineWidth', 0.8); % west
            line([horPos(1)+roadWidth, horPos(1)+roadWidth], [vertPos(2)-roadWidth/2, vertPos(2)+roadWidth/2], 'Color', 'w', 'LineWidth', 0.8); % east
        
            carHeight = vehWidth;
            % carWidth = size(redCar, 2) / size(redCar, 1) * carHeight; % keep the ratio
            carWidth = vehLength; % keep the original ratio

            % draw the red car
            image('CData', redCar, 'XData', [redCarPos(1)-carWidth/2, redCarPos(1)+carWidth/2], ...
                  'YData', [redCarPos(2)-carHeight/2, redCarPos(2)+carHeight/2], 'AlphaData', redCarAlpha);
            % draw the blue car
            image('CData', blueCar, 'XData', [blueCarPos(1)-carHeight/2, blueCarPos(1)+carHeight/2], ...
                 'YData', [blueCarPos(2)-carWidth/2, blueCarPos(2)+carWidth/2], 'AlphaData', blueCarAlpha);
    
            % set the axis
            axis equal;
            xlim([-25, 5]);
            ylim([-25, 5]);
            set(gca, 'FontName', 'Times', 'FontSize', 15, 'XTick', -25:5:5, 'YTick', -25:5:5, 'XTickLabelRotation', 0);
            box on; 

            xlabel('$\it{x}$ [m]', 'Interpreter', 'latex');

            % ylabel({'$\it{y}$'; '\textnormal{[m]}'}, 'Interpreter', 'latex', 'Rotation', 0, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'center');
             % if not vertical move the ylabel to the right a bit (+7)
            % if vertical move left a bit (-1)
            h = ylabel({'$\it{y}$'; '\textnormal{[m]}'}, 'Interpreter', 'latex', 'Rotation', 0, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'center');
            h.Position(1) = h.Position(1) - 2;  
            
        
            % Add title with Blue vehicle role
            title(sprintf('Blue Vehicle Role:  %s', blueCarRole), 'FontName', 'Times', 'FontSize', 15);
        
        
            % Add Legend
            % legendText = {['time:   ', sprintf('   %.1f', time), ' [s]'], ...
            %           ['\color{black}\color{blue}— \color{black}v = ', sprintf('%.1f', blueCarSpeed), ' [m/s]'], ...
            %           ['\color{black}\color{red}— \color{black}v = ', sprintf('%.1f', redCarSpeed), ' [m/s]']};
            % text(-24, -20, legendText, 'FontSize', 15, 'FontName', 'Times','BackgroundColor', 'white', 'EdgeColor', 'black', 'Units', 'data', 'Interpreter', 'tex');
            legendText = {['time: ', sprintf('    %.1f', time), ' [s]'], ...
                      ['\color{black}\color{blue}— \color{black}\it v \rm = ', sprintf('%.1f', blueCarSpeed), ' [m/s]'], ...
                      ['\color{black}\color{red}— \color{black}\it v \rm = ', sprintf('%.1f', redCarSpeed), ' [m/s]']};
            text(-24, -20, legendText, 'FontSize', 15, 'FontName', 'Times', 'BackgroundColor', 'white', 'EdgeColor', 'black', 'Units', 'data', 'Interpreter', 'tex');
            hold off;
        end
        function [carImage, alpha] = removeWhiteBackground(carImagePath)
            [carImage, ~, alpha] = imread(carImagePath);
        
            % create alpha channel if it does not exist
            if isempty(alpha)
                alpha = ones(size(carImage, 1), size(carImage, 2));
            end
        
            % remove white background
            whiteThreshold = 240; % threshold for white pixels
            whitePixels = all(carImage > whiteThreshold, 3);
            alpha(whitePixels) = 0;
        end
        function fig = plot_one_run(time_sim, x_sim, role_sim, time_to_plot, vertical)
            % by default plot 4 frame
            if nargin < 4 || isempty(time_to_plot)
                time_to_plot = [0.0, 1.0, 2.0, 3.0];
                % both in inches
                width = 16; 
                height = 5;
            else
                width = 5 * length(time_to_plot);
                height = 5;
            end
            if nargin < 5 || isempty(vertical)
                vertical = false;
            end
            if vertical
                www = width;
                width = height;
                height = www;
            end
            fig = Util.fig_setup_on_screen(width, height);
            for i = 1:length(time_to_plot)
                % find the closest time point in the simulation data
                time_index = find(abs(time_sim - time_to_plot(i)) < 1e-5);
            
                if isempty(time_index)
                    disp(['cannot find: ', num2str(time_to_plot(i))]);
                    continue;
                end
            
                % get the positions and speeds of the red and blue cars at the current time point
                redCarPos = [x_sim(1, time_index), x_sim(3, time_index)];
                redCarSpeed = x_sim(2, time_index);
                blueCarPos = [x_sim(5, time_index), x_sim(7, time_index)];
                blueCarSpeed = x_sim(8, time_index);
            
                % get the role of the blue car at the current time point
                blueCarRole = role_sim(time_index);
                if vertical
                    subplot(length(time_to_plot), 1, i);
                else
                    subplot(1, length(time_to_plot), i);
                end
                game.PlotIntersection.drawIntersection(time_to_plot(i), redCarPos, blueCarPos, ...
                                                       redCarSpeed, blueCarSpeed, blueCarRole, gca);            
            end
        end
    end
end