function fig = fig_setup_on_screen(width, height)
% the goal of this function is to set up the figure to be properly displayed 
% and stay consistence on the screen from different devices
% width and inches, mainly for publication purpose
fig = figure();
set(gcf, "unit", "inches");
% set(gcf, "Position", [23, 5, width, height]); % home, multiple screen
set(gcf, "Position", [0.5, 0.5, width, height]); % office, one screen
end
