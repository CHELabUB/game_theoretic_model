function [flag_who_arrived_first, flag_who_arrived_stop_line_first, could_stop] = ...
    simIntersectionFinishCheck(x_game_current, stop_line_pos, flag_who_arrived_first, flag_who_arrived_stop_line_first)
%% function that checks for final conditions
% the x is in the game coordinate of the NB vehicle
could_stop = false;
if flag_who_arrived_first == 0
  if x_game_current(1) >= 0 && x_game_current(3) < 0
      % north bound vehicle arrived first
      flag_who_arrived_first = 2;
  else 
      if x_game_current(3) >= 0 && x_game_current(1) < 0
          % east bound vehicle arrived first
          flag_who_arrived_first = 1;
      else
          if x_game_current(3) >= 0 && x_game_current(1) >= 0
              % arrived at the same time, very rare but could still
              % happen, only able to see if going finer mesh
              flag_who_arrived_first = 3;
          end
      end
  end
end

if flag_who_arrived_stop_line_first == 0
  % there can be bug here when both are zero
  if x_game_current(1) >= -stop_line_pos && x_game_current(3) < -stop_line_pos
      % other vehicle arrived first
      flag_who_arrived_stop_line_first = 2;
      could_stop = true;
  else 
      if x_game_current(3) >= -stop_line_pos && x_game_current(1) < -stop_line_pos
          flag_who_arrived_stop_line_first = 1;
          could_stop = true;
      else
          if x_game_current(3) >= -stop_line_pos && x_game_current(1) >= -stop_line_pos
              % arrived at the same time, very rare but could still
              % happen, only able to see if going finer mesh
              flag_who_arrived_stop_line_first = 3;
              could_stop = true;
          end
      end
  end
end
end
