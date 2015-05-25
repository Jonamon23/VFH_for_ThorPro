function picked_sector = pickBestValley(valleys, goal_sector)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This function chooses a direction when valid valleys are found. This is
%   accomplished by defining the best valley to pass through, and then choosing 
%   the best path to take through this valley.
%   To define the best valley, the difference is calculated between the target 
%   angle and each valley's start/end points. The valley with the minimum 
%   difference among all values is considered the best valley.  
% 
% Updated By: Jonathon Kreska
% Version Date: Jan 27, 2015
% Version: 1.0
% 
% Changelog:
%  1.0: Initial Release
% 
% Inputs:
%  valleys - 
%  goal_sector - 
% 
% Outputs:
%  picked_sector - 
% 
% Usage:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

valley_buffer = 6;

START = 1;
END = 2;
WIDTH = 3;
TOGOAL_S = 4;
TOGOAL_E = 5;

valleys(:, TOGOAL_S) = goal_sector-valleys(:,1);
  % represents the differences from the beginning of valleys
valleys(:, TOGOAL_E)= valleys(:,2)-goal_sector*ones(size(valleys(:,2)));
  % represents the difference from the ending of valleys

assignin('base', 'valleys', valleys); % DBG

% find the minimum of the valley start differences
[s1, i1] = min(abs(valleys(:, TOGOAL_S)));   
% find the minimum of the valley end differences
[s2, i2] = min(abs(valleys(:, TOGOAL_E)));
% if the target angle is closer to a valley start, find this
% valley and store its index in 'i'.

if ( s1 < s2 )
  % if the target angle's deflection from the beginning
  % of a valley and the end of a valley is greater, then
  % have the minimum valley width (so the vehicle has at
  % least the minimum valley width to go through), then
  % the steering angle is set to the target angle

  if (valleys(i1, TOGOAL_S) >= ((valley_buffer)/2))
    if (valleys(i1, TOGOAL_E) >= ((valley_buffer)/2))
      picked_sector = goal_sector; % the goal itself

      % if the deflection from the beginning of a valley is
      % greater than half of the minimum valley size but not
      % from the end of the valley, the vehicle is directed
      % to the valley's center.
      % steering angle = beginning_angle + valley width/2
    else
      picked_sector = (valleys(i1, START) + valleys(i1, END))/2;
      % center of valley
    end
    % if the target angle deflection from the beginning of
    % a valley is less than half of the minimum valley,
    % the vehicle is directed to the center of the valley

  else
    if (valleys(i1, WIDTH) < valley_buffer)
      % Center of the valley
      picked_sector = (valleys(i1, START) + valleys(i1, END))/2;
    else
      picked_sector = valleys(i1, START)+(valley_buffer/2);
      % Offset only by min_valley.
    end
  end

else
  % if the deflection from the end of a valley is
  % smaller, then use the same logic used above to
  % determine the steering angle (considering the end
  % of a valley instead of the beginning)
  if (valleys(i2, TOGOAL_E) >= ((valley_buffer)/2))
    if (valleys(i2, TOGOAL_S) >= ((valley_buffer)/2))
      picked_sector = goal_sector;  % the target itself
    else
      picked_sector = (valleys(i2, START)+valleys(i2, END))/2; % center of valley
    end

  else
    if (valleys(i2, WIDTH) < valley_buffer)
      % Center of the valley
      picked_sector = (valleys(i2, START)+valleys(i2, END))/2; % center of valley
    else
      picked_sector = valleys(i2, END)-(valley_buffer/2); % offset by buffer
    end
  end
end