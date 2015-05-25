function trap_escape_angle = ...
  trapDetermineEscape(polar_hist, sector_resolution, sensor)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This function calculates the escape angle if robot is trapped
% 
% Updated By: Jonathon Kreska
% Version Date: Jan 27, 2015
% Version: 1.0
% 
% Changelog:
%  1.0: Initial Release
% 
% Inputs:
%  polar_hist - 
%  sector_resolution - 
%  sensor - 
% 
% Outputs:
%  trap_escape_angle - 
% 
% Usage:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

chooseEscBasedOnGoalDirection = 0;

% NTU
% For Capacitops/Warrior
% left_th = [90 45] - sector_resolution/2;
% right_th = [-90 -45] + sector_resolution/2;

% For Thor/Thor Pro
left_th = [135 45] - sector_resolution/2;
right_th = [-135 -45] + sector_resolution/2;

left_sect = angleToSector(left_th, sector_resolution, sensor);
right_sect = angleToSector(right_th, sector_resolution, sensor);

obstaclesL = sum(polar_hist(left_sect(1):-1:left_sect(2)));
obstaclesR = sum(polar_hist(right_sect(1):1:right_sect(2)));

% min(map) might need to be flipped for thor pro
if (obstaclesL >= obstaclesR)
  trap_escape_angle = -135;     % turn right
else
  trap_escape_angle = +135;   % turn left
end

if (chooseEscBasedOnGoalDirection)
  if (goalAngle > 0)  % FIX - maybe use elseif or >= or <=
    trap_escape_angle = 90;
  end
  if (goalAngle < 0)
    trap_escape_angle = -90;
  end
end