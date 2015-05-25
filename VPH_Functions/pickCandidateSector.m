function picked_sector = ...
  pickCandidateSector(valleys, goal_sector, anti_sector, last_sector,...
                      linear_velocity, angular_velocity, prefer_narrow,...
                      sensor_min_angle, sector_resolution, MAX)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This function searches the valleys for all candidate directions, then 
%   determines the "cost" of each candidate. The candidate with the lowest cost 
%   is chosen.
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
%  anti_sector - 
%  last_sector
%  linear_velocity - 
%  angular_velocity - 
%  prefer_narrow - 
%  sensor_min_angle -
%  sector_resolution - 
%  MAX - 
% 
% Outputs:
%  picked_sector - 
% 
% Usage:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

center_sector = angleToSectorFloat(0, sensor_min_angle, sector_resolution); % Used elsewhere.

% NTU
% left_sector = angleToSectorFloat(135);  % Used elsewhere.

% Cost weights. Note the magnitude does not matter, only the ratio.

goal_weight = 10; % Cost of driving away from the goal.
approach_weight = 1;
    % Cost of driving away from where the angular velocity is already sending you 
last_weight = 1;
    % Cost of disagreeing with the previous cycle's decision
anti_weight = 0;
%anti_weight = -12;
    % Cost of driving AWAY from the anti-goal. Note this is a negative
    % cost, that is driving away from the anti-goal decreases your
    % cost, and driving towards the anti-goal increases your cost.

if (prefer_narrow)
  wide_weight = 7;
else
  wide_weight = 0;
end

wide_valley = 100; % TUN - Wide valleys are 6 sectors large

% Valley index definitions
START = 1; END = 2; WIDTH = 3; TOGOAL_S = 4; TOGOAL_E = 5; WIDE = 6;

valleys(:, TOGOAL_S) = goal_sector - valleys(:,START);
valleys(:, TOGOAL_E)= valleys(:,END) - goal_sector;
valleys(:, WIDE) = valleys(:, WIDTH) >= wide_valley;

% Identify all "narrow" valleys
narrow_valleys = valleys(~logical(valleys(:, WIDE)), 1:2);

% The remaining valleys are "wide"
wide_valleys = valleys(logical(valleys(:, WIDE)), 1:3);

SECTOR = 1;     WIDE = 2;   TOGOAL = 3; TOANTI = 4;
TOAPPROACH = 5; TOLAST = 6; COST = 7;

% Narrow valley's supply one candidate, down the center.
candidates(1:length(narrow_valleys), SECTOR) = ...
  narrow_valleys(:, START) + (narrow_valleys(:, END) ...
  - narrow_valleys(:, START))/2;

candidates(1:length(narrow_valleys), WIDE) = 0;

lc = size(candidates, 1)+1;
lw = size(wide_valleys, 1)-1;

% Wide valleys supply at least two candidates. The left side of it and
% the right side of it.
candidates(lc:lc+lw, SECTOR) = wide_valleys(:, START) + wide_valley/2;
candidates(lc:lc+lw, WIDE) = 1;

lc = size(candidates, 1)+1;
candidates(lc:lc+lw, SECTOR) = wide_valleys(:, END) - wide_valley/2;
candidates(lc:lc+lw, WIDE) = 1;

% lc = size(candidates, 1)+1;

% Wide valleys can also supply a candidate in the goal direction, if
% the valley includes the goal.
% goal_valley = valleys(  valleys(:, TOGOAL_S) > 0 & valleys(:, TOGOAL_E) > 0 );
% if(~isempty(goal_valley))
%   candidates(lc, [SECTOR, WIDE]) = [goal_sector, 1]; % The goal itself
% end

% Finally wide valley's can supply the forward (0�) candidate, if that
% is included in the valley.

lc = size(candidates, 1)+1;
center_valley = ...
  valleys( valleys(:, START) + wide_valley/2 < center_sector & ...
    valleys(:, END) -wide_valley/2 > center_sector );
if(~isempty(center_valley))
  candidates(lc, [SECTOR, WIDE]) = [center_valley, 1];
end

n = 360/sector_resolution; % 72 sectors

c = candidates(:, SECTOR);

% sector distance between candidate and goal
candidates(:, TOGOAL) = min ( [abs(c - goal_sector), ...
                               abs(c - goal_sector - n), ...
                               abs(c - goal_sector + n)], [], 2);

candidates(:, TOANTI) = min ( [abs(c - anti_sector), ...
                               abs(c - anti_sector - n), ...
                               abs(c - anti_sector + n)], [], 2);

% sector distance between candidate and sector we are currently
% approaching (based on angular velocity)

% First estimate the sector we are approaching

MaxTurnRate = MAX.TURNRATE_0MPH - (abs(linear_velocity))* ...
                (MAX.TURNRATE_0MPH - MAX.TURNRATE_5MPH)/5;
            % Calculate Maxturnrate at the current speed
            % This code borrowed from the Player/Stage project

if ( MaxTurnRate < 0 )
  MaxTurnRate = 1;
end

approach_sector = ...
  angleToSectorFloat((angular_velocity / MaxTurnRate * MAX.ACCESS_ANGLE),...
                     sensor_min_angle, sector_resolution);

if(approach_sector < 1)
  approach_sector = 1;
elseif(approach_sector > (3*n)/4)
  approach_sector = (3*n)/4;
end

candidates(:, TOAPPROACH) = abs(c - approach_sector);

% sector distance between candidate and sector we chose last time
candidates(:, TOLAST) = abs(c - last_sector);

% Compute the costs
candidates(:, COST) = goal_weight * candidates(:, TOGOAL) + ...
                      anti_weight * candidates(:, TOANTI) + ...
                      approach_weight * candidates(:, TOAPPROACH) + ...
                      last_weight * candidates(:, TOLAST) + ...
                      wide_weight * candidates(:, WIDE);

% assignin('base','candidates',candidates)

[~, index] = min(candidates(:, COST));
picked_sector = candidates(index, SECTOR);