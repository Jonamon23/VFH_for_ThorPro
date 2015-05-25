function [linear, angular] = ...
  determineSpeed(H, picked_sector, current_velocity, Hmax,...
                 sector_max, sensor_min_angle, sector_resolution, MAX)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This function Calculates Linear and Angular Velocities of the robot
% 
% Updated By: Jonathon Kreska
% Version Date: May 25, 2015
% Version: 1.1
% 
% Changelog:
%  1.0: Initial Release. Added Header
%  1.1: Changed Vmin and Vmax to global and removed from function call
% 
% Inputs:
%  H - 
%  picked_sector - 
%  current_velocity - 
%  Hmax - 
%  Vmax - 
%  Vmin - 
%  sector_max - 
%  sensor_min_angle - 
%  sector_resolution -
%  MAX -
%
% Outputs:
%  linear - 
%  angular - 
% 
% Usage:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Vmin Vmax;

MaxTurnRate = MAX.TURNRATE_0MPH - (abs(current_velocity))* ...
                (MAX.TURNRATE_0MPH - MAX.TURNRATE_5MPH)/5;
            % Calculate Maxturnrate at the current speed
            %  This code is borrowed from the Player/Stage project

if ( MaxTurnRate < 0 ) % Prevent negative max turning speed
  MaxTurnRate = 0;
end

angular = ((sectorToAngle(picked_sector,sensor_min_angle, sector_resolution)) / MAX.ACCESS_ANGLE) * ...
           MaxTurnRate;

if(abs(angular) > MaxTurnRate)
  angular = sign(angular) * MaxTurnRate;
end

max_linear = MAX.TURNRATE_5MPH ./ abs(angular) * 2.23;
                % 2.23 mps in mph

if (~isreal(max_linear))
  max_linear = Vmax;
end

t = round(picked_sector);
t = max(min(t, sector_max), 1);

tcenter = sector_max/2;
if (t > tcenter)
  d = +1;
  tlong = min(t+1, sector_max);
else
  d = -1;
  tlong = max(t-1, 1);
end

hc = max( H(tcenter:d:tlong) );

%hc = H(t);
hc = min( hc, Hmax); % this implement eq-9 page 16. where hc stands for hc''

% If there are no obstacles in front of the vehicle then go maximum speed 
% otherwise apply equation 8 in page 16 to adjust speed according to
% obstacle density.

if (Hmax == 0)
  linear = Vmax;
else
  linear = Vmax*(1-(hc/Hmax));% equation 8 in page 16
end

linear = min(max_linear, linear);

if (abs(linear) < .1) % MAG
  if(abs(angular) < 1) % MAG
    linear = Vmin;
  elseif(abs(angular) > MAX.TURNRATE_0MPH_THORARM)
    angular = sign(angular) * MAX.TURNRATE_0MPH_THORARM;
  end
end