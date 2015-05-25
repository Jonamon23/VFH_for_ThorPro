function VPH2015Algorithm(lidar_ranges, goal_angle) %, antigoal_angle)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This function is an implementation of VFH+ for use on Thor Pro.
% 
% Updated By: Jonathon Kreska
% Version Date: May 25, 2015
% Version: 1.4
% 
% Changelog:
%  1.0: Initial Release
%  1.1: Added Comments that demonstrate full understanding.
%       Defined inital global variables with one line. Saved ~.001sec in initial 
%        declaration and determineSpeed.
%  1.2: Moved many Tunable parameters to separate file.
%       Changed many function calls to pass value of global variables.
%       Passing by value prevents functions from changing the value and is 
%        argued that is faster than re-initializing a global variable.
%  1.3: Moved small functions inside this code
%  1.4: Added adaptive thresholding. Removed Remnants of Failed code attempts. 
%       Consolidated Debugging into vfh_state. Set anti-goal to 0.
% 
% Inputs:
%  map - lidar scan
%  goal_angle - angle of target from current robot heading
%  goal_dist - distance of target from current robot postion
%  antigoal_angle - tuning parameter?
%  antigoal_dist -  tuning parameter?
%  vfh_state - a bunch of saved parameters that allow the robot to determine if
%   it's inside of and escape a trap. Can be used for debugging each time VFH is 
%   run and can be used for future improvement. Knowledge of the previous step 
%   may be beneficial to the current.
% 
% Outputs:
%  linear_velocity - forward velocity in meters per second
%  angular_velocity - how much to rotate in ??? per second
%  vfh_state - see Inputs. Carries current state over to next iteration.
% 
% Usage:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load in Tuning Variables and Constants
global vfh_state lidar sector MAX Hmax;
% See 'tuneVPH.m' for variable descriptions 

%% Calculate the Maximum Number of Sectors
% This is based on lidar scan and defined elements per sector    

sector.count = round((lidar.max_angle - lidar.min_angle)/sector.alpha);

% % rscale = sector.alpha/lidar.resolution; % ???
% angleList = sectorToAngle(1:sector.count, lidar.min_angle, sector.alpha); 
%   % Array of Angles of all sectors 

% NTU - Old code to convert sector to angle; now inside function: sectorToAngle
% sectorToAngle(x) = x * sector.alpha + min_angle - sector.alpha/2
% angleToSector(x) = round((x - min_angle)/sector.alpha)

%% Identify Goal Sector
% Represent the goal direction in terms of an alpha degree quantized polar view, referred to as 
% the polar obstacle density (see equation 4, page 11 of VFH paper).
goal_sector = angleToSectorFloat(goal_angle, lidar.min_angle, sector.alpha);
% antigoal_sector = angleToSectorFloat(antigoal_angle, lidar.min_angle, sector.alpha);

%% Obtain Previous VFH State values and parameters

if (isfield(vfh_state, 'bin_hist'))
  bin_hist = vfh_state.bin_hist;  % gets previous H_b See VFH+ Equ 7
else  
  bin_hist = true(1, sector.count);
end

trap_escape_angle = vfh_state.trap_escape_angle;

if (isfield(vfh_state, 'picked_angle'))
  last_sector = angleToSector(vfh_state.picked_angle,sector.alpha,lidar);
else
  last_sector = 0;
end

prefer_narrow = vfh_state.prefer_narrow;

trap_speed = []; % clear trap speed value

linear_velocity = vfh_state.linear_velocity;
angular_velocity = vfh_state.angular_velocity;

% T_high = vfh_state.T_high;  % Get previous thresholds
% T_low = vfh_state.T_low;

%% Calculate Robot Radius for current iteration
% robot_radius = robot_radius_0mph + linear_velocity/2.23 * ...
%                (robot_radius_5mph - robot_radius_0mph);
  % Dynamically Calculates Robot Radius based on previous Linear Velocity
%  robot_radius = robot_radius_5mph - linear_velocity/2.23 * ...
%               (robot_radius_5mph - robot_radius_0mph);

%robot_radius = min(robot_radius_0mph, min(robot_radius_5mph, robot_radius));
%   % Finds whether the dynamic radius is less than high speed radius, then takes the 
%   %  smallest and compares it to the large stationary radius

% assignin('base','robot_radius',robot_radius); % DBG

%% Calculate Vector Magnitude
% lidar_ranges is the same as h_p in VFH+

d = lidar_ranges;
w_s = lidar.threshold*2;

b = 4/(2*w_s-1);
a = 1 + b*((w_s-1)/2)^2;

m = a - b*(d.^2); % certainty is 1 because we have a lidar
m(m<0) = 0; % Prevents negative numbers

%% Calculate Sector Polar Obstacle Density Histogram
H_p_k = calcSectorPolarObstacleDensity(lidar_ranges,m,sector);
% in this section we emphasize the edges of the polar histogram by increasing
% the amplitude for the last 15 degrees = 5 sectors on the left and right.
% the amplitude weight is higher for elements closer to the edge compared to those towards
% center
H_p_k(1:5) = H_p_k(1:5).*[2 1.8 1.6 1.4 1.2];
H_p_k(end:-1:end-4) = H_p_k(end:-1:end-4).*[2 1.8 1.6 1.4 1.2];

%% Jon's Hueristics for Dynamic Thresholding
% a = mean(H_p_k);
mi = min(H_p_k);
ma = max(H_p_k);
diff = ma-mi;

% disp(' ')
% disp(a)
% disp(mi)
% disp(ma)
% disp(diff)


% closest_dist = 330;
% 
% if a < closest_dist % VFH+ can work great
%   if a < 70
%     T_high = 70;
%     T_low  = 70*.88;
%   elseif ( a>closest_dist && s<=140)
%     T_high = closest_dist;
%     T_low  = closest_dist*.88;
%   elseif ( a>closest_dist && s>140)
%     T_high = min(closest_dist+50,a);
%     T_low  = min(closest_dist+50,a)*.88; 
%   else
%     T_high = a;
%     T_low  = a*.88;
%   end
%   
%   % Apply threshold hysteresis
%   bin_hist(H_p_k >= T_high) = true;   
%   bin_hist(H_p_k <= T_low) = false;   % This matches VFH+ Equ 7
%   % 0 = free    1 = Blocked
%   % Otherwise, Previous value is obtained from vfh_state loading at the beginning

% elseif ( a > closest_dist && min(H_p_k) < 30 )
% %   if (length(find(H_p_k < 20)) > 2 )
%     T_high = 200;
%     T_low  = 200*.88;
% %   end
%   
%   % Apply threshold hysteresis
%   bin_hist(H_p_k >= T_high) = true;   
%   bin_hist(H_p_k <= T_low) = false;   % This matches VFH+ Equ 7
%   % 0 = free    1 = Blocked
%   % Otherwise, Previous value is obtained from vfh_state loading at the beginning
%   
% elseif ((min(H_p_k) < closest_dist) && (s > 180))
%   T_high = closest_dist;
%   T_low  = closest_dist*.88;
%   
%   % Apply threshold hysteresis
%   bin_hist(H_p_k >= T_high) = true;   
%   bin_hist(H_p_k <= T_low) = false;   % This matches VFH+ Equ 7
%   % 0 = free    1 = Blocked
%   % Otherwise, Previous value is obtained from vfh_state loading at the beginning
% else
  if mi == 0
    if ma > 0 %min is 0 and max > 0
      T_low = 0.1*ma;
      T_high = 0.08*ma;
    else
      T_low = 1; % open field
      T_high = 1;
    end
  else % min has a value
    if diff > mi
      T_low = mi+.05*diff;
      T_high = mi+.1*diff; 
    else
      T_low = 1.05*mi;
      T_high = 1.1*mi;
    end
  end

  if T_high < 500
%     if ( a > closest_dist && min(H_p_k) < 30 )
% %     if (length(find(H_p_k < 20)) > 10 )
%       T_high = 200;
%       T_low  = 200*.88;

%     if ma < 500
%       T_high = 250;
%       T_low = 200;
%     else
%       T_high = 500;
%       T_low = 450;
%     end

    T_high = max(T_high,250);
    T_low = max(T_high,200);
    
  end


% Apply threshold hysteresis
bin_hist(H_p_k >= T_high) = true;   
bin_hist(H_p_k <= T_low) = false;   % This matches VFH+ Equ 7
% 0 = free    1 = Blocked
% Otherwise, Previous value is obtained from vfh_state loading at the beginning

% end
  
%   
% else % Thresholds get overloaded and Ballistic Navigation is Necessary
%   This allows the robot to nudge itself out of an overwhelming environment
%   until it sees a valley again. This addition is necessary to keep the max
%   threshold low enough so during normal operation, it doesn't hit a close
%   obstacle. Statistical analysis and trials found the highest threshold for
%   this.
%   disp('Ballistic Mode Engaged!')
%   H_ball = sum(reshape(m,sector.alpha/lidar.resolution,sector.count)); % sum danger values into sectors
%   bin_hist(H_ball > min(H_ball)) = true; % block all areas greater than the minimum
%   bin_hist(H_ball <= min(H_ball)+2) = false; %overwrite old posibilities
%   
% end

% try
%   valleys = [];
%   H_ball = sum(reshape(m,sector.alpha/lidar.resolution,sector.count)); % sum danger values into sectors
%   bin_hist=ones(size(H_ball));
%   i=1;
%   width=1;
%   disp('1')
%   while (width<15)  % If valleys are available...
%     bin_hist(H_ball < min(H_ball)+i) = false; 
%     i=i+1;
%     valleys = findValleys(bin_hist,sector.count);
%     if isempty(valleys)
%       width = 0;
%     else
%       width = max(valleys(:,3));
%     end
%       
%   end
%   disp(width)
% catch ME
%   disp(ME)
%   error('Here')
% end

% % OPT - Should be replaced with true vehicle dynamcis.
% if (angular_velocity > 10) % MAG
%   bin_hist(1:2) = false;
% elseif(angular_velocity < -10) % MAG
%   bin_hist(sector.count-1:sector.count) = false;
% end

%% Find Candidate Valleys
valleys = findValleys(bin_hist,sector.count);

if (~isempty(valleys))  % If valleys are available...
  picked_sector = ...
    pickCandidateSector(valleys, goal_sector, 0, last_sector,...
                        linear_velocity, angular_velocity, prefer_narrow,...
                        lidar.min_angle, sector.alpha, MAX);
  %   [~,index] = max(valleys(:,3));
  %   picked_sector = (valleys(index,2)-valleys(index,1))/2;
  picked_angle = sectorToAngle(double(picked_sector), lidar.min_angle, sector.alpha);

else  % There are no valleys... We are trapped
  % OPT - there are some traps, like the obstacle grouping trap deal that 
  % vfh still gets stuck in
  if (isempty(trap_escape_angle)) % If no trap escape angle
    trap_speed = 0;
    trap_escape_angle = trapDetermineEscape(H_p_k, sector.alpha, lidar);
    % Determine a direction
  end
  % Else we've been trapped for a while now, continue in the initial
  % direction picked.

  picked_angle = trap_escape_angle;
  picked_sector = angleToSectorFloat(trap_escape_angle, lidar.min_angle, sector.alpha);
  if (trap_escape_angle > 0) % turning right
    % Check left obstacles
    if(min(lidar_ranges(1:length(lidar_ranges)/2)) < .7) % MAG -> Must be smallest and middle element
      trap_speed = -.2; % MAG
    else
      trap_speed = 0;
    end

  else
    % Check right obstacles
    if (min(lidar_ranges(length(lidar_ranges)/2:length(lidar_ranges))) < .7) % MAG -> Must be middle and largest element
      trap_speed = -.2; % MAG
    else
      trap_speed = 0;
    end
  end
end

%% Determine Speed
[linear_velocity, angular_velocity] = ...
  determineSpeed(H_p_k, picked_sector, linear_velocity, Hmax, sector.count, ...
                 lidar.min_angle, sector.alpha, MAX);

% NTU - Slow down when the vehicle gets close to bread crumbs/waypoints
%   if (goal_dist < 1.5)
%     linear_velocity = linear_velocity/2;
%   end

if (~isempty(trap_speed))
  linear_velocity = trap_speed;
  % NTU - Disable trap escapes for testing right now.
  %    linear_velocity = 0;
  %    angular_velocity = 0;
  %    picked_angle = 0;
end

if (~isempty(valleys) && linear_velocity > 0.5) % MAG
  % If we've found a big enough valley to drive this fast, then
  % consider any trap we were in defeated.
  trap_escape_angle = [];
end

%% Return the current state for next session and debugging information
% Needed for next loop
vfh_state.linear_velocity = linear_velocity;
vfh_state.angular_velocity = angular_velocity;

vfh_state.bin_hist = bin_hist;

vfh_state.prefer_narrow = prefer_narrow;

vfh_state.picked_angle = picked_angle;
vfh_state.trap_escape_angle = trap_escape_angle;

%% Debug
vfh_state.m = m;
vfh_state.H_p_k = H_p_k;
vfh_state.T_high = T_high;
vfh_state.T_low = T_low;
vfh_state.picked_sector = picked_sector;

% vfh_state.valleys = valleys;