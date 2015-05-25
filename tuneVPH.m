%function tuneVPH
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This function initializes all the Tunable Global Variables for use in VPH
% 
% Updated By: Jonathon Kreska
% Version Date: May 25, 2015
% Version: 1.2
% 
% Changelog:
%  1.0: Initial Release. 
%  1.1: Added Header. Added Hmax. 
%  1.2: Formatting. Removed NTU parameters. Sector is a struct.
% 
% Inputs:
%  N/A
% 
% Outputs:
%  N/A
% 
% Usage:
%  tuneVFH
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Max angles and Turnrates
global MAX; MAX = struct;
MAX.TURNRATE_0MPH = 40;  %  Default Maxturnrate(deg/sec) at 0m/s  %80
MAX.TURNRATE_5MPH = 30;  %  Default Maxturnrate(deg/sec) at 2.23m/s  %60
MAX.TURNRATE_0MPH_THORARM = 60; %                                                                       
MAX.ACCESS_ANGLE = 85.0; %  If we chose this angle, we will go MaxTurnRate to get there.

%  MAX.TURNRATE_0MPH........... Max turnrate in deg/sec at 0m/s
%  MAX.TURNRATE_5MPH........... Max turnrate in deg/sec at 2.23m/s
%  MAX.TURNRATE_0MPH_THORARM... ???
%  MAX.ACCESS_ANGLE............ Angle at which the robot will use MAX_TURNRAT

%% Lidar
global lidar; lidar = struct;

lidar.threshold = 8.1;   % Consider objects > threshold meter distance negligable
  % 10 for Stage
  % 8.1 for Thor Pro for now...
% lidar.ranges = 8.2*ones(360,1); % include for broken 0.5 degree lidar
lidar.ranges = 25*ones(541,1); % initialize it this way for 0.25 degree lidar
  % Allows check to see if new data has been added
lidar.min_angle = -135; % Explicitly define parameters to reduce callback time
lidar.max_angle = 135;
lidar.resolution = 0.5;

%% Odom and Goal
global curr goal; curr = struct; goal = struct;
curr.x = []; curr.y = []; curr.theta = [];
goal.x = []; goal.y = [];

%% Lanes
global lanes; lanes = struct;
lanes.ranges = [];
lanes.min_angle = -90; % Explicitly define parameters to reduce callback time
lanes.max_angle = 90;
lanes.resolution = 0.5;

%% Sector
global sector; sector = struct;
sector.alpha = 3;      % how many degrees per sector

%% VFH Parameters
global Vmax Vmin robot_radius d_s r_rs Hmax;
% global robot_radius_0mph robot_radius_5mph L HScale threshold_high threshold_low 

Vmax = 1.5; % maximum vehicle speed m/s
Vmin = .5; %  m/s minimum speed with 0 angular velocity
           % (angular velocity with no linear velocity is allowed as well)

% NTU           
% robot_radius_0mph  = 0.45; % 0.19;    % Best robot radius at 0mph
% robot_radius_5mph = 0.25; % 0.19;     % Best robot radius at 5mph

robot_radius = 0.35; % 0.35 for Thor Pro % 0.4 for Cerberus
d_s = 0.1; % Minimum distance between the robot and obstacle (Safety buffer)
r_rs = robot_radius + d_s; % Enlarged safe radius

% NTU
% threshold_high = 250; % Put robot 1.5m from a barrel, set this value to max(m)
% % 170 works usually
% threshold_low = threshold_high*.88;  % This works...

Hmax = 2*250; %MAJ
% Used as a speed reducer when calculating the speed.
% Hmax in the reference paper page 16 eq 8-9 is said to be determined 
% imperically to cause sufficient reduction in speed, we simply chose it to
% be equal to double the maximum value of the histogram.

%% Initialize VFH State
global vfh_state; vfh_state = struct;

vfh_state.trap_escape_angle = [];
vfh_state.linear_velocity = 0; % meters/second
vfh_state.angular_velocity = 0; % degrees/second
vfh_state.prefer_narrow = 0;
vfh_state.T_high = 0;
vfh_state.T_low = 0;