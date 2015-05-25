%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This script tests the publishing and subscribing of MATLAB created VFH 
%   nodes connected to ROS core running a stage world. 
%   Goals are obtained from D*.
% 
% Author: Jonathon Kreska
% Version Date: Mar 17, 2015
% Version: 1.3
% 
% Changelog:
%  1.0: Initial Release
%  1.1: Moved Callbacks to Separate files and folder. 
%  1.2: Moved Tuning Parameters to different function that is called my this one
%  1.3: Added more debugging tools
% 
% Requirements: 
%  This machine: 
%   MATLAB ROS Support with supported MATLAB Version
%    http://www.mathworks.com/hardware-support/robot-operating-system.html
% 
%  ROS Host:
%   ROS Hydro (tested)
%   UDM IGVC stagerosworld from https://github.com/udmamrl/stageroscam.git
%    $ git clone https://github.com/udmamrl/stageroscam.git
%   Run: roslaunch stageroscam/IGVC2013_AutoNavBasic.launch (for example)
% 
% Usage:
%  Change IPs to yours
%  >> clear; close all
%  Click Run |>
%  
%  To Stop:
%  Ctrl+C
%  >clear all
% 
% Shortfalls:
%  Uses global variable to store callback data.
%  Only Plots data every time data is published, not every callback call
%  Needs code to cleanup node after stopping
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup
clear all; close all; clc;

% profile ON

addpath('ROS_Callback_Functions')
addpath('VPH_Functions')

% Running Parameters
using_Thor_Pro = true;
using_Breadcrumbs = false;
using_Localization = false;
using_Image_Processing = true;
Debugging = true;

% Set IP address of the host where ROS master is running (roscore)
rosMasterIp = '192.168.0.102';

% Set IP address of the local IP
localhostIp = '192.168.0.49'; 

% Initialize State and value variables
tuneVPH;

global vfh_state;

anti_goal.x=30; anti_goal.y=-30; % leave this until I remove anti-goal stuff

disp('Parameters Loaded!')

%% SUBSCRIBER SETUP
% Create Subscriber Node
global node;
node=rosmatlab.node('VFH',rosMasterIp,11311,'rosIP',localhostIp);

% LIDAR
% Setup Subscriber Node to look for lidar scans
global lidar;
if using_Thor_Pro
  lidar_sub=rosmatlab.subscriber('/scan','sensor_msgs/LaserScan',1,node);
  % Define Callback for Lidar
  lidar_sub.setOnNewMessageListeners({@getLidar});
else % Stage
  lidar_sub=rosmatlab.subscriber('/base_scan','sensor_msgs/LaserScan',1,node);
  % Define Callback for Lidar
  lidar_sub.setOnNewMessageListeners({@getStageLidar});
end
disp('Lidar Callback Ready')

% LANES
% Setup Subscriber Node to look for IP's Lane Line lidar scans
global lanes;
if using_Image_Processing
  lanes_sub=rosmatlab.subscriber('/matlab_base_scan','sensor_msgs/LaserScan',1,node);
  lanes_sub.setOnNewMessageListeners({@getLanes});   %Define Callback for Lanes
  disp('Lanes Callback Ready')
end

% GOAL
% Setup Subscriber Node to look for Goal Messages
global goal;
if using_Breadcrumbs
  goal_sub = rosmatlab.subscriber('/BC','geometry_msgs/Point',1,node);
  goal_sub.setOnNewMessageListeners({@getGoal});   % Define Callback for Goal
  disp('Goal Callback Ready')
else
  goal.x = 1000; goal.y = 0; % Make a goal 100m in front of robot
end

% % Waypoint % Sundy's Deprecated GPS waypoint nav topic
% % Setup Subscriber Node to look for Goal Messages
% goal_sub = rosmatlab.subscriber('Goal','geometry_msgs/Point',1,node);
% % Define Callback for Goal
% goal_sub.setOnNewMessageListeners({@getGoal});
% disp('Goal Callback Ready')


% ODOM
% Setup Subscriber Node to look for Odometry Messages
global curr;
if (using_Thor_Pro && ~using_Localization) % No Localization
  odom_sub=rosmatlab.subscriber('/ThorPro_Drivetrain/odom','nav_msgs/Odometry',1,node);
elseif (using_Thor_Pro && using_Localization) % With Localization
  odom_sub=rosmatlab.subscriber('/odom_combine','nav_msgs/Odometry',1,node);
else % You are using Stage
  odom_sub=rosmatlab.subscriber('/odom','nav_msgs/Odometry',1,node);
end
odom_sub.setOnNewMessageListeners({@getOdom}); % Define Callback for Odom
disp('Odom Callback Ready')

%% PUBLISHER SETUPipc
% CMD_VEL
% Setup Publisher Node as Topic as cmd_vel controlling movement
global cmd_vel_publisher
if using_Thor_Pro
  cmd_vel_publisher=rosmatlab.publisher('/ThorPro_Drivetrain/cmd_vel','geometry_msgs/Twist',node);
else % You are using Stage
  cmd_vel_publisher=rosmatlab.publisher('/cmd_vel','geometry_msgs/Twist',node);
end
disp('Cmd_vel Ready!')  

%% TIMER
t = timer('TimerFcn',{@publishSpeed},...
          'startDelay',1,...              % wait a second before publishing...
          'ExecutionMode','fixedRate',... % at a fixed rate...
          'Period', .070);                % 70msec
start(t);
disp('Timer Ready!')

%% Main Loop

while 1
  
  try 
    if (lidar.ranges(1)==8.2); % test to see if lidar is available
      lidar.ranges(542); % Create Error
    end
    
    try
      curr.x(1);  % test to see if Odom is available
      
      try
        goal.x(1);  % test to see if goals are available    
               
        try
          if using_Image_Processing
            lanes.ranges(1); % test to see if IP is available 
          else 
            lanes.ranges = lidar.ranges; % copy for code equality and simplicity
          end            
          lidar.combined = min(lidar.ranges,lanes.ranges); 
%           lidar.combined(lidar.combined>3) = 3;
          
          % VFH LOOP  
          tic

          if (curr.theta > pi)
            curr.theta = curr.theta - 2*pi;
          end

          phi=atan2(goal.y - curr.y,goal.x - curr.x);
          goal_heading = limitAngleRad(phi-curr.theta)*180/pi;
          % goal_dist = sqrt((goal.y - curr.y)^2+(goal.x - curr.x)^2);

          anti_phi=atan2(curr.y-anti_goal.y, curr.x-anti_goal.x);
          anti_goal_heading = limitAngleRad(anti_phi-curr.theta)*180/pi;
          % anti_goal_dist = sqrt((anti_goal.y - curr.y)^2+(anti_goal.x - curr.x)^2);
          

          VPH2015Algorithm(goal_heading, anti_goal_heading);
          
            
          % DEBUG
          if Debugging
%             if using_Image_Processing
%               subplot(611)
%               plot(linspace(135,-135,length(lanes.ranges)),lanes.ranges);
%               axis([-136 136 -.1 8.2])
% 
%               subplot(7,1,2)
%               plot(linspace(135,-135,length(lidar.ranges)),lidar.ranges);
%               axis([-136 136 -.1 8.2])
%             end

            subplot(411)
            plot(linspace(135,-135,length(lidar.combined)),lidar.combined)
            axis([-136 136 -.1 8.2])
            
            subplot(412)
            plot(linspace(135,-135,length(vfh_state.m)),vfh_state.m);
            axis([-136 136 -.1 8.5])

            subplot(413)
            plot(linspace(135,-135,length(vfh_state.H_p_k)),vfh_state.H_p_k);
            axis([-136 136 -.1 1.2*max(vfh_state.H_p_k)])
            hold on
            plot(linspace(135,-135,length(vfh_state.H_p_k)),vfh_state.T_high*ones(1,length(vfh_state.H_p_k)),'r');
            plot(linspace(135,-135,length(vfh_state.H_p_k)),vfh_state.T_low*ones(1,length(vfh_state.H_p_k)),'r');
            stem(-1*vfh_state.picked_angle,1.05*max(vfh_state.H_p_k),'r');
            stem(-1*goal_heading,1.1*max(vfh_state.H_p_k),'g');
            hold off 

            subplot(414)
            plot(linspace(135,-135,length(vfh_state.bin_hist)),vfh_state.bin_hist);
            axis([-136 136 -.1 1.1]);

            drawnow

            toc


          end % END DEBUGGING
          %END VFH LOOP
                  
        catch ME
          if(strcmp(ME.identifier,'MATLAB:badsubscript'))
            warning('No Lanes')
          else
            rethrow(ME)
          end   
        end
          
      catch ME
        if(strcmp(ME.identifier,'MATLAB:badsubscript'))
          warning('No Goals')
        else
          rethrow(ME)
        end      
      end
      
    catch ME
      if(strcmp(ME.identifier,'MATLAB:badsubscript'))
        warning('No Odom')
      else
        rethrow(ME)
      end      
    end
    
  catch ME
    if(strcmp(ME.identifier,'MATLAB:badsubscript'))
      warning('No Lidar')
    else
      rethrow(ME)
    end
  end
  
% profile viewer
  
end % END WHILE 1
%END OF LINE