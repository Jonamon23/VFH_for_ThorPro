%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This script tests the publishing and subscribing of MATLAB created VFH 
%   nodes connected to ROS core running a stage world. 
%   Goals are obtained from D*.
% 
% Author: Jonathon Kreska
% Version Date: May 25, 2015
% Version: 1.4
% 
% Changelog:
%  1.0: Initial Release
%  1.1: Moved Callbacks to Separate files and folder. 
%  1.2: Moved Tuning Parameters to different function that is called my this one
%  1.3: Added more debugging tools
%  1.4: Sent lidar ranges as parameter instead of a global variable.
%       Removed more anti-goal stuff. Simplified addpath calls. 
%       Moved tic/toc to better spot.
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
rosshutdown; clear all; close all; clc;

% addpath('ROS_Callback_Functions');
addpath('VPH_Functions');

% Running Parameters
using_Thor_Pro = true;
using_Breadcrumbs = false;
using_Localization = false;
using_Image_Processing = false;
Debugging = false;

% Set IP address of the host where ROS master is running (roscore)
rosinit('192.168.0.102','NodeHost','192.168.0.101','NodeName','/VFH')

% Initialize State and value variables
tuneVPH;
global vfh_state;

% anti_goal.x=30; anti_goal.y=-30; % leave this until I remove anti-goal stuff

disp('Parameters Loaded!')

%% SUBSCRIBER SETUP

% LIDAR
% Setup Subscriber Node to look for lidar scans
global lidar;

lidar_sub=rossubscriber('/scan',rostype.sensor_msgs_LaserScan);%,@getLidar);

disp('Lidar Callback Ready')

% LANES
% Setup Subscriber Node to look for IP's Lane Line lidar scans
global lanes;
if using_Image_Processing
  lanes_sub=rossubscriber('/matlab_base_scan',rostype.sensor_msgs_LaserScan);
  disp('Lanes Callback Ready')
end

% GOAL
% Setup Subscriber Node to look for Goal Messages
global goal;
if using_Breadcrumbs
  goal_sub = rossubscriber('/BC',rostype.geometry_msgs_Point);
  disp('Goal Callback Ready')
else
  goal.x = 1000; goal.y = 0; % Make a goal 100m in front of robot
end

% ODOM
% Setup Subscriber Node to look for Odometry Messages
global curr;
if (using_Thor_Pro && ~using_Localization) % No Localization
  odom_sub=rossubscriber('/ThorPro_Drivetrain/odom',rostype.nav_msgs_Odometry);
elseif (using_Thor_Pro && using_Localization) % With Localization
  odom_sub=rossubscriber('/odom_combine',rostype.nav_msgs_Odometry);
else % You are using Stage
  odom_sub=rossubscriber('/odom',rostype.nav_msgs_Odometry);
end
disp('Odom Callback Ready')

%% PUBLISHER SETUP
% CMD_VEL
% Setup Publisher Node as Topic as cmd_vel controlling movement
% global cmd_vel_publisher
if using_Thor_Pro
  cmd_vel_publisher=rospublisher('/ThorPro_Drivetrain/cmd_vel','geometry_msgs/Twist');
else % You are using Stage
  cmd_vel_publisher=rospublisher('/cmd_vel','geometry_msgs/Twist');
end
msg = rosmessage(cmd_vel_publisher);
disp('Cmd_vel Ready!')  

%% TIMER
% t = timer('TimerFcn',{@publishSpeed},...
%           'startDelay',1,...              % wait a second before publishing...
%           'ExecutionMode','fixedRate',... % at a fixed rate...
%           'Period', .50);                % 70msec
% start(t);
% disp('Timer Ready!')

%% Main Loop

while 1
  tic

  lidar.ranges = lidar_sub.LatestMessage.Ranges; % Read received data and store to ranges
  lidar.ranges(lidar.ranges<0.01) = lidar.threshold;
  lidar.ranges = flipud(lidar.ranges(1:540));
  lidar.ranges(lidar.ranges>3)=lidar.threshold;

  
%   try 
%     if lidar.ranges(1)==25 % test to see if lidar is available
%       lidar.ranges(542); % Create unique Error
%     end
%     
    odom_msg = odom_sub.LatestMessage;
    qx=odom_msg.Pose.Pose.Orientation.X;
    qy=odom_msg.Pose.Pose.Orientation.Y;
    qz=odom_msg.Pose.Pose.Orientation.Z;
    qw=odom_msg.Pose.Pose.Orientation.W;
    axang=quat2axang([qw qx qy qz]);

    curr.x=odom_msg.Pose.Pose.Position.X;
    curr.y=odom_msg.Pose.Pose.Position.Y;
    curr.theta=axang(4);
  
%     try
%       odom_sub.LatestMessage;
%       curr.x(1);  % test to see if Odom is available
%       
%       try
%         goal.x(1);  % test to see if goals are available    
%                
%         try

    if using_Breadcrumbs
        goal_msg = odom_sub.LatestMessage;
        goal.x =goal_msg.X;
        goal.y =goal_msg.Y;
    end
        

          if using_Image_Processing
            lanes.ranges = lanes_sub.LatestMessage.Ranges(1:540);
            % Mirror lanes 45 degrees behind the robot
            lanes.ranges(90:-1:1)=lanes.ranges(91:1:180);
            lanes.ranges(540:-1:451)=lanes.ranges(361:1:450);
            
            lanes.ranges(1); % test to see if IP is available 
          else 
            lanes.ranges = lidar.ranges(1:540); % copy for code equality and simplicity
          end   
          
          lidar.combined = min(lidar.ranges,lanes.ranges); 
%           lidar.combined(lidar.combined>3) = 3; 

          if (curr.theta > pi)
            curr.theta = curr.theta - 2*pi;
          end

          phi=atan2(goal.y - curr.y,goal.x - curr.x);
          goal_heading = limitAngleRad(phi-curr.theta)*180/pi;
          % goal_dist = sqrt((goal.y - curr.y)^2+(goal.x - curr.x)^2);

          % anti_phi=atan2(curr.y-anti_goal.y, curr.x-anti_goal.x);
          % anti_goal_heading = limitAngleRad(anti_phi-curr.theta)*180/pi;
          % anti_goal_dist = sqrt((anti_goal.y - curr.y)^2+(anti_goal.x - curr.x)^2);
          
          lidar_ranges = lidar.combined'; % lock in ranges for this loop

          VPH2015Algorithm(lidar_ranges, goal_heading);%, anti_goal_heading);
          
             if(sqrt((goal.y - curr.y)^2+(goal.x - curr.x)^2) > .3) % Until you reach the goal...

                  % Extract Lin and Ang Vel
                  linear_velocity = vfh_state.linear_velocity;
                  angular_velocity = vfh_state.angular_velocity;

                  angular_velocity_rad = angular_velocity *pi/180;

                  if (linear_velocity == 0)
                    linear_velocity = .02;
                  end

                % elseif(goal.x == [] || goal.y == []) % Not working for empty goal FIX
                %   disp('Waiting for Replan');
                %   linear_velocity = 0;
                %   angular_velocity_rad = 0;

             else

                  disp('Waiting for new Goal');
                  linear_velocity = 0;
                  angular_velocity_rad = 0;

             end

            % Set linear and Angular Velocity and define ROS Message Types



            msg.Linear.X = linear_velocity;
            msg.Angular.Z = angular_velocity_rad;
            send(cmd_vel_publisher,msg)
            
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
            
          end % END DEBUGGING
          
                  
%         catch ME
%           if(strcmp(ME.identifier,'MATLAB:badsubscript'))
%             warning('No Lanes')
%           else
%             rethrow(ME)
%           end   
%         end
%           
%       catch ME
%         if(strcmp(ME.identifier,'MATLAB:badsubscript'))
%           warning('No Goals')
%         else
%           rethrow(ME)
%         end      
%       end
%       
%     catch ME
%       if(strcmp(ME.identifier,'MATLAB:badsubscript'))
%         warning('No Odom')
%       else
%         rethrow(ME)
%       end      
%     end
%     
%   catch ME
%     if(strcmp(ME.identifier,'MATLAB:badsubscript'))
%       warning('No Lidar')
%     else
%       rethrow(ME)
%     end
%   end
%   
toc
% profile viewer
  
end % END WHILE 1
%END OF LINE