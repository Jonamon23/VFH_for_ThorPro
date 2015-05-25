% Jon's Test function for VFH
% Dumbs down a lot of stuff and combines multiple "run VFH" functions I found

addpath('C:\Users\Jon\Dropbox\SCHOOL\Local Nav (VFH)\VPH JON')
addpath('C:\Users\Jon\Dropbox\SCHOOL\Local Nav (VFH)\VPH JON\VPH_Functions')

global sensor_min_angle sensor_max_angle sensor_resolution;
global vfh_state;


tic

goal.x=-18.71;
goal.y=5.95;
anti_goal.x=30;
anti_goal.y=-30;
Vmax = 2.5; % maximum vehicle speed m/s

curr.x = 0;
curr.y = 0;

curr.theta = 0;

if curr.theta > pi
  curr.theta = curr.theta - 2*pi;
end

phi=atan2(goal.y - curr.y,goal.x - curr.x);
goal_heading = limitAngleRad(phi-curr.theta)*180/pi;
goal_dist = sqrt((goal.y - curr.y)^2+(goal.x - curr.x)^2);

% For simulation%
anti_phi=atan2(curr.y-anti_goal.y, curr.x-anti_goal.x);
anti_goal_heading = limitAngleRad(anti_phi-curr.theta)*180/pi;
anti_goal_dist = sqrt((anti_goal.y - curr.y)^2+(anti_goal.x - curr.x)^2);


if (goal_dist < .5)
  vfhStop = true; 
  continue;
end

sensor_min_angle = -135;
sensor_max_angle = 135;
sensor_resolution = .25;

%create fake lidar scan
lidar = 30*ones(1,1081); % initial lidar with maximum distance of 8 
lidar(300:360) = 7;     % obstacle with distance of 7
lidar(460:580) = 5;     % obstacle with distance of 5
lidar(840:920) = 6;     % obstacle with distance of 6

lidarRanges = lidar';
% lidarRanges(lidarRanges == 0) = lidarMaxRange; ???

[linear_velocity, angular_velocity] = ...
   VPH2010Algorithm(lidarRanges, goal_heading, goal_dist, ...
                    anti_goal_heading ,anti_goal_dist, Vmax, vfh_state);

angular_velocity_rad = angular_velocity *pi/180;
twistMsg.linear.x = linear_velocity;
twistMsg.angular.z = angular_velocity_rad;

toc

disp(twistMsg.linear.x)
disp(twistMsg.angular.z)