% Setup
clear all; close all; clc;

addpath('C:\Users\Jonathon\Dropbox\SCHOOL\Local Nav (VFH)\VFH for ThorPro')
addpath('C:\Users\Jonathon\Dropbox\SCHOOL\Local Nav (VFH)\VFH for ThorPro\ROS_Callback_Functions')
addpath('C:\Users\Jonathon\Dropbox\SCHOOL\Local Nav (VFH)\VFH for ThorPro\VPH_Functions')

% Set IP address of the host where ROS master is running (roscore)
rosMasterIp = '192.168.0.102';

% Set IP address of the local IP
localhostIp = '192.168.0.124'; 

tuneVPH;

global node;
node=rosmatlab.node('VFH',rosMasterIp,11311,'rosIP',localhostIp);

% LIDAR
% Setup Subscriber Node to look for lidar scans
global lidar;

  lidar_sub=rosmatlab.subscriber('/scan','sensor_msgs/LaserScan',1,node);
  % Define Callback for Lidar
  lidar_sub.setOnNewMessageListeners({@getLidar});

disp('Lidar Callback Ready')

while 1
  plot(lidar.ranges)
%   polar(linspace(225,90,length(lidar.ranges)).*pi/180,lidar.ranges');
  drawnow
end