%# Comments with %# are Jon's

%system('roslaunch stageroscam IGVC2013_AutoNavBasic3.launch'); 
%system('gnome-terminal --tab --title="Stage" -x bash -c "roslaunch stageroscam IGVC2013_AutoNavBasic4.launch"');
  %#runs launch files
clear, close all
clc

%addpath('/home/administrator/ros/mplab-ros-pkg/matlab_bridge');
%addpath('/home/administrator/ros/mplab-ros-pkg/matlab_bridge/VFHP/VPHSupport');
  %#I think these include the current directory and the support directory

jmb_init(); 
  %#Initalizes ros java

% Change this to your 
MASTER_URI='http://localhost:11311';
NODE_NAME='vph';
node=jmb_init_node(NODE_NAME, MASTER_URI);

% Create a Subscriber
lidarSub=edu.ucsd.SubscriberAdapter(node,'/base_scan','sensor_msgs/LaserScan');
odomSub=edu.ucsd.SubscriberAdapter(node,'/base_pose_ground_truth','nav_msgs/Odometry');
twistPub=jmb_init_publisher(node,'/cmd_vel','geometry_msgs/Twist');


timeout=2;
logger=node.getLog();
vfhStop=false;


goal.x=-18.71;
goal.y=5.95;
Vmax = 2.5; % maximum vehicle speed m/s

while (~vfhStop)    
%for i=1:10
  twistMsg =[];
  twistMsg=org.ros.message.geometry_msgs.Twist();
  lasermsg=lidarSub.takeMessage(timeout);
  odommsg=odomSub.takeMessage(timeout);
  if((~isempty(odommsg)) && (~isempty(lasermsg)))
    curr.x = -1 * odommsg.pose.pose.position.y;
    curr.y = odommsg.pose.pose.position.x;
    x= odommsg.pose.pose.orientation.x;
    y= odommsg.pose.pose.orientation.y;
    z= odommsg.pose.pose.orientation.z;
    w= odommsg.pose.pose.orientation.w;
    [curr.theta, pitch, roll] = quat2angle([w x y z]);
    curr.theta = (mod(curr.theta+pi, 2*pi )-pi);
    curr.theta = curr.theta+pi/2;
    if curr.theta > pi
      curr.theta = curr.theta - 2*pi;
    end

    phi=atan2(goal.y - curr.y,goal.x - curr.x);
    goal_heading = limitAngleRad(phi-curr.theta)*180/pi;
    goal_dist = sqrt((goal.y - curr.y)^2+(goal.x - curr.x)^2);

    if (goal_dist < .5)
        vfhStop =true;
        continue;
    end
    lidarMinAngle = lasermsg.angle_min;
    sensor_min_angle = round(lasermsg.angle_min *180/pi); 
    % There in the main cases of 270 deg lidar and 360 deg
    % virtual-lidar, the min and max angle are spilling over due to
    % accuracy in representing angles in radians.
    lidarMaxAngle = lasermsg.angle_max;
    sensor_max_angle = lasermsg.angle_max *180/pi;
    % There in the main cases of 270 deg lidar and 360 deg
    % virtual-lidar, the min and max angle are spilling over due to
    % accuracy in representing angles in radians.        
    lidarResolution = lasermsg.angle_increment;
    sensor_resolution = round (100*lasermsg.angle_increment *180/pi)/100;

    lidarMaxRange = lasermsg.range_max;
    lidarMinRanges = lasermsg.range_min;
    lidarRanges = lasermsg.ranges;
    lidarRanges(lidarRanges == 0) = lidarMaxRange; 
    lidarRanges(lidarRanges >= 8) = 8;
    % set lidar reading of 0 'nothing'to the maximum lidar range 
    % in compliance with player/stage convention.

    %         [linear_velocity, angular_velocity] = ...
    %             VPH2010Algorithm(lidarRanges, goal_heading, goal_dist, ...
    %                         anti_goal_heading ,anti_goal_dist, Vmax, vfh_state);
    [linear_velocity, angular_velocity] = vfh_fixed(lidarRanges, 0.5,goal_heading);


    angular_velocity_rad = angular_velocity *pi/180;
    twistMsg.linear.x = linear_velocity;
    twistMsg.angular.z = angular_velocity_rad;
    twistPub.publish(twistMsg);

    %fprintf('q0 %f q1 %f q2 %f q3 %f\n',x, y, z, w);
    %fprintf('yaw %f pitch %f roll %f\n',curr.theta, pitch, roll);
    %fprintf('x %f y %f theta %f\n',curr.x, curr.y, curr.theta*180/pi);
    %fprintf('absolute goale heading %f\n',phi*180/pi);
    %fprintf('curTheta %f Goal_Theta %f antGoal_theta %f GoalDist %f linVel %f angVel %f\n'...
    %  ,curr.theta*180/pi,goal_heading,anti_goal_heading,goal_dist,linear_velocity,angular_velocity);
    %pause(.2);
  end
end
%node.shutdown()
