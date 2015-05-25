function publishSpeed(obj,event)
global cmd_vel_publisher vfh_state goal curr;

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

msg = rosmessage(cmd_vel_publisher);

msg.Linear.X = linear_velocity;
msg.Angular.Z = angular_velocity_rad;
send(cmd_vel_publisher,msg)