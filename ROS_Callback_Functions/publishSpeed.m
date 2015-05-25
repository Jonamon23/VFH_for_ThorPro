function publishSpeed(obj,event)
global node cmd_vel_publisher vfh_state goal curr;

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
twistMsg = rosmatlab.message('geometry_msgs/Twist',node);

msg_lin = twistMsg.getLinear;
msg_ang = twistMsg.getAngular;

% Put into respective slot in Twist msg
msg_lin.setX(linear_velocity);
twistMsg.setLinear(msg_lin);     % Linear Set

msg_ang.setZ(angular_velocity_rad);
twistMsg.setAngular(msg_ang);    % Angular set

cmd_vel_publisher.publish(twistMsg);   % Publish velocities