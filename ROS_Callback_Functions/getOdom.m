function getOdom(~,odom_msg) % Odom Callback

global curr;

% qx=odom_msg.getPose.getPose.getOrientation.getX;
% qy=odom_msg.getPose.getPose.getOrientation.getY;
% qz=odom_msg.getPose.getPose.getOrientation.getZ;
% qw=odom_msg.getPose.getPose.getOrientation.getW;
% [theta, ~, ~]=quat2angle([qw qx qy qz]);
% 
% curr.x=odom_msg.getPose.getPose.getPosition.getX;
% curr.y=odom_msg.getPose.getPose.getPosition.getY;
% curr.theta=theta;

qx=odom_msg.Pose.Pose.Orientation.X;
qy=odom_msg.Pose.Pose.Orientation.Y;
qz=odom_msg.Pose.Pose.Orientation.Z;
qw=odom_msg.Pose.Pose.Orientation.W;
axang=quat2axang([qw qx qy qz]);


curr.x=odom_msg.Pose.Pose.Position.X;
curr.y=odom_msg.Pose.Pose.Position.Y;
curr.theta=axang(4);