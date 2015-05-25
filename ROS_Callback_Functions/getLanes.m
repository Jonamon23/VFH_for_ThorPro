function getLanes(lanes_msg) % Lidar Lanes Callback

global lanes;

lanes.ranges = lanes_msg.getRanges;    % Read received data and store to ranges
lanes.ranges = lanes.ranges(1:540);
% Mirror lanes 45 degrees behind the robot
lanes.ranges(90:-1:1)=lanes.ranges(91:1:180);
lanes.ranges(540:-1:451)=lanes.ranges(361:1:450);

% lanes.min_angle = -90; % Explicitly define parameters since they change/are wrong
% lanes.max_angle = 90;
% lanes.resolution = 0.5;