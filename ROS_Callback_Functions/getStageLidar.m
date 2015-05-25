% Lidar Callback
function getStageLidar(lidar_msg)
global lidar;

lidar.ranges = lidar_msg.getRanges;    % Read received data and store to ranges

lidar.ranges = lidar.ranges(1:1080);

lidar.ranges(lidar.ranges>lidar.threshold) = lidar.threshold; % prevent infite values

lidar.min_angle = (180*lidar_msg.getAngleMin)/pi;
lidar.max_angle = (180*lidar_msg.getAngleMax)/pi;
lidar.resolution = (180*lidar_msg.getAngleIncrement)/pi;