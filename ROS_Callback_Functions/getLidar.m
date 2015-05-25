function getLidar(~,lidar_msg) % Lidar Callback


global lidar;

lidar.ranges = lidar_msg.Ranges; % Read received data and store to ranges

%% Thor Pro with 270 degree Lidar
% Sick LMS111 over ethernet

lidar.ranges(lidar.ranges<0.01) = lidar.threshold;
lidar.ranges = flipud(lidar.ranges(1:540));
lidar.ranges(lidar.ranges>3)=lidar.threshold;

% lidar.min_angle = -135; % Explicitly define parameters just cuz
% lidar.max_angle = 135;
% lidar.resolution = 0.5;

%% Thor Pro (0.5 degree Resolution Interlace method)
% Sick LMS2XX at 500K Baud

% if(length(scan)==181)                     % If scan is 181 elements,
%   lidar.ranges(1:2:end-1)= scan(1:end-1); % interlace in odd elements
% elseif(length(scan)==180)                 % If scan is 180 elements,
%   lidar.ranges(2:2:end)= scan;            % interlace in even elements
% else
%   error('Error in lidar reading!!!!')
% end
% 
% lidar.ranges(lidar.ranges>lidar.threshold) = lidar.threshold; % prevent infinte values
% lidar.ranges(isnan(lidar.ranges)) = lidar.threshold; % prevent NaN values
% 

% lidar.min_angle = -90; % Explicitly define parameters since they change/are wrong
% lidar.max_angle = 90;
% lidar.resolution = 0.5;

%% Thor Pro (1 degree Res)

% lidar.ranges = lidar.ranges(1:180);

% lidar.ranges(lidar.ranges>lidar.threshold) = lidar.threshold; % prevent infinte values
% lidar.ranges(isnan(lidar.ranges)) = lidar.threshold; % prevent NaN values

% lidar.min_angle = -90;
% lidar.max_angle = 90;
% lidar.resolution = 1;