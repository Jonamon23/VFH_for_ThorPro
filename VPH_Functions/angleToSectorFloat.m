function sector = angleToSectorFloat(angle, sensor_min_angle, sector_resolution)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This function Converts Angle to Corresponding Sector Number
% 
% Updated By: Jonathon Kreska
% Version Date: Jan 27, 2015
% Version: 1.0
% 
% Changelog:
%  1.0: Initial Release
% 
% Inputs:
%  angle - 
%  sensor_min_angle - 
%  sector_resolution - 
% 
% Outputs:
%  sector - 
% 
% Usage:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  NTU
%  angle = max(min(angle, -180), 180);
%  clampWithinRange(angle, sensor_min_angle + sector_resolution/2, sensor_max_angle);

sector = (angle - sensor_min_angle) / sector_resolution;