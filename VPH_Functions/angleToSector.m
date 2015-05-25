function sector = angleToSector(angle, sector_resolution, sensor)
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
%  sector_resolution - 
%  sensor - 
% 
% Outputs:
%  sector - 
% 
% Usage:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
angle = max(min(angle, sensor.max_angle), sensor.min_angle + sector_resolution/2);
%  % NTU
%  clampWithinRange(angle, sensor.min_angle + sector_resolution/2, ...
%                   sensor.max_angle);

sector = round( (angle - sensor.min_angle) / sector_resolution );