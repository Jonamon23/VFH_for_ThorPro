function angle = sectorToAngle(sector, sensor_min_angle, sector_resolution)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This function Converts Sector Number to Corresponding Angle
% 
% Updated By: Jonathon Kreska
% Version Date: Jan 27, 2015
% Version: 1.0
% 
% Changelog:
%  1.0: Initial Release
% 
% Inputs:
%  sector - Array of sector numbers OR single sector number
%  sensor_min_angle - 
%  sector_resolution - 
% 
% Outputs:
%  angle - Array of sector angles OR single sector angle
% 
% Usage:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
angle = sector * sector_resolution + sensor_min_angle - sector_resolution/2;