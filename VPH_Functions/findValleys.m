function valleys = findValleys(bin_hist,sector_max)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Abstract:
%  This function checks for available sectors to go through using edge detection on 
%   bin_hist, the beginning and end of valleys can be found.
% 
% Updated By: Jonathon Kreska
% Version Date: Jan 27, 2015
% Version: 1.0
% 
% Changelog:
%  1.0: Initial Release
% 
% Inputs:
%  bin_hist - represents sectors with an obstacle density less than the threshold
% 
% Outputs:
%  valleys - 
% 
% Usage:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup
START = 1; END = 2; WIDTH = 3;
min_valley = 5; % TUN
% After sector expansion, you only need 1 sector free
if(sum(bin_hist) == sector_max)
  % The only valleys are left or right. We are trapped.
  valleys = [];
  return;
end

%% Find the boundaries of "valleys", or sets of sectors.
  % Remember: 0 = free   1 = blocked
edges = [bin_hist, 1] - [1, bin_hist];
  % ie: [(1 1 0 0 0 1 1), 0] - [0, (1 1 0 0 0 1 1)] = [0 0 -1 0 0 0 1 0]
  % So valleys start at -1 and end at the zero before the 1

valley_start = find(edges < 0); % Falling edges are starts of valleys
valley_end = find(edges > 0) - 1; % Rising edges are ends of valleys.

valley_width = valley_end - valley_start + 1;

%% Locate Good valleys (greater than min_valley width)
good_valleys = valley_width >= min_valley;
good_count = sum(good_valleys);
if(good_count < 1) 
  valleys = [];
  return;
end

%% Return only the good_valleys
valleys = zeros(good_count, 5, 'single');
valleys(:, START) = valley_start(good_valleys);
valleys(:, END) = valley_end(good_valleys);
valleys(:, WIDTH) = valley_width(good_valleys);