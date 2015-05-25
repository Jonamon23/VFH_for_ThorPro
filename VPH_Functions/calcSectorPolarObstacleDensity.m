function H_p_k = calcSectorPolarObstacleDensity(h_p,m,sector)
%%%%%
% Updated by: Jonathon Kreska
% Version Date: May 25, 2015
% Version: 1.1
% 
% Changelog:
%  1.0: Initial Release
%  1.1: Removed Debugging.
%
%
%%%%%

global r_rs lidar;

enlargement_angle = (180/pi)*real(asin(r_rs./h_p));

lidar_angles = (lidar.min_angle:lidar.resolution:lidar.max_angle-lidar.resolution) + (lidar.max_angle+1);

LEFT = 1; RIGHT = 2; DANGER = 3;
expanded_h_p(:,LEFT) = lidar_angles - enlargement_angle;
expanded_h_p(:,RIGHT) = lidar_angles + enlargement_angle;

expanded_h_p(expanded_h_p<0) = 0; % puts negative values into sector 1

h_prime = floor(expanded_h_p/sector.alpha)+1; % translate angles to sectors

h_prime(h_prime>sector.count) = sector.count; % limit sectors to max num of sectors

%m(1:round(end/5)) = .8*m(1:round(end/5));
%m(round(4*end/5):end) = .8*m(round(4*end/5):end);

h_prime = [h_prime,m']; % add danger vlaues to third column

H_p = zeros(length(lidar_angles),sector.count); % init array for speed

for i = 1:length(h_prime)
  H_p(i,h_prime(i,LEFT):h_prime(i,RIGHT)) = h_prime(i,DANGER); % do magic
end 

H_p_k = sum(H_p); % Sum each column's dangers into a single row