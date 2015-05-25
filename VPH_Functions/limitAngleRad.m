function theta = limitAngleRad( phi )
% This function takes an angle in radians and limits it in the range [-pi, pi]
theta = mod(phi+pi,2*pi) - pi;