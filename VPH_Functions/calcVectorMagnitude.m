function m = calcVectorMagnitude(d,w_s)

b = 4/(2*w_s-1);
a = 1 + b*((w_s-1)/2)^2;

m = a - b*(d.^2); % certainty is 1 because we have a lidar
m(m<0) = 0; % Prevents negative numbers