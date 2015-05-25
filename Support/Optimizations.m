%% Optimization testing
%{
 This file contains possible optimizations, used experiments, and results
  of the experiements.
 They ones with sufficent proof will be marked "Implemented" once put into 
  the current version of the code along with its version number.
%}
%% Optimization 1 - Implemented in v1.1
%{ 
Observation: 
 global sensor_min_angle sensor_max_angle sensor_resolution;
 global sector_resolution sector_max rscale angleList;
 global MAX_TURNRATE_0MPH MAX_TURNRATE_5MPH MAX_TURNRATE_0MPH_THORARM;
 global MAX_ACCESS_ANGLE;

 By defining global variables using global in each line, it may take 
  longer to define all these global variables. 

Proposed Optimization:
 Use a single "global" on the first line and use "..." to continue to the 
 next line for the other global variables.
%}
% Testing
clear; clc
total = 0;
for loop = 1:100000

  % Original Time:
  tic
  global sensor_min_angle sensor_max_angle sensor_resolution;
  global sector_resolution sector_max rscale angleList;
  global MAX_TURNRATE_0MPH MAX_TURNRATE_5MPH MAX_TURNRATE_0MPH_THORARM;
  global MAX_ACCESS_ANGLE;
  orig_time = toc;

  % Optimized Time:
  tic
  global sensor_min_angle sensor_max_angle sensor_resolution...
         sector_resolution sector_max rscale angleList MAX_TURNRATE_0MPH... 
         MAX_TURNRATE_5MPH MAX_TURNRATE_0MPH_THORARM MAX_ACCESS_ANGLE;
  opt_time = toc;

  time_savings = orig_time - opt_time;
  total = time_savings + total;
end
average_savings = total/loop

%{ 
Results
 If the variables are re-initialized each time (like in the actual code), 
  then the average savings was around 1.65E-06 seconds
%}
%% Optimization 2 
%{
Observation:
 sectorToAngle is only called 4 times and is a one line function besides its 
  global variable initialization. 

Proposed Optimization:
 Integrate single line into main code and add a preceeding comment that you are
  converting between sector to angle. This will save time interms of fucntion
  call, global variable initialization, and return to main.
%}
%% Optimization 3
%{
Observation:
 Lidar is translated Horizontal to vertical initially and again before the polar
  sector conversion

Proposed Optimization:
 Modify parts of code that use the vertical form and only use horizontal form
%}
%% Optimization 4
%{
Observation:
 Variables are global and passed to functions as well. This gives change access
 to functions that don't change the variable.

Proposed Optimization:
 Remove as many global variables are possible and use passed variables.
%}