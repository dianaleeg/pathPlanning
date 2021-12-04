%% Path Planning for Drone
% Move drone using RRT Planner
%% 
% Load map of house 

clear all
close all
clc

% load image and create Occupancy map from image
[city_occgrid, city_occgrid_unscaled] = loadMap('city_map.png', 50);
show(city_occgrid_unscaled)

%Set the start and goal states.
start = [30,35,0];
goal = [5,5,0];

%%
UAVsize = 0.2;
inflated_city_occgrid = inflateMap(city_occgrid, UAVsize, 1.0)
[time_traveled, dist_traveled, pthObj, solnInfo] = rrt(inflated_city_occgrid, start, goal)

if not(isfolder('figures'))
    mkdir('figures')
end
    
plotSolvedPath(city_occgrid_unscaled,solnInfo,pthObj,'RRT - City Occupancy Grid with Path','/figures/rrt_city_path_1.png');